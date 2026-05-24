#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rb23_audio_node.py

Nó ROS 2 responsável pelo áudio local do GuardROS.

Funções principais:
-------------------
1. Assinar o modo de áudio em /rb23/audio_mode.
2. Assinar o áudio recebido do robô em /rb23/audio/rx.
3. Reproduzir no speaker local quando estiver em modo "ouvir".
4. Capturar o microfone local quando estiver em modo "falar".
5. Publicar áudio capturado em /rb23/audio/tx.

Arquitetura:
------------
RB23 -> rb23_driver_node -> /rb23/audio/rx -> rb23_audio_node -> speaker local
microfone local -> rb23_audio_node -> /rb23/audio/tx -> rb23_driver_node -> RB23

Modos de operação:
------------------
- MODE_SILENCE:
    não reproduz e não transmite
- MODE_LISTEN:
    reproduz áudio vindo do robô
- MODE_TALK:
    captura microfone e publica para o driver

Observações:
------------
- Este nó usa sounddevice para falar com os dispositivos de áudio do PC.
- Ele foi escrito para ser didático e fortemente comentado.
- A lógica foi inspirada no comportamento validado do cliente Linux,
  mas adaptada para a arquitetura ROS 2 modular.
"""

from collections import deque
import threading
import time
from typing import Deque, Optional

import numpy as np
import rclpy
from guardros_msgs.msg import AudioFrame
from guardros_msgs.msg import AudioMode
from rclpy.node import Node

try:
    import sounddevice as sd
    SOUNDDEVICE_AVAILABLE = True
    SOUNDDEVICE_IMPORT_ERROR = "-"
except Exception as exc:
    sd = None
    SOUNDDEVICE_AVAILABLE = False
    SOUNDDEVICE_IMPORT_ERROR = str(exc)


# ============================================================
# CONFIGURAÇÕES DE ÁUDIO
# ============================================================

# Tópicos ROS usados por este nó.
AUDIO_MODE_TOPIC = "/rb23/audio_mode"
AUDIO_RX_TOPIC = "/rb23/audio/rx"
AUDIO_TX_TOPIC = "/rb23/audio/tx"

# ------------------------------------------------------------
# REPRODUÇÃO DO ÁUDIO VINDO DO ROBÔ
# ------------------------------------------------------------
# O driver publica o áudio RX com estes metadados.
AUDIO_RX_EXPECTED_ENCODING = "pcm_u8"
AUDIO_RX_EXPECTED_SAMPLE_RATE = 16000
AUDIO_RX_EXPECTED_CHANNELS = 1

# Para o speaker local, preferimos tocar em 48 kHz,
# fazendo um upsample simples por interpolação.
AUDIO_OUTPUT_RATE = 48000
AUDIO_RX_UPSAMPLE_FACTOR = 3
AUDIO_OUTPUT_BLOCKSIZE = 1024

# Fila de buffers já convertidos para float32.
AUDIO_RX_QUEUE_MAX = 100

# ------------------------------------------------------------
# CAPTURA DE MICROFONE PARA ENVIAR AO ROBÔ
# ------------------------------------------------------------
AUDIO_TX_ENCODING = "pcm_s16le"
AUDIO_TX_SAMPLE_RATE = 48000
AUDIO_TX_CHANNELS = 1
AUDIO_TX_SAMPLES_PER_PACKET = 1024

# Ganho aplicado ao microfone.
AUDIO_TX_GAIN = 0.35

# Noise gate simples:
# abaixo deste RMS, enviamos silêncio real.
AUDIO_TX_GATE_RMS = 450.0


class RB23AudioNode(Node):
    """
    Nó ROS 2 de áudio local do GuardROS.
    """

    def __init__(self) -> None:
        super().__init__("rb23_audio_node")

        # =========================================================
        # PARÂMETROS
        # =========================================================
        self.declare_parameter("audio_tx_gain", AUDIO_TX_GAIN)
        self.declare_parameter("audio_tx_gate_rms", AUDIO_TX_GATE_RMS)

        self.audio_tx_gain = float(self.get_parameter("audio_tx_gain").value)
        self.audio_tx_gate_rms = float(self.get_parameter("audio_tx_gate_rms").value)

        # =========================================================
        # ESTADO INTERNO
        # =========================================================
        self.state_lock = threading.RLock()

        # Modo inicial: silêncio.
        self.current_audio_mode = AudioMode.MODE_SILENCE

        # Último estado textual para logs/diagnóstico.
        self.audio_status = "silêncio"

        # Fila de áudio RX pronto para tocar.
        self.audio_rx_queue: Deque[np.ndarray] = deque(maxlen=AUDIO_RX_QUEUE_MAX)

        # Streams locais.
        self.audio_output_stream = None
        self.audio_input_stream = None

        self.audio_output_started = False
        self.audio_input_started = False

        self.last_audio_rx_time = 0.0
        self.last_audio_tx_time = 0.0

        self.audio_packets_rx = 0
        self.audio_packets_tx = 0
        self.audio_packets_rx_dropped = 0
        self.audio_played_chunks = 0

        self.audio_error = "-"

        # =========================================================
        # ROS PUB/SUB
        # =========================================================
        self.audio_tx_pub = self.create_publisher(
            AudioFrame,
            AUDIO_TX_TOPIC,
            10,
        )

        self.create_subscription(
            AudioMode,
            AUDIO_MODE_TOPIC,
            self.audio_mode_callback,
            10,
        )

        self.create_subscription(
            AudioFrame,
            AUDIO_RX_TOPIC,
            self.audio_rx_callback,
            10,
        )

        # =========================================================
        # INICIALIZAÇÃO DAS STREAMS DE ÁUDIO
        # =========================================================
        self.setup_audio_output()
        self.setup_audio_input()

        if SOUNDDEVICE_AVAILABLE:
            self.get_logger().info(
                "rb23_audio_node iniciado com sounddevice disponível."
            )
        else:
            self.get_logger().warn(
                "sounddevice não está disponível. "
                f"Detalhe: {SOUNDDEVICE_IMPORT_ERROR}"
            )

    # =========================================================
    # CALLBACKS ROS
    # =========================================================

    def audio_mode_callback(self, msg: AudioMode) -> None:
        """
        Atualiza o modo atual de áudio.
        """
        with self.state_lock:
            self.current_audio_mode = int(msg.mode)

            # Sempre limpamos a fila ao trocar de modo para evitar
            # tocar áudio antigo quando o usuário alterna rapidamente.
            self.audio_rx_queue.clear()

            if self.current_audio_mode == AudioMode.MODE_SILENCE:
                self.audio_status = "silêncio bilateral"
            elif self.current_audio_mode == AudioMode.MODE_LISTEN:
                self.audio_status = "ouvindo robô"
            elif self.current_audio_mode == AudioMode.MODE_TALK:
                self.audio_status = "falando com robô"
            else:
                self.audio_status = f"modo desconhecido ({self.current_audio_mode})"

        self.get_logger().info(f"Modo de áudio alterado para: {self.audio_status}")

    def audio_rx_callback(self, msg: AudioFrame) -> None:
        """
        Recebe um frame de áudio vindo do robô e o prepara para reprodução local.
        """
        if msg.encoding != AUDIO_RX_EXPECTED_ENCODING:
            return

        if msg.channels != AUDIO_RX_EXPECTED_CHANNELS:
            return

        # Neste projeto esperamos 16 kHz no RX.
        # Caso no futuro o driver publique outro sample_rate,
        # podemos generalizar a conversão.
        if int(msg.sample_rate) != AUDIO_RX_EXPECTED_SAMPLE_RATE:
            return

        with self.state_lock:
            if self.current_audio_mode != AudioMode.MODE_LISTEN:
                return

        try:
            payload = np.array(msg.data, dtype=np.uint8).astype(np.float32)

            if payload.size == 0:
                return

            # Conversão semelhante à usada no cliente Linux:
            # uint8 centrado em 128 -> float em torno de zero.
            mono = (payload - 128.0) / 128.0

            # Remove offset DC.
            mono = mono - float(np.mean(mono))

            # Upsample simples para 48 kHz.
            if AUDIO_RX_UPSAMPLE_FACTOR > 1 and mono.size >= 2:
                x_old = np.arange(mono.size, dtype=np.float32)
                x_new = np.linspace(
                    0,
                    mono.size - 1,
                    mono.size * AUDIO_RX_UPSAMPLE_FACTOR,
                    dtype=np.float32
                )
                mono = np.interp(x_new, x_old, mono).astype(np.float32)

            # Pequeno ganho para melhorar a audição.
            mono = np.clip(mono * 1.4, -1.0, 1.0)

            with self.state_lock:
                if len(self.audio_rx_queue) >= AUDIO_RX_QUEUE_MAX:
                    self.audio_packets_rx_dropped += 1
                    return

                self.audio_rx_queue.append(mono)
                self.audio_packets_rx += 1
                self.last_audio_rx_time = time.time()

        except Exception as exc:
            with self.state_lock:
                self.audio_error = str(exc)

    # =========================================================
    # SETUP DAS STREAMS DE ÁUDIO
    # =========================================================

    def setup_audio_output(self) -> None:
        """
        Inicializa a stream de saída de áudio local.
        """
        if not SOUNDDEVICE_AVAILABLE:
            with self.state_lock:
                self.audio_output_started = False
                self.audio_error = SOUNDDEVICE_IMPORT_ERROR
            return

        try:
            self.audio_output_stream = sd.OutputStream(
                samplerate=AUDIO_OUTPUT_RATE,
                channels=1,
                dtype="float32",
                blocksize=AUDIO_OUTPUT_BLOCKSIZE,
                callback=self.audio_output_callback,
                latency="low",
            )
            self.audio_output_stream.start()

            with self.state_lock:
                self.audio_output_started = True

        except Exception as exc:
            with self.state_lock:
                self.audio_output_started = False
                self.audio_error = str(exc)

            self.get_logger().error(f"Falha ao abrir saída de áudio: {exc}")

    def setup_audio_input(self) -> None:
        """
        Inicializa a stream de entrada de áudio local (microfone).
        """
        if not SOUNDDEVICE_AVAILABLE:
            with self.state_lock:
                self.audio_input_started = False
                self.audio_error = SOUNDDEVICE_IMPORT_ERROR
            return

        try:
            self.audio_input_stream = sd.InputStream(
                samplerate=AUDIO_TX_SAMPLE_RATE,
                channels=AUDIO_TX_CHANNELS,
                dtype="int16",
                blocksize=AUDIO_TX_SAMPLES_PER_PACKET,
                callback=self.audio_input_callback,
                latency="low",
            )
            self.audio_input_stream.start()

            with self.state_lock:
                self.audio_input_started = True

        except Exception as exc:
            with self.state_lock:
                self.audio_input_started = False
                self.audio_error = str(exc)

            self.get_logger().error(f"Falha ao abrir microfone: {exc}")

    # =========================================================
    # CALLBACKS DO SOUNDDEVICE
    # =========================================================

    def audio_output_callback(self, outdata, frames, time_info, status) -> None:
        """
        Callback chamado pelo sounddevice para preencher o speaker local.
        """
        if status:
            with self.state_lock:
                self.audio_error = f"callback saída: {status}"

        outdata.fill(0)

        with self.state_lock:
            if self.current_audio_mode != AudioMode.MODE_LISTEN:
                return

        filled = 0

        while filled < frames:
            with self.state_lock:
                chunk = self.audio_rx_queue[0] if self.audio_rx_queue else None

            if chunk is None or len(chunk) == 0:
                break

            take = min(frames - filled, len(chunk))
            outdata[filled:filled + take, 0] = chunk[:take]
            filled += take

            with self.state_lock:
                if take >= len(self.audio_rx_queue[0]):
                    self.audio_rx_queue.popleft()
                else:
                    self.audio_rx_queue[0] = self.audio_rx_queue[0][take:]

        if filled > 0:
            with self.state_lock:
                self.audio_played_chunks += 1

    def audio_input_callback(self, indata, frames, time_info, status) -> None:
        """
        Callback chamado pelo sounddevice quando há novo bloco do microfone.
        """
        if status:
            with self.state_lock:
                self.audio_error = f"callback entrada: {status}"

        with self.state_lock:
            if self.current_audio_mode != AudioMode.MODE_TALK:
                return

        try:
            mono_f = indata[:, 0].astype(np.float32, copy=True)

            # Remove offset DC.
            mono_f -= float(np.mean(mono_f))

            rms = float(np.sqrt(np.mean(np.square(mono_f))) if mono_f.size else 0.0)

            # Noise gate:
            # se estiver abaixo do limiar, enviamos silêncio real.
            if rms < self.audio_tx_gate_rms:
                mono_i16 = np.zeros(AUDIO_TX_SAMPLES_PER_PACKET, dtype=np.int16)
            else:
                if self.audio_tx_gain != 1.0:
                    mono_f *= self.audio_tx_gain

                mono_f = np.clip(mono_f, -32768, 32767)
                mono_i16 = mono_f.astype(np.int16)

            payload = mono_i16.tobytes()

            msg = AudioFrame()
            msg.data = list(payload)
            msg.sample_rate = AUDIO_TX_SAMPLE_RATE
            msg.channels = AUDIO_TX_CHANNELS
            msg.encoding = AUDIO_TX_ENCODING
            msg.samples_per_channel = AUDIO_TX_SAMPLES_PER_PACKET

            self.audio_tx_pub.publish(msg)

            with self.state_lock:
                self.audio_packets_tx += 1
                self.last_audio_tx_time = time.time()

        except Exception as exc:
            with self.state_lock:
                self.audio_error = str(exc)

    # =========================================================
    # ENCERRAMENTO
    # =========================================================

    def close(self) -> None:
        """
        Fecha as streams de áudio de forma organizada.
        """
        try:
            if self.audio_output_stream is not None:
                self.audio_output_stream.stop()
                self.audio_output_stream.close()
        except Exception:
            pass

        try:
            if self.audio_input_stream is not None:
                self.audio_input_stream.stop()
                self.audio_input_stream.close()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RB23AudioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()