#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rb23_driver_node.py

Driver ROS 2 para o GuardBot RollerBot 23 (RB23).

Este nó tem como função principal:
1. Enviar continuamente comandos UDP para o robô.
2. Receber a telemetria JSON enviada pelo robô.
3. Publicar essa telemetria em:
   - um tópico bruto de depuração (/rb23/telemetry_json)
   - vários tópicos ROS tipados, mais adequados para integração com outros nós
4. Receber comandos ROS 2 em tópicos e convertê-los
   para o formato interno usado pelo protocolo UDP do RB23.
5. Detectar frames JPEG vindos no mesmo fluxo UDP e publicá-los
   em um tópico ROS de imagem comprimida:
   - /rb23/camera/image/compressed
6. Detectar pacotes de áudio vindos no mesmo fluxo UDP e publicá-los
   em um tópico ROS tipado:
   - /rb23/audio/rx
7. Receber frames de áudio ROS em /rb23/audio/tx e encapsulá-los
   no formato esperado pelo robô.

Arquitetura atual:
------------------
Tópicos ROS -> rb23_driver_node -> UDP -> RB23

Tópicos publicados:
-------------------
- /rb23/telemetry_json              -> std_msgs/String
- /rb23/battery                     -> guardros_msgs/Battery
- /rb23/temperatures                -> guardros_msgs/Temperatures
- /rb23/drive_status                -> guardros_msgs/DriveStatus
- /rb23/magnetometer                -> guardros_msgs/Magnetometer
- /rb23/connection_status           -> guardros_msgs/ConnectionStatus
- /rb23/command_state               -> guardros_msgs/CommandState
- /rb23/camera/image/compressed     -> sensor_msgs/CompressedImage
- /rb23/audio/rx                    -> guardros_msgs/AudioFrame

Tópicos assinados:
------------------
- /rb23/cmd_vel            -> geometry_msgs/Twist
- /rb23/force_rpi          -> std_msgs/Bool
- /rb23/cam_stable         -> std_msgs/Bool
- /rb23/cam_angle          -> std_msgs/Float32
- /rb23/audio/tx           -> guardros_msgs/AudioFrame

Observações importantes:
------------------------
- O protocolo do RB23 usa um byte inicial de identificação antes do JSON.
- Alguns valores da telemetria podem vir como listas de um único elemento.
  Por isso o código trata esse caso explicitamente.
- O percentual da bateria é estimado a partir da tensão, pois até o momento
  não identificamos no JSON um campo explícito de porcentagem.
- O fluxo UDP do RB23 pode carregar diferentes tipos de dados no mesmo socket:
  telemetria, vídeo, áudio e outros pacotes.
- Este arquivo foi escrito de forma didática para facilitar manutenção futura.

Fail-safe:
----------
Este driver implementa um timeout de segurança para /rb23/cmd_vel.
Se o teclado (ou qualquer outro nó futuro) parar de publicar comandos,
o driver força automaticamente:
    speed = 0.0
    rotation = 0.0
Isso evita que o robô continue em movimento indefinidamente.
"""

import json
import socket
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from guardros_msgs.msg import AudioFrame
from guardros_msgs.msg import Battery
from guardros_msgs.msg import CommandState
from guardros_msgs.msg import ConnectionStatus
from guardros_msgs.msg import DriveStatus
from guardros_msgs.msg import Magnetometer
from guardros_msgs.msg import Temperatures
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


# ============================================================
# CONFIGURAÇÕES GERAIS DO DRIVER
# ============================================================

# Endereço do servidor atualmente utilizado pelo ecossistema RB23.
SERVER_HOST = "RB23-Brazil.harv-guardbot.org"
SERVER_PORT = 11000

# ROBOT_ID padrão caso o usuário não forneça outro via parâmetro ROS.
DEFAULT_ROBOT_ID = 1

# Timeout do socket UDP.
SOCKET_TIMEOUT = 0.2

# Período de publicação ROS.
PUBLISH_PERIOD = 0.10

# Período de envio contínuo de comandos ao robô.
SEND_PERIOD = 0.05

# Timeout de segurança para /rb23/cmd_vel.
DEFAULT_CMD_VEL_TIMEOUT_SEC = 0.30

# Estimativa provisória para bateria de lítio nominal 12 V.
BATTERY_EMPTY_VOLTAGE = 9.0
BATTERY_FULL_VOLTAGE = 12.6


# ============================================================
# CONFIGURAÇÕES DE VÍDEO
# ============================================================

# Tópico ROS onde o JPEG extraído do UDP será publicado.
VIDEO_TOPIC_COMPRESSED = "/rb23/camera/image/compressed"

# Pacotes muito pequenos dificilmente contêm um JPEG útil.
VIDEO_PACKET_MIN_BYTES = 256

# Marcadores padrão de início e fim de imagem JPEG.
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"


# ============================================================
# CONFIGURAÇÕES DE ÁUDIO
# ============================================================

# Tópicos ROS de áudio.
AUDIO_RX_TOPIC = "/rb23/audio/rx"
AUDIO_TX_TOPIC = "/rb23/audio/tx"

# Identificador esperado para pacotes de áudio RX vindos do robô.
# Esta convenção já foi observada e utilizada no cliente Linux.
AUDIO_RX_ID_BASE = 0x80

# Prefixo usado no envio de áudio TX ao robô.
# Formato observado/validado anteriormente:
#   0x01 + 1024 amostras PCM16 little-endian mono
AUDIO_TX_PREFIX = 0x01

# Metadados ROS para o áudio recebido do robô.
# No cliente Linux, o áudio RX foi tratado como PCM unsigned 8-bit,
# mono, e depois ampliado para reprodução local.
# Aqui no driver publicamos o frame bruto e informamos esses metadados.
AUDIO_RX_ENCODING = "pcm_u8"
AUDIO_RX_CHANNELS = 1
AUDIO_RX_SAMPLE_RATE = 16000

# Metadados esperados para o áudio TX vindo do ROS.
AUDIO_TX_EXPECTED_ENCODING = "pcm_s16le"
AUDIO_TX_EXPECTED_CHANNELS = 1
AUDIO_TX_EXPECTED_SAMPLE_RATE = 48000
AUDIO_TX_SAMPLES_PER_PACKET = 1024
AUDIO_TX_PAYLOAD_BYTES = 1 + 2 * AUDIO_TX_SAMPLES_PER_PACKET

# Limite inferior razoável para diferenciar áudio RX de ruído.
AUDIO_RX_PACKET_MIN = 64

# Quantos bytes do cabeçalho do pacote RX devem ser ignorados
# antes dos dados de áudio. No protocolo observado, o primeiro
# byte é o ID do pacote.
AUDIO_RX_HEADER_SKIP = 1


# ============================================================
# FUNÇÕES AUXILIARES
# ============================================================

def now_millis() -> float:
    """
    Retorna o tempo atual em milissegundos.
    """
    return time.time() * 1000.0


def compute_tx_id(robot_id: int) -> int:
    """
    Calcula o identificador de transmissão (TX_ID) usado nos pacotes
    enviados ao robô.
    """
    return ((0xC0 + robot_id) & 0x7F)


def compute_rx_id(robot_id: int) -> int:
    """
    Calcula o identificador de recepção (RX_ID) esperado nos pacotes
    de telemetria vindos do robô.
    """
    return (0xC0 + robot_id) & 0xFF


def compute_audio_rx_id(robot_id: int) -> int:
    """
    Calcula o identificador de recepção dos pacotes de áudio vindos do robô.
    """
    return (AUDIO_RX_ID_BASE + robot_id) & 0xFF


def build_command_packet(tx_id: int, payload: Dict[str, Any]) -> bytes:
    """
    Monta o pacote UDP de comando a ser enviado ao robô.

    Estrutura:
        1 byte de identificador TX + JSON UTF-8
    """
    data = dict(payload)
    data["absolute_ping_millis"] = now_millis()
    payload_bytes = json.dumps(data, separators=(",", ":")).encode("utf-8")
    return bytes([tx_id]) + payload_bytes


def try_decode_telemetry(packet: bytes) -> Optional[Dict[str, Any]]:
    """
    Tenta decodificar um pacote UDP recebido como telemetria JSON.

    Estrutura esperada:
        1 byte de identificador RX + JSON UTF-8

    Se o pacote não for JSON válido, retorna None.
    """
    if not packet or len(packet) < 2:
        return None

    msg_id = packet[0]
    payload = packet[1:]

    try:
        data = json.loads(payload.decode("utf-8"))
    except Exception:
        return None

    return {"msg_id": msg_id, "data": data}


def extract_jpeg_from_packet(packet: bytes) -> Optional[bytes]:
    """
    Procura um frame JPEG dentro de um pacote UDP bruto.

    Estratégia:
    - localizar o marcador de início JPEG (FFD8)
    - localizar o marcador de fim JPEG (FFD9)
    - extrair apenas esse trecho

    Retorna:
    - bytes do JPEG, se encontrado
    - None, se o pacote não contiver um JPEG válido
    """
    if not packet or len(packet) < VIDEO_PACKET_MIN_BYTES:
        return None

    start = packet.find(JPEG_SOI)
    if start < 0:
        return None

    end = packet.find(JPEG_EOI, start + 2)
    if end < 0:
        return None

    jpeg_bytes = packet[start:end + 2]

    if len(jpeg_bytes) < VIDEO_PACKET_MIN_BYTES:
        return None

    return jpeg_bytes


def is_probable_audio_rx(packet: bytes, expected_audio_rx_id: int) -> bool:
    """
    Tenta determinar se um pacote UDP bruto parece ser um pacote de áudio RX.

    Critérios usados:
    - tamanho mínimo razoável
    - primeiro byte compatível com o ID de áudio esperado
    - não conter assinatura típica de JPEG
    """
    if not packet or len(packet) < AUDIO_RX_PACKET_MIN:
        return False

    if packet[0] != expected_audio_rx_id:
        return False

    if JPEG_SOI in packet or JPEG_EOI in packet:
        return False

    return True


def extract_audio_rx_payload(packet: bytes, expected_audio_rx_id: int) -> Optional[bytes]:
    """
    Extrai o payload de áudio vindo do robô.

    Neste estágio, não decodificamos o áudio para float nem fazemos
    upsample. O driver apenas publica o payload bruto em ROS.

    Retorna:
    - bytes do áudio, se o pacote parecer válido
    - None, caso contrário
    """
    if not is_probable_audio_rx(packet, expected_audio_rx_id):
        return None

    payload = packet[AUDIO_RX_HEADER_SKIP:]

    if len(payload) < (AUDIO_RX_PACKET_MIN - AUDIO_RX_HEADER_SKIP):
        return None

    return payload


def clamp(value: float, low: float, high: float) -> float:
    """
    Limita um valor ao intervalo [low, high].
    """
    return max(low, min(value, high))


def safe_first(value: Any) -> Any:
    """
    Se o valor vier como lista não vazia, retorna o primeiro elemento.
    Caso contrário, retorna o próprio valor.
    """
    if isinstance(value, list) and value:
        return value[0]
    return value


def to_float(value: Any, default: float = 0.0) -> float:
    """
    Converte um valor qualquer para float de forma robusta.
    """
    value = safe_first(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def to_int(value: Any, default: int = 0) -> int:
    """
    Converte um valor qualquer para int de forma robusta.
    """
    value = safe_first(value)
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def to_bool(value: Any, default: bool = False) -> bool:
    """
    Converte um valor qualquer para bool de forma robusta.
    """
    value = safe_first(value)

    if isinstance(value, bool):
        return value

    if isinstance(value, (int, float)):
        return bool(value)

    return default


def estimate_battery_percentage(voltage: float) -> float:
    """
    Estima o percentual da bateria a partir da tensão.
    """
    if BATTERY_FULL_VOLTAGE <= BATTERY_EMPTY_VOLTAGE:
        return 0.0

    percentage = 100.0 * (voltage - BATTERY_EMPTY_VOLTAGE) / (
        BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE
    )
    return clamp(percentage, 0.0, 100.0)


# ============================================================
# CLASSE PRINCIPAL DO NÓ ROS
# ============================================================

class RB23RosDriver(Node):
    """
    Nó ROS 2 principal do GuardROS para comunicação com o RB23.

    Responsabilidades:
    - abrir e manter socket UDP conectado ao servidor do robô
    - enviar comandos ao robô em uma thread dedicada
    - receber telemetria/vídeo/áudio em outra thread dedicada
    - publicar esses dados em tópicos ROS
    - receber comandos ROS e convertê-los para o protocolo do RB23
    """

    def __init__(self) -> None:
        super().__init__("rb23_driver_node")

        # --------------------------------------------------------
        # PARÂMETROS ROS
        # --------------------------------------------------------
        self.declare_parameter("robot_id", DEFAULT_ROBOT_ID)
        self.declare_parameter("cmd_vel_timeout_sec", DEFAULT_CMD_VEL_TIMEOUT_SEC)

        self.server_host = SERVER_HOST
        self.server_port = SERVER_PORT
        self.robot_id = int(self.get_parameter("robot_id").value)
        self.cmd_vel_timeout_sec = float(
            self.get_parameter("cmd_vel_timeout_sec").value
        )

        # IDs do protocolo UDP do RB23.
        self.tx_id = compute_tx_id(self.robot_id)
        self.rx_id = compute_rx_id(self.robot_id)
        self.audio_rx_id = compute_audio_rx_id(self.robot_id)

        # --------------------------------------------------------
        # PUBLISHERS ROS
        # --------------------------------------------------------
        self.telemetry_pub = self.create_publisher(
            String,
            "/rb23/telemetry_json",
            10,
        )

        self.battery_pub = self.create_publisher(
            Battery,
            "/rb23/battery",
            10,
        )

        self.temperatures_pub = self.create_publisher(
            Temperatures,
            "/rb23/temperatures",
            10,
        )

        self.drive_status_pub = self.create_publisher(
            DriveStatus,
            "/rb23/drive_status",
            10,
        )

        self.magnetometer_pub = self.create_publisher(
            Magnetometer,
            "/rb23/magnetometer",
            10,
        )

        self.connection_status_pub = self.create_publisher(
            ConnectionStatus,
            "/rb23/connection_status",
            10,
        )

        self.command_state_pub = self.create_publisher(
            CommandState,
            "/rb23/command_state",
            10,
        )

        # Publisher do vídeo comprimido.
        self.camera_compressed_pub = self.create_publisher(
            CompressedImage,
            VIDEO_TOPIC_COMPRESSED,
            10,
        )

        # Publisher do áudio RX.
        self.audio_rx_pub = self.create_publisher(
            AudioFrame,
            AUDIO_RX_TOPIC,
            10,
        )

        # --------------------------------------------------------
        # SUBSCRIBERS ROS
        # --------------------------------------------------------
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/rb23/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.force_rpi_sub = self.create_subscription(
            Bool,
            "/rb23/force_rpi",
            self.force_rpi_callback,
            10,
        )

        self.cam_stable_sub = self.create_subscription(
            Bool,
            "/rb23/cam_stable",
            self.cam_stable_callback,
            10,
        )

        self.cam_angle_sub = self.create_subscription(
            Float32,
            "/rb23/cam_angle",
            self.cam_angle_callback,
            10,
        )

        self.audio_tx_sub = self.create_subscription(
            AudioFrame,
            AUDIO_TX_TOPIC,
            self.audio_tx_callback,
            10,
        )

        # --------------------------------------------------------
        # ESTADO INTERNO COMPARTILHADO ENTRE THREADS
        # --------------------------------------------------------
        self.state_lock = threading.Lock()
        self.socket_lock = threading.Lock()

        # Última telemetria recebida do robô.
        self.latest_telemetry: Dict[str, Any] = {}

        # Informações auxiliares de recepção da telemetria.
        self.last_rx_id: Optional[int] = None
        self.last_rx_time: float = 0.0
        self.last_published_rx_time: float = 0.0

        # Estado interno de vídeo para diagnóstico básico.
        self.video_packets_ok: int = 0
        self.video_packets_non_jpeg: int = 0
        self.last_video_time: float = 0.0
        self.first_video_packet_logged: bool = False

        # Estado interno de áudio para diagnóstico básico.
        self.audio_packets_ok: int = 0
        self.audio_packets_non_audio: int = 0
        self.audio_tx_packets_sent: int = 0
        self.last_audio_rx_time: float = 0.0
        self.last_audio_tx_time: float = 0.0
        self.last_audio_rx_bytes: int = 0
        self.last_audio_tx_bytes: int = 0
        self.first_audio_packet_logged: bool = False

        # Estado de comando atualmente enviado ao robô.
        self.command_state: Dict[str, Any] = {
            "cam_angle": 0.0,
            "cam_stable": 1,
            "force_rpi": 0,
            "speed": 0.0,
            "rotation": 0.0,
        }

        # Instante do último /rb23/cmd_vel recebido.
        self.last_cmd_vel_time: float = 0.0

        self.first_packet_logged = False
        self.running = True

        # --------------------------------------------------------
        # SOCKET UDP
        # --------------------------------------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(SOCKET_TIMEOUT)
        self.sock.connect((self.server_host, self.server_port))

        self.get_logger().info(
            f"RB23 driver iniciado | "
            f"host={self.server_host}:{self.server_port} | "
            f"robot_id={self.robot_id} | "
            f"TX_ID=0x{self.tx_id:02X} | "
            f"RX_ID=0x{self.rx_id:02X} | "
            f"AUDIO_RX_ID=0x{self.audio_rx_id:02X} | "
            f"cmd_vel_timeout={self.cmd_vel_timeout_sec:.2f}s | "
            f"video_topic={VIDEO_TOPIC_COMPRESSED} | "
            f"audio_rx_topic={AUDIO_RX_TOPIC} | "
            f"audio_tx_topic={AUDIO_TX_TOPIC}"
        )

        # --------------------------------------------------------
        # THREADS DE ENVIO E RECEPÇÃO
        # --------------------------------------------------------
        self.receiver_thread = threading.Thread(
            target=self.receiver_loop,
            daemon=True,
        )
        self.sender_thread = threading.Thread(
            target=self.sender_loop,
            daemon=True,
        )

        self.receiver_thread.start()
        self.sender_thread.start()

        # --------------------------------------------------------
        # TIMER ROS DE PUBLICAÇÃO
        # --------------------------------------------------------
        self.publish_timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_telemetry,
        )

    # =========================================================
    # CALLBACKS DOS TÓPICOS DE COMANDO
    # =========================================================

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Recebe comandos ROS 2 do tópico /rb23/cmd_vel e atualiza o estado
        interno de comando do driver.
        """
        with self.state_lock:
            self.command_state["speed"] = float(msg.linear.x)
            self.command_state["rotation"] = -float(msg.angular.z)
            self.last_cmd_vel_time = time.time()

    def force_rpi_callback(self, msg: Bool) -> None:
        """
        Atualiza o campo force_rpi do estado interno de comando.
        """
        with self.state_lock:
            self.command_state["force_rpi"] = 1 if msg.data else 0

    def cam_stable_callback(self, msg: Bool) -> None:
        """
        Atualiza o campo cam_stable do estado interno de comando.
        """
        with self.state_lock:
            self.command_state["cam_stable"] = 1 if msg.data else 0

    def cam_angle_callback(self, msg: Float32) -> None:
        """
        Atualiza o campo cam_angle do estado interno de comando.
        """
        with self.state_lock:
            self.command_state["cam_angle"] = float(msg.data)

    def audio_tx_callback(self, msg: AudioFrame) -> None:
        """
        Recebe um frame de áudio ROS e o envia ao robô via UDP.

        Formato esperado:
        - encoding = pcm_s16le
        - channels = 1
        - sample_rate = 48000
        - samples_per_channel = 1024
        - data = 2048 bytes (1024 amostras int16 little-endian)

        Observação:
        Para não interromper o restante do sistema, adotamos uma validação
        tolerante: o driver avisa por log se os metadados não coincidirem,
        mas ainda tenta encapsular e enviar o payload recebido caso o
        tamanho esteja correto.
        """
        payload_bytes = bytes(msg.data)

        if len(payload_bytes) != (2 * AUDIO_TX_SAMPLES_PER_PACKET):
            self.get_logger().warn(
                "Frame de áudio TX ignorado: tamanho inválido. "
                f"Recebido {len(payload_bytes)} bytes, esperado {2 * AUDIO_TX_SAMPLES_PER_PACKET}."
            )
            return

        if msg.channels != AUDIO_TX_EXPECTED_CHANNELS:
            self.get_logger().warn(
                f"Áudio TX com channels={msg.channels}, esperado={AUDIO_TX_EXPECTED_CHANNELS}."
            )

        if msg.sample_rate != AUDIO_TX_EXPECTED_SAMPLE_RATE:
            self.get_logger().warn(
                f"Áudio TX com sample_rate={msg.sample_rate}, esperado={AUDIO_TX_EXPECTED_SAMPLE_RATE}."
            )

        if msg.encoding != AUDIO_TX_EXPECTED_ENCODING:
            self.get_logger().warn(
                f"Áudio TX com encoding='{msg.encoding}', esperado='{AUDIO_TX_EXPECTED_ENCODING}'."
            )

        if msg.samples_per_channel != AUDIO_TX_SAMPLES_PER_PACKET:
            self.get_logger().warn(
                f"Áudio TX com samples_per_channel={msg.samples_per_channel}, "
                f"esperado={AUDIO_TX_SAMPLES_PER_PACKET}."
            )

        packet = bytes([AUDIO_TX_PREFIX]) + payload_bytes

        if len(packet) != AUDIO_TX_PAYLOAD_BYTES:
            self.get_logger().warn(
                "Pacote de áudio TX não enviado: tamanho final inválido. "
                f"Recebido {len(packet)} bytes, esperado {AUDIO_TX_PAYLOAD_BYTES}."
            )
            return

        try:
            with self.socket_lock:
                self.sock.send(packet)

            with self.state_lock:
                self.audio_tx_packets_sent += 1
                self.last_audio_tx_time = time.time()
                self.last_audio_tx_bytes = len(packet)

        except OSError:
            pass
        except Exception as exc:
            self.get_logger().warn(f"Erro ao enviar áudio TX via UDP: {exc}")

    # =========================================================
    # FUNÇÕES DE EXTRAÇÃO DE MENSAGENS
    # =========================================================

    def extract_battery_msg(self, telemetry: Dict[str, Any]) -> Optional[Battery]:
        raw_volts = telemetry.get("volts")
        raw_amps = telemetry.get("amps")

        if raw_volts is None and raw_amps is None:
            return None

        voltage = to_float(raw_volts, 0.0)
        current = to_float(raw_amps, 0.0)
        power = voltage * current
        percentage = estimate_battery_percentage(voltage)

        msg = Battery()
        msg.voltage = round(voltage, 2)
        msg.current = round(current, 2)
        msg.power = round(power, 2)
        msg.percentage = round(percentage, 2)
        return msg

    def extract_temperatures_msg(self, telemetry: Dict[str, Any]) -> Optional[Temperatures]:
        raw_pcb_temp = telemetry.get("pcb_temp")
        raw_cpu_temp = telemetry.get("cpu_temp")

        if raw_pcb_temp is None and raw_cpu_temp is None:
            return None

        msg = Temperatures()
        msg.pcb_temp = round(to_float(raw_pcb_temp, 0.0), 2)
        msg.cpu_temp = round(to_float(raw_cpu_temp, 0.0), 2)
        return msg

    def extract_drive_status_msg(self, telemetry: Dict[str, Any]) -> Optional[DriveStatus]:
        raw_pitch = telemetry.get("pitch")
        raw_rotation = telemetry.get("rotation")
        raw_m1 = telemetry.get("m1")
        raw_m2 = telemetry.get("m2")

        if raw_pitch is None and raw_rotation is None and raw_m1 is None and raw_m2 is None:
            return None

        msg = DriveStatus()
        msg.pitch = round(to_float(raw_pitch, 0.0), 2)
        msg.rotation = round(to_float(raw_rotation, 0.0), 2)
        msg.motor_left = round(to_float(raw_m1, 0.0), 2)
        msg.motor_right = round(to_float(raw_m2, 0.0), 2)
        return msg

    def extract_magnetometer_msg(self, telemetry: Dict[str, Any]) -> Optional[Magnetometer]:
        raw_x = telemetry.get("mag_x")
        raw_y = telemetry.get("mag_y")
        raw_z = telemetry.get("mag_z")

        if raw_x is None and raw_y is None and raw_z is None:
            return None

        msg = Magnetometer()
        msg.x = round(to_float(raw_x, 0.0), 2)
        msg.y = round(to_float(raw_y, 0.0), 2)
        msg.z = round(to_float(raw_z, 0.0), 2)
        return msg

    def extract_connection_status_msg(self, telemetry: Dict[str, Any]) -> Optional[ConnectionStatus]:
        raw_robot_server_ping = telemetry.get("robot_server_ping")
        raw_client_server_ping = telemetry.get("client_server_ping")
        raw_local_controller = telemetry.get("local_controller")
        raw_follow_mode = telemetry.get("follow_mode")
        raw_button = telemetry.get("button")

        if (
            raw_robot_server_ping is None
            and raw_client_server_ping is None
            and raw_local_controller is None
            and raw_follow_mode is None
            and raw_button is None
        ):
            return None

        msg = ConnectionStatus()
        msg.robot_server_ping = round(to_float(raw_robot_server_ping, 0.0), 2)
        msg.client_server_ping = round(to_float(raw_client_server_ping, 0.0), 2)
        msg.local_controller = to_bool(raw_local_controller, False)
        msg.follow_mode = to_bool(raw_follow_mode, False)
        msg.button = to_int(raw_button, 0)
        return msg

    def extract_command_state_msg(self) -> CommandState:
        with self.state_lock:
            cmd = dict(self.command_state)

        msg = CommandState()
        msg.speed = round(to_float(cmd.get("speed"), 0.0), 2)
        msg.rotation = round(to_float(cmd.get("rotation"), 0.0), 2)
        msg.cam_angle = round(to_float(cmd.get("cam_angle"), 0.0), 2)
        msg.cam_stable = to_bool(cmd.get("cam_stable"), False)
        msg.force_rpi = to_bool(cmd.get("force_rpi"), False)
        return msg

    def build_audio_rx_msg(self, audio_payload: bytes) -> AudioFrame:
        """
        Constrói uma mensagem ROS do tipo AudioFrame a partir de um
        payload bruto de áudio RX vindo do robô.
        """
        msg = AudioFrame()
        msg.data = list(audio_payload)
        msg.sample_rate = AUDIO_RX_SAMPLE_RATE
        msg.channels = AUDIO_RX_CHANNELS
        msg.encoding = AUDIO_RX_ENCODING
        msg.samples_per_channel = len(audio_payload)
        return msg

    # =========================================================
    # FUNÇÕES DE VÍDEO
    # =========================================================

    def publish_compressed_video(self, jpeg_bytes: bytes) -> None:
        """
        Publica um frame JPEG no tópico ROS de imagem comprimida.
        """
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = jpeg_bytes
        self.camera_compressed_pub.publish(msg)

    # =========================================================
    # THREAD DE ENVIO CONTÍNUO
    # =========================================================

    def sender_loop(self) -> None:
        """
        Thread responsável por enviar continuamente o estado de comando ao robô.
        """
        while self.running:
            try:
                with self.state_lock:
                    if self.last_cmd_vel_time > 0.0:
                        elapsed = time.time() - self.last_cmd_vel_time
                        if elapsed > self.cmd_vel_timeout_sec:
                            self.command_state["speed"] = 0.0
                            self.command_state["rotation"] = 0.0

                    packet = build_command_packet(self.tx_id, self.command_state)

                with self.socket_lock:
                    self.sock.send(packet)

            except OSError:
                break
            except Exception as exc:
                self.get_logger().warn(f"Erro no send UDP: {exc}")

            time.sleep(SEND_PERIOD)

    # =========================================================
    # THREAD DE RECEPÇÃO
    # =========================================================

    def receiver_loop(self) -> None:
        """
        Thread responsável por receber pacotes UDP do robô.

        Estratégia adotada:
        1. Tentar interpretar o pacote como telemetria JSON.
        2. Se não for telemetria, tentar interpretar como vídeo JPEG.
        3. Se não for vídeo, tentar interpretar como áudio RX.
        4. Caso não seja nenhum dos três, ignorar.
        """
        while self.running:
            try:
                with self.socket_lock:
                    packet = self.sock.recv(65535)

            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as exc:
                self.get_logger().warn(f"Erro no recv UDP: {exc}")
                continue

            # ------------------------------------------------------
            # 1. TENTATIVA DE TELEMETRIA JSON
            # ------------------------------------------------------
            decoded = try_decode_telemetry(packet)
            if decoded is not None:
                msg_id = decoded["msg_id"]
                data = decoded["data"]

                if msg_id == self.rx_id:
                    with self.state_lock:
                        self.latest_telemetry = data
                        self.last_rx_id = msg_id
                        self.last_rx_time = time.time()

                    if not self.first_packet_logged:
                        self.first_packet_logged = True
                        self.get_logger().info(
                            "Primeiro pacote de telemetria recebido com sucesso."
                        )

                continue

            # ------------------------------------------------------
            # 2. TENTATIVA DE EXTRAÇÃO DE VÍDEO JPEG
            # ------------------------------------------------------
            jpeg_bytes = extract_jpeg_from_packet(packet)
            if jpeg_bytes is not None:
                try:
                    self.publish_compressed_video(jpeg_bytes)

                    with self.state_lock:
                        self.video_packets_ok += 1
                        self.last_video_time = time.time()

                    if not self.first_video_packet_logged:
                        self.first_video_packet_logged = True
                        self.get_logger().info(
                            "Primeiro frame JPEG recebido e publicado com sucesso."
                        )

                except Exception as exc:
                    self.get_logger().warn(
                        f"Falha ao publicar frame comprimido: {exc}"
                    )

                continue

            # ------------------------------------------------------
            # 3. TENTATIVA DE EXTRAÇÃO DE ÁUDIO RX
            # ------------------------------------------------------
            audio_payload = extract_audio_rx_payload(packet, self.audio_rx_id)
            if audio_payload is not None:
                try:
                    audio_msg = self.build_audio_rx_msg(audio_payload)
                    self.audio_rx_pub.publish(audio_msg)

                    with self.state_lock:
                        self.audio_packets_ok += 1
                        self.last_audio_rx_time = time.time()
                        self.last_audio_rx_bytes = len(audio_payload)

                    if not self.first_audio_packet_logged:
                        self.first_audio_packet_logged = True
                        self.get_logger().info(
                            "Primeiro pacote de áudio RX recebido e publicado com sucesso."
                        )

                except Exception as exc:
                    self.get_logger().warn(
                        f"Falha ao publicar frame de áudio RX: {exc}"
                    )

                continue

            # ------------------------------------------------------
            # 4. PACOTE DESCONHECIDO / IGNORADO
            # ------------------------------------------------------
            with self.state_lock:
                self.video_packets_non_jpeg += 1
                self.audio_packets_non_audio += 1

    # =========================================================
    # PUBLICAÇÃO ROS
    # =========================================================

    def publish_telemetry(self) -> None:
        """
        Publica a última telemetria recebida em todos os tópicos ROS relevantes.
        """
        with self.state_lock:
            if not self.latest_telemetry:
                return

            if self.last_rx_time == self.last_published_rx_time:
                return

            payload = dict(self.latest_telemetry)
            self.last_published_rx_time = self.last_rx_time

        telemetry_msg = String()
        telemetry_msg.data = json.dumps(payload, separators=(",", ":"))
        self.telemetry_pub.publish(telemetry_msg)

        battery_msg = self.extract_battery_msg(payload)
        if battery_msg is not None:
            self.battery_pub.publish(battery_msg)

        temperatures_msg = self.extract_temperatures_msg(payload)
        if temperatures_msg is not None:
            self.temperatures_pub.publish(temperatures_msg)

        drive_status_msg = self.extract_drive_status_msg(payload)
        if drive_status_msg is not None:
            self.drive_status_pub.publish(drive_status_msg)

        magnetometer_msg = self.extract_magnetometer_msg(payload)
        if magnetometer_msg is not None:
            self.magnetometer_pub.publish(magnetometer_msg)

        connection_status_msg = self.extract_connection_status_msg(payload)
        if connection_status_msg is not None:
            self.connection_status_pub.publish(connection_status_msg)

        command_state_msg = self.extract_command_state_msg()
        self.command_state_pub.publish(command_state_msg)

    # =========================================================
    # ENCERRAMENTO
    # =========================================================

    def close(self) -> None:
        """
        Encerra o driver de forma organizada.
        """
        self.running = False

        with self.state_lock:
            self.command_state["speed"] = 0.0
            self.command_state["rotation"] = 0.0

        try:
            with self.socket_lock:
                self.sock.close()
        except Exception:
            pass

        if self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=1.0)

        if self.sender_thread.is_alive():
            self.sender_thread.join(timeout=1.0)


# ============================================================
# FUNÇÃO MAIN
# ============================================================

def main(args=None) -> None:
    """
    Ponto de entrada do nó ROS 2.
    """
    rclpy.init(args=args)
    node = RB23RosDriver()

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