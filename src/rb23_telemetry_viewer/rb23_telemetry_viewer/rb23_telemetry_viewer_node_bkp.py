#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rb23_telemetry_viewer_node.py

Nó ROS 2 para visualização textual da telemetria do GuardROS / RB23
em um terminal dedicado.

Objetivo:
---------
Exibir em tempo real, de forma organizada e amigável, os principais
dados de telemetria publicados pelo rb23_driver_node, além do estado
do modo de áudio selecionado no sistema.

Arquitetura:
------------
RB23 -> rb23_driver_node -> tópicos ROS -> rb23_telemetry_viewer_node

Tópicos assinados:
------------------
- /rb23/battery
- /rb23/temperatures
- /rb23/drive_status
- /rb23/magnetometer
- /rb23/connection_status
- /rb23/command_state
- /rb23/telemetry_json
- /rb23/audio_mode
- /rb23/audio/rx
- /rb23/audio/tx

Melhorias desta versão:
-----------------------
1. Interface textual com 3 colunas.
2. Inclusão do estado de áudio no painel.
3. Inclusão de indicadores básicos de atividade de áudio RX/TX.

Observação:
-----------
Este nó não envia comandos ao robô.
Ele apenas observa tópicos ROS e organiza os dados em tela.
"""

import curses
import json
import time
from typing import Optional

import rclpy
from guardros_msgs.msg import AudioFrame
from guardros_msgs.msg import AudioMode
from guardros_msgs.msg import Battery
from guardros_msgs.msg import CommandState
from guardros_msgs.msg import ConnectionStatus
from guardros_msgs.msg import DriveStatus
from guardros_msgs.msg import Magnetometer
from guardros_msgs.msg import Temperatures
from rclpy.node import Node
from std_msgs.msg import String


class RB23TelemetryViewerNode(Node):
    """
    Nó ROS 2 responsável por mostrar a telemetria do RB23 no terminal.
    """

    def __init__(self) -> None:
        super().__init__("rb23_telemetry_viewer_node")

        # =========================================================
        # PARÂMETROS
        # =========================================================
        self.declare_parameter("screen_rate_hz", 10.0)
        self.declare_parameter("stale_timeout_sec", 1.0)

        self.screen_rate_hz = float(self.get_parameter("screen_rate_hz").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)

        # =========================================================
        # ESTADO INTERNO
        # =========================================================
        self.running = True

        # Última mensagem recebida de cada tópico.
        self.battery_msg: Optional[Battery] = None
        self.temperatures_msg: Optional[Temperatures] = None
        self.drive_status_msg: Optional[DriveStatus] = None
        self.magnetometer_msg: Optional[Magnetometer] = None
        self.connection_status_msg: Optional[ConnectionStatus] = None
        self.command_state_msg: Optional[CommandState] = None
        self.audio_mode_msg: Optional[AudioMode] = None
        self.audio_rx_msg: Optional[AudioFrame] = None
        self.audio_tx_msg: Optional[AudioFrame] = None

        # Instantes de recepção.
        self.last_battery_time = 0.0
        self.last_temperatures_time = 0.0
        self.last_drive_status_time = 0.0
        self.last_magnetometer_time = 0.0
        self.last_connection_status_time = 0.0
        self.last_command_state_time = 0.0
        self.last_audio_mode_time = 0.0
        self.last_audio_rx_time = 0.0
        self.last_audio_tx_time = 0.0
        self.last_any_telemetry_time = 0.0

        # Resumo do JSON bruto.
        self.raw_json_preview = "-"
        self.raw_json_keys_preview = "-"
        self.raw_json_field_count = 0

        # =========================================================
        # SUBSCRIBERS
        # =========================================================
        self.create_subscription(
            Battery,
            "/rb23/battery",
            self.battery_callback,
            10
        )

        self.create_subscription(
            Temperatures,
            "/rb23/temperatures",
            self.temperatures_callback,
            10
        )

        self.create_subscription(
            DriveStatus,
            "/rb23/drive_status",
            self.drive_status_callback,
            10
        )

        self.create_subscription(
            Magnetometer,
            "/rb23/magnetometer",
            self.magnetometer_callback,
            10
        )

        self.create_subscription(
            ConnectionStatus,
            "/rb23/connection_status",
            self.connection_status_callback,
            10
        )

        self.create_subscription(
            CommandState,
            "/rb23/command_state",
            self.command_state_callback,
            10
        )

        self.create_subscription(
            String,
            "/rb23/telemetry_json",
            self.telemetry_json_callback,
            10
        )

        self.create_subscription(
            AudioMode,
            "/rb23/audio_mode",
            self.audio_mode_callback,
            10
        )

        self.create_subscription(
            AudioFrame,
            "/rb23/audio/rx",
            self.audio_rx_callback,
            10
        )

        self.create_subscription(
            AudioFrame,
            "/rb23/audio/tx",
            self.audio_tx_callback,
            10
        )

        self.get_logger().info(
            "RB23 telemetry viewer iniciado com layout em 3 colunas e suporte a áudio."
        )

    # =========================================================
    # CALLBACKS
    # =========================================================

    def mark_rx(self) -> None:
        self.last_any_telemetry_time = time.time()

    def battery_callback(self, msg: Battery) -> None:
        self.battery_msg = msg
        self.last_battery_time = time.time()
        self.mark_rx()

    def temperatures_callback(self, msg: Temperatures) -> None:
        self.temperatures_msg = msg
        self.last_temperatures_time = time.time()
        self.mark_rx()

    def drive_status_callback(self, msg: DriveStatus) -> None:
        self.drive_status_msg = msg
        self.last_drive_status_time = time.time()
        self.mark_rx()

    def magnetometer_callback(self, msg: Magnetometer) -> None:
        self.magnetometer_msg = msg
        self.last_magnetometer_time = time.time()
        self.mark_rx()

    def connection_status_callback(self, msg: ConnectionStatus) -> None:
        self.connection_status_msg = msg
        self.last_connection_status_time = time.time()
        self.mark_rx()

    def command_state_callback(self, msg: CommandState) -> None:
        self.command_state_msg = msg
        self.last_command_state_time = time.time()
        self.mark_rx()

    def audio_mode_callback(self, msg: AudioMode) -> None:
        self.audio_mode_msg = msg
        self.last_audio_mode_time = time.time()

    def audio_rx_callback(self, msg: AudioFrame) -> None:
        self.audio_rx_msg = msg
        self.last_audio_rx_time = time.time()

    def audio_tx_callback(self, msg: AudioFrame) -> None:
        self.audio_tx_msg = msg
        self.last_audio_tx_time = time.time()

    def telemetry_json_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            keys = sorted(list(data.keys()))
            self.raw_json_field_count = len(keys)

            self.raw_json_keys_preview = ", ".join(keys[:8])
            if len(keys) > 8:
                self.raw_json_keys_preview += ", ..."

            self.raw_json_preview = msg.data[:110]
            if len(msg.data) > 110:
                self.raw_json_preview += "..."

        except Exception:
            self.raw_json_preview = msg.data[:110]
            if len(msg.data) > 110:
                self.raw_json_preview += "..."
            self.raw_json_keys_preview = "-"
            self.raw_json_field_count = 0

    # =========================================================
    # FUNÇÕES AUXILIARES
    # =========================================================

    def age_text(self, stamp: float) -> str:
        if stamp <= 0.0:
            return "nunca"
        dt = time.time() - stamp
        return f"{dt:.2f} s"

    def freshness_label(self, stamp: float) -> str:
        if stamp <= 0.0:
            return "SEM DADO"

        dt = time.time() - stamp

        if dt <= self.stale_timeout_sec:
            return "OK"

        return "ANTIGO"

    def safe_bool_text(self, value: bool) -> str:
        return "ON" if value else "OFF"

    def audio_mode_text(self) -> str:
        if self.audio_mode_msg is None:
            return "SEM DADO"

        mode = self.audio_mode_msg.mode

        if mode == AudioMode.MODE_SILENCE:
            return "silêncio bilateral"
        if mode == AudioMode.MODE_LISTEN:
            return "ouvindo robô"
        if mode == AudioMode.MODE_TALK:
            return "falando com robô"

        return f"desconhecido ({mode})"

    def audio_frame_summary(self, msg: Optional[AudioFrame]) -> str:
        if msg is None:
            return "SEM DADO"

        return (
            f"{msg.encoding}, {msg.sample_rate} Hz, "
            f"{msg.channels} ch, {msg.samples_per_channel} amostras"
        )

    # =========================================================
    # MONTAGEM DOS BLOCOS DE TEXTO
    # =========================================================

    def make_col1_items(self) -> list[str]:
        items = [
            f"Status geral          : {self.freshness_label(self.last_any_telemetry_time)}",
            f"Última telemetria     : {self.age_text(self.last_any_telemetry_time)}",
            "",
            "[CONEXÃO]",
        ]

        if self.connection_status_msg is None:
            items.append("Sem dados de conexão.")
        else:
            c = self.connection_status_msg
            items.extend([
                f"robot_server_ping     : {c.robot_server_ping:.2f} ms",
                f"client_server_ping    : {c.client_server_ping:.2f} ms",
                f"local_controller      : {self.safe_bool_text(c.local_controller)}",
                f"follow_mode           : {self.safe_bool_text(c.follow_mode)}",
                f"button                : {c.button}",
                f"idade conexão         : {self.age_text(self.last_connection_status_time)}",
            ])

        items.extend([
            "",
            "[COMANDO ATUAL]",
        ])

        if self.command_state_msg is None:
            items.append("Sem dados de comando.")
        else:
            cmd = self.command_state_msg
            items.extend([
                f"speed                 : {cmd.speed:.2f}",
                f"rotation              : {cmd.rotation:.2f}",
                f"cam_angle             : {cmd.cam_angle:.2f}",
                f"cam_stable            : {self.safe_bool_text(cmd.cam_stable)}",
                f"force_rpi             : {self.safe_bool_text(cmd.force_rpi)}",
                f"idade comando         : {self.age_text(self.last_command_state_time)}",
            ])

        return items

    def make_col2_items(self) -> list[str]:
        items = [
            "[BATERIA]",
        ]

        if self.battery_msg is None:
            items.append("Sem dados de bateria.")
        else:
            b = self.battery_msg
            items.extend([
                f"tensão                : {b.voltage:.2f} V",
                f"corrente              : {b.current:.2f} A",
                f"potência              : {b.power:.2f} W",
                f"percentual estimado   : {b.percentage:.2f} %",
                f"idade bateria         : {self.age_text(self.last_battery_time)}",
            ])

        items.extend([
            "",
            "[TEMPERATURAS]",
        ])

        if self.temperatures_msg is None:
            items.append("Sem dados de temperatura.")
        else:
            t = self.temperatures_msg
            items.extend([
                f"pcb_temp              : {t.pcb_temp:.2f} °C",
                f"cpu_temp              : {t.cpu_temp:.2f} °C",
                f"idade temperatura     : {self.age_text(self.last_temperatures_time)}",
            ])

        items.extend([
            "",
            "[MOVIMENTO]",
        ])

        if self.drive_status_msg is None:
            items.append("Sem dados de movimento.")
        else:
            d = self.drive_status_msg
            items.extend([
                f"pitch                 : {d.pitch:.2f}",
                f"rotation              : {d.rotation:.2f}",
                f"motor_left            : {d.motor_left:.2f}",
                f"motor_right           : {d.motor_right:.2f}",
                f"idade movimento       : {self.age_text(self.last_drive_status_time)}",
            ])

        return items

    def make_col3_items(self) -> list[str]:
        items = [
            "[ÁUDIO]",
            f"modo atual            : {self.audio_mode_text()}",
            f"idade modo            : {self.age_text(self.last_audio_mode_time)}",
            f"último RX             : {self.age_text(self.last_audio_rx_time)}",
            f"último TX             : {self.age_text(self.last_audio_tx_time)}",
            f"resumo RX             : {self.audio_frame_summary(self.audio_rx_msg)}",
            f"resumo TX             : {self.audio_frame_summary(self.audio_tx_msg)}",
            "",
            "[MAGNETÔMETRO]",
        ]

        if self.magnetometer_msg is None:
            items.append("Sem dados de magnetômetro.")
        else:
            m = self.magnetometer_msg
            items.extend([
                f"mag_x                 : {m.x:.2f}",
                f"mag_y                 : {m.y:.2f}",
                f"mag_z                 : {m.z:.2f}",
                f"idade magnetômetro    : {self.age_text(self.last_magnetometer_time)}",
            ])

        items.extend([
            "",
            "[DIAGNÓSTICO JSON]",
            f"campos detectados     : {self.raw_json_field_count}",
            f"chaves                : {self.raw_json_keys_preview}",
            f"preview               : {self.raw_json_preview}",
            "",
            "[ENCERRAMENTO]",
            "ESC ou Ctrl+C",
        ])

        return items

    # =========================================================
    # DESENHO DE BLOCOS
    # =========================================================

    def draw_column(
        self,
        stdscr,
        start_y: int,
        start_x: int,
        width: int,
        lines: list[str]
    ) -> None:
        max_y, max_x = stdscr.getmaxyx()

        if start_x >= max_x - 1:
            return

        width = max(10, min(width, max_x - start_x - 1))

        y = start_y
        for line in lines:
            if y >= max_y - 1:
                break

            try:
                stdscr.addnstr(y, start_x, line, width)
            except curses.error:
                pass

            y += 1

    # =========================================================
    # RENDERIZAÇÃO PRINCIPAL
    # =========================================================

    def render_screen(self, stdscr) -> None:
        stdscr.erase()

        max_y, max_x = stdscr.getmaxyx()

        header = "================ GUARDROS - TELEMETRY VIEWER ================"

        try:
            stdscr.addnstr(0, 0, header, max_x - 1)
        except curses.error:
            pass

        usable_width = max_x - 4
        col_width = max(24, usable_width // 3)

        col1_x = 0
        col2_x = col_width + 2
        col3_x = 2 * (col_width + 2)

        start_y = 2

        col1_lines = self.make_col1_items()
        col2_lines = self.make_col2_items()
        col3_lines = self.make_col3_items()

        self.draw_column(stdscr, start_y, col1_x, col_width, col1_lines)
        self.draw_column(stdscr, start_y, col2_x, col_width, col2_lines)
        self.draw_column(stdscr, start_y, col3_x, col_width, col3_lines)

        try:
            stdscr.refresh()
        except curses.error:
            pass

    # =========================================================
    # LOOP PRINCIPAL DO CURSES
    # =========================================================

    def curses_loop(self, stdscr) -> None:
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)
        curses.noecho()
        curses.cbreak()

        draw_period = 1.0 / max(self.screen_rate_hz, 1.0)
        last_draw = 0.0

        while self.running and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.0)

                ch = stdscr.getch()

                if ch == 27:
                    self.running = False
                    break

                now = time.time()

                if now - last_draw >= draw_period:
                    self.render_screen(stdscr)
                    last_draw = now

                time.sleep(0.01)

            except KeyboardInterrupt:
                self.running = False
                break

            except Exception as exc:
                self.get_logger().error(f"Erro no telemetry viewer: {exc}")
                time.sleep(0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RB23TelemetryViewerNode()

    try:
        curses.wrapper(node.curses_loop)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()