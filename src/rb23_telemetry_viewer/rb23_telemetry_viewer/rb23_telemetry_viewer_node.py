#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rb23_telemetry_viewer_node.py

ROS 2 node for text-based telemetry visualization of GuardROS / RB23
in a dedicated terminal window.

Purpose:
--------
Display, in real time and in an organized user-friendly layout, the main
telemetry data published by rb23_driver_node, as well as the currently
selected audio mode in the system.

Architecture:
-------------
RB23 -> rb23_driver_node -> ROS topics -> rb23_telemetry_viewer_node

Subscribed topics:
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

Current version improvements:
-----------------------------
1. Three-column text interface.
2. Audio state included in the telemetry panel.
3. Basic activity indicators for audio RX/TX.

Important note:
---------------
This node does not send commands to the robot.
It only observes ROS topics and organizes the data on screen.
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
    ROS 2 node responsible for displaying RB23 telemetry in the terminal.
    """

    def __init__(self) -> None:
        super().__init__("rb23_telemetry_viewer_node")

        # =========================================================
        # PARAMETERS
        # =========================================================
        self.declare_parameter("screen_rate_hz", 10.0)
        self.declare_parameter("stale_timeout_sec", 1.0)

        self.screen_rate_hz = float(self.get_parameter("screen_rate_hz").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)

        # =========================================================
        # INTERNAL STATE
        # =========================================================
        self.running = True

        # Most recent message received from each topic.
        self.battery_msg: Optional[Battery] = None
        self.temperatures_msg: Optional[Temperatures] = None
        self.drive_status_msg: Optional[DriveStatus] = None
        self.magnetometer_msg: Optional[Magnetometer] = None
        self.connection_status_msg: Optional[ConnectionStatus] = None
        self.command_state_msg: Optional[CommandState] = None
        self.audio_mode_msg: Optional[AudioMode] = None
        self.audio_rx_msg: Optional[AudioFrame] = None
        self.audio_tx_msg: Optional[AudioFrame] = None

        # Reception timestamps for each data group.
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

        # Raw JSON summary for quick debugging.
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
            "RB23 telemetry viewer started with a three-column layout and audio support."
        )

    # =========================================================
    # CALLBACKS
    # =========================================================

    def mark_rx(self) -> None:
        """
        Mark the timestamp of the most recent telemetry-related reception.

        This function is used by the callbacks that represent the main robot
        telemetry stream. It provides a simple global freshness indicator for
        the terminal interface.
        """
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
        """
        Store a compact preview of the raw telemetry JSON.

        The complete JSON string may be too long for the terminal layout, so
        this callback extracts the number of fields, a short list of keys, and
        a short preview of the raw payload.
        """
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
    # HELPER FUNCTIONS
    # =========================================================

    def age_text(self, stamp: float) -> str:
        """
        Return a human-readable age for a timestamp.
        """
        if stamp <= 0.0:
            return "never"
        dt = time.time() - stamp
        return f"{dt:.2f} s"

    def freshness_label(self, stamp: float) -> str:
        """
        Return a simple freshness label for a timestamp.

        This is useful for quickly identifying whether the displayed data is
        current or stale.
        """
        if stamp <= 0.0:
            return "NO DATA"

        dt = time.time() - stamp

        if dt <= self.stale_timeout_sec:
            return "OK"

        return "STALE"

    def safe_bool_text(self, value: bool) -> str:
        """
        Convert a boolean value into a compact ON/OFF text.
        """
        return "ON" if value else "OFF"

    def audio_mode_text(self) -> str:
        """
        Convert the current audio mode code into a readable text string.
        """
        if self.audio_mode_msg is None:
            return "NO DATA"

        mode = self.audio_mode_msg.mode

        if mode == AudioMode.MODE_SILENCE:
            return "bidirectional silence"
        if mode == AudioMode.MODE_LISTEN:
            return "listening to robot"
        if mode == AudioMode.MODE_TALK:
            return "talking to robot"

        return f"unknown ({mode})"

    def audio_frame_summary(self, msg: Optional[AudioFrame]) -> str:
        """
        Build a compact summary of an audio frame.
        """
        if msg is None:
            return "NO DATA"

        return (
            f"{msg.encoding}, {msg.sample_rate} Hz, "
            f"{msg.channels} ch, {msg.samples_per_channel} samples"
        )

    # =========================================================
    # TEXT BLOCK ASSEMBLY
    # =========================================================

    def make_col1_items(self) -> list[str]:
        """
        Build the first column of the terminal interface.
        """
        items = [
            f"Overall status        : {self.freshness_label(self.last_any_telemetry_time)}",
            f"Last telemetry        : {self.age_text(self.last_any_telemetry_time)}",
            "",
            "[CONNECTION]",
        ]

        if self.connection_status_msg is None:
            items.append("No connection data.")
        else:
            c = self.connection_status_msg
            items.extend([
                f"robot_server_ping     : {c.robot_server_ping:.2f} ms",
                f"client_server_ping    : {c.client_server_ping:.2f} ms",
                f"local_controller      : {self.safe_bool_text(c.local_controller)}",
                f"follow_mode           : {self.safe_bool_text(c.follow_mode)}",
                f"button                : {c.button}",
                f"connection age        : {self.age_text(self.last_connection_status_time)}",
            ])

        items.extend([
            "",
            "[CURRENT COMMAND]",
        ])

        if self.command_state_msg is None:
            items.append("No command data.")
        else:
            cmd = self.command_state_msg
            items.extend([
                f"speed                 : {cmd.speed:.2f}",
                f"rotation              : {cmd.rotation:.2f}",
                f"cam_angle             : {cmd.cam_angle:.2f}",
                f"cam_stable            : {self.safe_bool_text(cmd.cam_stable)}",
                f"force_rpi             : {self.safe_bool_text(cmd.force_rpi)}",
                f"command age           : {self.age_text(self.last_command_state_time)}",
            ])

        return items

    def make_col2_items(self) -> list[str]:
        """
        Build the second column of the terminal interface.
        """
        items = [
            "[BATTERY]",
        ]

        if self.battery_msg is None:
            items.append("No battery data.")
        else:
            b = self.battery_msg
            items.extend([
                f"voltage               : {b.voltage:.2f} V",
                f"current               : {b.current:.2f} A",
                f"power                 : {b.power:.2f} W",
                f"estimated percentage  : {b.percentage:.2f} %",
                f"battery age           : {self.age_text(self.last_battery_time)}",
            ])

        items.extend([
            "",
            "[TEMPERATURES]",
        ])

        if self.temperatures_msg is None:
            items.append("No temperature data.")
        else:
            t = self.temperatures_msg
            items.extend([
                f"pcb_temp              : {t.pcb_temp:.2f} °C",
                f"cpu_temp              : {t.cpu_temp:.2f} °C",
                f"temperature age       : {self.age_text(self.last_temperatures_time)}",
            ])

        items.extend([
            "",
            "[MOTION]",
        ])

        if self.drive_status_msg is None:
            items.append("No motion data.")
        else:
            d = self.drive_status_msg
            items.extend([
                f"pitch                 : {d.pitch:.2f}",
                f"rotation              : {d.rotation:.2f}",
                f"motor_left            : {d.motor_left:.2f}",
                f"motor_right           : {d.motor_right:.2f}",
                f"motion age            : {self.age_text(self.last_drive_status_time)}",
            ])

        return items

    def make_col3_items(self) -> list[str]:
        """
        Build the third column of the terminal interface.
        """
        items = [
            "[AUDIO]",
            f"current mode          : {self.audio_mode_text()}",
            f"mode age              : {self.age_text(self.last_audio_mode_time)}",
            f"last RX               : {self.age_text(self.last_audio_rx_time)}",
            f"last TX               : {self.age_text(self.last_audio_tx_time)}",
            f"RX summary            : {self.audio_frame_summary(self.audio_rx_msg)}",
            f"TX summary            : {self.audio_frame_summary(self.audio_tx_msg)}",
            "",
            "[MAGNETOMETER]",
        ]

        if self.magnetometer_msg is None:
            items.append("No magnetometer data.")
        else:
            m = self.magnetometer_msg
            items.extend([
                f"mag_x                 : {m.x:.2f}",
                f"mag_y                 : {m.y:.2f}",
                f"mag_z                 : {m.z:.2f}",
                f"magnetometer age      : {self.age_text(self.last_magnetometer_time)}",
            ])

        items.extend([
            "",
            "[JSON DIAGNOSTIC]",
            f"detected fields       : {self.raw_json_field_count}",
            f"keys                  : {self.raw_json_keys_preview}",
            f"preview               : {self.raw_json_preview}",
            "",
            "[EXIT]",
            "ESC or Ctrl+C",
        ])

        return items

    # =========================================================
    # TEXT BLOCK DRAWING
    # =========================================================

    def draw_column(
        self,
        stdscr,
        start_y: int,
        start_x: int,
        width: int,
        lines: list[str]
    ) -> None:
        """
        Draw one column of text on the curses screen.
        """
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
    # MAIN RENDERING
    # =========================================================

    def render_screen(self, stdscr) -> None:
        """
        Render the complete three-column telemetry interface.
        """
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
    # MAIN CURSES LOOP
    # =========================================================

    def curses_loop(self, stdscr) -> None:
        """
        Main loop for the curses-based terminal interface.
        """
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
                self.get_logger().error(f"Telemetry viewer loop error: {exc}")
                time.sleep(0.05)


def main(args=None) -> None:
    """
    Main entry point for the ROS 2 node.
    """
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
