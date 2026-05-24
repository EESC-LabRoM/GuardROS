#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rb23_experiment_logger_node.py

ROS 2 experiment logger node for GuardROS / RollerBot 23.

Purpose
-------
This node records experimental data from the GuardROS system into structured CSV and JSONL files.
It is intended to support quantitative evaluation for scientific publication,
especially for metrics such as:

1. Command publication rate.
2. Telemetry update rate.
3. Video frame rate.
4. Audio RX/TX activity.
5. Diagnostic counters from operational nodes.
6. Timing, jitter, and data availability during experimental runs.

Design philosophy
-----------------
This logger follows a non-invasive architecture:

- It does not send commands to the robot.
- It does not modify the behavior of the driver, teleoperation, video, audio,
  or telemetry viewer nodes.
- It only subscribes to existing ROS topics and diagnostic topics.
- It writes structured CSV and JSONL files that can be analyzed later with Python.

Expected architecture
---------------------
Operational nodes publish data:

keyboard_teleop_node       -> /rb23/cmd_vel
rb23_driver_node           -> /rb23/telemetry_json
rb23_driver_node           -> /rb23/camera/image/compressed
rb23_driver_node           -> /rb23/audio/rx
rb23_audio_node            -> /rb23/audio/tx
instrumented nodes         -> /rb23/diagnostics/*

This logger subscribes to those topics and writes:

session_metadata.csv
control_log.csv
aux_commands_log.csv
telemetry_log.csv
typed_telemetry_log.csv
video_log.csv
audio_log.csv
diagnostics_driver_log.jsonl
diagnostics_teleop_log.jsonl
diagnostics_audio_log.jsonl
diagnostics_video_log.jsonl


Default log directory
---------------------
By default, this logger stores data relative to the current working directory:

    Path.cwd() / "experiments" / "logs"

Therefore, if the logger is launched from the root of the GuardROS workspace,
the logs will be stored inside the workspace itself:

    guardros_ws/experiments/logs

This avoids hard-coded user-specific paths inside the Python source code.
For scripted execution, the startup script should pass the base_log_dir
parameter explicitly using the WORKSPACE variable.

Important note
--------------
This node uses wall-clock time from Python's time.time() for CSV timestamps.
This is adequate for experimental logging at the application level. If future
experiments require sub-millisecond synchronization between multiple machines,
a more advanced clock synchronization strategy should be considered.
"""

import csv
import json
import os
import platform
import socket
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from guardros_msgs.msg import AudioFrame
from guardros_msgs.msg import AudioMode
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
# DEFAULT TOPIC NAMES
# ============================================================

CMD_VEL_TOPIC = "/rb23/cmd_vel"
FORCE_RPI_TOPIC = "/rb23/force_rpi"
CAM_STABLE_TOPIC = "/rb23/cam_stable"
CAM_ANGLE_TOPIC = "/rb23/cam_angle"
AUDIO_MODE_TOPIC = "/rb23/audio_mode"

TELEMETRY_JSON_TOPIC = "/rb23/telemetry_json"
BATTERY_TOPIC = "/rb23/battery"
TEMPERATURES_TOPIC = "/rb23/temperatures"
DRIVE_STATUS_TOPIC = "/rb23/drive_status"
MAGNETOMETER_TOPIC = "/rb23/magnetometer"
CONNECTION_STATUS_TOPIC = "/rb23/connection_status"
COMMAND_STATE_TOPIC = "/rb23/command_state"

VIDEO_TOPIC = "/rb23/camera/image/compressed"
AUDIO_RX_TOPIC = "/rb23/audio/rx"
AUDIO_TX_TOPIC = "/rb23/audio/tx"

DRIVER_DIAG_TOPIC = "/rb23/diagnostics/driver"
TELEOP_DIAG_TOPIC = "/rb23/diagnostics/teleop"
AUDIO_DIAG_TOPIC = "/rb23/diagnostics/audio"
VIDEO_DIAG_TOPIC = "/rb23/diagnostics/video"


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def wall_time() -> float:
    """
    Return the current wall-clock time in seconds.

    The value is stored in CSV files as a floating-point Unix timestamp.
    This makes it easy to compute time intervals later using pandas.
    """
    return time.time()


def iso_time() -> str:
    """
    Return the current local time as an ISO-like string.

    This is mostly useful for metadata and human-readable session folders.
    """
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def safe_json_loads(text: str) -> Optional[Dict[str, Any]]:
    """
    Safely decode a JSON string into a dictionary.

    If decoding fails, None is returned instead of raising an exception.
    This prevents malformed diagnostic messages from interrupting the logger.
    """
    try:
        data = json.loads(text)
        if isinstance(data, dict):
            return data
        return {"value": data}
    except Exception:
        return None


def flatten_dict(data: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    """
    Flatten a nested dictionary into a single-level dictionary.

    Example:
        {"a": {"b": 1}} becomes {"a.b": 1}

    This is useful for diagnostic JSON messages, because different nodes may
    publish nested diagnostic structures. Flattening makes the output easier
    to save in a CSV-like format.
    """
    flat: Dict[str, Any] = {}

    for key, value in data.items():
        new_key = f"{prefix}.{key}" if prefix else str(key)

        if isinstance(value, dict):
            flat.update(flatten_dict(value, new_key))
        else:
            flat[new_key] = value

    return flat


class CsvWriter:
    """
    Small helper class for safe CSV writing.

    Each log file has a fixed header. Rows are dictionaries. Missing fields are
    automatically written as empty cells. Unknown extra fields are ignored to
    keep the CSV structure stable during one experimental session.
    """

    def __init__(self, path: Path, fieldnames: list[str]) -> None:
        self.path = path
        self.fieldnames = fieldnames

        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.file.flush()

    def write(self, row: Dict[str, Any]) -> None:
        safe_row = {name: row.get(name, "") for name in self.fieldnames}
        self.writer.writerow(safe_row)
        self.file.flush()

    def close(self) -> None:
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass


class JsonLinesWriter:
    """
    Helper class for diagnostic logs saved as JSON Lines.

    Diagnostics may contain variable fields depending on the node and on future
    instrumentation. JSON Lines is safer than plain CSV for this type of data.

    Each line is a valid JSON object and can later be loaded with pandas using:

        pandas.read_json("file.jsonl", lines=True)
    """

    def __init__(self, path: Path) -> None:
        self.path = path
        self.file = open(self.path, "w", encoding="utf-8")

    def write(self, row: Dict[str, Any]) -> None:
        self.file.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
        self.file.flush()

    def close(self) -> None:
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass


# ============================================================
# MAIN LOGGER NODE
# ============================================================

class RB23ExperimentLoggerNode(Node):
    """
    ROS 2 node responsible for recording GuardROS experimental data.
    """

    def __init__(self) -> None:
        super().__init__("rb23_experiment_logger_node")

        # ========================================================
        # ROS PARAMETERS
        # ========================================================
        # The base directory defines where all experimental sessions will be
        # stored. The default is relative to the current working directory.
        #
        # This design avoids hard-coded absolute paths in the Python code.
        # If the node is launched from the root of the GuardROS workspace,
        # the default log directory becomes:
        #
        #     guardros_ws/experiments/logs
        #
        # For robust scripted execution, the startup script can override this
        # parameter using:
        #
        #     -p base_log_dir:="$WORKSPACE/experiments/logs"
        #
        # This way, the user only needs to configure the WORKSPACE variable
        # in the startup script, not this Python file.
        self.declare_parameter(
            "base_log_dir",
            str(Path.cwd() / "experiments" / "logs"),
        )

        # Optional label used to identify the experiment condition.
        # Examples: nominal, video_on, audio_on, stress_test, long_run.
        self.declare_parameter("experiment_label", "manual_run")

        # If true, the logger creates a new timestamped folder for this session.
        self.declare_parameter("create_session_folder", True)

        self.base_log_dir = Path(str(self.get_parameter("base_log_dir").value))
        self.experiment_label = str(self.get_parameter("experiment_label").value)
        self.create_session_folder = bool(
            self.get_parameter("create_session_folder").value
        )

        # ========================================================
        # SESSION DIRECTORY
        # ========================================================
        session_stamp = iso_time()

        if self.create_session_folder:
            safe_label = self.experiment_label.replace(" ", "_")
            self.session_dir = self.base_log_dir / f"{session_stamp}_{safe_label}"
        else:
            self.session_dir = self.base_log_dir

        self.session_dir.mkdir(parents=True, exist_ok=True)

        # ========================================================
        # INTERNAL COUNTERS
        # ========================================================
        self.start_time = wall_time()

        self.cmd_vel_count = 0
        self.telemetry_json_count = 0
        self.video_msg_count = 0
        self.audio_rx_count = 0
        self.audio_tx_count = 0

        self.last_cmd_vel_time = 0.0
        self.last_telemetry_json_time = 0.0
        self.last_video_msg_time = 0.0
        self.last_audio_rx_time = 0.0
        self.last_audio_tx_time = 0.0

        # ========================================================
        # CSV / JSONL WRITERS
        # ========================================================
        self.control_writer = CsvWriter(
            self.session_dir / "control_log.csv",
            [
                "timestamp",
                "elapsed_sec",
                "seq",
                "linear_x",
                "angular_z",
                "dt_since_previous_sec",
            ],
        )

        self.aux_commands_writer = CsvWriter(
            self.session_dir / "aux_commands_log.csv",
            [
                "timestamp",
                "elapsed_sec",
                "topic",
                "value",
            ],
        )

        self.telemetry_writer = CsvWriter(
            self.session_dir / "telemetry_log.csv",
            [
                "timestamp",
                "elapsed_sec",
                "seq",
                "payload_size_bytes",
                "field_count",
                "keys_preview",
                "dt_since_previous_sec",
                "raw_json",
            ],
        )

        self.typed_telemetry_writer = CsvWriter(
            self.session_dir / "typed_telemetry_log.csv",
            [
                "timestamp",
                "elapsed_sec",
                "source",
                "voltage",
                "current",
                "power",
                "percentage",
                "pcb_temp",
                "cpu_temp",
                "pitch",
                "rotation",
                "motor_left",
                "motor_right",
                "mag_x",
                "mag_y",
                "mag_z",
                "robot_server_ping",
                "client_server_ping",
                "local_controller",
                "follow_mode",
                "button",
                "cmd_speed",
                "cmd_rotation",
                "cmd_cam_angle",
                "cmd_cam_stable",
                "cmd_force_rpi",
            ],
        )

        self.video_writer = CsvWriter(
            self.session_dir / "video_log.csv",
            [
                "timestamp",
                "elapsed_sec",
                "seq",
                "payload_size_bytes",
                "format",
                "dt_since_previous_sec",
            ],
        )

        self.audio_writer = CsvWriter(
            self.session_dir / "audio_log.csv",
            [
                "timestamp",
                "elapsed_sec",
                "seq",
                "direction",
                "payload_size_bytes",
                "sample_rate",
                "channels",
                "encoding",
                "samples_per_channel",
                "dt_since_previous_sec",
            ],
        )

        # Diagnostic logs are saved as JSON Lines because diagnostic messages
        # can evolve over time and may contain variable fields.
        self.driver_diag_writer = JsonLinesWriter(
            self.session_dir / "diagnostics_driver_log.jsonl"
        )
        self.teleop_diag_writer = JsonLinesWriter(
            self.session_dir / "diagnostics_teleop_log.jsonl"
        )
        self.audio_diag_writer = JsonLinesWriter(
            self.session_dir / "diagnostics_audio_log.jsonl"
        )
        self.video_diag_writer = JsonLinesWriter(
            self.session_dir / "diagnostics_video_log.jsonl"
        )

        self.metadata_writer = CsvWriter(
            self.session_dir / "session_metadata.csv",
            ["key", "value"],
        )

        self.write_metadata()

        # ========================================================
        # SUBSCRIBERS
        # ========================================================
        self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_callback, 10)

        self.create_subscription(Bool, FORCE_RPI_TOPIC, self.force_rpi_callback, 10)
        self.create_subscription(Bool, CAM_STABLE_TOPIC, self.cam_stable_callback, 10)
        self.create_subscription(Float32, CAM_ANGLE_TOPIC, self.cam_angle_callback, 10)
        self.create_subscription(AudioMode, AUDIO_MODE_TOPIC, self.audio_mode_callback, 10)

        self.create_subscription(
            String,
            TELEMETRY_JSON_TOPIC,
            self.telemetry_json_callback,
            10,
        )

        self.create_subscription(Battery, BATTERY_TOPIC, self.battery_callback, 10)
        self.create_subscription(
            Temperatures,
            TEMPERATURES_TOPIC,
            self.temperatures_callback,
            10,
        )
        self.create_subscription(
            DriveStatus,
            DRIVE_STATUS_TOPIC,
            self.drive_status_callback,
            10,
        )
        self.create_subscription(
            Magnetometer,
            MAGNETOMETER_TOPIC,
            self.magnetometer_callback,
            10,
        )
        self.create_subscription(
            ConnectionStatus,
            CONNECTION_STATUS_TOPIC,
            self.connection_status_callback,
            10,
        )
        self.create_subscription(
            CommandState,
            COMMAND_STATE_TOPIC,
            self.command_state_callback,
            10,
        )

        self.create_subscription(
            CompressedImage,
            VIDEO_TOPIC,
            self.video_callback,
            10,
        )

        self.create_subscription(AudioFrame, AUDIO_RX_TOPIC, self.audio_rx_callback, 10)
        self.create_subscription(AudioFrame, AUDIO_TX_TOPIC, self.audio_tx_callback, 10)

        self.create_subscription(String, DRIVER_DIAG_TOPIC, self.driver_diag_callback, 10)
        self.create_subscription(String, TELEOP_DIAG_TOPIC, self.teleop_diag_callback, 10)
        self.create_subscription(String, AUDIO_DIAG_TOPIC, self.audio_diag_callback, 10)
        self.create_subscription(String, VIDEO_DIAG_TOPIC, self.video_diag_callback, 10)

        self.get_logger().info(
            f"RB23 experiment logger started. Session directory: {self.session_dir}"
        )

    # ========================================================
    # METADATA
    # ========================================================

    def write_metadata(self) -> None:
        """
        Write basic session metadata.

        This file helps identify the experimental condition, the machine, and
        the software environment used during the run.
        """
        metadata = {
            "session_start_unix": self.start_time,
            "session_start_local": datetime.now().isoformat(timespec="seconds"),
            "experiment_label": self.experiment_label,
            "base_log_dir": str(self.base_log_dir),
            "session_dir": str(self.session_dir),
            "current_working_directory": str(Path.cwd()),
            "hostname": socket.gethostname(),
            "user": os.environ.get("USER", ""),
            "platform": platform.platform(),
            "python_version": platform.python_version(),
        }

        for key, value in metadata.items():
            self.metadata_writer.write({"key": key, "value": value})

    def elapsed(self, timestamp: Optional[float] = None) -> float:
        """
        Return elapsed time since the beginning of the logging session.
        """
        if timestamp is None:
            timestamp = wall_time()
        return timestamp - self.start_time

    # ========================================================
    # CONTROL CALLBACKS
    # ========================================================

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Log motion commands published to /rb23/cmd_vel.
        """
        now = wall_time()
        dt = "" if self.last_cmd_vel_time <= 0.0 else now - self.last_cmd_vel_time

        self.cmd_vel_count += 1
        self.last_cmd_vel_time = now

        self.control_writer.write(
            {
                "timestamp": now,
                "elapsed_sec": self.elapsed(now),
                "seq": self.cmd_vel_count,
                "linear_x": float(msg.linear.x),
                "angular_z": float(msg.angular.z),
                "dt_since_previous_sec": dt,
            }
        )

    def force_rpi_callback(self, msg: Bool) -> None:
        self.log_aux_command(FORCE_RPI_TOPIC, bool(msg.data))

    def cam_stable_callback(self, msg: Bool) -> None:
        self.log_aux_command(CAM_STABLE_TOPIC, bool(msg.data))

    def cam_angle_callback(self, msg: Float32) -> None:
        self.log_aux_command(CAM_ANGLE_TOPIC, float(msg.data))

    def audio_mode_callback(self, msg: AudioMode) -> None:
        self.log_aux_command(AUDIO_MODE_TOPIC, int(msg.mode))

    def log_aux_command(self, topic: str, value: Any) -> None:
        """
        Log auxiliary command topics.

        Auxiliary commands are not continuous velocity commands, but they are
        useful for reconstructing the experiment timeline.
        """
        now = wall_time()
        self.aux_commands_writer.write(
            {
                "timestamp": now,
                "elapsed_sec": self.elapsed(now),
                "topic": topic,
                "value": value,
            }
        )

    # ========================================================
    # RAW TELEMETRY CALLBACK
    # ========================================================

    def telemetry_json_callback(self, msg: String) -> None:
        """
        Log raw JSON telemetry published by the driver.

        This is the most complete telemetry representation currently available
        in GuardROS. Typed topics are useful for ROS integration, but the raw
        JSON is valuable for offline analysis because it preserves all fields.
        """
        now = wall_time()
        dt = "" if self.last_telemetry_json_time <= 0.0 else now - self.last_telemetry_json_time

        self.telemetry_json_count += 1
        self.last_telemetry_json_time = now

        decoded = safe_json_loads(msg.data)
        if decoded is None:
            field_count = 0
            keys_preview = ""
        else:
            keys = sorted(decoded.keys())
            field_count = len(keys)
            keys_preview = ",".join(keys[:12])

        self.telemetry_writer.write(
            {
                "timestamp": now,
                "elapsed_sec": self.elapsed(now),
                "seq": self.telemetry_json_count,
                "payload_size_bytes": len(msg.data.encode("utf-8")),
                "field_count": field_count,
                "keys_preview": keys_preview,
                "dt_since_previous_sec": dt,
                "raw_json": msg.data,
            }
        )

    # ========================================================
    # TYPED TELEMETRY CALLBACKS
    # ========================================================

    def battery_callback(self, msg: Battery) -> None:
        now = wall_time()
        self.typed_telemetry_writer.write({"timestamp": now, "elapsed_sec": self.elapsed(now), "source": BATTERY_TOPIC, "voltage": msg.voltage, "current": msg.current, "power": msg.power, "percentage": msg.percentage})

    def temperatures_callback(self, msg: Temperatures) -> None:
        now = wall_time()
        self.typed_telemetry_writer.write({"timestamp": now, "elapsed_sec": self.elapsed(now), "source": TEMPERATURES_TOPIC, "pcb_temp": msg.pcb_temp, "cpu_temp": msg.cpu_temp})

    def drive_status_callback(self, msg: DriveStatus) -> None:
        now = wall_time()
        self.typed_telemetry_writer.write({"timestamp": now, "elapsed_sec": self.elapsed(now), "source": DRIVE_STATUS_TOPIC, "pitch": msg.pitch, "rotation": msg.rotation, "motor_left": msg.motor_left, "motor_right": msg.motor_right})

    def magnetometer_callback(self, msg: Magnetometer) -> None:
        now = wall_time()
        self.typed_telemetry_writer.write({"timestamp": now, "elapsed_sec": self.elapsed(now), "source": MAGNETOMETER_TOPIC, "mag_x": msg.x, "mag_y": msg.y, "mag_z": msg.z})

    def connection_status_callback(self, msg: ConnectionStatus) -> None:
        now = wall_time()
        self.typed_telemetry_writer.write({"timestamp": now, "elapsed_sec": self.elapsed(now), "source": CONNECTION_STATUS_TOPIC, "robot_server_ping": msg.robot_server_ping, "client_server_ping": msg.client_server_ping, "local_controller": msg.local_controller, "follow_mode": msg.follow_mode, "button": msg.button})

    def command_state_callback(self, msg: CommandState) -> None:
        now = wall_time()
        self.typed_telemetry_writer.write({"timestamp": now, "elapsed_sec": self.elapsed(now), "source": COMMAND_STATE_TOPIC, "cmd_speed": msg.speed, "cmd_rotation": msg.rotation, "cmd_cam_angle": msg.cam_angle, "cmd_cam_stable": msg.cam_stable, "cmd_force_rpi": msg.force_rpi})

    # ========================================================
    # VIDEO CALLBACK
    # ========================================================

    def video_callback(self, msg: CompressedImage) -> None:
        """
        Log compressed image messages.

        The logger does not decode or display the frame. It only records timing,
        payload size, and format. Decoding is handled by the video viewer node.
        """
        now = wall_time()
        dt = "" if self.last_video_msg_time <= 0.0 else now - self.last_video_msg_time

        self.video_msg_count += 1
        self.last_video_msg_time = now

        self.video_writer.write(
            {
                "timestamp": now,
                "elapsed_sec": self.elapsed(now),
                "seq": self.video_msg_count,
                "payload_size_bytes": len(msg.data),
                "format": msg.format,
                "dt_since_previous_sec": dt,
            }
        )

    # ========================================================
    # AUDIO CALLBACKS
    # ========================================================

    def audio_rx_callback(self, msg: AudioFrame) -> None:
        self.log_audio_frame("rx", msg)

    def audio_tx_callback(self, msg: AudioFrame) -> None:
        self.log_audio_frame("tx", msg)

    def log_audio_frame(self, direction: str, msg: AudioFrame) -> None:
        """
        Log audio frames flowing through GuardROS.

        Direction:
        - rx: audio received from the robot and published by the driver.
        - tx: audio captured from the local microphone and sent to the driver.
        """
        now = wall_time()

        if direction == "rx":
            dt = "" if self.last_audio_rx_time <= 0.0 else now - self.last_audio_rx_time
            self.audio_rx_count += 1
            seq = self.audio_rx_count
            self.last_audio_rx_time = now
        else:
            dt = "" if self.last_audio_tx_time <= 0.0 else now - self.last_audio_tx_time
            self.audio_tx_count += 1
            seq = self.audio_tx_count
            self.last_audio_tx_time = now

        self.audio_writer.write(
            {
                "timestamp": now,
                "elapsed_sec": self.elapsed(now),
                "seq": seq,
                "direction": direction,
                "payload_size_bytes": len(msg.data),
                "sample_rate": msg.sample_rate,
                "channels": msg.channels,
                "encoding": msg.encoding,
                "samples_per_channel": msg.samples_per_channel,
                "dt_since_previous_sec": dt,
            }
        )

    # ========================================================
    # DIAGNOSTIC CALLBACKS
    # ========================================================

    def driver_diag_callback(self, msg: String) -> None:
        self.log_diagnostic("driver", msg, self.driver_diag_writer)

    def teleop_diag_callback(self, msg: String) -> None:
        self.log_diagnostic("teleop", msg, self.teleop_diag_writer)

    def audio_diag_callback(self, msg: String) -> None:
        self.log_diagnostic("audio", msg, self.audio_diag_writer)

    def video_diag_callback(self, msg: String) -> None:
        self.log_diagnostic("video", msg, self.video_diag_writer)

    def log_diagnostic(self, source: str, msg: String, writer: JsonLinesWriter) -> None:
        """
        Log diagnostic JSON messages from instrumented nodes.

        Diagnostic topics are expected to contain JSON strings. If a message
        cannot be decoded as JSON, the raw string is still saved.
        """
        now = wall_time()
        decoded = safe_json_loads(msg.data)

        if decoded is None:
            row = {"timestamp": now, "elapsed_sec": self.elapsed(now), "source": source, "decode_ok": False, "raw": msg.data}
        else:
            row = {"timestamp": now, "elapsed_sec": self.elapsed(now), "source": source, "decode_ok": True}
            row.update(flatten_dict(decoded))

        writer.write(row)

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def close(self) -> None:
        """
        Close all open files.

        This method should be called during node shutdown to avoid incomplete
        CSV files and to make sure all buffered data is written to disk.
        """
        writers = [
            self.control_writer,
            self.aux_commands_writer,
            self.telemetry_writer,
            self.typed_telemetry_writer,
            self.video_writer,
            self.audio_writer,
            self.driver_diag_writer,
            self.teleop_diag_writer,
            self.audio_diag_writer,
            self.video_diag_writer,
            self.metadata_writer,
        ]

        for writer in writers:
            try:
                writer.close()
            except Exception:
                pass


def main(args=None) -> None:
    """
    Entry point for the ROS 2 experiment logger node.
    """
    rclpy.init(args=args)
    node = RB23ExperimentLoggerNode()

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