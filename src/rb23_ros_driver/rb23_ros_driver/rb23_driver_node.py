#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rb23_driver_node.py

ROS 2 driver for the GuardBot RollerBot 23 (RB23).

Main responsibilities:
----------------------
1. Continuously send UDP command packets to the robot.
2. Receive JSON telemetry packets sent by the robot.
3. Publish telemetry through:
   - a raw debugging topic (/rb23/telemetry_json)
   - multiple typed ROS topics suitable for integration with other nodes
4. Receive ROS 2 commands and convert them to the internal command format
   used by the RB23 UDP protocol.
5. Detect JPEG frames arriving in the same UDP stream and publish them as
   compressed ROS images:
   - /rb23/camera/image/compressed
6. Detect audio packets arriving in the same UDP stream and publish them as
   typed ROS audio messages:
   - /rb23/audio/rx
7. Receive ROS audio frames on /rb23/audio/tx and encapsulate them in the
   format expected by the robot.
8. Publish internal diagnostic information on /rb23/diagnostics/driver so that
   a future experiment logger can record timing, packet counters, and failure
   events without changing the robot firmware or hardware.

Current architecture:
---------------------
ROS topics -> rb23_driver_node -> UDP -> RB23

Published topics:
-----------------
- /rb23/telemetry_json              -> std_msgs/String
- /rb23/battery                     -> guardros_msgs/Battery
- /rb23/temperatures                -> guardros_msgs/Temperatures
- /rb23/drive_status                -> guardros_msgs/DriveStatus
- /rb23/magnetometer                -> guardros_msgs/Magnetometer
- /rb23/connection_status           -> guardros_msgs/ConnectionStatus
- /rb23/command_state               -> guardros_msgs/CommandState
- /rb23/camera/image/compressed     -> sensor_msgs/CompressedImage
- /rb23/audio/rx                    -> guardros_msgs/AudioFrame
- /rb23/diagnostics/driver          -> std_msgs/String containing JSON diagnostics

Subscribed topics:
------------------
- /rb23/cmd_vel            -> geometry_msgs/Twist
- /rb23/force_rpi          -> std_msgs/Bool
- /rb23/cam_stable         -> std_msgs/Bool
- /rb23/cam_angle          -> std_msgs/Float32
- /rb23/audio/tx           -> guardros_msgs/AudioFrame

Important notes:
----------------
- The RB23 protocol uses a one-byte identifier before the JSON payload.
- Some telemetry values may arrive as single-element lists. This is handled
  explicitly in the conversion helper functions.
- Battery percentage is currently estimated from voltage because an explicit
  percentage field has not yet been identified in the JSON telemetry.
- The RB23 UDP stream may carry different data types on the same socket:
  telemetry, video, audio, and unknown packets.
- This file is intentionally written with detailed comments to support future
  publication, reproducibility, and maintenance by other engineers.

Fail-safe:
----------
This driver implements a safety timeout for /rb23/cmd_vel. If the keyboard
teleoperation node, or any future command node, stops publishing commands, the
driver automatically forces:
    speed = 0.0
    rotation = 0.0
This prevents the robot from continuing to move indefinitely.
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
# GENERAL DRIVER SETTINGS
# ============================================================

# Server address currently used by the RB23 ecosystem.
SERVER_HOST = "RB23-Brazil.harv-guardbot.org"
SERVER_PORT = 11000

# Default ROBOT_ID used when the user does not provide another value through a ROS parameter.
DEFAULT_ROBOT_ID = 1

# UDP socket timeout.
SOCKET_TIMEOUT = 0.2

# ROS telemetry publishing period.
PUBLISH_PERIOD = 0.10

# Continuous command transmission period.
SEND_PERIOD = 0.05

# Topic used to publish internal driver diagnostics.
# The future experiment logger will subscribe to this topic and save the
# diagnostic stream to CSV files for IEEE RA-P experiments.
DRIVER_DIAGNOSTICS_TOPIC = "/rb23/diagnostics/driver"

# Period used to publish driver diagnostics.
# This value is intentionally slower than SEND_PERIOD to avoid unnecessary
# diagnostic traffic while still capturing the driver state during experiments.
DIAGNOSTICS_PERIOD = 0.50

# Safety timeout for /rb23/cmd_vel.
DEFAULT_CMD_VEL_TIMEOUT_SEC = 0.30

# Temporary voltage-based estimate for a nominal 12 V lithium battery.
BATTERY_EMPTY_VOLTAGE = 9.0
BATTERY_FULL_VOLTAGE = 12.6


# ============================================================
# VIDEO SETTINGS
# ============================================================

# ROS topic where JPEG frames extracted from UDP packets are published.
VIDEO_TOPIC_COMPRESSED = "/rb23/camera/image/compressed"

# Very small packets are unlikely to contain useful JPEG frames.
VIDEO_PACKET_MIN_BYTES = 256

# Standard JPEG start-of-image and end-of-image markers.
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"


# ============================================================
# AUDIO SETTINGS
# ============================================================

# ROS audio topics.
AUDIO_RX_TOPIC = "/rb23/audio/rx"
AUDIO_TX_TOPIC = "/rb23/audio/tx"

# Expected identifier for audio RX packets coming from the robot.
# This convention was previously observed and validated in the Linux client.
AUDIO_RX_ID_BASE = 0x80

# Prefix used when sending TX audio to the robot.
# Previously observed and validated format:
#   0x01 + 1024 amostras PCM16 little-endian mono
AUDIO_TX_PREFIX = 0x01

# ROS metadata used for audio received from the robot.
# No cliente Linux, o áudio RX foi tratado como PCM unsigned 8-bit,
# mono, e depois ampliado para reprodução local.
# Aqui no driver publicamos o frame bruto e informamos esses metadados.
AUDIO_RX_ENCODING = "pcm_u8"
AUDIO_RX_CHANNELS = 1
AUDIO_RX_SAMPLE_RATE = 16000

# Expected metadata for TX audio received from ROS.
AUDIO_TX_EXPECTED_ENCODING = "pcm_s16le"
AUDIO_TX_EXPECTED_CHANNELS = 1
AUDIO_TX_EXPECTED_SAMPLE_RATE = 48000
AUDIO_TX_SAMPLES_PER_PACKET = 1024
AUDIO_TX_PAYLOAD_BYTES = 1 + 2 * AUDIO_TX_SAMPLES_PER_PACKET

# Reasonable lower bound used to distinguish RX audio from noise.
AUDIO_RX_PACKET_MIN = 64

# Number of RX packet header bytes that must be skipped
# before the audio data. In the observed protocol, the first
# byte is the packet ID.
AUDIO_RX_HEADER_SKIP = 1


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def now_millis() -> float:
    """
    Return the current time in milliseconds.
    """
    return time.time() * 1000.0


def compute_tx_id(robot_id: int) -> int:
    """
    Compute the transmission identifier (TX_ID) used in packets sent to the robot.
    """
    return ((0xC0 + robot_id) & 0x7F)


def compute_rx_id(robot_id: int) -> int:
    """
    Compute the expected reception identifier (RX_ID) used in telemetry packets from the robot.
    """
    return (0xC0 + robot_id) & 0xFF


def compute_audio_rx_id(robot_id: int) -> int:
    """
    Compute the expected reception identifier for audio packets coming from the robot.
    """
    return (AUDIO_RX_ID_BASE + robot_id) & 0xFF


def build_command_packet(tx_id: int, payload: Dict[str, Any]) -> bytes:
    """
    Build the UDP command packet sent to the robot.

    Structure:
        1 TX identifier byte + UTF-8 JSON payload
    """
    data = dict(payload)
    data["absolute_ping_millis"] = now_millis()
    payload_bytes = json.dumps(data, separators=(",", ":")).encode("utf-8")
    return bytes([tx_id]) + payload_bytes


def try_decode_telemetry(packet: bytes) -> Optional[Dict[str, Any]]:
    """
    Try to decode a received UDP packet as JSON telemetry.

    Expected structure:
        1 RX identifier byte + UTF-8 JSON payload

    If the packet is not valid JSON, return None.
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
    Search for a JPEG frame inside a raw UDP packet.

    Strategy:
    - locate the JPEG start marker (FFD8)
    - locate the JPEG end marker (FFD9)
    - extract only that byte range

    Returns:
    - JPEG bytes, if found
    - None, if the packet does not contain a valid JPEG frame
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
    Try to determine whether a raw UDP packet looks like an RX audio packet.

    Criteria used:
    - reasonable minimum size
    - first byte compatible with the expected audio ID
    - absence of typical JPEG markers
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
    Extract the audio payload coming from the robot.

    At this stage, the driver does not decode audio to float and does not
    perform upsampling. It only publishes the raw payload through ROS.

    Returns:
    - audio bytes, if the packet appears valid
    - None otherwise
    """
    if not is_probable_audio_rx(packet, expected_audio_rx_id):
        return None

    payload = packet[AUDIO_RX_HEADER_SKIP:]

    if len(payload) < (AUDIO_RX_PACKET_MIN - AUDIO_RX_HEADER_SKIP):
        return None

    return payload


def clamp(value: float, low: float, high: float) -> float:
    """
    Clamp a value to the interval [low, high].
    """
    return max(low, min(value, high))


def safe_first(value: Any) -> Any:
    """
    If the value arrives as a non-empty list, return its first element.
    Otherwise, return the value itself.
    """
    if isinstance(value, list) and value:
        return value[0]
    return value


def to_float(value: Any, default: float = 0.0) -> float:
    """
    Robustly convert any value to float.
    """
    value = safe_first(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def to_int(value: Any, default: int = 0) -> int:
    """
    Robustly convert any value to int.
    """
    value = safe_first(value)
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def to_bool(value: Any, default: bool = False) -> bool:
    """
    Robustly convert any value to bool.
    """
    value = safe_first(value)

    if isinstance(value, bool):
        return value

    if isinstance(value, (int, float)):
        return bool(value)

    return default


def estimate_battery_percentage(voltage: float) -> float:
    """
    Estimate battery percentage from voltage.
    """
    if BATTERY_FULL_VOLTAGE <= BATTERY_EMPTY_VOLTAGE:
        return 0.0

    percentage = 100.0 * (voltage - BATTERY_EMPTY_VOLTAGE) / (
        BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE
    )
    return clamp(percentage, 0.0, 100.0)


# ============================================================
# MAIN ROS NODE CLASS
# ============================================================

class RB23RosDriver(Node):
    """
    Main GuardROS ROS 2 node responsible for communication with the RB23.

    Responsibilities:
    - open and maintain a UDP socket connected to the robot server
    - send commands to the robot in a dedicated thread
    - receive telemetry/video/audio in another dedicated thread
    - publish these data streams as ROS topics
    - receive ROS commands and convert them to the RB23 protocol
    """

    def __init__(self) -> None:
        super().__init__("rb23_driver_node")

        # --------------------------------------------------------
        # ROS PARAMETERS
        # --------------------------------------------------------
        self.declare_parameter("robot_id", DEFAULT_ROBOT_ID)
        self.declare_parameter("cmd_vel_timeout_sec", DEFAULT_CMD_VEL_TIMEOUT_SEC)

        self.server_host = SERVER_HOST
        self.server_port = SERVER_PORT
        self.robot_id = int(self.get_parameter("robot_id").value)
        self.cmd_vel_timeout_sec = float(
            self.get_parameter("cmd_vel_timeout_sec").value
        )

        # RB23 UDP protocol identifiers.
        self.tx_id = compute_tx_id(self.robot_id)
        self.rx_id = compute_rx_id(self.robot_id)
        self.audio_rx_id = compute_audio_rx_id(self.robot_id)

        # --------------------------------------------------------
        # ROS PUBLISHERS
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

        # Compressed video publisher.
        self.camera_compressed_pub = self.create_publisher(
            CompressedImage,
            VIDEO_TOPIC_COMPRESSED,
            10,
        )

        # RX audio publisher.
        self.audio_rx_pub = self.create_publisher(
            AudioFrame,
            AUDIO_RX_TOPIC,
            10,
        )

        # Internal driver diagnostics publisher.
        # This topic does not control the robot and does not change the driver
        # behavior. It only exposes counters and timing information that would
        # otherwise remain hidden inside this node. The future experiment logger
        # will subscribe to this topic and save the messages for offline analysis.
        self.driver_diagnostics_pub = self.create_publisher(
            String,
            DRIVER_DIAGNOSTICS_TOPIC,
            10,
        )

        # --------------------------------------------------------
        # ROS SUBSCRIBERS
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
        # INTERNAL STATE SHARED ACROSS THREADS
        # --------------------------------------------------------
        self.state_lock = threading.Lock()
        self.socket_lock = threading.Lock()

        # Last telemetry dictionary received from the robot.
        self.latest_telemetry: Dict[str, Any] = {}

        # Auxiliary telemetry reception state.
        self.last_rx_id: Optional[int] = None
        self.last_rx_time: float = 0.0
        self.last_published_rx_time: float = 0.0

        # Internal video state used for basic diagnostics.
        self.video_packets_ok: int = 0
        self.video_packets_non_jpeg: int = 0
        self.last_video_time: float = 0.0
        self.first_video_packet_logged: bool = False

        # Internal audio state used for basic diagnostics.
        self.audio_packets_ok: int = 0
        self.audio_packets_non_audio: int = 0
        self.audio_tx_packets_sent: int = 0
        self.last_audio_rx_time: float = 0.0
        self.last_audio_tx_time: float = 0.0
        self.last_audio_rx_bytes: int = 0
        self.last_audio_tx_bytes: int = 0
        self.first_audio_packet_logged: bool = False

        # Internal counters used for experimental diagnostics.
        # These counters are deliberately stored inside the driver because some
        # events cannot be observed accurately by an external logger. Examples
        # include UDP send attempts, socket timeouts, packet classification, and
        # internal command callback timing.
        self.cmd_vel_messages_rx: int = 0
        self.force_rpi_messages_rx: int = 0
        self.cam_stable_messages_rx: int = 0
        self.cam_angle_messages_rx: int = 0
        self.audio_tx_messages_rx: int = 0

        self.udp_command_packets_sent: int = 0
        self.udp_command_send_errors: int = 0
        self.udp_audio_packets_sent: int = 0
        self.udp_audio_send_errors: int = 0
        self.udp_packets_rx_total: int = 0
        self.udp_socket_timeouts: int = 0
        self.udp_recv_errors: int = 0
        self.udp_unknown_packets: int = 0

        self.telemetry_packets_rx: int = 0
        self.telemetry_packets_published: int = 0
        self.telemetry_packets_wrong_id: int = 0
        self.telemetry_decode_attempts_ok: int = 0
        self.telemetry_decode_attempts_failed: int = 0

        self.last_udp_command_send_time: float = 0.0
        self.last_udp_rx_time: float = 0.0
        self.last_cmd_vel_rx_time: float = 0.0
        self.last_driver_error: str = "-"

        # Current command state being sent to the robot.
        self.command_state: Dict[str, Any] = {
            "cam_angle": 0.0,
            "cam_stable": 1,
            "force_rpi": 0,
            "speed": 0.0,
            "rotation": 0.0,
        }

        # Time of the last received /rb23/cmd_vel message.
        self.last_cmd_vel_time: float = 0.0

        self.first_packet_logged = False
        self.running = True

        # --------------------------------------------------------
        # UDP SOCKET
        # --------------------------------------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(SOCKET_TIMEOUT)
        self.sock.connect((self.server_host, self.server_port))

        self.get_logger().info(
            f"RB23 driver started | "
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
        # SENDER AND RECEIVER THREADS
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
        # ROS PUBLISHING TIMER
        # --------------------------------------------------------
        self.publish_timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_telemetry,
        )

        # Periodic diagnostics timer.
        # This timer publishes a compact JSON snapshot of internal driver state.
        # The message is intentionally encoded as std_msgs/String to avoid adding
        # a new custom message type at this stage of the project. Later, if the
        # diagnostic schema becomes stable, it can be migrated to a dedicated
        # guardros_msgs message definition.
        self.diagnostics_timer = self.create_timer(
            DIAGNOSTICS_PERIOD,
            self.publish_driver_diagnostics,
        )

    # =========================================================
    # COMMAND TOPIC CALLBACKS
    # =========================================================

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Receive ROS 2 commands from /rb23/cmd_vel and update the internal driver command state.
        """
        with self.state_lock:
            self.command_state["speed"] = float(msg.linear.x)
            self.command_state["rotation"] = -float(msg.angular.z)
            self.last_cmd_vel_time = time.time()
            self.last_cmd_vel_rx_time = self.last_cmd_vel_time
            self.cmd_vel_messages_rx += 1

    def force_rpi_callback(self, msg: Bool) -> None:
        """
        Update the force_rpi field in the internal command state.
        """
        with self.state_lock:
            self.command_state["force_rpi"] = 1 if msg.data else 0
            self.force_rpi_messages_rx += 1

    def cam_stable_callback(self, msg: Bool) -> None:
        """
        Update the cam_stable field in the internal command state.
        """
        with self.state_lock:
            self.command_state["cam_stable"] = 1 if msg.data else 0
            self.cam_stable_messages_rx += 1

    def cam_angle_callback(self, msg: Float32) -> None:
        """
        Update the cam_angle field in the internal command state.
        """
        with self.state_lock:
            self.command_state["cam_angle"] = float(msg.data)
            self.cam_angle_messages_rx += 1

    def audio_tx_callback(self, msg: AudioFrame) -> None:
        """
        Receive one ROS audio frame and send it to the robot through UDP.

        Formato expected:
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

        with self.state_lock:
            self.audio_tx_messages_rx += 1

        if len(payload_bytes) != (2 * AUDIO_TX_SAMPLES_PER_PACKET):
            self.get_logger().warn(
                "TX audio frame ignored: invalid size. "
                f"Received {len(payload_bytes)} bytes, expected {2 * AUDIO_TX_SAMPLES_PER_PACKET}."
            )
            return

        if msg.channels != AUDIO_TX_EXPECTED_CHANNELS:
            self.get_logger().warn(
                f"TX audio with channels={msg.channels}, expected={AUDIO_TX_EXPECTED_CHANNELS}."
            )

        if msg.sample_rate != AUDIO_TX_EXPECTED_SAMPLE_RATE:
            self.get_logger().warn(
                f"TX audio with sample_rate={msg.sample_rate}, expected={AUDIO_TX_EXPECTED_SAMPLE_RATE}."
            )

        if msg.encoding != AUDIO_TX_EXPECTED_ENCODING:
            self.get_logger().warn(
                f"TX audio with encoding='{msg.encoding}', expected='{AUDIO_TX_EXPECTED_ENCODING}'."
            )

        if msg.samples_per_channel != AUDIO_TX_SAMPLES_PER_PACKET:
            self.get_logger().warn(
                f"TX audio with samples_per_channel={msg.samples_per_channel}, "
                f"expected={AUDIO_TX_SAMPLES_PER_PACKET}."
            )

        packet = bytes([AUDIO_TX_PREFIX]) + payload_bytes

        if len(packet) != AUDIO_TX_PAYLOAD_BYTES:
            self.get_logger().warn(
                "TX audio packet not sent: invalid final size. "
                f"Received {len(packet)} bytes, expected {AUDIO_TX_PAYLOAD_BYTES}."
            )
            return

        try:
            with self.socket_lock:
                self.sock.send(packet)

            with self.state_lock:
                self.audio_tx_packets_sent += 1
                self.udp_audio_packets_sent += 1
                self.last_audio_tx_time = time.time()
                self.last_audio_tx_bytes = len(packet)

        except OSError as exc:
            with self.state_lock:
                self.udp_audio_send_errors += 1
                self.last_driver_error = f"audio_tx_os_error: {exc}"
        except Exception as exc:
            with self.state_lock:
                self.udp_audio_send_errors += 1
                self.last_driver_error = f"audio_tx_error: {exc}"
            self.get_logger().warn(f"Error while sending TX audio through UDP: {exc}")

    # =========================================================
    # MESSAGE EXTRACTION FUNCTIONS
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
        Build a ROS AudioFrame message from a raw RX audio payload coming from the robot.
        """
        msg = AudioFrame()
        msg.data = list(audio_payload)
        msg.sample_rate = AUDIO_RX_SAMPLE_RATE
        msg.channels = AUDIO_RX_CHANNELS
        msg.encoding = AUDIO_RX_ENCODING
        msg.samples_per_channel = len(audio_payload)
        return msg

    # =========================================================
    # VIDEO FUNCTIONS
    # =========================================================

    def publish_compressed_video(self, jpeg_bytes: bytes) -> None:
        """
        Publish one JPEG frame on the ROS compressed image topic.
        """
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = jpeg_bytes
        self.camera_compressed_pub.publish(msg)

    # =========================================================
    # CONTINUOUS SENDER THREAD
    # =========================================================

    def sender_loop(self) -> None:
        """
        Thread responsible for continuously sending the command state to the robot.
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

                with self.state_lock:
                    self.udp_command_packets_sent += 1
                    self.last_udp_command_send_time = time.time()

            except OSError as exc:
                with self.state_lock:
                    self.udp_command_send_errors += 1
                    self.last_driver_error = f"command_send_os_error: {exc}"
                break
            except Exception as exc:
                with self.state_lock:
                    self.udp_command_send_errors += 1
                    self.last_driver_error = f"command_send_error: {exc}"
                self.get_logger().warn(f"UDP send error: {exc}")

            time.sleep(SEND_PERIOD)

    # =========================================================
    # RECEIVER THREAD
    # =========================================================

    def receiver_loop(self) -> None:
        """
        Thread responsible for receiving UDP packets from the robot.

        Adopted strategy:
        1. Try to interpret the packet as JSON telemetry.
        2. If it is not telemetry, try to extract a JPEG video frame.
        3. If it is not video, try to interpret it as RX audio.
        4. If it matches none of these categories, ignore it.
        """
        while self.running:
            try:
                with self.socket_lock:
                    packet = self.sock.recv(65535)

            except socket.timeout:
                with self.state_lock:
                    self.udp_socket_timeouts += 1
                continue
            except OSError as exc:
                with self.state_lock:
                    self.udp_recv_errors += 1
                    self.last_driver_error = f"recv_os_error: {exc}"
                break
            except Exception as exc:
                with self.state_lock:
                    self.udp_recv_errors += 1
                    self.last_driver_error = f"recv_error: {exc}"
                self.get_logger().warn(f"UDP receive error: {exc}")
                continue

            with self.state_lock:
                self.udp_packets_rx_total += 1
                self.last_udp_rx_time = time.time()

            # ------------------------------------------------------
            # 1. JSON TELEMETRY DECODING ATTEMPT
            # ------------------------------------------------------
            decoded = try_decode_telemetry(packet)
            if decoded is not None:
                with self.state_lock:
                    self.telemetry_decode_attempts_ok += 1
                msg_id = decoded["msg_id"]
                data = decoded["data"]

                if msg_id == self.rx_id:
                    with self.state_lock:
                        self.latest_telemetry = data
                        self.last_rx_id = msg_id
                        self.last_rx_time = time.time()
                        self.telemetry_packets_rx += 1

                    if not self.first_packet_logged:
                        self.first_packet_logged = True
                        self.get_logger().info(
                            "First telemetry packet received successfully."
                        )

                else:
                    with self.state_lock:
                        self.telemetry_packets_wrong_id += 1

                continue
            else:
                with self.state_lock:
                    self.telemetry_decode_attempts_failed += 1

            # ------------------------------------------------------
            # 2. JPEG VIDEO EXTRACTION ATTEMPT
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
                            "First JPEG frame received and published successfully."
                        )

                except Exception as exc:
                    self.get_logger().warn(
                        f"Failed to publish compressed frame: {exc}"
                    )

                continue

            # ------------------------------------------------------
            # 3. RX AUDIO EXTRACTION ATTEMPT
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
                            "First RX audio packet received and published successfully."
                        )

                except Exception as exc:
                    self.get_logger().warn(
                        f"Failed to publish RX audio frame: {exc}"
                    )

                continue

            # ------------------------------------------------------
            # 4. UNKNOWN / IGNORED PACKET
            # ------------------------------------------------------
            with self.state_lock:
                self.video_packets_non_jpeg += 1
                self.audio_packets_non_audio += 1
                self.udp_unknown_packets += 1

    # =========================================================
    # ROS PUBLICATION
    # =========================================================

    def publish_telemetry(self) -> None:
        """
        Publish the latest received telemetry on all relevant ROS topics.
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

        with self.state_lock:
            self.telemetry_packets_published += 1


    # =========================================================
    # DRIVER DIAGNOSTICS PUBLICATION
    # =========================================================

    def publish_driver_diagnostics(self) -> None:
        """
        Publish a compact diagnostic snapshot of the internal driver state.

        The future experiment logger will subscribe to this topic and write the
        JSON payload to a CSV or JSONL file. This keeps the driver focused on
        robot communication while still exposing measurements that cannot be
        reconstructed from public ROS topics alone.

        The diagnostic message is intentionally non-invasive:
        - it does not change command values;
        - it does not change UDP packet formatting;
        - it does not modify the robot firmware or hardware;
        - it only publishes counters, timestamps, and basic status values.
        """
        now = time.time()

        with self.state_lock:
            command_state_copy = dict(self.command_state)

            diagnostics = {
                "node": "rb23_driver_node",
                "timestamp": now,
                "robot_id": self.robot_id,
                "server_host": self.server_host,
                "server_port": self.server_port,
                "tx_id": self.tx_id,
                "rx_id": self.rx_id,
                "audio_rx_id": self.audio_rx_id,
                "send_period_sec": SEND_PERIOD,
                "publish_period_sec": PUBLISH_PERIOD,
                "socket_timeout_sec": SOCKET_TIMEOUT,
                "cmd_vel_timeout_sec": self.cmd_vel_timeout_sec,

                "cmd_vel_messages_rx": self.cmd_vel_messages_rx,
                "force_rpi_messages_rx": self.force_rpi_messages_rx,
                "cam_stable_messages_rx": self.cam_stable_messages_rx,
                "cam_angle_messages_rx": self.cam_angle_messages_rx,
                "audio_tx_messages_rx": self.audio_tx_messages_rx,

                "udp_command_packets_sent": self.udp_command_packets_sent,
                "udp_command_send_errors": self.udp_command_send_errors,
                "udp_audio_packets_sent": self.udp_audio_packets_sent,
                "udp_audio_send_errors": self.udp_audio_send_errors,
                "udp_packets_rx_total": self.udp_packets_rx_total,
                "udp_socket_timeouts": self.udp_socket_timeouts,
                "udp_recv_errors": self.udp_recv_errors,
                "udp_unknown_packets": self.udp_unknown_packets,

                "telemetry_packets_rx": self.telemetry_packets_rx,
                "telemetry_packets_published": self.telemetry_packets_published,
                "telemetry_packets_wrong_id": self.telemetry_packets_wrong_id,
                "telemetry_decode_attempts_ok": self.telemetry_decode_attempts_ok,
                "telemetry_decode_attempts_failed": self.telemetry_decode_attempts_failed,

                "video_packets_ok": self.video_packets_ok,
                "video_packets_non_jpeg": self.video_packets_non_jpeg,
                "audio_packets_ok": self.audio_packets_ok,
                "audio_packets_non_audio": self.audio_packets_non_audio,
                "audio_tx_packets_sent": self.audio_tx_packets_sent,
                "last_audio_rx_bytes": self.last_audio_rx_bytes,
                "last_audio_tx_bytes": self.last_audio_tx_bytes,

                "last_cmd_vel_rx_time": self.last_cmd_vel_rx_time,
                "last_udp_command_send_time": self.last_udp_command_send_time,
                "last_udp_rx_time": self.last_udp_rx_time,
                "last_telemetry_rx_time": self.last_rx_time,
                "last_video_time": self.last_video_time,
                "last_audio_rx_time": self.last_audio_rx_time,
                "last_audio_tx_time": self.last_audio_tx_time,

                "age_cmd_vel_rx_sec": now - self.last_cmd_vel_rx_time if self.last_cmd_vel_rx_time > 0.0 else None,
                "age_udp_command_send_sec": now - self.last_udp_command_send_time if self.last_udp_command_send_time > 0.0 else None,
                "age_udp_rx_sec": now - self.last_udp_rx_time if self.last_udp_rx_time > 0.0 else None,
                "age_telemetry_rx_sec": now - self.last_rx_time if self.last_rx_time > 0.0 else None,
                "age_video_sec": now - self.last_video_time if self.last_video_time > 0.0 else None,
                "age_audio_rx_sec": now - self.last_audio_rx_time if self.last_audio_rx_time > 0.0 else None,
                "age_audio_tx_sec": now - self.last_audio_tx_time if self.last_audio_tx_time > 0.0 else None,

                "command_state": command_state_copy,
                "last_driver_error": self.last_driver_error,
            }

        msg = String()
        msg.data = json.dumps(diagnostics, separators=(",", ":"))
        self.driver_diagnostics_pub.publish(msg)

    # =========================================================
    # SHUTDOWN
    # =========================================================

    def close(self) -> None:
        """
        Shut down the driver in an orderly way.
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
# MAIN FUNCTION
# ============================================================

def main(args=None) -> None:
    """
    ROS 2 node entry point.
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