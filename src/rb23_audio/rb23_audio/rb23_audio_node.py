#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rb23_audio_node.py

ROS 2 node responsible for local audio handling in GuardROS.

Main functions:
---------------
1. Subscribe to the audio mode topic at /rb23/audio_mode.
2. Subscribe to audio received from the robot at /rb23/audio/rx.
3. Play robot audio through the local speaker when the node is in "listen" mode.
4. Capture local microphone audio when the node is in "talk" mode.
5. Publish captured microphone audio to /rb23/audio/tx.
6. Publish internal audio diagnostics to /rb23/diagnostics/audio for experimental logging.

Architecture:
-------------
RB23 -> rb23_driver_node -> /rb23/audio/rx -> rb23_audio_node -> local speaker
local microphone -> rb23_audio_node -> /rb23/audio/tx -> rb23_driver_node -> RB23

Diagnostic architecture for experiments:
----------------------------------------
rb23_audio_node -> /rb23/diagnostics/audio -> future rb23_experiment_logger_node -> CSV files

Operating modes:
----------------
- MODE_SILENCE:
    the node neither plays received audio nor transmits microphone audio
- MODE_LISTEN:
    the node plays audio received from the robot
- MODE_TALK:
    the node captures the local microphone and publishes audio frames to the driver

Important notes:
----------------
- This node uses sounddevice to communicate with the local PC audio devices.
- The operational audio behavior was kept intentionally simple and transparent.
- The implementation was inspired by the previously validated Linux client behavior,
  but adapted to the modular ROS 2 architecture used by GuardROS.
- The diagnostic topic does not change robot behavior. It only exposes internal
  counters, timestamps, queue state, and error information so that experiments can
  be logged later by a dedicated logger node.
"""

from collections import deque
import json
import threading
import time
from typing import Deque, Optional

import numpy as np
import rclpy
from guardros_msgs.msg import AudioFrame
from guardros_msgs.msg import AudioMode
from rclpy.node import Node
from std_msgs.msg import String

try:
    import sounddevice as sd
    SOUNDDEVICE_AVAILABLE = True
    SOUNDDEVICE_IMPORT_ERROR = "-"
except Exception as exc:
    sd = None
    SOUNDDEVICE_AVAILABLE = False
    SOUNDDEVICE_IMPORT_ERROR = str(exc)


# ============================================================
# AUDIO CONFIGURATION
# ============================================================

# ROS topics used by this node.
AUDIO_MODE_TOPIC = "/rb23/audio_mode"
AUDIO_RX_TOPIC = "/rb23/audio/rx"
AUDIO_TX_TOPIC = "/rb23/audio/tx"

# Diagnostic topic used by the future experiment logger.
#
# This topic publishes JSON strings with internal audio-state information.
# The logger node will subscribe to this topic and write the received messages
# to a CSV file. Keeping file writing outside this node preserves separation
# of responsibilities:
#   - rb23_audio_node: real-time audio handling
#   - rb23_experiment_logger_node: persistent experimental data logging
AUDIO_DIAGNOSTICS_TOPIC = "/rb23/diagnostics/audio"

# Diagnostic publication rate.
#
# A moderate rate is enough because this topic is meant to record the state of
# the audio subsystem over time, not to store every individual audio sample.
AUDIO_DIAGNOSTICS_RATE_HZ = 1.0

# ------------------------------------------------------------
# PLAYBACK OF AUDIO RECEIVED FROM THE ROBOT
# ------------------------------------------------------------
# The driver publishes RX audio with these metadata.
AUDIO_RX_EXPECTED_ENCODING = "pcm_u8"
AUDIO_RX_EXPECTED_SAMPLE_RATE = 16000
AUDIO_RX_EXPECTED_CHANNELS = 1

# For the local speaker, we prefer to play at 48 kHz.
# A simple interpolation-based upsampling is used.
AUDIO_OUTPUT_RATE = 48000
AUDIO_RX_UPSAMPLE_FACTOR = 3
AUDIO_OUTPUT_BLOCKSIZE = 1024

# Queue of audio buffers already converted to float32.
AUDIO_RX_QUEUE_MAX = 100

# ------------------------------------------------------------
# MICROPHONE CAPTURE FOR TRANSMISSION TO THE ROBOT
# ------------------------------------------------------------
AUDIO_TX_ENCODING = "pcm_s16le"
AUDIO_TX_SAMPLE_RATE = 48000
AUDIO_TX_CHANNELS = 1
AUDIO_TX_SAMPLES_PER_PACKET = 1024

# Gain applied to the local microphone signal.
AUDIO_TX_GAIN = 0.35

# Simple noise gate.
# Below this RMS value, the node sends real digital silence.
AUDIO_TX_GATE_RMS = 450.0


class RB23AudioNode(Node):
    """
    Local audio node for GuardROS.

    This node does not communicate directly with the RB23 robot.
    It communicates only through ROS topics:
    - It receives robot audio from the driver through /rb23/audio/rx.
    - It sends microphone audio to the driver through /rb23/audio/tx.
    - It receives the desired audio mode from /rb23/audio_mode.
    - It publishes diagnostic information through /rb23/diagnostics/audio.
    """

    def __init__(self) -> None:
        super().__init__("rb23_audio_node")

        # =========================================================
        # ROS PARAMETERS
        # =========================================================
        self.declare_parameter("audio_tx_gain", AUDIO_TX_GAIN)
        self.declare_parameter("audio_tx_gate_rms", AUDIO_TX_GATE_RMS)
        self.declare_parameter("diagnostics_rate_hz", AUDIO_DIAGNOSTICS_RATE_HZ)

        self.audio_tx_gain = float(self.get_parameter("audio_tx_gain").value)
        self.audio_tx_gate_rms = float(self.get_parameter("audio_tx_gate_rms").value)
        self.diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)

        # =========================================================
        # INTERNAL STATE
        # =========================================================
        self.state_lock = threading.RLock()

        # Initial audio mode: bilateral silence.
        self.current_audio_mode = AudioMode.MODE_SILENCE

        # Last human-readable state used for logs and diagnostics.
        self.audio_status = "silence"

        # Queue of RX audio chunks ready for local playback.
        self.audio_rx_queue: Deque[np.ndarray] = deque(maxlen=AUDIO_RX_QUEUE_MAX)

        # Local audio streams.
        self.audio_output_stream = None
        self.audio_input_stream = None

        self.audio_output_started = False
        self.audio_input_started = False

        # Timestamps of relevant audio events.
        #
        # time.time() is used here because these values are intended for
        # external CSV-based experimental analysis. The important property is
        # consistency within a given run, not synchronization with the robot.
        self.node_start_time = time.time()
        self.last_audio_rx_time = 0.0
        self.last_audio_tx_time = 0.0
        self.last_audio_mode_change_time = 0.0
        self.last_diagnostics_time = 0.0

        # Audio packet counters.
        #
        # These counters are useful for verifying whether audio was active
        # during an experiment and whether the local playback queue overflowed.
        self.audio_packets_rx = 0
        self.audio_packets_tx = 0
        self.audio_packets_rx_dropped = 0
        self.audio_played_chunks = 0

        # Counters for rejected RX packets.
        #
        # These help the experiment logger distinguish between missing audio
        # and audio frames that were received but ignored because their metadata
        # did not match the expected format.
        self.audio_rx_rejected_encoding = 0
        self.audio_rx_rejected_channels = 0
        self.audio_rx_rejected_sample_rate = 0
        self.audio_rx_rejected_empty_payload = 0
        self.audio_rx_ignored_wrong_mode = 0

        # Microphone statistics.
        #
        # These are not used for audio processing itself. They are exposed to
        # diagnostics so that later we can document whether the microphone stream
        # was actually active during an experimental run.
        self.last_microphone_rms = 0.0
        self.microphone_blocks_captured = 0
        self.microphone_blocks_silenced_by_gate = 0

        # Last error string. The value "-" means no relevant error was recorded.
        self.audio_error = "-"

        # =========================================================
        # ROS PUBLISHERS AND SUBSCRIBERS
        # =========================================================
        self.audio_tx_pub = self.create_publisher(
            AudioFrame,
            AUDIO_TX_TOPIC,
            10,
        )

        # Diagnostic publisher for the future logger node.
        #
        # The message type is std_msgs/String containing a compact JSON object.
        # This avoids creating a custom diagnostic message at this stage and
        # keeps the experimental instrumentation lightweight.
        self.audio_diagnostics_pub = self.create_publisher(
            String,
            AUDIO_DIAGNOSTICS_TOPIC,
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

        # Timer used to periodically publish audio diagnostics.
        #
        # This timer is intentionally independent from the audio callbacks so
        # that diagnostic publication does not interfere with real-time audio
        # capture or playback.
        diagnostics_period = 1.0 / max(self.diagnostics_rate_hz, 0.1)
        self.diagnostics_timer = self.create_timer(
            diagnostics_period,
            self.publish_audio_diagnostics,
        )

        # =========================================================
        # AUDIO STREAM INITIALIZATION
        # =========================================================
        self.setup_audio_output()
        self.setup_audio_input()

        if SOUNDDEVICE_AVAILABLE:
            self.get_logger().info(
                "rb23_audio_node started with sounddevice available."
            )
        else:
            self.get_logger().warn(
                "sounddevice is not available. "
                f"Detail: {SOUNDDEVICE_IMPORT_ERROR}"
            )

    # =========================================================
    # ROS CALLBACKS
    # =========================================================

    def audio_mode_callback(self, msg: AudioMode) -> None:
        """
        Update the current audio mode.

        The RX playback queue is cleared whenever the mode changes. This avoids
        playing outdated robot audio when the user switches quickly between
        listening, talking, and silence.
        """
        with self.state_lock:
            self.current_audio_mode = int(msg.mode)
            self.last_audio_mode_change_time = time.time()

            # Clear the playback queue to avoid delayed stale audio.
            self.audio_rx_queue.clear()

            if self.current_audio_mode == AudioMode.MODE_SILENCE:
                self.audio_status = "bilateral silence"
            elif self.current_audio_mode == AudioMode.MODE_LISTEN:
                self.audio_status = "listening to robot"
            elif self.current_audio_mode == AudioMode.MODE_TALK:
                self.audio_status = "talking to robot"
            else:
                self.audio_status = f"unknown mode ({self.current_audio_mode})"

        self.get_logger().info(f"Audio mode changed to: {self.audio_status}")

    def audio_rx_callback(self, msg: AudioFrame) -> None:
        """
        Receive one audio frame from the robot and prepare it for local playback.

        Only frames matching the expected metadata are accepted. Rejected frames
        are counted for diagnostics, because this information helps distinguish
        communication problems from format mismatch problems during experiments.
        """
        if msg.encoding != AUDIO_RX_EXPECTED_ENCODING:
            with self.state_lock:
                self.audio_rx_rejected_encoding += 1
            return

        if msg.channels != AUDIO_RX_EXPECTED_CHANNELS:
            with self.state_lock:
                self.audio_rx_rejected_channels += 1
            return

        # In the current GuardROS implementation, RX audio is expected at 16 kHz.
        # If the driver publishes a different sample rate in the future, this
        # conversion can be generalized.
        if int(msg.sample_rate) != AUDIO_RX_EXPECTED_SAMPLE_RATE:
            with self.state_lock:
                self.audio_rx_rejected_sample_rate += 1
            return

        with self.state_lock:
            if self.current_audio_mode != AudioMode.MODE_LISTEN:
                self.audio_rx_ignored_wrong_mode += 1
                return

        try:
            payload = np.array(msg.data, dtype=np.uint8).astype(np.float32)

            if payload.size == 0:
                with self.state_lock:
                    self.audio_rx_rejected_empty_payload += 1
                return

            # Conversion similar to the previously validated Linux client:
            # unsigned 8-bit audio centered at 128 is mapped to a float signal
            # approximately centered around zero.
            mono = (payload - 128.0) / 128.0

            # Remove DC offset.
            mono = mono - float(np.mean(mono))

            # Simple interpolation-based upsampling to 48 kHz.
            if AUDIO_RX_UPSAMPLE_FACTOR > 1 and mono.size >= 2:
                x_old = np.arange(mono.size, dtype=np.float32)
                x_new = np.linspace(
                    0,
                    mono.size - 1,
                    mono.size * AUDIO_RX_UPSAMPLE_FACTOR,
                    dtype=np.float32
                )
                mono = np.interp(x_new, x_old, mono).astype(np.float32)

            # Small gain to improve audibility.
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
    # AUDIO STREAM SETUP
    # =========================================================

    def setup_audio_output(self) -> None:
        """
        Initialize the local audio output stream.

        The output stream is kept open even when the node is not in listen mode.
        The callback simply outputs zeros unless robot audio is available and the
        current mode is MODE_LISTEN.
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

            self.get_logger().error(f"Failed to open audio output stream: {exc}")

    def setup_audio_input(self) -> None:
        """
        Initialize the local audio input stream.

        The input stream is kept open, but microphone frames are only published
        when the current audio mode is MODE_TALK.
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

            self.get_logger().error(f"Failed to open microphone input stream: {exc}")

    # =========================================================
    # SOUNDDEVICE CALLBACKS
    # =========================================================

    def audio_output_callback(self, outdata, frames, time_info, status) -> None:
        """
        Callback called by sounddevice to fill the local speaker buffer.

        This callback must remain lightweight because it is part of the audio
        real-time path. For that reason, it only consumes already-prepared
        buffers from the RX queue and updates simple counters.
        """
        if status:
            with self.state_lock:
                self.audio_error = f"output callback: {status}"

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
        Callback called by sounddevice whenever a new microphone block arrives.

        When the node is in MODE_TALK, the microphone block is converted to the
        expected AudioFrame format and published to /rb23/audio/tx. The driver
        is responsible for encapsulating that ROS audio frame into the RB23 UDP
        protocol.
        """
        if status:
            with self.state_lock:
                self.audio_error = f"input callback: {status}"

        with self.state_lock:
            if self.current_audio_mode != AudioMode.MODE_TALK:
                return

        try:
            mono_f = indata[:, 0].astype(np.float32, copy=True)

            # Remove DC offset.
            mono_f -= float(np.mean(mono_f))

            rms = float(np.sqrt(np.mean(np.square(mono_f))) if mono_f.size else 0.0)

            with self.state_lock:
                self.last_microphone_rms = rms
                self.microphone_blocks_captured += 1

            # Noise gate:
            # if the input signal is below the threshold, send real silence.
            if rms < self.audio_tx_gate_rms:
                mono_i16 = np.zeros(AUDIO_TX_SAMPLES_PER_PACKET, dtype=np.int16)
                with self.state_lock:
                    self.microphone_blocks_silenced_by_gate += 1
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
    # DIAGNOSTICS
    # =========================================================

    def publish_audio_diagnostics(self) -> None:
        """
        Publish a compact JSON diagnostic snapshot of the audio subsystem.

        This diagnostic message is designed for the future experiment logger.
        It exposes internal information that cannot be reconstructed only by
        subscribing to /rb23/audio/rx and /rb23/audio/tx, such as:
        - RX queue occupancy
        - local playback drops
        - local stream initialization status
        - microphone RMS
        - audio mode changes
        - internal audio errors

        The node does not write CSV files directly. File writing will be handled
        by a dedicated logger node so that audio handling remains independent
        from experimental data storage.
        """
        now = time.time()

        with self.state_lock:
            queue_len = len(self.audio_rx_queue)

            diagnostics = {
                "timestamp_unix": now,
                "node_name": self.get_name(),
                "audio_mode": int(self.current_audio_mode),
                "audio_status": self.audio_status,
                "sounddevice_available": bool(SOUNDDEVICE_AVAILABLE),
                "sounddevice_import_error": SOUNDDEVICE_IMPORT_ERROR,
                "audio_output_started": bool(self.audio_output_started),
                "audio_input_started": bool(self.audio_input_started),
                "audio_output_rate_hz": int(AUDIO_OUTPUT_RATE),
                "audio_input_rate_hz": int(AUDIO_TX_SAMPLE_RATE),
                "rx_expected_encoding": AUDIO_RX_EXPECTED_ENCODING,
                "rx_expected_sample_rate": int(AUDIO_RX_EXPECTED_SAMPLE_RATE),
                "rx_expected_channels": int(AUDIO_RX_EXPECTED_CHANNELS),
                "tx_encoding": AUDIO_TX_ENCODING,
                "tx_sample_rate": int(AUDIO_TX_SAMPLE_RATE),
                "tx_channels": int(AUDIO_TX_CHANNELS),
                "tx_samples_per_packet": int(AUDIO_TX_SAMPLES_PER_PACKET),
                "audio_packets_rx": int(self.audio_packets_rx),
                "audio_packets_tx": int(self.audio_packets_tx),
                "audio_packets_rx_dropped": int(self.audio_packets_rx_dropped),
                "audio_played_chunks": int(self.audio_played_chunks),
                "audio_rx_queue_len": int(queue_len),
                "audio_rx_queue_max": int(AUDIO_RX_QUEUE_MAX),
                "audio_rx_rejected_encoding": int(self.audio_rx_rejected_encoding),
                "audio_rx_rejected_channels": int(self.audio_rx_rejected_channels),
                "audio_rx_rejected_sample_rate": int(self.audio_rx_rejected_sample_rate),
                "audio_rx_rejected_empty_payload": int(self.audio_rx_rejected_empty_payload),
                "audio_rx_ignored_wrong_mode": int(self.audio_rx_ignored_wrong_mode),
                "microphone_blocks_captured": int(self.microphone_blocks_captured),
                "microphone_blocks_silenced_by_gate": int(self.microphone_blocks_silenced_by_gate),
                "last_microphone_rms": float(round(self.last_microphone_rms, 3)),
                "audio_tx_gain": float(self.audio_tx_gain),
                "audio_tx_gate_rms": float(self.audio_tx_gate_rms),
                "last_audio_rx_time_unix": float(self.last_audio_rx_time),
                "last_audio_tx_time_unix": float(self.last_audio_tx_time),
                "last_audio_mode_change_time_unix": float(self.last_audio_mode_change_time),
                "seconds_since_last_audio_rx": (
                    None if self.last_audio_rx_time <= 0.0 else round(now - self.last_audio_rx_time, 6)
                ),
                "seconds_since_last_audio_tx": (
                    None if self.last_audio_tx_time <= 0.0 else round(now - self.last_audio_tx_time, 6)
                ),
                "seconds_since_mode_change": (
                    None if self.last_audio_mode_change_time <= 0.0 else round(now - self.last_audio_mode_change_time, 6)
                ),
                "uptime_sec": float(round(now - self.node_start_time, 6)),
                "audio_error": self.audio_error,
            }

            self.last_diagnostics_time = now

        msg = String()
        msg.data = json.dumps(diagnostics, separators=(",", ":"))
        self.audio_diagnostics_pub.publish(msg)

    # =========================================================
    # SHUTDOWN
    # =========================================================

    def close(self) -> None:
        """
        Close local audio streams in an organized way.
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
