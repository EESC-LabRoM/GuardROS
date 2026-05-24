#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rb23_video_viewer_node.py

ROS 2 node responsible for displaying the compressed video stream (JPEG)
published by the RollerBot 23 driver.

Desired architecture:
    RB23 -> rb23_driver_node -> /rb23/camera/image/compressed -> rb23_video_viewer_node

Main responsibilities:
1. Subscribe to the compressed image topic published by the driver.
2. Receive JPEG bytes through ROS 2 messages.
3. Decode the JPEG stream using OpenCV.
4. Display the decoded image in a local window.
5. Publish lightweight diagnostic information for experimental logging.

Why is this node separated from the driver?
------------------------------------------
In ROS-based systems, separating responsibilities is a good architectural
practice:
- the driver handles communication with the robot;
- this node handles only visual presentation.

This separation improves maintainability, debugging, and future extensions,
such as:
- video recording;
- OpenCV-based processing;
- object detection;
- graphical dashboards;
- experimental logging for scientific evaluation.

Visual improvement implemented:
-------------------------------
This node uses a letterbox/pillarbox strategy:
- the image is resized while preserving its original aspect ratio;
- the resized image is centered inside a fixed display area;
- the remaining area is filled with black pixels.

This avoids:
- horizontal distortion;
- vertical distortion;
- overly large or inconvenient OpenCV windows.

Diagnostic topic:
-----------------
This node publishes diagnostic snapshots to:
- /rb23/diagnostics/video -> std_msgs/msg/String containing JSON data

The diagnostic topic is intended to be consumed by a future experiment logger.
The viewer itself does not write CSV files. It only exposes internal runtime
information as a ROS topic so that a dedicated logger node can store the data
in a clean and centralized way.
"""

import json
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


# ============================================================
# GENERAL CONFIGURATION
# ============================================================

# Name of the OpenCV window shown on the screen.
WINDOW_NAME = "RB23 Camera Viewer"

# ROS topic where the driver publishes the JPEG video stream.
DEFAULT_IMAGE_TOPIC = "/rb23/camera/image/compressed"

# ROS topic used to publish video diagnostics for the future experiment logger.
VIDEO_DIAGNOSTICS_TOPIC = "/rb23/diagnostics/video"

# Display timer period.
# 0.03 s corresponds to approximately 33 Hz of visual refresh.
DISPLAY_PERIOD = 0.03

# Diagnostic publication period.
# A low-rate diagnostic topic is enough for logging counters, timestamps,
# frame size, and decoding status without overloading the ROS graph.
DIAGNOSTICS_PERIOD = 1.0

# Fixed display area.
# 720x405 is a comfortable 16:9 window size for this application.
WINDOW_WIDTH = 720
WINDOW_HEIGHT = 405


# ============================================================
# IMAGE HELPER FUNCTIONS
# ============================================================

def letterbox_frame(frame: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    """
    Resize an image while preserving its aspect ratio and place it centered
    inside a fixed-size black canvas.

    This prevents image distortion and keeps the video presentation visually
    organized.

    Strategy:
    1. Measure the original image size.
    2. Compute the largest scale factor that allows the image to fit inside
       the desired display area.
    3. Resize the image while preserving its aspect ratio.
    4. Create a fixed-size black canvas.
    5. Copy the resized image to the center of the canvas.

    The result is similar to a standard video player behavior:
    - if horizontal space is missing, side bars appear;
    - if vertical space is missing, top/bottom bars appear.

    Parameters
    ----------
    frame:
        Original OpenCV image in BGR format.

    target_width:
        Desired final display width.

    target_height:
        Desired final display height.

    Returns
    -------
    canvas:
        Final image ready to be displayed by OpenCV.
    """
    if frame is None:
        return np.zeros((target_height, target_width, 3), dtype=np.uint8)

    original_height, original_width = frame.shape[:2]

    # Simple protection against anomalous or corrupted frames.
    if original_width <= 0 or original_height <= 0:
        return np.zeros((target_height, target_width, 3), dtype=np.uint8)

    # Largest scale factor that allows the image to fit inside the fixed area.
    scale = min(target_width / original_width, target_height / original_height)

    # New image size, preserving the original aspect ratio.
    new_width = max(1, int(original_width * scale))
    new_height = max(1, int(original_height * scale))

    # INTER_AREA usually works well when reducing image size.
    resized = cv2.resize(
        frame,
        (new_width, new_height),
        interpolation=cv2.INTER_AREA,
    )

    # Fixed black canvas where the resized image will be centered.
    canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)

    # Compute offsets for centering.
    x_offset = (target_width - new_width) // 2
    y_offset = (target_height - new_height) // 2

    # Copy the resized image into the center of the canvas.
    canvas[
        y_offset:y_offset + new_height,
        x_offset:x_offset + new_width
    ] = resized

    return canvas


# ============================================================
# MAIN NODE CLASS
# ============================================================

class RB23VideoViewerNode(Node):
    """
    ROS 2 node used to display the RB23 video stream.

    This node:
    - subscribes to a sensor_msgs/msg/CompressedImage topic;
    - receives JPEG bytes;
    - decodes the latest received frame;
    - displays the decoded frame in an OpenCV window;
    - publishes lightweight runtime diagnostics as JSON.

    Note
    ----
    The image is displayed from a timer instead of directly inside the ROS
    callback. This design keeps the subscriber callback focused on receiving
    data and makes the GUI update loop more stable.
    """

    def __init__(self) -> None:
        super().__init__("rb23_video_viewer_node")

        # --------------------------------------------------------
        # ROS PARAMETERS
        # --------------------------------------------------------
        # This parameter allows changing the video topic in the future without
        # editing the source code.
        self.declare_parameter("image_topic", DEFAULT_IMAGE_TOPIC)
        self.image_topic = str(self.get_parameter("image_topic").value)

        # --------------------------------------------------------
        # INTERNAL STATE
        # --------------------------------------------------------
        # A lock is used because the subscriber callback and the display timer
        # may access the same frame data at different moments.
        self.frame_lock = threading.Lock()

        # Most recent decoded frame.
        self.latest_frame: Optional[np.ndarray] = None

        # Counters useful for debugging and experimental evaluation.
        self.frames_received = 0
        self.frames_decoded = 0
        self.decode_failures = 0
        self.frames_displayed = 0
        self.empty_display_cycles = 0

        # Timestamp-related state variables.
        # These values allow the future logger to estimate frame intervals,
        # video availability, video startup delay, and periods without frames.
        self.node_start_time = time.time()
        self.first_frame_rx_time = 0.0
        self.last_frame_rx_time = 0.0
        self.last_frame_decoded_time = 0.0
        self.last_frame_displayed_time = 0.0

        # Last received/decoded frame metadata.
        self.last_frame_bytes = 0
        self.last_frame_width = 0
        self.last_frame_height = 0
        self.last_decode_error = "-"

        # --------------------------------------------------------
        # ROS PUBLISHERS
        # --------------------------------------------------------
        # Diagnostic publisher used by the future experiment logger.
        # The data is encoded as JSON inside std_msgs/String to avoid creating
        # a custom diagnostic message before the experimental protocol is final.
        self.video_diagnostics_pub = self.create_publisher(
            String,
            VIDEO_DIAGNOSTICS_TOPIC,
            10,
        )

        # --------------------------------------------------------
        # ROS SUBSCRIBER
        # --------------------------------------------------------
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10,
        )

        # --------------------------------------------------------
        # TIMERS
        # --------------------------------------------------------
        # Timer responsible for updating the OpenCV window.
        self.display_timer = self.create_timer(
            DISPLAY_PERIOD,
            self.display_timer_callback,
        )

        # Timer responsible for publishing diagnostic snapshots.
        # This keeps diagnostics independent from the image callback rate.
        self.diagnostics_timer = self.create_timer(
            DIAGNOSTICS_PERIOD,
            self.publish_video_diagnostics,
        )

        # --------------------------------------------------------
        # OPENCV WINDOW
        # --------------------------------------------------------
        # The window is created only once and resized to a comfortable initial
        # size for the desktop environment.
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

        self.get_logger().info(
            f"RB23 video viewer started | topic={self.image_topic} | "
            f"window={WINDOW_WIDTH}x{WINDOW_HEIGHT} | "
            f"diagnostics_topic={VIDEO_DIAGNOSTICS_TOPIC}"
        )

    # =========================================================
    # IMAGE TOPIC CALLBACK
    # =========================================================

    def image_callback(self, msg: CompressedImage) -> None:
        """
        Callback executed whenever a new JPEG message arrives.

        Steps:
        1. Read the JPEG bytes from msg.data.
        2. Convert the byte buffer to a NumPy vector.
        3. Decode the JPEG into an OpenCV BGR image.
        4. Store the most recent decoded frame.
        5. Update counters and timestamps used by the experiment logger.
        """
        now = time.time()

        self.frames_received += 1
        self.last_frame_rx_time = now
        self.last_frame_bytes = len(msg.data)

        if self.first_frame_rx_time <= 0.0:
            self.first_frame_rx_time = now

        try:
            # Convert JPEG bytes to a NumPy uint8 vector.
            np_buffer = np.frombuffer(msg.data, dtype=np.uint8)

            # Decode the JPEG into a color BGR OpenCV image.
            frame = cv2.imdecode(np_buffer, cv2.IMREAD_COLOR)

            # If OpenCV cannot decode the frame, count the failure and exit.
            if frame is None:
                self.decode_failures += 1
                self.last_decode_error = "cv2.imdecode returned None"
                return

            frame_height, frame_width = frame.shape[:2]

            # Store the most recent frame in a thread-safe way.
            with self.frame_lock:
                self.latest_frame = frame

            self.frames_decoded += 1
            self.last_frame_decoded_time = time.time()
            self.last_frame_width = int(frame_width)
            self.last_frame_height = int(frame_height)
            self.last_decode_error = "-"

        except Exception as exc:
            self.decode_failures += 1
            self.last_decode_error = str(exc)
            self.get_logger().warn(f"Failed to decode JPEG frame: {exc}")

    # =========================================================
    # DISPLAY TIMER
    # =========================================================

    def display_timer_callback(self) -> None:
        """
        Timer responsible for displaying the latest frame in the OpenCV window.

        This separation is useful because:
        - the ROS callback is responsible for receiving data;
        - the timer is responsible for updating the graphical interface.
        """
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()

        # If no frame has arrived yet, keep the OpenCV window responsive.
        if frame is None:
            self.empty_display_cycles += 1
            cv2.waitKey(1)
            return

        # Apply the letterbox presentation:
        # the image is fitted into a fixed canvas without distortion.
        display_frame = letterbox_frame(
            frame,
            WINDOW_WIDTH,
            WINDOW_HEIGHT,
        )

        # Show the final display frame.
        cv2.imshow(WINDOW_NAME, display_frame)
        self.frames_displayed += 1
        self.last_frame_displayed_time = time.time()

        # Keep the OpenCV window responsive.
        key = cv2.waitKey(1) & 0xFF

        # Optional shutdown shortcut: q or ESC closes the ROS node.
        if key in (27, ord("q"), ord("Q")):
            self.get_logger().info("Closing video viewer by keyboard command.")
            rclpy.shutdown()

    # =========================================================
    # DIAGNOSTIC PUBLISHING
    # =========================================================

    def publish_video_diagnostics(self) -> None:
        """
        Publish a lightweight diagnostic snapshot for the video viewer.

        The future experiment logger will subscribe to this topic and save the
        JSON payload to disk. This node does not write CSV files directly,
        preserving a clean separation between visualization and data logging.

        The diagnostic fields are useful for estimating:
        - video frame rate;
        - decode failure rate;
        - time to first frame;
        - time since the last received/decoded/displayed frame;
        - frame size and resolution;
        - whether the visual pipeline is active.
        """
        now = time.time()

        time_to_first_frame = None
        if self.first_frame_rx_time > 0.0:
            time_to_first_frame = self.first_frame_rx_time - self.node_start_time

        diagnostics = {
            "timestamp": now,
            "node_name": "rb23_video_viewer_node",
            "image_topic": self.image_topic,
            "window_name": WINDOW_NAME,
            "window_width": WINDOW_WIDTH,
            "window_height": WINDOW_HEIGHT,
            "display_period_sec": DISPLAY_PERIOD,
            "frames_received": self.frames_received,
            "frames_decoded": self.frames_decoded,
            "decode_failures": self.decode_failures,
            "frames_displayed": self.frames_displayed,
            "empty_display_cycles": self.empty_display_cycles,
            "last_frame_bytes": self.last_frame_bytes,
            "last_frame_width": self.last_frame_width,
            "last_frame_height": self.last_frame_height,
            "node_uptime_sec": now - self.node_start_time,
            "time_to_first_frame_sec": time_to_first_frame,
            "age_last_frame_rx_sec": None if self.last_frame_rx_time <= 0.0 else now - self.last_frame_rx_time,
            "age_last_frame_decoded_sec": None if self.last_frame_decoded_time <= 0.0 else now - self.last_frame_decoded_time,
            "age_last_frame_displayed_sec": None if self.last_frame_displayed_time <= 0.0 else now - self.last_frame_displayed_time,
            "last_decode_error": self.last_decode_error,
        }

        msg = String()
        msg.data = json.dumps(diagnostics, separators=(",", ":"))
        self.video_diagnostics_pub.publish(msg)

    # =========================================================
    # SHUTDOWN
    # =========================================================

    def close(self) -> None:
        """
        Close the OpenCV window in an organized way.
        """
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


# ============================================================
# MAIN
# ============================================================

def main(args=None) -> None:
    """
    ROS 2 entry point for this node.
    """
    rclpy.init(args=args)
    node = RB23VideoViewerNode()

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
