#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
keyboard_teleop_node.py

ROS 2 keyboard teleoperation node for GuardROS / RB23.

Purpose:
--------
Read the keyboard in real time and publish ROS 2 commands that are consumed
by other GuardROS nodes, mainly:
- rb23_driver_node
- rb23_audio_node

Current architecture:
---------------------
keyboard -> ROS topics -> driver / audio -> RB23

Published topics:
-----------------
- /rb23/cmd_vel                -> geometry_msgs/msg/Twist
- /rb23/force_rpi              -> std_msgs/msg/Bool
- /rb23/cam_stable             -> std_msgs/msg/Bool
- /rb23/cam_angle              -> std_msgs/msg/Float32
- /rb23/audio_mode             -> guardros_msgs/msg/AudioMode
- /rb23/diagnostics/teleop     -> std_msgs/msg/String, JSON payload

Main features:
--------------
1. Accepts WASD keys and arrow keys for robot motion.
2. Publishes Twist messages on /rb23/cmd_vel.
3. Allows control of additional RB23 functions:
   - force_rpi
   - camera stabilization
   - camera angle offset
4. Allows selection of the audio mode:
   - bilateral silence
   - listen to the robot
   - talk to the robot
5. Shows the key mapping to the user in the terminal.
6. Works in a "video-game car" style:
   - while a motion key is detected, the command remains active
   - when the key is released, the node returns to zero velocity
7. Uses curses for real-time keyboard reading in a Linux terminal.
8. Publishes a lightweight diagnostic topic intended for experimental logging.
9. The code is intentionally verbose and heavily commented to support
   publication, maintenance, and future reuse by other researchers.

Important note:
---------------
This node does not communicate directly with the robot.
It only publishes ROS topics.
The node that communicates with the RB23 is rb23_driver_node.
"""

import curses
import json
import time
from typing import Dict, Any

import rclpy
from geometry_msgs.msg import Twist
from guardros_msgs.msg import AudioMode
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


# ============================================================
# ROS TOPIC NAMES
# ============================================================

# Motion command topic consumed by rb23_driver_node.
CMD_VEL_TOPIC = "/rb23/cmd_vel"

# Auxiliary command topics consumed by rb23_driver_node.
FORCE_RPI_TOPIC = "/rb23/force_rpi"
CAM_STABLE_TOPIC = "/rb23/cam_stable"
CAM_ANGLE_TOPIC = "/rb23/cam_angle"

# Audio mode topic consumed by rb23_audio_node.
AUDIO_MODE_TOPIC = "/rb23/audio_mode"

# Diagnostic topic intended for the future experiment logger.
# This topic does not change the teleoperation behavior. It only exposes
# internal teleop state and counters in a machine-readable JSON message.
TELEOP_DIAGNOSTICS_TOPIC = "/rb23/diagnostics/teleop"


class KeyboardTeleopNode(Node):
    """
    ROS 2 node responsible for:
    - capturing keyboard input from the terminal
    - converting keys into ROS commands
    - publishing velocity commands on /rb23/cmd_vel
    - publishing auxiliary RB23 commands on dedicated topics
    - publishing the audio mode on /rb23/audio_mode
    - publishing diagnostic snapshots for experimental logging
    """

    def __init__(self) -> None:
        super().__init__("rb23_keyboard_teleop_node")

        # =========================================================
        # ROS PARAMETERS
        # =========================================================
        # These parameters allow the teleoperation behavior to be tuned
        # without editing the source code.
        self.declare_parameter("linear_speed", 0.30)
        self.declare_parameter("angular_speed", 1.00)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("key_hold_timeout", 0.18)

        # Camera control parameters.
        self.declare_parameter("cam_angle_step", 0.10)
        self.declare_parameter("cam_angle_min", -1.50)
        self.declare_parameter("cam_angle_max", 1.50)

        # Diagnostic publication rate.
        # A low rate is enough because high-frequency command timing can be
        # obtained directly by the future logger from /rb23/cmd_vel.
        # The diagnostic topic is meant to expose internal counters and state.
        self.declare_parameter("diagnostics_rate_hz", 2.0)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.key_hold_timeout = float(self.get_parameter("key_hold_timeout").value)

        self.cam_angle_step = float(self.get_parameter("cam_angle_step").value)
        self.cam_angle_min = float(self.get_parameter("cam_angle_min").value)
        self.cam_angle_max = float(self.get_parameter("cam_angle_max").value)
        self.diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)

        # =========================================================
        # ROS PUBLISHERS
        # =========================================================
        # Standard ROS mobile-robot velocity publisher.
        # Adopted convention:
        #   linear.x  -> linear motion command (forward / backward)
        #   angular.z -> yaw-rate command (turning)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10
        )

        # Auxiliary RB23 command publishers.
        self.force_rpi_pub = self.create_publisher(
            Bool,
            FORCE_RPI_TOPIC,
            10
        )

        self.cam_stable_pub = self.create_publisher(
            Bool,
            CAM_STABLE_TOPIC,
            10
        )

        self.cam_angle_pub = self.create_publisher(
            Float32,
            CAM_ANGLE_TOPIC,
            10
        )

        # Audio mode publisher consumed by rb23_audio_node.
        self.audio_mode_pub = self.create_publisher(
            AudioMode,
            AUDIO_MODE_TOPIC,
            10
        )

        # Diagnostic publisher for the future logger node.
        # The payload is JSON inside std_msgs/String to avoid creating a new
        # custom message during the first experimental-instrumentation phase.
        self.teleop_diag_pub = self.create_publisher(
            String,
            TELEOP_DIAGNOSTICS_TOPIC,
            10
        )

        # =========================================================
        # INTERNAL MOTION STATE
        # =========================================================
        # Timestamp of the last detected motion key.
        # This is used to implement the "stop when the key is released" behavior.
        self.last_motion_key_time = 0.0

        # Current desired velocities.
        self.current_linear = 0.0
        self.current_angular = 0.0

        # =========================================================
        # INTERNAL AUXILIARY-COMMAND STATE
        # =========================================================
        # Initial values consistent with the base command state used by the driver.
        self.force_rpi = False
        self.cam_stable = True
        self.cam_angle = 0.0

        # =========================================================
        # INTERNAL AUDIO STATE
        # =========================================================
        # The initial audio mode is bilateral silence.
        self.audio_mode = AudioMode.MODE_SILENCE

        # =========================================================
        # EXPERIMENTAL DIAGNOSTIC STATE
        # =========================================================
        # These counters and timestamps are not required for teleoperation itself.
        # They are exposed only to support reproducible experiments and later
        # statistical analysis for the RA-P article.
        self.node_start_time = time.time()
        self.cmd_vel_publish_count = 0
        self.force_rpi_publish_count = 0
        self.cam_stable_publish_count = 0
        self.cam_angle_publish_count = 0
        self.audio_mode_publish_count = 0
        self.diagnostics_publish_count = 0

        self.key_event_count = 0
        self.motion_key_event_count = 0
        self.aux_key_event_count = 0
        self.audio_key_event_count = 0
        self.stop_event_count = 0
        self.timeout_stop_count = 0

        self.last_cmd_vel_publish_time = 0.0
        self.last_aux_publish_time = 0.0
        self.last_audio_mode_publish_time = 0.0
        self.last_diagnostics_publish_time = 0.0
        self.last_key_time = 0.0

        # =========================================================
        # INFORMATION SHOWN ON SCREEN
        # =========================================================
        self.last_key_name = "-"
        self.last_action = "waiting for command"

        # Flag used to control the main loop exit.
        self.running = True

        # =========================================================
        # PUBLICATION TIMERS
        # =========================================================
        # Even when the robot is stopped, continuously publishing zero velocity
        # is useful to keep a consistent command stream.
        publish_period = 1.0 / self.publish_rate_hz
        self.publish_timer = self.create_timer(
            publish_period,
            self.publish_cmd_vel
        )

        # The diagnostic timer publishes a compact state snapshot. The future
        # experiment logger will subscribe to this topic and save it to CSV.
        diagnostics_period = 1.0 / max(self.diagnostics_rate_hz, 0.1)
        self.diagnostics_timer = self.create_timer(
            diagnostics_period,
            self.publish_teleop_diagnostics
        )

        self.get_logger().info("Keyboard teleoperation node initialized successfully.")

        # Publish the initial audio mode as soon as the node starts.
        self.publish_audio_mode()

    # =========================================================
    # GENERAL AUXILIARY FUNCTIONS
    # =========================================================

    def clamp(self, value: float, low: float, high: float) -> float:
        """
        Limit a value to the interval [low, high].
        """
        return max(low, min(value, high))

    def audio_mode_to_text(self, mode: int) -> str:
        """
        Convert the numeric audio mode code into human-readable text.
        """
        if mode == AudioMode.MODE_SILENCE:
            return "bilateral silence"
        if mode == AudioMode.MODE_LISTEN:
            return "listen to robot"
        if mode == AudioMode.MODE_TALK:
            return "talk to robot"
        return f"unknown ({mode})"

    # =========================================================
    # MOTION AUXILIARY FUNCTIONS
    # =========================================================

    def set_motion(
        self,
        linear: float,
        angular: float,
        key_name: str,
        action: str
    ) -> None:
        """
        Update the current motion command state.

        Parameters:
        -----------
        linear:
            Desired linear velocity along the x axis.

        angular:
            Desired angular velocity around the z axis.

        key_name:
            Human-readable name of the pressed key.

        action:
            Textual description of the action.
        """
        self.current_linear = linear
        self.current_angular = angular
        self.last_key_name = key_name
        self.last_action = action
        self.last_motion_key_time = time.time()
        self.last_key_time = self.last_motion_key_time
        self.motion_key_event_count += 1

    def stop_motion(
        self,
        key_name: str = "-",
        action: str = "stopped",
        due_to_timeout: bool = False
    ) -> None:
        """
        Immediately set the motion command to zero.

        The due_to_timeout flag allows us to separate deliberate stop commands
        from automatic stops caused by key-release timeout. This distinction is
        useful during experimental analysis because timeout stops are part of
        the teleoperation behavior rather than explicit user commands.
        """
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_key_name = key_name
        self.last_action = action
        self.stop_event_count += 1

        if due_to_timeout:
            self.timeout_stop_count += 1

    def update_key_timeout(self) -> None:
        """
        Implement the "held key" behavior.

        If more time than the configured limit passes without detecting a new
        motion key, the command is automatically set to zero.
        """
        if self.last_motion_key_time <= 0.0:
            return

        elapsed = time.time() - self.last_motion_key_time

        if elapsed > self.key_hold_timeout:
            # Only count a timeout stop when the command was not already zero.
            was_moving = (abs(self.current_linear) > 0.0) or (abs(self.current_angular) > 0.0)
            self.stop_motion(
                action="stopped (key released)",
                due_to_timeout=was_moving
            )

    def publish_cmd_vel(self) -> None:
        """
        Periodically publish a Twist message on /rb23/cmd_vel.
        """
        # Before publishing, check whether the command expired.
        self.update_key_timeout()

        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular

        self.cmd_vel_pub.publish(msg)

        self.cmd_vel_publish_count += 1
        self.last_cmd_vel_publish_time = time.time()

    # =========================================================
    # AUXILIARY COMMAND PUBLICATION FUNCTIONS
    # =========================================================

    def publish_force_rpi(self) -> None:
        """
        Publish the current force_rpi state.
        """
        msg = Bool()
        msg.data = self.force_rpi
        self.force_rpi_pub.publish(msg)

        self.force_rpi_publish_count += 1
        self.last_aux_publish_time = time.time()

    def publish_cam_stable(self) -> None:
        """
        Publish the current camera-stabilization state.
        """
        msg = Bool()
        msg.data = self.cam_stable
        self.cam_stable_pub.publish(msg)

        self.cam_stable_publish_count += 1
        self.last_aux_publish_time = time.time()

    def publish_cam_angle(self) -> None:
        """
        Publish the current camera angle offset.
        """
        msg = Float32()
        msg.data = float(self.cam_angle)
        self.cam_angle_pub.publish(msg)

        self.cam_angle_publish_count += 1
        self.last_aux_publish_time = time.time()

    def publish_audio_mode(self) -> None:
        """
        Publish the current audio mode.
        """
        msg = AudioMode()
        msg.mode = int(self.audio_mode)
        self.audio_mode_pub.publish(msg)

        self.audio_mode_publish_count += 1
        self.last_audio_mode_publish_time = time.time()

    def set_audio_mode(self, mode: int, key_name: str) -> None:
        """
        Update and publish the audio mode.
        """
        self.audio_mode = mode
        self.last_key_name = key_name
        self.last_action = f"audio -> {self.audio_mode_to_text(self.audio_mode)}"
        self.audio_key_event_count += 1
        self.last_key_time = time.time()
        self.publish_audio_mode()

    # =========================================================
    # DIAGNOSTIC PUBLICATION
    # =========================================================

    def build_teleop_diagnostics_dict(self) -> Dict[str, Any]:
        """
        Build a dictionary containing the current diagnostic state.

        This diagnostic snapshot is designed for logging, not for control.
        It intentionally avoids large payloads and contains only scalar values,
        counters, timestamps, and the current teleoperation state. The future
        experiment logger can subscribe to /rb23/diagnostics/teleop and write
        these JSON fields to a CSV file.
        """
        now = time.time()
        uptime = now - self.node_start_time

        return {
            "timestamp": now,
            "node_name": self.get_name(),
            "uptime_sec": uptime,
            "publish_rate_hz_configured": self.publish_rate_hz,
            "key_hold_timeout_sec": self.key_hold_timeout,
            "linear_speed_configured": self.linear_speed,
            "angular_speed_configured": self.angular_speed,
            "current_linear": self.current_linear,
            "current_angular": self.current_angular,
            "force_rpi": self.force_rpi,
            "cam_stable": self.cam_stable,
            "cam_angle": self.cam_angle,
            "audio_mode": int(self.audio_mode),
            "audio_mode_text": self.audio_mode_to_text(int(self.audio_mode)),
            "last_key_name": self.last_key_name,
            "last_action": self.last_action,
            "cmd_vel_publish_count": self.cmd_vel_publish_count,
            "force_rpi_publish_count": self.force_rpi_publish_count,
            "cam_stable_publish_count": self.cam_stable_publish_count,
            "cam_angle_publish_count": self.cam_angle_publish_count,
            "audio_mode_publish_count": self.audio_mode_publish_count,
            "diagnostics_publish_count": self.diagnostics_publish_count,
            "key_event_count": self.key_event_count,
            "motion_key_event_count": self.motion_key_event_count,
            "aux_key_event_count": self.aux_key_event_count,
            "audio_key_event_count": self.audio_key_event_count,
            "stop_event_count": self.stop_event_count,
            "timeout_stop_count": self.timeout_stop_count,
            "last_cmd_vel_publish_time": self.last_cmd_vel_publish_time,
            "last_aux_publish_time": self.last_aux_publish_time,
            "last_audio_mode_publish_time": self.last_audio_mode_publish_time,
            "last_key_time": self.last_key_time,
        }

    def publish_teleop_diagnostics(self) -> None:
        """
        Publish a JSON diagnostic snapshot for experimental logging.

        The future logger node will be responsible for writing this information
        to disk. Keeping file writing outside this teleoperation node preserves
        the node's original responsibility: interactive command generation.
        """
        self.diagnostics_publish_count += 1
        self.last_diagnostics_publish_time = time.time()

        msg = String()
        msg.data = json.dumps(
            self.build_teleop_diagnostics_dict(),
            separators=(",", ":")
        )
        self.teleop_diag_pub.publish(msg)

    # =========================================================
    # KEY HANDLING
    # =========================================================

    def handle_key(self, ch: int) -> None:
        """
        Convert keyboard input into motion and auxiliary commands.

        Mapping:
        --------
        Motion:
        - w or arrow up      -> forward
        - s or arrow down    -> backward
        - a or arrow left    -> turn left
        - d or arrow right   -> turn right
        - x or space         -> immediate stop

        Auxiliary commands:
        - f -> toggle force_rpi
        - c -> toggle cam_stable
        - u -> increase cam_angle
        - j -> decrease cam_angle

        Audio commands:
        - 1 -> bilateral silence
        - 2 -> listen to the robot
        - 3 -> talk to the robot

        Shutdown:
        - ESC -> exit
        """
        # Ignore special curses codes that are not useful commands.
        if ch in (curses.KEY_MOUSE, curses.KEY_RESIZE):
            return

        self.key_event_count += 1
        self.last_key_time = time.time()

        # ---------------------------------------------------------
        # MOTION
        # ---------------------------------------------------------

        # Forward
        if ch in (ord("w"), ord("W"), curses.KEY_UP):
            self.set_motion(
                linear=+self.linear_speed,
                angular=0.0,
                key_name="w / ↑",
                action="forward"
            )

        # Backward
        elif ch in (ord("s"), ord("S"), curses.KEY_DOWN):
            self.set_motion(
                linear=-self.linear_speed,
                angular=0.0,
                key_name="s / ↓",
                action="backward"
            )

        # Turn left
        elif ch in (ord("a"), ord("A"), curses.KEY_LEFT):
            self.set_motion(
                linear=0.0,
                angular=+self.angular_speed,
                key_name="a / ←",
                action="turn left"
            )

        # Turn right
        elif ch in (ord("d"), ord("D"), curses.KEY_RIGHT):
            self.set_motion(
                linear=0.0,
                angular=-self.angular_speed,
                key_name="d / →",
                action="turn right"
            )

        # Immediate stop
        elif ch in (ord("x"), ord("X"), ord(" ")):
            self.stop_motion(
                key_name="x / space",
                action="immediate stop"
            )

        # ---------------------------------------------------------
        # RB23 AUXILIARY COMMANDS
        # ---------------------------------------------------------

        # Toggle force_rpi
        elif ch in (ord("f"), ord("F")):
            self.force_rpi = not self.force_rpi
            self.last_key_name = "f"
            self.last_action = f"force_rpi -> {self.force_rpi}"
            self.aux_key_event_count += 1
            self.publish_force_rpi()

        # Toggle camera stabilization
        elif ch in (ord("c"), ord("C")):
            self.cam_stable = not self.cam_stable
            self.last_key_name = "c"
            self.last_action = f"cam_stable -> {self.cam_stable}"
            self.aux_key_event_count += 1
            self.publish_cam_stable()

        # Increase camera angle
        elif ch in (ord("u"), ord("U")):
            self.cam_angle = self.clamp(
                self.cam_angle + self.cam_angle_step,
                self.cam_angle_min,
                self.cam_angle_max
            )
            self.last_key_name = "u"
            self.last_action = f"cam_angle -> {self.cam_angle:.2f}"
            self.aux_key_event_count += 1
            self.publish_cam_angle()

        # Decrease camera angle
        elif ch in (ord("j"), ord("J")):
            self.cam_angle = self.clamp(
                self.cam_angle - self.cam_angle_step,
                self.cam_angle_min,
                self.cam_angle_max
            )
            self.last_key_name = "j"
            self.last_action = f"cam_angle -> {self.cam_angle:.2f}"
            self.aux_key_event_count += 1
            self.publish_cam_angle()

        # ---------------------------------------------------------
        # AUDIO COMMANDS
        # ---------------------------------------------------------

        # Bilateral silence
        elif ch == ord("1"):
            self.set_audio_mode(AudioMode.MODE_SILENCE, "1")

        # Listen to the robot
        elif ch == ord("2"):
            self.set_audio_mode(AudioMode.MODE_LISTEN, "2")

        # Talk to the robot
        elif ch == ord("3"):
            self.set_audio_mode(AudioMode.MODE_TALK, "3")

        # ---------------------------------------------------------
        # SHUTDOWN
        # ---------------------------------------------------------

        elif ch == 27:
            self.last_key_name = "ESC"
            self.last_action = "shutting down"
            self.running = False

    # =========================================================
    # ROBUST ARROW-KEY READING
    # =========================================================

    def get_next_key(self, stdscr) -> int:
        """
        Read one terminal key and attempt to correctly interpret arrow keys.

        In many Linux terminals, arrow keys arrive as escape sequences starting
        with ESC (27). This function explicitly handles these sequences.
        """
        ch = stdscr.getch()

        # If this is not an ESC sequence, return the key directly.
        if ch != 27:
            return ch

        # Short delay to allow the remaining bytes of the escape sequence to arrive.
        time.sleep(0.002)

        next1 = stdscr.getch()
        if next1 == -1:
            return 27

        if next1 != 91:
            return 27

        next2 = stdscr.getch()
        if next2 == -1:
            return 27

        if next2 == 65:
            return curses.KEY_UP
        if next2 == 66:
            return curses.KEY_DOWN
        if next2 == 67:
            return curses.KEY_RIGHT
        if next2 == 68:
            return curses.KEY_LEFT

        return 27

    # =========================================================
    # TEXT INTERFACE
    # =========================================================

    def render_screen(self, stdscr) -> None:
        """
        Draw the text interface in the terminal.
        """
        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()

        lines = [
            "================ GUARDROS - KEYBOARD TELEOP ================",
            "",
            "Motion:",
            "  w or ↑  : forward",
            "  s or ↓  : backward",
            "  a or ←  : turn left",
            "  d or →  : turn right",
            "  x/space : immediate stop",
            "",
            "RB23 auxiliary commands:",
            "  f       : toggle force_rpi",
            "  c       : toggle camera stabilization",
            "  u       : increase camera angle",
            "  j       : decrease camera angle",
            "",
            "Audio:",
            "  1       : bilateral silence",
            "  2       : listen to robot",
            "  3       : talk to robot",
            "",
            "Shutdown:",
            "  ESC     : exit",
            "  Ctrl+C  : exit",
            "",
            f"Configured linear speed       : {self.linear_speed:.3f}",
            f"Configured angular speed      : {self.angular_speed:.3f}",
            f"Key timeout                   : {self.key_hold_timeout:.3f} s",
            f"Camera angle step             : {self.cam_angle_step:.2f}",
            f"Diagnostics topic             : {TELEOP_DIAGNOSTICS_TOPIC}",
            "",
            f"Last key                      : {self.last_key_name}",
            f"Last action                   : {self.last_action}",
            "",
            f"current linear.x              : {self.current_linear:.3f}",
            f"current angular.z             : {self.current_angular:.3f}",
            f"current force_rpi             : {self.force_rpi}",
            f"current cam_stable            : {self.cam_stable}",
            f"current cam_angle             : {self.cam_angle:.2f}",
            f"current audio_mode            : {self.audio_mode_to_text(self.audio_mode)}",
            "",
            f"cmd_vel publications          : {self.cmd_vel_publish_count}",
            f"key events                    : {self.key_event_count}",
            f"timeout stops                 : {self.timeout_stop_count}",
            "",
            f"Publishing on {CMD_VEL_TOPIC} ...",
            f"Publishing on {AUDIO_MODE_TOPIC} ...",
            f"Publishing on {TELEOP_DIAGNOSTICS_TOPIC} ...",
        ]

        for i, line in enumerate(lines):
            if i >= max_y - 1:
                break

            try:
                stdscr.addnstr(i, 0, line, max_x - 1)
            except curses.error:
                # In small terminals, some lines may not fit.
                pass

        try:
            stdscr.refresh()
        except curses.error:
            pass

    # =========================================================
    # MAIN CURSES LOOP
    # =========================================================

    def curses_loop(self, stdscr) -> None:
        """
        Main loop of the keyboard interface.
        """
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)
        curses.noecho()
        curses.cbreak()

        last_draw = 0.0
        draw_period = 0.05

        while self.running and rclpy.ok():
            try:
                # Process ROS callbacks without blocking the keyboard loop.
                rclpy.spin_once(self, timeout_sec=0.0)

                ch = self.get_next_key(stdscr)
                if ch != -1:
                    self.handle_key(ch)

                now = time.time()
                if now - last_draw >= draw_period:
                    self.render_screen(stdscr)
                    last_draw = now

                time.sleep(0.01)

            except KeyboardInterrupt:
                self.last_key_name = "Ctrl+C"
                self.last_action = "shutting down due to keyboard interrupt"
                self.running = False
                break

            except Exception as exc:
                self.get_logger().error(f"Error in curses loop: {exc}")
                time.sleep(0.05)


def main(args=None) -> None:
    """
    Node entry point.
    """
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        curses.wrapper(node.curses_loop)

    except KeyboardInterrupt:
        # If an interruption still escapes the internal loop, proceed to
        # the standard shutdown sequence.
        pass

    finally:
        # Before destroying the node, try to publish one final stop command
        # if the ROS context is still valid.
        if rclpy.ok():
            try:
                zero_msg = Twist()
                node.cmd_vel_pub.publish(zero_msg)

                # Also republish the current audio mode so the final state is
                # explicit at shutdown.
                node.publish_audio_mode()

                # Publish one final diagnostic message so the experiment logger
                # can record the shutdown state.
                node.publish_teleop_diagnostics()

                time.sleep(0.05)
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
