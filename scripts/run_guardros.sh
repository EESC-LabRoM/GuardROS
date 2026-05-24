#!/usr/bin/env bash

WORKSPACE="/path/to/RollerBot23/guardros_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WORKSPACE/install/setup.bash"

PID_FILE="/tmp/guardros_pids.txt"

EXPERIMENT_LOG_DIR="$WORKSPACE/experiments/logs"
EXPERIMENT_LABEL="manual_run"

# ==============================
# ROBOT_ID INPUT
# ==============================

echo "=============================================="
echo "GuardROS - Initial Configuration"
echo "=============================================="
echo ""

read -p "Enter the ROBOT_ID [default = 1]: " USER_ROBOT_ID

if [ -z "$USER_ROBOT_ID" ]; then
    USER_ROBOT_ID=1
fi

if ! [[ "$USER_ROBOT_ID" =~ ^[0-9]+$ ]]; then
    echo "Error: invalid ROBOT_ID"
    exit 1
fi

if [ "$USER_ROBOT_ID" -lt 0 ] || [ "$USER_ROBOT_ID" -gt 63 ]; then
    echo "Error: ROBOT_ID out of range (0–63)"
    exit 1
fi

echo "ROBOT_ID: $USER_ROBOT_ID"
echo "Experiment log directory: $EXPERIMENT_LOG_DIR"
echo ""

mkdir -p "$EXPERIMENT_LOG_DIR"

# ==============================
# PID LIST
# ==============================

GUARDROS_PIDS=()

# ==============================
# TERMINAL LAUNCH FUNCTION
# ==============================

open_terminal () {
    gnome-terminal --title="$2" -- bash -c "$1" &
    PID=$!
    GUARDROS_PIDS+=($PID)
    sleep 0.6
}

# ==============================
# NODE COMMANDS
# ==============================

DRIVER_CMD="
cd '$WORKSPACE'
source '$ROS_SETUP'
source '$WS_SETUP'
echo 'DRIVER - ROBOT_ID=$USER_ROBOT_ID'
ros2 run rb23_ros_driver rb23_driver_node --ros-args -p robot_id:=$USER_ROBOT_ID
exec bash
"

TELEOP_CMD="
cd '$WORKSPACE'
source '$ROS_SETUP'
source '$WS_SETUP'
echo 'TELEOP'
ros2 run rb23_keyboard_teleop keyboard_teleop_node
exec bash
"

VIDEO_CMD="
cd '$WORKSPACE'
source '$ROS_SETUP'
source '$WS_SETUP'
echo 'VIDEO VIEWER'
ros2 run rb23_video_viewer rb23_video_viewer_node
exec bash
"

TELEMETRY_CMD="
cd '$WORKSPACE'
source '$ROS_SETUP'
source '$WS_SETUP'
echo 'TELEMETRY VIEWER'
ros2 run rb23_telemetry_viewer rb23_telemetry_viewer_node
exec bash
"

AUDIO_CMD="
cd '$WORKSPACE'
source '$ROS_SETUP'
source '$WS_SETUP'
echo 'AUDIO NODE'
ros2 run rb23_audio rb23_audio_node
exec bash
"

LOGGER_CMD="
cd '$WORKSPACE'
source '$ROS_SETUP'
source '$WS_SETUP'
echo 'EXPERIMENT LOGGER'
echo 'Logs will be saved to: $EXPERIMENT_LOG_DIR'
ros2 run rb23_experiment_logger rb23_experiment_logger_node --ros-args -p base_log_dir:='$EXPERIMENT_LOG_DIR' -p experiment_label:='$EXPERIMENT_LABEL'
exec bash
"

# ==============================
# OPEN TERMINALS
# ==============================

open_terminal "$DRIVER_CMD" "GuardROS - DRIVER"
open_terminal "$TELEOP_CMD" "GuardROS - TELEOP"
open_terminal "$VIDEO_CMD" "GuardROS - VIDEO VIEWER"
open_terminal "$TELEMETRY_CMD" "GuardROS - TELEMETRY VIEWER"
open_terminal "$AUDIO_CMD" "GuardROS - AUDIO NODE"
open_terminal "$LOGGER_CMD" "GuardROS - EXPERIMENT LOGGER"

# ==============================
# SAVE PIDS
# ==============================

echo "${GUARDROS_PIDS[@]}" > "$PID_FILE"

echo ""
echo "GuardROS started successfully."
echo "Experiment logger enabled."
echo "Logs will be saved to $EXPERIMENT_LOG_DIR"
echo "PIDs saved to $PID_FILE"
