#!/usr/bin/env bash

echo "Shutting down GuardROS..."

# Terminate all project nodes
pkill -f rb23_driver_node
pkill -f keyboard_teleop_node
pkill -f rb23_video_viewer_node
pkill -f rb23_telemetry_viewer_node
pkill -f rb23_audio_node

echo "ROS nodes terminated."

# Optional: close terminal windows as well
pkill -f "GuardROS - DRIVER"
pkill -f "GuardROS - TELEOP"
pkill -f "GuardROS - VIDEO VIEWER"
pkill -f "GuardROS - TELEMETRY VIEWER"
pkill -f "GuardROS - AUDIO NODE"

echo "Terminal windows closed."
echo "GuardROS shutdown complete."