#!/bin/bash
set -e

# Source ROS 2 Jazzy
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  source /opt/ros/jazzy/setup.bash
fi

# Source the workspace overlay if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source /ros2_ws/install/setup.bash
fi

# Go to the workspace by default (nice QoL)
cd /ros2_ws

# Hand off to whatever was passed (default is "bash")
exec "$@"

