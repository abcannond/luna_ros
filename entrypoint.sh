#!/bin/bash
set -e

# Source ROS 2 base setup
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace setup if it exists
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# If no arguments, launch interactive login shell so aliases work
if [ $# -eq 0 ]; then
    exec bash -il
else
    exec "$@"
fi
