#!/bin/bash
set -e

# Allow X11 connections for GUI
xhost +local:docker

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Only source workspace if it exists (bind mount will make it available)
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# If no command is passed, open bash
if [ $# -eq 0 ]; then
    exec bash
else
    exec "$@"
fi
