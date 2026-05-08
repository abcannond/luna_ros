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

# Go to the workspace by default (nice QoL)
cd /ros2_ws

source install/setup.bash

#turn on CAN interfaces automatically 
#turn them off first so we dont get kicked out of the container with "network busy"
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 up type can bitrate 250000
sudo ip link set can1 up type can bitrate 1000000

# If no arguments, launch interactive login shell so aliases work
if [ $# -eq 0 ]; then
    exec bash -il
else
    exec "$@"
fi

