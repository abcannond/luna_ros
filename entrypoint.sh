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

# Turn on CAN interfaces automatically
# Turn them off first so we don't get kicked out of the container with "network busy"
#: << 'COMMENT'
bring_up_can() {
    local iface=$1
    local bitrate=$2
    local max_retries=5
    local retry_delay=1

    echo "Configuring $iface at bitrate $bitrate..."

    # Check if interface exists
    if ! ip link show "$iface" &>/dev/null; then
        echo "ERROR: Interface $iface does not exist. Skipping."
        return 0
    fi

    # Bring down with retries (ignore errors if already down)
    for i in $(seq 1 $max_retries); do
        if sudo ip link set "$iface" down 2>/dev/null; then
            echo "$iface brought down successfully."
            break
        fi
        echo "Attempt $i/$max_retries: Failed to bring $iface down, retrying in ${retry_delay}s..."
        sleep $retry_delay
    done

    # Small delay to let the interface settle
    sleep 0.5

    # Bring up with retries
    for i in $(seq 1 $max_retries); do
        if sudo ip link set "$iface" up type can bitrate "$bitrate" 2>/dev/null; then
            echo "$iface is up at bitrate $bitrate."
            return 0
        fi
        echo "Attempt $i/$max_retries: Failed to bring $iface up, retrying in ${retry_delay}s..."
        sleep $retry_delay
    done

    echo "ERROR: Failed to bring up $iface after $max_retries attempts."
    return 0
}

bring_up_can can0 250000
bring_up_can can1 1000000
#COMMENT

# Sync clock to fix build cache clock skew
sudo date -s "$(date -u '+%Y-%m-%d %H:%M:%S')" > /dev/null 2>&1 || true

# If no arguments, launch interactive login shell so aliases work
if [ $# -eq 0 ]; then
    exec bash -il
else
    exec "$@"
fi

