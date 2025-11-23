#!/bin/bash
set -e

IMAGE_NAME=$1

if [ -z "$IMAGE_NAME" ]; then
    echo "Usage: ./run_ros_image.sh <image_name>"
    exit 1
fi

# Enable X11 on host
xhost +local:docker

# Detect if NVIDIA GPU exists on host
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU detected: enabling --gpus all"
    GPU_FLAGS="--gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all"
else
    echo "No NVIDIA GPU detected: using standard Mesa OpenGL"
    GPU_FLAGS=""
fi

HOST_WS=$(pwd)/ros2_ws

docker run -it \
    $GPU_FLAGS \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume "$HOST_WS":/ros2_ws:rw \
    --network host \
    $IMAGE_NAME \
    bash
