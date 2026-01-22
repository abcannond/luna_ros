#!/usr/bin/env bash
set -e

IMAGE_NAME=$1

if [ -z "$IMAGE_NAME" ]; then
    echo "Usage: $0 <docker_image_name>"
    exit 1
fi

# Allow X11 access to local docker containers
xhost +local:root

# Detect GPU
if command -v nvidia-smi &>/dev/null; then
    echo "NVIDIA GPU detected: enabling --gpus all"
    GPU_FLAGS="--gpus all \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all"
else
    echo "No NVIDIA GPU detected: using software rendering"
    GPU_FLAGS="-e LIBGL_ALWAYS_SOFTWARE=1"
fi

# Mount workspace
HOST_WS=$(pwd)/ros2_ws

docker run -it \
    $GPU_FLAGS \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env LIBGL_ALWAYS_INDIRECT=0 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$HOST_WS":/ros2_ws:rw \
    --network host \
    --privileged \
    $IMAGE_NAME \
    bash
