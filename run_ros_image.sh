#!/usr/bin/env bash
set -e

IMAGE_NAME=$1

if [ -z "$IMAGE_NAME" ]; then
    echo "Usage: $0 <docker_image_name>"
    exit 1
fi

# Allow X11 access from local root and SSH-forwarded sessions
xhost +local:root 2>/dev/null || true
xhost +localhost 2>/dev/null || true

# Gazebo GUI uses OpenGL/EGL; gz_bringup defaults to server-only (-s) in Docker (see gz_server_only).
# No Mesa overrides here — they conflict with EGL when a hardware device is selected.

# Mount workspace
HOST_WS=$(pwd)/ros2_ws

docker run -it \
    --group-add video \
    --device /dev/input:/dev/input \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env LIBGL_ALWAYS_INDIRECT=0 \
    -e OGRE_RTT_MODE=Copy \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$HOME/.Xauthority:/root/.Xauthority:ro" \
    --env XAUTHORITY=/root/.Xauthority \
    --volume "$HOST_WS":/ros2_ws:rw \
    --network host \
    --privileged \
    $IMAGE_NAME \
    bash
