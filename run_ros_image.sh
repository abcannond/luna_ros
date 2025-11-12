#!/bin/bash
set -e

# Enable X11 for GUI
xhost +local:docker

# Workspace path on host
HOST_WS=$(pwd)/ros2_ws

# Image name passed as first argument
IMAGE_NAME=$1

docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume "$HOST_WS":/ros2_ws:rw \
  --device /dev/dri:/dev/dri \
  --network host \
  $IMAGE_NAME \
  bash
