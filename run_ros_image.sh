#!/usr/bin/env bash
set -e

# Enable X11 for GUI (run on host)
xhost +local:docker || true

# Workspace path on host
HOST_WS="$(pwd)/ros2_ws"

# Image name passed as first argument
IMAGE_NAME="$1"

# Only pass /dev/dri if it exists (Jetson has it, Mac might not)
EXTRA_DEV=""
if [ -e /dev/dri ]; then
  EXTRA_DEV="--device /dev/dri:/dev/dri"
fi

docker run -it \
  --env DISPLAY="$DISPLAY" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume "$HOST_WS":/ros2_ws:rw \
  $EXTRA_DEV \
  --network host \
  --entrypoint /bin/bash \
  "$IMAGE_NAME"

