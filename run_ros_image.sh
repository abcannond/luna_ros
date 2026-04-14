#!/usr/bin/env bash
set -e

IMAGE_NAME=$1

if [ -z "$IMAGE_NAME" ]; then
    echo "Usage: $0 <docker_image_name>"
    exit 1
fi

# Allow X11 access to local docker containers
xhost +local:root

# Detect GPU — NVIDIA: use container toolkit + GLVND (no forced Mesa iris / host libGL overlays).
# Non-NVIDIA: keep software-rendering workarounds (iris + host DRI/GL mounts).
if command -v nvidia-smi &>/dev/null; then
    echo "NVIDIA GPU detected: enabling --gpus all"
    GPU_FLAGS="--gpus all \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e __NV_PRIME_RENDER_OFFLOAD=1 \
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia"
    HOST_GL_MOUNTS=""
else
    echo "No NVIDIA GPU detected: using software rendering"
    GPU_FLAGS="-e LIBGL_ALWAYS_SOFTWARE=1 \
        -e MESA_LOADER_DRIVER_OVERRIDE=iris \
        -e GALLIUM_DRIVER=iris"
    HOST_GL_MOUNTS="-v /usr/lib/x86_64-linux-gnu/dri:/usr/lib/x86_64-linux-gnu/dri:ro \
        -v /usr/lib/x86_64-linux-gnu/libdrm.so.2:/usr/lib/x86_64-linux-gnu/libdrm.so.2:ro \
        -v /usr/lib/x86_64-linux-gnu/libEGL.so.1:/usr/lib/x86_64-linux-gnu/libEGL.so.1:ro \
        -v /usr/lib/x86_64-linux-gnu/libGL.so.1:/usr/lib/x86_64-linux-gnu/libGL.so.1:ro"
fi

# Mount workspace
HOST_WS=$(pwd)/ros2_ws

docker run -it \
    --device=/dev/dri \
    --group-add video \
    $GPU_FLAGS \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env LIBGL_ALWAYS_INDIRECT=0 \
    -e GZ_RENDER_ENGINE=ogre2 \
    -e OGRE_RTT_MODE=Copy \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$HOST_WS":/ros2_ws:rw \
    $HOST_GL_MOUNTS \
    --network host \
    --privileged \
    $IMAGE_NAME \
    bash
