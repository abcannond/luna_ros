# Running on Hardware (Jetson + RealSense D455 + 4× Nexigo N980P)

This guide covers running the same functional camera and mapping stack on real hardware: an **Intel RealSense D455** depth camera and **4× Nexigo N980P** USB webcams for fiducial localization, typically on an **NVIDIA Jetson** inside the project’s Docker container.

## Hardware Overview

| Device | Role | Topics (after bringup) |
|--------|------|------------------------|
| Intel RealSense D455 | Depth + RGB for RTAB-Map, Nav2 | `/camera/camera/color/*`, `/camera/camera/depth/*` |
| 4× Nexigo N980P | Fiducial (ArUco) localization | `/fid_cams/front_left_camera/*`, `front_right`, `back_left`, `back_right` |

Topic names match simulation so you can use the same mapping and Nav2 launch files.

## Prerequisites

- NVIDIA Jetson (or x86 host) with Docker.
- RealSense D455 connected via USB 3.
- Four Nexigo N980P (or compatible V4L2 USB cams) connected.
- Built workspace and `luna_ros` image (see main README).

## One-time: find USB camera devices

RealSense usually gets the first `/dev/video*` nodes. The four fiducial cams will be on other indices. On the Jetson (or host):

```bash
# List V4L2 devices
v4l2-ctl --list-devices
```

Note the device paths for the four webcams (e.g. `/dev/video2`, `/dev/video4`, `/dev/video6`, `/dev/video8`). You will pass these as launch arguments if they differ from the defaults.

## Running in Docker on Jetson

From the repo root (with the image built for your platform, e.g. using `Dockerfile.jetson`):

```bash
./run_ros_image.sh luna_ros:latest
```

Inside the container:

```bash
cd /ros2_ws
source install/setup.bash
```

## Terminal 1: Hardware bringup (cameras + TF)

Starts the RealSense D455 and the four fiducial webcams, and publishes `base_link` → `camera_link` so the rest of the stack can run unchanged.

```bash
ros2 launch lunabot_2425 hardware_bringup.launch.py
```

If your fiducial cameras are on different devices, override the defaults:

```bash
ros2 launch lunabot_2425 hardware_bringup.launch.py \
  fid_front_left_dev:=/dev/video2 \
  fid_front_right_dev:=/dev/video4 \
  fid_back_left_dev:=/dev/video6 \
  fid_back_right_dev:=/dev/video8
```

Check that topics are published:

```bash
ros2 topic list | grep -E "camera|fid_cams"
```

## Terminal 2: RTAB-Map + Nav2 (mapping and navigation)

Uses the same pipeline as in simulation, with real time and no Gazebo TFs. **Odom must be provided by your robot** (e.g. wheel odometry) on `/odom` and as `odom` → `base_link` on `/tf`.

```bash
ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py
```

RViz opens with the Nav2 + map config. Use the Nav2 Goal tool to send goals once you have a map and odometry.

## Optional: Fiducial localizer

If you use ArUco markers for localization, run the fiducial localizer with **hardware** params (no sim time):

```bash
ros2 launch fiducial_localizer multi_camera_fiducial.launch.py \
  params_file:=$(ros2 pkg prefix fiducial_localizer)/share/fiducial_localizer/params/multi_camera_hardware.yaml \
  use_sim_time:=false
```

Create `multi_camera_hardware.yaml` from `multi_camera_sim.yaml` if needed (same topic names; only `use_sim_time` and any arena-specific pose hints may differ).

## Docker on Jetson (Jetson-specific image)

For ARM64 Jetson, use a Jetson-compatible base image. Example `Dockerfile.jetson`:

```dockerfile
# Use NVIDIA L4T ROS2 image (adjust tag for your JetPack)
FROM nvcr.io/nvidia/l4t-ml:r35.2.1-py3
# Or: FROM dustynv/ros:jazzy-ros-base-l4t-r35.2.1
SHELL ["/bin/bash", "-c"]
# Install ROS2 Jazzy, realsense2_camera, v4l2_camera, Nav2, RTAB-Map, etc.
# Then same RUN/COPY as main Dockerfile for workspace and entrypoint.
```

Build and run:

```bash
docker build -f Dockerfile.jetson -t luna_ros:jetson .
./run_ros_image.sh luna_ros:jetson
```

Exact base image and package install steps depend on your JetPack and ROS2 distro; the launch flow above is unchanged.

## Troubleshooting

- **RealSense not found**: Ensure USB 3 and udev rules are installed (`sudo apt install ros-jazzy-realsense2-camera` and run the suggested `realsense2_camera` udev script if any).
- **Wrong fiducial devices**: Use `v4l2-ctl --list-devices` and set `fid_*_dev` launch arguments to the correct `/dev/video*` nodes.
- **No odom / TF**: RTAB-Map and Nav2 need `odom` → `base_link` on `/tf`. Start your robot base driver so it publishes odometry and this transform.
- **Namespace mismatch**: If the RealSense driver does not support `camera_namespace:=camera/camera`, remap its topics to `/camera/camera/color/image_raw` (and depth/camera_info) in a small launch or relay node so the rest of the stack still sees the expected names.
