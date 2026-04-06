# Running on hardware

This doc describes how to run the same camera and mapping stack on real hardware: **RealSense D455** for depth and RGB, and **4× Nexigo N980P** (or compatible) for fiducial cameras, typically on an **NVIDIA Jetson** inside the project’s Docker container.

**See also** [STACK_GUIDE.md](STACK_GUIDE.md), [ARCHITECTURE.md](ARCHITECTURE.md), [DEPLOYMENT.md](DEPLOYMENT.md).

---

## Field checklist (sign off when validated on the robot)

Use this as a lab worksheet. Date and initials each line when someone has verified it on your hardware.

| Step | Verified |
|------|----------|
| Power and estop behavior understood | |
| RealSense on USB 3, stable `ros2 topic hz` on color and depth | |
| Four fiducial cams enumerated (`v4l2-ctl --list-devices`) and launch args set | |
| `hardware_bringup.launch.py` runs without driver errors | |
| `/odom` publishing and `odom`→`base_link` on `/tf` from the base driver | |
| `rtabmap_nav2_hardware.launch.py` with `use_sim_time:=false`, `sim:=false` | |
| `tf2_echo map base_link` stable while driving slowly | |
| Nav2 goal in RViz completes in a safe test area | |

Unresolved items belong in [COMPETITION_READINESS.md](COMPETITION_READINESS.md).

---

## Sim vs hardware: what changes

The pipeline (camera → RTAB-Map + depth processing → Nav2) is the same. What changes is the data source, `use_sim_time`, sim-only static TFs, and who owns **`odom`→`base_link`**.

| In sim (default competition) | On hardware |
|------------------------------|-------------|
| Gazebo + bridge publish `camera/camera/*` | RealSense driver publishes the same topic names (`camera/camera`) |
| **`rgbd_odometry`** plus `enable_odom_tf: false` on `luna_cont` | Your **base driver** must publish `/odom` and **`odom`→`base_link`** (unless you intentionally mirror the sim rgbd odom setup) |
| `competition_sim` publishes **`world`→`map`** at spawn | You align map or initial pose per field procedure |
| `use_sim_time:=true` | Mapping launch uses **`use_sim_time:=false`** and **`sim:=false`** |

Run **two terminals** on hardware: cameras and camera TF first (`hardware_bringup`), then mapping and Nav2 (`rtabmap_nav2_hardware.launch.py`).

---

## What’s in place

So you know what each launch does:

- **`hardware_bringup.launch.py`** (lunabot_2425): Starts the RealSense D455 with namespace `camera/camera` so topics match sim. Starts 4× usb_cam nodes for the fiducial cams (e.g. `/fid_cams/front_left_camera`, …) with MJPEG decode so all cams fit on USB 2.0. **frame_id** is set to `front_left_camera_link_optical`, etc., so image/camera_info headers match what the fiducial localizer expects. Publishes static TF `base_link`→`camera_link` (depth) and **base_link**→**each fiducial camera optical frame** (front_left_camera_link_optical, …) so the multi-camera fiducial localizer can use all 4 cams. Tune the static TF xyz/yaw for your robot if needed (see launch file).

- **`rtabmap_nav2_hardware.launch.py`** (luna_mapping): Includes `rtabmap_nav2_sim.launch.py` with `use_sim_time:=false` and `sim:=false`. By default it also starts the **multi-camera fiducial localizer** (params: `multi_camera_hardware.yaml`, use_sim_time false) so all 4 fiducial cams are used for pose. Use `launch_fiducial:=false` to skip fiducial.

- **Shared launch** (`rtabmap_nav2_sim.launch.py`): All static TFs that are sim-only are behind `IfCondition(sim)`. When you call it with `sim:=false` (as the hardware launch does), those nodes are not started. On hardware you rely on `hardware_bringup` for `base_link`→`camera_link` and your robot driver for `odom`→`base_link` and `/odom`.

- **Frame fixers** run on hardware too. They subscribe to the RealSense topics and republish to `*_fixed` with consistent frame_ids (e.g. `camera_depth_optical_frame`). No separate “hardware” config is needed for the mapping launch; the same remappings work.

---

## Devices and topics

| Device | Role | Topics (after bringup) |
|--------|------|------------------------|
| Intel RealSense D455 | Depth + RGB for RTAB-Map and Nav2 | `/camera/camera/color/*`, `/camera/camera/depth/*` |
| 4× Nexigo N980P | Fiducial (ArUco) localization | `/fid_cams/front_left_camera/*`, front_right, back_left, back_right |

Topic names match simulation so the same mapping and Nav2 configs apply.

---

## Prerequisites

- NVIDIA Jetson (or x86 host) with Docker.
- RealSense D455 connected via USB 3.
- Four Nexigo N980P (or compatible V4L2 USB cams) connected.
- Built workspace and `luna_ros` image (see main [README](../README.md)).

---

## Find USB devices (one-time)

RealSense usually gets the first `/dev/video*` devices; the four fiducial cams will be on others. On the Jetson or host:

```bash
v4l2-ctl --list-devices
```

Note the device paths for the four webcams (e.g. `/dev/video2`, `/dev/video4`, `/dev/video6`, `/dev/video8`). You’ll pass these as launch arguments if they differ from the defaults.

---

## Run in Docker

From the repo root. If you’re on ARM64 (Jetson), use a Jetson-compatible image (e.g. built with `Dockerfile.jetson`).

```bash
./run_ros_image.sh luna_ros:latest
```

Inside the container:

```bash
cd /ros2_ws && source install/setup.bash
```

---

## Terminal 1: Cameras + TF

This starts the RealSense and fiducial cams and publishes `base_link`→`camera_link` so the mapping stack can run unchanged.

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

Check that topics are publishing:

```bash
ros2 topic list | grep -E "camera|fid_cams"
```

---

## Terminal 2: RTAB-Map + Nav2 + Fiducial (optional)

Uses the same pipeline as in simulation, with real time and no Gazebo TFs. **Odom must come from your robot:** `/odom` and the `odom`→`base_link` transform on `/tf`. By default this also starts the **multi-camera fiducial localizer** so all 4 Nexigo cams are used for ArUco-based pose (publishes `/fiducial_pose` and `map`→`odom` when a marker is seen).

```bash
ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py
```

To run without the fiducial node:

```bash
ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py launch_fiducial:=false
```

RViz opens with the Nav2 + map config. Use the Nav2 Goal tool to send goals once you have a map and odometry.

---

## Fiducial localizer (all 4 cameras)

The fiducial localizer subscribes to all 4 wheel-pod camera topics (`/fid_cams/front_left_camera/image_raw`, etc.) and their `camera_info`. It needs TF from `base_link` to each camera’s optical frame (`front_left_camera_link_optical`, …). **Hardware bringup** sets each usb_cam node's `frame_id` to that name and publishes the corresponding static TFs, so no extra setup is required. When any camera sees an ArUco marker, the node computes the robot pose in the world frame and publishes it and `map`→`odom`. Configure your arena marker pose in `fiducial_localizer/params/multi_camera_hardware.yaml` (`world_to_marker_xyz`, `world_to_marker_yaw`).

---

## Docker on Jetson

For ARM64 Jetson, use a Jetson-compatible base image (e.g. L4T or `dustynv/ros`). Example using `Dockerfile.jetson`:

```bash
docker build -f Dockerfile.jetson -t luna_ros:jetson .
./run_ros_image.sh luna_ros:jetson
```

The launch flow (Terminal 1 = hardware_bringup, Terminal 2 = rtabmap_nav2_hardware) is the same; the base image and package install steps depend on your JetPack and ROS 2 distro.

---

## Troubleshooting

| Problem | What to do |
|--------|------------|
| **RealSense not found** | Ensure USB 3 and udev rules are installed. Install `ros-jazzy-realsense2-camera` and run the udev script it suggests if any. |
| **Wrong fiducial devices** | Run `v4l2-ctl --list-devices` and set the `fid_*_dev` launch arguments to the correct `/dev/video*` nodes. Defaults: `/dev/video0,2,4,6` for 4 cams. |
| **"No space left on device" (cameras)** | USB 2.0 bandwidth limit. Fiducial cams use usb_cam with MJPEG; if you see this, ensure you're not running extra uncompressed streams. |
| **No odom / TF** | RTAB-Map and Nav2 need `odom`→`base_link` on `/tf`. Start your robot base driver so it publishes odometry and this transform. |
| **Namespace mismatch** | If the RealSense driver can’t use `camera_namespace:=camera/camera`, remap its topics to `/camera/camera/color/image_raw` (and depth/camera_info) in a small launch or relay node so the rest of the stack still sees the expected names. |
