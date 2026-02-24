# Luna ROS — Simulation and Navigation Stack

ROS 2 Jazzy stack for the WPI Lunabotics robot: **Gazebo Harmonic** simulation, **RTAB-Map** SLAM, **Nav2** navigation, and the same camera pipeline for **real hardware** (Intel RealSense D455 + 4× Nexigo N980P fiducial cams). Runs in Docker for a consistent environment (Ubuntu 24.04; Jetson supported via a separate image).

## What it does

- **Simulation:** Gazebo runs the robot with a simulated RealSense D455 and four fiducial cameras. You can use the UCF or Artemis arena (`world:=ucf_arena` or `world:=artemis_arena`).
- **Mapping:** RTAB-Map builds an occupancy grid from the depth camera and publishes `/map`.
- **Navigation:** Depth is converted to a 2D scan and point cloud; Nav2 plans and drives the robot to goals set in RViz.
- **Fiducials:** Four wheel-pod cameras (sim or real) are used for ArUco-based localization when enabled.

The same functional camera and mapping code is intended for **real hardware**: one RealSense D455 and four USB webcams (e.g. Nexigo N980P) for fiducial localization, typically on an **NVIDIA Jetson** in the same Docker setup.

## Quick start (simulation)

**First time (host):**

```bash
git clone --recurse-submodules <repo_url>
cd luna_ros
docker build -t luna_ros:latest .
./run_ros_image.sh luna_ros:latest
```

**Inside the container:**

```bash
cd /ros2_ws
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Single command (sim):** one launch runs Gazebo, then after ~18 s starts RTAB-Map, Nav2, and RViz. No second terminal needed.

```bash
ros2 launch lunabot_2425 gz_bringup.launch.py
```

Use the **Nav2 Goal** tool in RViz to send navigation goals.

- **Gazebo only (no mapping):**  
  `ros2 launch lunabot_2425 gz_bringup.launch.py launch_mapping:=false`
- **Other arena:**  
  `ros2 launch lunabot_2425 gz_bringup.launch.py world:=artemis_arena`

## Quick start (hardware)

On a Jetson (or host) with a RealSense D455 and four USB fiducial cams:

1. **Terminal 1 — cameras + TF**  
   `ros2 launch lunabot_2425 hardware_bringup.launch.py`  
   (Override `fid_*_dev` if your `/dev/video*` devices differ; see [docs/HARDWARE.md](docs/HARDWARE.md).)

2. **Terminal 2 — mapping + Nav2**  
   `ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py`  

Your robot must publish odometry (`/odom` and `odom` → `base_link` on `/tf`). Full hardware details, device setup, and optional fiducial localizer: **[docs/HARDWARE.md](docs/HARDWARE.md)**.

## Docs and layout

| Doc / file | Purpose |
|------------|--------|
| [LAUNCH_COMMANDS.md](LAUNCH_COMMANDS.md) | Full sim guide: manual terminals, options, troubleshooting |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Hardware: Jetson, RealSense D455, 4× Nexigo N980P, Docker |
| [Commands to run and what.txt](Commands%20to%20run%20and%20what.txt) | Short command and topic reference |
| `Dockerfile` | Main image (x86_64, Ubuntu 24.04, ROS 2 Jazzy) |
| `Dockerfile.jetson` | Jetson (ARM64) image template |

## Requirements

- Docker, Linux with X11 (e.g. Ubuntu 24.04)
- For hardware: RealSense D455, 4× USB webcams (e.g. Nexigo N980P), optional Jetson
