# Luna ROS

ROS 2 Jazzy autonomy stack for the **WPI Lunabotics** robot. Provides SLAM, navigation, and fiducial-based localization for the Lunabotics competition arena.

**Sensors:** Intel RealSense D455 (depth + RGB) and 4x Nexigo N980P (ArUco fiducial localization).

**Stack:** Gazebo Harmonic simulation, RTAB-Map visual SLAM, Nav2 autonomous navigation, quad-swerve drivetrain controller via ros2_control. Runs in Docker on Ubuntu 24.04 (x86) or Jetson (ARM64).

---

## Packages

| Package | Description |
|---------|-------------|
| `lunabot_2425` | Robot URDF, launch files, sim/hardware bringup |
| `luna_control` | Quad-swerve ros2_control controller (steer + drive) |
| `luna_mapping` | RTAB-Map SLAM, depth processing, Nav2 integration |
| `luna_nav` | Nav2 config, arena zone publisher |
| `fiducial_localizer` | Multi-camera ArUco pose estimation (map->odom) |
| `depth_to_pointcloud` | Depth image to PointCloud2 conversion |
| `luna_ros2_worlds` | Gazebo arena worlds (UCF, Artemis) and models |

---

## Quick start — simulation

**First time:** See [INSTALL.md](INSTALL.md) for clone, Docker build, and workspace setup.

Inside the container:

```bash
cd /ros2_ws && source install/setup.bash
ros2 launch lunabot_2425 gz_bringup.launch.py
```

One command launches Gazebo, RTAB-Map, Nav2, and RViz. Use the **Nav2 Goal** tool in RViz to navigate.

| Variant | Command |
|---------|---------|
| Gazebo only (no mapping) | `gz_bringup.launch.py launch_mapping:=false` |
| Artemis arena | `gz_bringup.launch.py world:=artemis_arena` |

---

## Quick start — hardware

**Terminal 1** — cameras and TF:

```bash
ros2 launch lunabot_2425 hardware_bringup.launch.py
```

**Terminal 2** — RTAB-Map + Nav2:

```bash
ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py
```

Your robot driver must publish `/odom` and the `odom` -> `base_link` transform.

---

## Documentation

| Doc | Purpose |
|-----|---------|
| [INSTALL.md](INSTALL.md) | Clone, build, Docker setup |
| [docs/STACK_GUIDE.md](docs/STACK_GUIDE.md) | Pipeline details, features, tuning, troubleshooting |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Hardware setup (Jetson, RealSense, fiducial cams) |
| [docs/STACK_REVIEW_LUNABOTICS.md](docs/STACK_REVIEW_LUNABOTICS.md) | Competition fit analysis, risks, compute limits |

---

## Architecture

```
Camera (D455) ──> RTAB-Map (SLAM) ──> /map (occupancy grid)
     │                                      │
     ├──> Depth FOV Filter ──> Point Cloud ──> Nav2 Costmaps ──> Path Planning
     │                    └──> LaserScan                              │
     │                                                               v
4x Nexigo ──> Fiducial Localizer ──> map->odom TF          /cmd_vel -> Robot
```

---

## Requirements

- Docker, Linux with X11 (Ubuntu 24.04 recommended)
- GPU recommended (for Gazebo rendering)
- **Hardware:** Intel RealSense D455, 4x USB webcams (Nexigo N980P), optional NVIDIA Jetson
