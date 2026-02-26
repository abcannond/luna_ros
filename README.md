# Luna ROS

ROS 2 Jazzy stack for the WPI Lunabotics robot: **Gazebo Harmonic** sim, **RTAB-Map** SLAM, **Nav2**, and the same pipeline on **real hardware** (RealSense D455 + 4× Nexigo N980P). Runs in Docker (Ubuntu 24.04; Jetson via separate image).

---

## What it does

- **Sim:** Gazebo + simulated RealSense D455 (UCF or Artemis arena). RTAB-Map builds `/map`; depth → scan + point cloud; Nav2 plans and drives to goals in RViz.
- **Hardware:** Same camera and mapping stack on Jetson with RealSense D455 and four USB fiducial cams.

---

## Quick start — simulation

**First time:** [INSTALL.md](INSTALL.md) (clone, Docker, build).

Inside the container:

```bash
cd /ros2_ws && source install/setup.bash
ros2 launch lunabot_2425 gz_bringup.launch.py
```

One launch runs Gazebo, then RTAB-Map, Nav2, and RViz. Use the **Nav2 Goal** tool in RViz.

- Gazebo only: `launch_mapping:=false`
- Other arena: `world:=artemis_arena`

---

## Quick start — hardware

1. **Terminal 1:** `ros2 launch lunabot_2425 hardware_bringup.launch.py`
2. **Terminal 2:** `ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py`

Robot must publish `/odom` and `odom`→`base_link`. Details: [docs/HARDWARE.md](docs/HARDWARE.md).

---

## Documentation

| Doc | Purpose |
|-----|---------|
| [INSTALL.md](INSTALL.md) | Install, build, Docker |
| [docs/STACK_GUIDE.md](docs/STACK_GUIDE.md) | Launching sim, features, design, troubleshooting |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Hardware (Jetson, RealSense, fiducial cams) |
| [docs/STACK_REVIEW_LUNABOTICS.md](docs/STACK_REVIEW_LUNABOTICS.md) | Competition fit, ground crop, risks, compute |

---

## Requirements

- Docker, Linux with X11 (e.g. Ubuntu 24.04)
- Hardware: RealSense D455, 4× USB webcams (e.g. Nexigo N980P), optional Jetson
