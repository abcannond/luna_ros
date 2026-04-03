
# WPI Lunabotics ROS2 Stack

ROS2 Jazzy / Gazebo Harmonic simulation and hardware stack for the WPI Lunabotics 2024-25 robot. Includes SLAM (RTAB-Map), Nav2 navigation, ArUco fiducial localization, Wavefront Frontier Detection (WFD), arena zone classification, and a competition mission FSM.

## Quick Start (Docker)

```bash
./run_ros_image.sh [image_id]
```

Inside the container:

```bash
cd /ros2_ws && source install/setup.bash
# Full competition simulation (Gazebo + RTAB-Map + Nav2 + autonomy):
ros2 launch lunabot_2425 competition_sim.launch.py world:=ucf_arena
```

Wait ~30–40s for all nodes to initialize, then start a mission:

```bash
ros2 service call /mission_supervisor/start_mission std_srvs/srv/Trigger {}
```

Step-by-step (arenas, timing, what the mission does): [docs/COMPETITION_SIM.md](docs/COMPETITION_SIM.md).

## Documentation

| Document | Description |
|----------|-------------|
| [docs/COMPETITION_SIM.md](docs/COMPETITION_SIM.md) | **Competition sim: launch + `start_mission` flow** |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | System architecture, package map, data flow, TF tree |
| [docs/STACK_GUIDE.md](docs/STACK_GUIDE.md) | Launching sim, tuning, troubleshooting |
| [docs/AUTONOMY.md](docs/AUTONOMY.md) | Zones, ArUco, WFD frontier explorer, mission FSM |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Physical robot setup (RealSense, fiducial cams, Jetson) |
| [docs/COMPETITION_PREP_SSH_SEQUENCE.md](docs/COMPETITION_PREP_SSH_SEQUENCE.md) | Competition-day checklist and SSH sequence |

## Key Packages

- **lunabot_2425** -- URDF, Gazebo worlds, launch files, zone configs
- **luna_nav** -- Nav2 params, zone publisher, frontier explorer (WFD), mission supervisor
- **luna_mapping** -- RTAB-Map SLAM, depth processing, Nav2 integration
- **fiducial_localizer** -- Multi-camera ArUco detection for global pose
- **luna_control** -- Quad-swerve drivetrain controller

## How to run (Docker details)

Run the container with `./run_ros_image.sh [image_id]`.
This sets up the X host properly, allowing Docker to present a visible GUI (tested on Ubuntu 24.04).
