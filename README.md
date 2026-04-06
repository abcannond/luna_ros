
# WPI Lunabotics ROS2 Stack

ROS2 Jazzy / Gazebo Harmonic simulation and hardware stack for the WPI Lunabotics 2024-25 robot. Includes SLAM (RTAB-Map), Nav2 navigation, ArUco fiducial localization, Wavefront Frontier Detection (WFD), arena zone classification, and a competition mission FSM.

## Stack at a glance

The workspace is built to run **the same Nav2 and mapping core** in **sim** (Gazebo) and on **hardware** (Jetson, RealSense, fiducial webcams), with different drivers and `use_sim_time`.

| Piece | Package | Role |
|-------|---------|------|
| Robot + worlds + sim/hardware launches | `lunabot_2425` | URDF, Gazebo, `competition_sim`, `gz_bringup`, `hardware_bringup`, zones YAML, controllers |
| Mapping + depth pipeline | `luna_mapping` | RTAB-Map, frame fixers, `rtabmap_nav2_sim` / `_hardware` |
| Nav2 + mission + zones + WFD | `luna_nav` | `nav2_rtabmap_params.yaml`, zone_publisher, frontier_explorer, mission_supervisor |
| Drive controller | `luna_control` | `LunaController` (`luna_cont` in `robot_controllers.yaml`) |
| ArUco | `fiducial_localizer` | Multi-camera markers, `/fiducial_pose` |
| Arena assets | `luna_ros2_worlds` | Worlds and models (submodule) |

Legacy **`quad_swerve_controller`** is not loaded by current bringup (see that package’s README). Details, TF, topic tables, YAML index, and launch inventory: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

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

Eight files, written for a small team handoff. Read **COMPETITION_SIM** first if you only run sim; **ARCHITECTURE** when you change launches or params.

| Document | Description |
|----------|-------------|
| [docs/COMPETITION_SIM.md](docs/COMPETITION_SIM.md) | **Sim:** full launch, `start_mission`, optional two-terminal flow, **field pit checklist** (SSH, boot order) |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | Sim vs hardware profiles and which launch to use |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Packages, topics, TF, launch tree, **YAML index**, **package/launch inventory**, verification log |
| [docs/STACK_GUIDE.md](docs/STACK_GUIDE.md) | Launch variants, tuning, pipeline detail, **debugging quick reference** |
| [docs/AUTONOMY.md](docs/AUTONOMY.md) | Zones, ArUco, WFD, mission FSM |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Physical setup, USB, two-terminal hardware bringup, field checklist |
| [docs/COMPETITION_READINESS.md](docs/COMPETITION_READINESS.md) | Known gaps, risks, next steps (leads) |
| [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) | Pre-merge checklist, tests, CI |

## Key Packages

- **lunabot_2425** -- URDF, Gazebo worlds, launch files, zone configs
- **luna_nav** -- Nav2 params, zone publisher, frontier explorer (WFD), mission supervisor
- **luna_mapping** -- RTAB-Map SLAM, depth processing, Nav2 integration
- **fiducial_localizer** -- Multi-camera ArUco detection for global pose
- **luna_control** -- Quad-swerve drivetrain controller

## How to run (Docker details)

Run the container with `./run_ros_image.sh [image_id]`.
This sets up the X host properly, allowing Docker to present a visible GUI (tested on Ubuntu 24.04).
