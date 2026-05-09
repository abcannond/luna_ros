# System Architecture

Software architecture for the WPI Lunabotics ROS 2 stack (sim and hardware).

**See also** [README.md](../README.md) (quick start and stack at a glance).

---

## Package responsibility matrix

| Package | Responsibility |
|---------|----------------|
| `lunabot_2425` | URDF/xacro, Gazebo bridges, sim and hardware launches, zone YAML, `robot_controllers.yaml` |
| `luna_control` | `ros2_control` plugin `LunaController` (`luna_cont` in config) |
| `quad_swerve_controller` | Legacy controller package; **not** loaded by current bringup (see package README) |
| `luna_mapping` | RTAB-Map SLAM, depth pipeline, frame fixers, `rtabmap_nav2_sim.launch.py` / hardware launch |
| `luna_nav` | Nav2 YAML, zone_publisher, frontier_explorer, mission_supervisor |
| `fiducial_localizer` | Multi-camera ArUco, `/fiducial_pose` |
| `depth_to_pointcloud` | Depth to PointCloud2 when mapping launch is off (`gz_bringup` `launch_mapping:=false`) |
| `luna_ros2_worlds` | Gazebo worlds and models (submodule) |
| `twist_stamper` | `/cmd_vel` to `/cmd_vel_stamped` for `LunaController` (submodule) |
| `teleop_twist_keyboard` | Keyboard teleop (submodule) |

---

## Data flow (competition sim)

Sensors and bridges feed fixed-frame topics. RTAB-Map builds `/map` and publishes `map`→`odom`. With `use_rtabmap_odom:=true`, `rgbd_odometry` publishes `odom`→`base_link` and `/odom`. Nav2 plans in `map` and tracks the same TF chain. Mission supervisor sends goals in `world` (aligned to `map` via static transform from `competition_sim.launch.py`). Zone publisher classifies pose in `map`. Frontier explorer reads `/map` when enabled.

Fiducial node publishes `/fiducial_pose` for mission gating; robust fusion into `map`→`odom` is still a documented gap (see [COMPETITION_READINESS.md](COMPETITION_READINESS.md)).

---

## Topic map (key topics)

| Topic | Type | Typical publisher | Notes |
|-------|------|-------------------|--------|
| `/map` | `nav_msgs/OccupancyGrid` | rtabmap | Nav2 global costmap, frontier_explorer |
| `/odom` | `nav_msgs/Odometry` | rgbd_odometry when `use_rtabmap_odom:=true` | Else wheel or driver odom per bringup |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2, mission_supervisor (zeros), teleop | Routed via twist_stamper to controller |
| `/current_zone` | `std_msgs/String` | zone_publisher | mission_supervisor |
| `/mission_state` | `std_msgs/String` | mission_supervisor | |
| `/exploration_status` | `std_msgs/String` | frontier_explorer | |
| `/fiducial_pose` | `geometry_msgs/PoseStamped` | fiducial_localizer | |

---

## TF frame hierarchy (sim, default competition)

- **`world`** — Gazebo world frame. Static transform **`world`→`map`** at robot spawn (from `competition_sim.launch.py`).
- **`map`** — SLAM map frame. **`map`→`odom`** from RTAB-Map.
- **`odom`** — Smooth odometry frame. **`odom`→`base_link`** from `rgbd_odometry` when `use_rtabmap_odom:=true`. `luna_cont` sets **`enable_odom_tf: false`** so wheels do not also publish this link.
- **`base_link`** — Robot root from URDF; `robot_state_publisher` publishes fixed and joint transforms to cameras and wheels.

Nav2 `global_frame` is **`map`**. Mission goals use frame **`world`** to match zone YAML; it coincides with `map` at spawn offset only (see launch).

---

## Competition sim launch hierarchy

```
competition_sim.launch.py
 ├── gz_bringup.launch.py
 │    ├── rsp.launch.py
 │    ├── Gazebo + spawn + bridges + twist_stamper + controller spawners
 │    └── rtabmap_nav2_sim.launch.py (delayed ~25 s)
 │         ├── depth pipeline, frame fixers, RTAB-Map, rgbd_odometry (if enabled), Nav2, RViz
 │         └── optional fiducial launch
 ├── static world→map (spawn alignment)
 └── (delayed ~30 s) zone_publisher, frontier_explorer, mission_supervisor
```

---

## Configuration index

This section lists **where** settings live. It does not duplicate full parameter dumps. Open the linked files for values.

### Navigation and costmaps

| Area | File |
|------|------|
| Nav2 stack (local and global costmaps, DWB, behaviors, velocity smoother) | [`ros2_ws/src/luna_nav/config/nav2_rtabmap_params.yaml`](../ros2_ws/src/luna_nav/config/nav2_rtabmap_params.yaml) |
| Alternate Nav2 file (if used) | [`ros2_ws/src/luna_nav/config/nav2_params.yaml`](../ros2_ws/src/luna_nav/config/nav2_params.yaml) |

### SLAM and depth pipeline

| Area | File |
|------|------|
| RTAB-Map node params (sim) | [`ros2_ws/src/luna_mapping/config/rtabmap_sim.yaml`](../ros2_ws/src/luna_mapping/config/rtabmap_sim.yaml) |
| Mapping launch (node graph, remaps, `use_rtabmap_odom`, RViz path) | [`ros2_ws/src/luna_mapping/launch/rtabmap_nav2_sim.launch.py`](../ros2_ws/src/luna_mapping/launch/rtabmap_nav2_sim.launch.py) |
| Hardware mapping entry | [`ros2_ws/src/luna_mapping/launch/rtabmap_nav2_hardware.launch.py`](../ros2_ws/src/luna_mapping/launch/rtabmap_nav2_hardware.launch.py) |
| Experimental RViz configs | [`ros2_ws/src/luna_mapping/config/experimental/`](../ros2_ws/src/luna_mapping/config/experimental/) |
| Default competition RViz | [`ros2_ws/src/luna_mapping/config/competition_sim.rviz`](../ros2_ws/src/luna_mapping/config/competition_sim.rviz) |

### Robot control

| Area | File |
|------|------|
| Controller manager and `luna_cont` (`enable_odom_tf`, wheel names, `command_topic`) | [`ros2_ws/src/lunabot_2425/config/robot_controllers.yaml`](../ros2_ws/src/lunabot_2425/config/robot_controllers.yaml) |

### Autonomy and arenas

| Area | File |
|------|------|
| Mission phases and default waypoints | [`ros2_ws/src/luna_nav/config/mission_phases.yaml`](../ros2_ws/src/luna_nav/config/mission_phases.yaml) |
| UCF zone rectangles and waypoints | [`ros2_ws/src/lunabot_2425/config/zones/ucf_zones.yaml`](../ros2_ws/src/lunabot_2425/config/zones/ucf_zones.yaml) |
| Artemis zone rectangles and waypoints | [`ros2_ws/src/lunabot_2425/config/zones/artemis_zones.yaml`](../ros2_ws/src/lunabot_2425/config/zones/artemis_zones.yaml) |
| Zone publisher defaults | [`ros2_ws/src/luna_nav/config/zone_publisher.yaml`](../ros2_ws/src/luna_nav/config/zone_publisher.yaml) |

### Fiducial

| Area | File |
|------|------|
| Sim multi-camera params | [`ros2_ws/src/fiducial_localizer/params/multi_camera_sim.yaml`](../ros2_ws/src/fiducial_localizer/params/multi_camera_sim.yaml) |
| Hardware multi-camera params | [`ros2_ws/src/fiducial_localizer/params/multi_camera_hardware.yaml`](../ros2_ws/src/fiducial_localizer/params/multi_camera_hardware.yaml) |

### Mission supervisor (runtime params)

Set from launch (see `competition_sim.launch.py`) or override on the command line.

| Parameter | Role |
|-----------|------|
| `goal_frame_id` | Frame for `NavigateToPose` (default `world` for zone alignment in sim) |
| `map_warmup_s` | Stationary delay before first goal |
| `debug_ndjson_log` | Optional file and ingest logging (default **false**) |
| `debug_ndjson_path` | NDJSON path when debug logging is on (empty uses `/tmp/luna_mission_supervisor.ndjson`) |

### Depth FOV filter

Launched from `rtabmap_nav2_sim.launch.py`. Params include `ground_crop_bottom`, `debug_ndjson_log`, `debug_ndjson_path` (same pattern as mission supervisor; default logging off).

---

## Repository inventory

Team checklist for packages, launches, and doc accuracy. Update the **verification log** when you confirm a path on sim or hardware.

### Packages

| Package | Build | Purpose | Sim | Hardware | Primary entry |
|---------|-------|---------|-----|----------|----------------|
| `lunabot_2425` | ament_cmake | URDF, Gazebo bringup, zones YAML, robot_controllers | yes | yes | `competition_sim.launch.py`, `gz_bringup.launch.py`, `hardware_bringup.launch.py` |
| `luna_nav` | ament_python | Nav2 params, zone_publisher, frontier_explorer, mission_supervisor | yes | yes | Included from `competition_sim.launch.py`; `nav2_bringup_rtabmap.launch.py` |
| `luna_mapping` | ament_python | RTAB-Map sim and hardware launches, depth pipeline, RViz configs | yes | yes | `rtabmap_nav2_sim.launch.py`, `rtabmap_nav2_hardware.launch.py` |
| `luna_control` | ament_cmake | `LunaController` plugin for quad-swerve (used as `luna_cont`) | yes | yes | Loaded via `robot_controllers.yaml` |
| `quad_swerve_controller` | ament_cmake | Older ros2_control controller package (not loaded by current bringup) | no | no | See package README |
| `fiducial_localizer` | ament_python | Multi-camera ArUco, `/fiducial_pose` | yes | yes | `multi_camera_fiducial.launch.py` (included from mapping/hardware launches) |
| `depth_to_pointcloud` | ament_cmake | Depth image to cloud when mapping launch is off | yes | optional | `gz_bringup.launch.py` when `launch_mapping:=false` |
| `luna_ros2_worlds` | ament_cmake | SDF worlds and models (submodule) | yes | no | Included by `gz_bringup`; optional standalone arena launches |
| `twist_stamper` | ament_python | `/cmd_vel` to `/cmd_vel_stamped` | yes | yes | Submodule; `gz_bringup.launch.py` |
| `teleop_twist_keyboard` | ament_python | Keyboard teleop | dev | dev | Submodule; manual `ros2 run` |

### Launch files (classification)

| Launch | Classification | Notes |
|--------|----------------|-------|
| `lunabot_2425/competition_sim.launch.py` | **supported** | Full competition sim + autonomy |
| `lunabot_2425/gz_bringup.launch.py` | **supported** | Gazebo, bridges, controllers, optional mapping |
| `lunabot_2425/hardware_bringup.launch.py` | **supported** | Physical robot bringup; flags `launch_localizer:=true launch_rviz:=true` enable the smoke-test workflow |
| `lunabot_2425/rsp.launch.py` | **supported** | robot_state_publisher only |
| `luna_mapping/rtabmap_nav2_sim.launch.py` | **supported** | RTAB-Map + Nav2 for sim (included from gz_bringup) |
| `luna_mapping/rtabmap_nav2_hardware.launch.py` | **supported** | RTAB-Map + Nav2 for robot |
| `luna_mapping/d455_rtabmap.launch.py` | **dev** | D455-focused mapping bringup |
| `luna_nav/nav2_bringup_rtabmap.launch.py` | **dev / advanced** | Nav2-centric bringup; not the default sim stack |
| `luna_nav/rs_launch_custom.py` | **dev** | Custom RealSense launch helper |
| `fiducial_localizer/*.launch.py` | **supported** | Standalone fiducial runs |
| `luna_ros2_worlds/ucf_arena.launch.py` | **dev** | Arena-only helper |
| `luna_ros2_worlds/arena_b.launch.py` | **dev** | Arena-only helper |

### Submodules

| Path | Policy |
|------|--------|
| `teleop_twist_keyboard` | Upstream ROS teleop package; pin commits in team notes if you fork behavior |
| `twist_stamper` | Small relay; keep aligned with ROS distro expectations |
| `luna_ros2_worlds` | Worlds and meshes; document commit in release notes when arena geometry changes |

### Sim TF and odom (doc contract)

These points match the **TF frame hierarchy** and **Topic map** sections above:

- **`/odom` publisher** — In default competition sim with `use_rtabmap_odom:=true`, visual odometry from RTAB-Map publishes `odom`→`base_link`. `luna_cont` has `enable_odom_tf: false` so wheel odom does not publish that TF.
- **`map`→`odom`** — RTAB-Map publishes this correction. A static `world`→`map` transform is published by `competition_sim.launch.py` at the spawn pose so goals in `world` align with the map.

### Verification log

| Date | What ran | Environment | Result |
|------|-----------|---------------|--------|
| 2026-04-05 | `scripts/smoke_workspace.sh` + `colcon test --packages-select luna_mapping` | Docker `luna_ros:latest`, workspace bind-mounted | pass |

When you complete a full sim run on a clean checkout, add a row here or note it in your PR.

---

## Sources

- **Wavefront Frontier Detector:** Topiwala, Inani, Kathpal. "Frontier Based Exploration for Autonomous Robot." arXiv:1806.03581. https://arxiv.org/abs/1806.03581
- **Nav2:** https://navigation.ros.org/
- **RTAB-Map:** http://introlab.github.io/rtabmap/
