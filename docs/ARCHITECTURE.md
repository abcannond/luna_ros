# System Architecture

This document describes the software architecture of the WPI Lunabotics autonomous navigation and competition stack.

**Related docs:** [AUTONOMY.md](AUTONOMY.md), [STACK_GUIDE.md](STACK_GUIDE.md)

---

## Package Responsibility Matrix

| Package | Responsibility |
|---------|---------------|
| `lunabot_2425` | URDF/xacro, Gazebo worlds/bridges, launch files (sim + hardware), RViz configs, zone YAML definitions |
| `luna_control` | `ros2_control` controller plugin for quad-swerve drivetrain, odometry |
| `luna_mapping` | RTAB-Map SLAM, depth processing pipeline, frame fixers, Nav2 integration launch |
| `luna_nav` | Nav2 parameters, zone publisher, frontier explorer (WFD), mission supervisor FSM |
| `fiducial_localizer` | ArUco marker detection from multi-camera setup, global pose in starting zone |
| `depth_to_pointcloud` | Depth image to PointCloud2 conversion (standalone, used when mapping is off) |
| `luna_ros2_worlds` | Gazebo Harmonic SDF worlds and models (submodule) |
| `twist_stamper` | Twist -> TwistStamped relay for controller compatibility |
| `teleop_twist_keyboard` | Keyboard teleoperation (submodule) |

---

## Data Flow

```
Gazebo Sim / Hardware Sensors
         |
         v
+-------------------+     +---------------------+
| RealSense D455    |     | Fiducial Cameras    |
| (RGB + Depth)     |     | (4x USB webcams)    |
+--------+----------+     +----------+----------+
         |                            |
    Frame fixers               fiducial_localizer
         |                            |
         v                            v
+-------------------+     +---------------------+
| RTAB-Map SLAM     |     | /fiducial_pose      |
| -> /map, TF       |     | -> map->odom TF     |
+--------+----------+     +---------------------+
         |
    +----+----+
    |         |
    v         v
+-------+ +----------+
| Nav2  | | /map     |
| Stack | | topic    |
+---+---+ +----+-----+
    |          |
    v          v
+--------+ +---------------------+
| Robot  | | Frontier Explorer   |
| Motion | | (WFD on /map)       |
+--------+ +----------+----------+
                       |
                       v
              +------------------+
              | Mission          |
              | Supervisor (FSM) |
              +--------+---------+
                       |
                       v
              +------------------+
              | Zone Publisher   |
              | -> /current_zone |
              +------------------+
```

---

## Topic Map (Key Topics)

| Topic | Type | Publisher | Subscribers |
|-------|------|----------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | rtabmap | Nav2, frontier_explorer |
| `/odom` | `nav_msgs/Odometry` | luna_controller | rtabmap, Nav2 |
| `/current_zone` | `std_msgs/String` | zone_publisher | mission_supervisor |
| `/mission_state` | `std_msgs/String` | mission_supervisor | (logging/UI) |
| `/mission_dashboard` | `std_msgs/String` | mission_supervisor | (logging/UI) |
| `/exploration_status` | `std_msgs/String` | frontier_explorer | mission_supervisor |
| `/frontier_goals` | `geometry_msgs/PoseArray` | frontier_explorer | (RViz) |
| `/frontier_markers` | `visualization_msgs/MarkerArray` | frontier_explorer | (RViz) |
| `/zone_markers` | `visualization_msgs/MarkerArray` | zone_publisher | (RViz) |
| `/fiducial_pose` | `geometry_msgs/PoseStamped` | fiducial_localizer | mission_supervisor |
| `/fiducial_status` | `visualization_msgs/Marker` | fiducial_localizer | (RViz) |

---

## TF Frame Hierarchy

```
map
 └── odom                    (published by luna_controller or RTAB-Map)
      └── base_link          (robot center)
           ├── depth_camera_link
           │    ├── camera_color_optical_frame
           │    └── camera_depth_optical_frame
           ├── front_left_camera_link
           │    └── front_left_camera_link_optical
           ├── front_right_camera_link
           │    └── front_right_camera_link_optical
           ├── back_left_camera_link
           │    └── back_left_camera_link_optical
           └── back_right_camera_link
                └── back_right_camera_link_optical
```

---

## Competition Sim Launch Hierarchy

```
competition_sim.launch.py
 ├── gz_bringup.launch.py (world, controllers, bridges)
 │    ├── rsp.launch.py (robot_state_publisher)
 │    ├── Gazebo Harmonic (world SDF)
 │    ├── ros_gz_bridge (topics + TF)
 │    ├── twist_stamper
 │    ├── controller spawners (joint_state_broadcaster, luna_cont)
 │    └── rtabmap_nav2_sim.launch.py (delayed 25s)
 │         ├── RTAB-Map SLAM
 │         ├── Frame fixers
 │         ├── Depth processing
 │         ├── Nav2 stack (full)
 │         ├── RViz (competition_sim.rviz)
 │         └── fiducial_localizer (optional, launch_fiducials:=true)
 ├── zone_publisher (delayed 30s)
 ├── frontier_explorer (delayed 30s, starts disabled)
 └── mission_supervisor (delayed 30s)
```

---

## Sources

- **Wavefront Frontier Detector:** Topiwala, Inani, Kathpal. "Frontier Based Exploration for Autonomous Robot." arXiv:1806.03581. https://arxiv.org/abs/1806.03581
- **Lunabotics Rules:** 2025 Lunabotics Guidebook. https://fsi.ucf.edu/wp-content/uploads/sites/4/2025/01/lunaboticsguidebook-2025-1.pdf
- **Nav2:** https://navigation.ros.org/
- **RTAB-Map:** http://introlab.github.io/rtabmap/
