COMMANDS.md
================================

**Overview**
This file contains commands for launching Gazebo, RTAB-Map, Nav2, RViz,
teleoperation, and diagnostics.

Installation and build steps are documented separately in INSTALL.md.

**TERMINAL 1: Gazebo (Must run first)**
  ```bash
    dbld
    drun
  ```
*Inside container:*
  ```bash
      colcon build
      source /opt/ros/jazzy/setup.bash
      cd /ros2_ws && source install/setup.bash
      ros2 launch lunabot_2425 gz_bringup.launch.py
  ```
*Wait until Gazebo is fully up and the robot is spawned.*

**TERMINAL 2: RTAB-Map + Nav2 (Run only after Gazebo is up)**
  *New host terminal:*
  ```bash
    dbash
  ```
  *Inside container:*
  ```bash
    cd /ros2_ws && source install/setup.bash
    source /opt/ros/jazzy/setup.bash
    ros2 launch luna_mapping rtabmap_nav2_sim.launch.py
  ```                                                    
  *Wait until RTAB-Map and Nav2 messages appear.*
---

**TERMINAL 2 (Optional): Teleop**

  *One launch (gate + keyboard, same as `drive` topic wiring):*

  ```bash
    source /opt/ros/jazzy/setup.bash
    cd /ros2_ws && source install/setup.bash
    ros2 launch lunabot_2425 joy_teleop.launch.py
  ```

The `drive` alias publishes to `/teleop_cmd_vel_raw`; the gate forwards to `/cmd_vel` only while armed.

Controls:
  i = forward
  , = backward
  j = left
  l = right
  k = stop

Inside container:

  cd /ros2_ws
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch lunabot_2425 joy_teleop.launch.py

Terminal prints controller input by default (`joy_echo`). To silence printouts:

  ros2 launch lunabot_2425 joy_teleop.launch.py joy_echo:=false

- Left stick: forward/back + strafe (holonomic)
- Right stick X: rotate
- Hold A (enable) to drive; release to stop (deadman)
- Input map: see docs/CONTROLLER_MAP.md

---

## TERMINAL 5 (Optional): Diagnostics
(Run only after Terminals 1 and 2 are up)

Inside container:

  dbash
  cd /ros2_ws && source install/setup.bash

Commands:

  ros2 topic list
  ros2 topic hz /map
  ros2 topic hz /scan
  ros2 topic hz /odom
  ros2 run tf2_tools view_frames
  ros2 run tf2_ros tf2_echo odom base_link

---

## Main Topics (Reference)

  /map                              Occupancy grid (RTAB-Map)
  /scan                             2D scan from depth (Nav2 obstacle layer)
  /camera/camera/depth/color/points Point cloud (Nav2 voxel layer)
  /camera/camera/color/image_raw    RGB image
  /odom                             Odometry
  /cmd_vel                          Velocity commands (Nav2 or teleop)
  /local_costmap/costmap            Nav2 local costmap
  /global_costmap/costmap           Nav2 global costmap
  /plan                             Global path
  /local_plan                       Local path

---

## RTAB-Map + Nav2 Launch Arguments
(Optional)

  use_rtabmap_odom:=true    Use RTAB-Map visual odom instead of Gazebo odom
  use_fiducial_odom:=true   Enable fiducial marker localization
  launch_rviz:=true         Start RViz with the stack
  launch_nav2:=false        Mapping only (no Nav2)

Example:

  ros2 launch luna_mapping rtabmap_nav2_sim.launch.py use_rtabmap_odom:=true

---

## Zone Publisher (luna_nav)

Publishes the current zone name on `/current_zone` from `/odom`.

Build (from workspace root; if you hit permission errors on `install/`, use a custom install base):

  colcon build --packages-select luna_nav

  # Or with custom install path:
  # colcon build --build-base ./build_luna --install-base ./install_luna --packages-select luna_nav
  # then: source install_luna/setup.bash

  source install/setup.bash
  ros2 run luna_nav zone_publisher

In another terminal, echo the topic:

  ros2 topic echo /current_zone

Zone logic (first match): starting zone (0–2, 0–2), excavation (0–2.5, 0–11), obstacle (4.38–6.38, 0–11 minus 4.38–6.88×0–1.5), construction (7–12, 0–11), else "outside bounds". -->