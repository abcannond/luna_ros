How to Run the Luna ROS Simulation and Navigation Stack
========================================================

This guide walks you through getting the full system running: Gazebo (robot and depth camera), RTAB-Map (mapping), and Nav2 (navigation). Follow the steps in order.

How it works (big picture)
--------------------------

1. Gazebo runs the robot and a simulated RealSense D455 depth camera. It publishes RGB images, depth images, point clouds, and odometry.

2. RTAB-Map takes the RGB and depth images and does SLAM: it builds a map and publishes an occupancy grid on /map.

3. Depth processing turns the depth data into a 2D laser-style scan on /scan (so Nav2 can treat it like a lidar) and a 3D point cloud for the costmap.

4. Nav2 uses /map and the scan/point cloud to build costmaps, plan paths, and send velocity commands on /cmd_vel so the robot drives to a goal.

Basically: Gazebo sends camera data into RTAB-Map (map) and depth processing (obstacles), then Nav2 plans and drives the robot.

**Single command:** Run `ros2 launch lunabot_2425 gz_bringup.launch.py` — it starts Gazebo, waits ~18 s for the simulation to load, then automatically starts RTAB-Map, Nav2, and RViz (with Nav2 plugin, map, costmaps, and planner). No need for a second terminal. To run Gazebo only without mapping, use `launch_mapping:=false`.

**Design goal: sim reflects real hardware and arena.** The end goal is to run this stack on real hardware (robot + RealSense D455) in a real arena. The simulation is tuned to mirror that as closely as possible: same sensor model (D455), same topic names and pipeline (RTAB-Map + Nav2), same map resolution and Nav2 params, and an arena-style world (ucf_arena). Map and navigation quality are prioritized over sim runtime so that behavior in sim transfers reliably to the real robot.

Prerequisites: first-time setup
-------------------------------

Do this once (or when you clone on a new machine).

On your host machine (outside Docker):

  git clone git@github.com:abcannond/luna_ros.git
  cd luna_ros

  git submodule update --init --recursive

  docker build -t luna_ros:latest .

  ./run_ros_image.sh luna_ros:latest

Run docker build and run_ros_image.sh from the repo root. The trailing dot in docker build is required. run_ros_image.sh mounts your ./ros2_ws into the container as /ros2_ws.

Inside the container (first time, or after you change packages):

  cd /ros2_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install
  source install/setup.bash

You may see: not found: ".../quad_swerve_controller/.../local_setup.bash". That's harmless; you can ignore it. It doesn't affect Gazebo, RTAB-Map, or Nav2.

If you see "package 'twist_stamper' not found", build it: `colcon build --symlink-install --packages-select twist_stamper` then `source install/setup.bash`.

**Duplicate package names / CMakeCache:** `run_ros_image.sh` mounts `$(pwd)/ros2_ws` into the container as `/ros2_ws`. You must run `drun` (or `./run_ros_image.sh`) from the **repo root** (the luna_ros folder). If you run from a parent directory, the mounted workspace can contain the wrong layout (e.g. both `src` and `luna_ros/ros2_ws/src`), leading to duplicate package names and CMake errors. Fix: from inside the container, `cd /ros2_ws && rm -rf build install`, then `source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash`. Always run `drun` from repo root.

Single command: sim (Gazebo + RTAB-Map + Nav2 + RViz)
-----------------------------------------------------

**What it does:** Starts everything in one launch: Gazebo (robot, world, bridges), waits ~18 seconds for the simulation to load, then automatically starts RTAB-Map, Nav2, and RViz with the Nav2 plugin, occupancy grid, costmaps, and planner displays. One terminal only.

  cd /ros2_ws
  source install/setup.bash

  ros2 launch lunabot_2425 gz_bringup.launch.py

  # Artemis arena:
  ros2 launch lunabot_2425 gz_bringup.launch.py world:=artemis_arena

  # Gazebo only (no mapping, no RTAB-Map/Nav2):
  ros2 launch lunabot_2425 gz_bringup.launch.py launch_mapping:=false

Use the Nav2 Goal tool in RViz to send navigation goals.

Manual: Terminal 1 — Gazebo (simulation)
-------------------------------

What it does: Starts the Gazebo world and spawns the robot with the depth camera (and optional wheel-pod cameras). This has to be running before you start Terminal 2.

Where to run: In the same shell where you ran ./run_ros_image.sh, or in a new terminal after docker exec -it <CONTAINER_ID> /bin/bash.

  cd /ros2_ws
  source install/setup.bash

  UCF arena (default):
    ros2 launch lunabot_2425 gz_bringup.launch.py
  or explicitly:
    ros2 launch lunabot_2425 gz_bringup.launch.py world:=ucf_arena

  Artemis arena:
    ros2 launch lunabot_2425 gz_bringup.launch.py world:=artemis_arena

The robot spawn position is different per arena (UCF: near barriers; Artemis: from artemis_arenas branch). If the world fails to load with "model://... could not be resolved", build and install luna_ros2_worlds: `colcon build --symlink-install --packages-select luna_ros2_worlds` then `source install/setup.bash`.

What you should see: A Gazebo window opens with the robot in the world. Log messages about the robot spawning and camera topics being bridged. No errors about missing packages.

Leave this terminal running. Then move on to Terminal 2.

Manual: Terminal 2 — RTAB-Map + Nav2 (mapping and navigation)
----------------------------------------------------

What it does: Starts RTAB-Map (builds the map from the depth camera), depth-to-scan and depth-to-pointcloud, and the full Nav2 stack (costmaps, planner, controller). After this is up, you have a map and the robot can navigate to goals.

Where to run: Open a new terminal on your host. Attach to the same container:

  docker ps
  docker exec -it <CONTAINER_ID> /bin/bash

Replace <CONTAINER_ID> with the ID or name from docker ps (e.g. 69efb91d8358 or the name shown).

Inside the container:

  cd /ros2_ws
  source install/setup.bash

  ros2 launch luna_mapping rtabmap_nav2_sim.launch.py launch_rviz:=true

What you should see: Lines like rtabmap (X): Rate=1.00s... (RTAB-Map is processing frames). No "Could not convert rgb/depth msgs" errors. Nav2 nodes and camera/depth processing nodes starting.

Leave this terminal running. Once it's up, the system is ready for RViz and navigation goals.

Terminal 3 (optional): RViz — visualize and send goals
------------------------------------------------------

What it does: Opens RViz with a config that shows the map, costmaps, robot, and sensor data. You use the Nav2 Goal tool to click a goal; Nav2 will plan and drive the robot there.

Where to run: New host terminal, then exec into the container again.

  docker exec -it <CONTAINER_ID> /bin/bash
  cd /ros2_ws && source install/setup.bash

  rviz2 -d $(ros2 pkg prefix luna_mapping)/share/luna_mapping/config/rtabmap_nav2.rviz

Use the Nav2 Goal tool (in the toolbar) to send navigation goals. Don't use the generic Set Goal if you still have it; we use only the Nav2 Goal tool to avoid a topic conflict.

If you prefer to start RViz without the config, run rviz2 and add displays by hand: Map → /map, Local Costmap → /local_costmap/costmap, Global Costmap → /global_costmap/costmap, PointCloud2 → /camera/camera/depth/color/points, LaserScan → /scan, RobotModel → /robot_description, TF.

Using the RViz GUI: depth camera and Nav2 (in depth)
----------------------------------------------------

This section explains what you see in RViz when using the rtabmap_nav2.rviz config, what each display means, and what to look for to confirm things are working.

**What RViz is showing**

RViz is a 3D visualization tool. The config loads several "displays" that subscribe to ROS topics. Everything is drawn in a **Fixed Frame** (usually `map` or `odom`). The camera view can be rotated and zoomed with the mouse; the bottom-left "Views" panel lets you save and switch camera angles.

**Main displays and what to expect**

- **Map** (topic: `/map`)  
  RTAB-Map’s occupancy grid. You should see the explored environment as grey (unknown), white (free), and black (occupied). The map fills in as the robot moves and RTAB-Map processes depth images. If the map stays mostly grey, the robot may not be moving, RTAB-Map may not be receiving images, or there may be TF issues (check Terminal 2 for errors).

- **Global Costmap** (topic: `/global_costmap/costmap`)  
  Nav2’s world-wide cost layer. Similar to the map but used for global planning: green = low cost (preferred), red = high cost (obstacles). It should align with the map and obstacles. If it’s empty or wrong, check that `/scan` and/or the point cloud are publishing and that the costmap parameters in the Nav2 config match your sensor setup.

- **Local Costmap** (topic: `/local_costmap/costmap`)  
  Nav2’s local window around the robot. It updates quickly and shows nearby obstacles. You should see it move with the robot. If it doesn’t, TF (e.g. `map` → `odom` → `base_link`) may be broken or the costmap source topics may not be publishing.

- **PointCloud2** (topic: `/camera/camera/depth/color/points`)  
  3D points from the depth camera. You should see a cloud of points (often colorized by height or RGB) in front of the robot, updating as the robot or scene moves. This is the raw depth data; Nav2 can use it for the voxel layer. If you see nothing, check that Gazebo (Terminal 1) and the depth/pointcloud nodes (Terminal 2) are running and that the topic is published (`ros2 topic hz /camera/camera/depth/color/points`).

- **LaserScan** (topic: `/scan`)  
  2D slice from the depth camera (depth-to-scan). Nav2 uses this for the obstacle layer. You should see a fan or arc of points in the plane of the scan. If it’s missing, the depth-to-scan node may not be running or may be misconfigured (frame_id, topic names).

- **RobotModel** (topic: `/robot_description`)  
  The URDF of the robot. You should see the robot mesh and links. It should sit on the map at the robot’s current pose. If the robot appears in the wrong place or doesn’t move, check TF: `map` → `odom` → `base_link` and that `/robot_description` is published.

- **TF**  
  Shows coordinate frames as axes. Useful to verify that `map`, `odom`, and `base_link` (and sensor frames) are connected. If you see "No transform from X to Y" or disconnected trees, fix TF before trusting the map or Nav2.

**Nav2: sending goals and what to look for**

1. **Tool**: In the toolbar, use **"Nav2 Goal"** (or "Goal Pose"). Do **not** use the generic "Set Goal" (that uses a different action and can cause conflicts).

2. **Sending a goal**: Click "Nav2 Goal", then click in the 3D view to set the goal position; often you can drag to set orientation (depends on the tool). Nav2 will plan a path and the robot will drive there.

3. **What you should see**:
   - A **path** (often on `/plan` or `/local_plan`) drawn as a line from the robot toward the goal.
   - The **local costmap** updating around the robot.
   - The **robot** (RobotModel) moving on the map as the robot drives.
   - In the terminal running Nav2, you may see goal received, plan computed, and controller feedback.

4. **If the robot doesn’t move**: Check that `/cmd_vel` is being published (Nav2 controller), that the drive controller (luna_cont) is active and subscribed to the right topic, and that TF is correct. Use `ros2 topic echo /cmd_vel` and check for TF errors in RViz or with `ros2 run tf2_ros tf2_echo map base_link`.

5. **If the path is bad or the robot gets stuck**: The costmaps may be too conservative or too aggressive (inflation, obstacle layer). Tune the Nav2 costmap and controller parameters. Ensure `/scan` and the point cloud are correct so obstacles are detected.

**Depth camera–specific notes**

- The **point cloud** is in the camera’s optical frame; RViz (and Nav2) use TF to show it in the fixed frame. If the cloud appears in the wrong place, check camera frame_id and TF (e.g. `base_link` → `camera_link` → `camera_depth_optical_frame`).
- **Range**: Simulated depth has a limited range; distant walls may not appear in the point cloud or scan. That’s normal.
- **Rate**: If the cloud or scan updates slowly, check Gazebo and the bridge; in sim, GPU and bridge rate can limit throughput.

**Quick sanity checklist**

- Map: fills in when the robot moves; not stuck all grey.
- PointCloud2 and LaserScan: visible and updating.
- RobotModel: at the correct pose and moving with the robot.
- Global/Local Costmap: aligned with the map and obstacles.
- Nav2 Goal: path appears and robot moves (or you see clear errors in the Nav2 terminal).

For more details on topics and frames, see "Main topics" and "Diagnostics" in this guide and in Commands to run and what.txt.

Terminal 4 (optional): Drive the robot manually
---------------------------------------------------------

What it does: Lets you drive the robot with the keyboard. One terminal: source, then run drive (alias starts twist_stamper in background and teleop in foreground).

Where to run: One terminal in the container (docker exec -it \<CONTAINER_ID\> /bin/bash, or dbash).

  cd /ros2_ws && source install/setup.bash
  drive

Controls: i = forward, comma = backward, j = left, l = right, k = stop. q/z and w/x change speed.

Terminal 5 (optional): Monitoring and diagnostics
--------------------------------------------------

What it does: Lets you check that topics and TF are working. Run these after Terminal 1 and Terminal 2 have been up for a bit (otherwise you'll see "topic does not appear to be published yet" or TF errors).

  docker exec -it <CONTAINER_ID> /bin/bash
  cd /ros2_ws && source install/setup.bash

  ros2 topic list
  ros2 topic hz /map
  ros2 topic hz /scan
  ros2 topic hz /camera/camera/depth/color/points
  ros2 topic hz /odom
  ros2 run tf2_tools view_frames
  ros2 run tf2_ros tf2_echo odom base_link

view_frames creates a PDF in the current directory. Ctrl+C to stop the hz commands.

Launch order summary
--------------------

**Single command:** `ros2 launch lunabot_2425 gz_bringup.launch.py` — does steps 1–3 automatically.

**Manual (run mapping separately):**
1. Terminal 1: Gazebo — wait until the world and robot are up.
2. Terminal 2: RTAB-Map + Nav2 (launch_rviz:=true) — wait until you see RTAB-Map and Nav2 running.
3. Terminal 3 (optional): RViz only — if you used launch_rviz:=false in step 2.
4. Terminal 4 (optional): Teleop — drive manually.
5. Terminal 5 (optional): Diagnostics — only after 1 and 2 are running.

Customizing the RTAB-Map + Nav2 launch
---------------------------------------

You can pass arguments when launching Terminal 2:

  Use RTAB-Map visual odometry instead of Gazebo odom:
  ros2 launch luna_mapping rtabmap_nav2_sim.launch.py use_rtabmap_odom:=true


  Start RViz automatically with the stack:
  ros2 launch luna_mapping rtabmap_nav2_sim.launch.py launch_rviz:=true

  Mapping only (no Nav2):
  ros2 launch luna_mapping rtabmap_nav2_sim.launch.py launch_nav2:=false

World selection (Gazebo, Terminal 1)
------------------------------------

Both arenas use the same launch file; only the world argument changes:

  UCF arena (default):
  ros2 launch lunabot_2425 gz_bringup.launch.py
  or: ros2 launch lunabot_2425 gz_bringup.launch.py world:=ucf_arena

  Artemis arena:
  ros2 launch lunabot_2425 gz_bringup.launch.py world:=artemis_arena

The robot spawn position is different per arena (UCF: near barriers; Artemis: from artemis_arenas branch). If the world fails to load with "model://... could not be resolved", build and install luna_ros2_worlds: colcon build --symlink-install --packages-select luna_ros2_worlds then source install/setup.bash.

Troubleshooting
---------------

"not found: .../quad_swerve_controller/.../local_setup.bash"
  Harmless. The workspace still references the ignored quad_swerve_controller package. It doesn't affect Gazebo, RTAB-Map, or Nav2; you can ignore it.

RViz: "could not create publisher ... rt/goal_pose with incompatible type"
  Use only the Nav2 Goal tool. If you loaded an old config, reload rtabmap_nav2.rviz or remove the extra Set Goal tool so only the Nav2 GoalTool is there.

"Could not find a connection between 'odom' and 'base_link'" or "Tf has two or more unconnected trees"
  Usually means you ran diagnostics before the stack was fully up. Start Terminal 1 (Gazebo) and Terminal 2 (RTAB-Map + Nav2), wait until you see RTAB-Map and costmap activity, then run tf2_echo or view_frames.

"topic [/map] does not appear to be published yet"
  Terminal 2 (RTAB-Map + Nav2) isn't running yet, or RTAB-Map hasn't started publishing. Launch Terminal 2 and wait a few seconds before checking topics.

General checks: ros2 topic list, ros2 topic hz <topic_name>, ros2 node list, ros2 node info <node_name>, ros2 run tf2_tools view_frames.

Common bugs and workflow
-------------------------
**TF tree (sim):** `map` → `odom` → `base_link`. The sim launch publishes a static odom → base_link (identity) so the tree is connected. map → odom comes from RTAB-Map (SLAM).
**Robot jerking / Nav2 "Localization inactive":** With RTAB-Map SLAM, the Nav2 panel shows "Localization inactive" because we don't run AMCL or lifecycle_manager_localization—localization comes from RTAB-Map (map→odom TF). That indicator can be ignored; Nav2 uses the TF chain (map→odom→base_link) for pose. If the robot jerks or doesn't follow goals, check TF: `ros2 run tf2_ros tf2_echo map base_link`. Start Gazebo first, then the stack.
**Velocity chain:** Nav2 → velocity_smoother → `/cmd_vel` → gz_bridge → Gazebo. Only one source should drive the robot.


Troubleshooting (in depth)
---------------------------

**Robot doesn’t move when driving (teleop)**  
- `ros2 topic list | grep cmd_vel` — you should see /cmd_vel and controller topics.  
- Find which topic the controller subscribes to: `ros2 node list`, then `ros2 node info /controller_manager` (or /model/mooncake/controller_manager); under Subscribers, find the cmd_vel topic. Alternatively `ros2 topic list -t | grep TwistStamped` and `ros2 topic info <topic> --verbose` to see which has Subscription count 1.  
- Confirm /cmd_vel is published when you press keys: `ros2 topic echo /cmd_vel`.  
- `ros2 control list_controllers -c /controller_manager` — luna_cont should show "active". If not, restart the sim and wait for "joint_state_broadcaster active — starting luna_cont...".

**No map / "map frame does not exist"**  
- Gazebo (Terminal 1) must run first; RTAB-Map needs camera data from the bridge.  
- After git pull, rebuild: `colcon build --symlink-install --packages-select luna_mapping`, then `source install/setup.bash`. Restart Gazebo and RTAB-Map+Nav2.  
- Verify camera topics (both launches running): `ros2 topic hz /camera/camera/color/image_raw`, `.../image_raw_fixed`, `.../depth/image_rect_raw_fixed`. If *_fixed topics show 0 Hz, the fixer nodes may not be receiving from the bridge.

**Blocky or fragmented costmap**  
- The costmap combines the static map and the obstacle layer (/scan). If map→base_link is missing or stale, /scan is projected in the wrong place. Check: `ros2 run tf2_ros tf2_echo map odom` and `ros2 run tf2_ros tf2_echo map base_link`.  
- Start Gazebo first, then RTAB-Map+Nav2.  
- The global costmap uses a small observation_source_delay (e.g. 0.2 s) so TF is available when transforming /scan to map; this can reduce misaligned patches.

**Duplicate nodes**  
- Run the RTAB-Map+Nav2 launch in **only one** terminal. If you run it twice, you get duplicate nodes and "nodes share an exact name" warnings.

Quick reference (commands only)
-------------------------------

Single command: ros2 launch lunabot_2425 gz_bringup.launch.py

Manual (separate mapping):
Terminal 1: ros2 launch lunabot_2425 gz_bringup.launch.py
Terminal 2: ros2 launch luna_mapping rtabmap_nav2_sim.launch.py launch_rviz:=true
Terminal 3: rviz2 -d $(ros2 pkg prefix luna_mapping)/share/luna_mapping/config/rtabmap_nav2.rviz
Terminal 4: dbash, then cd /ros2_ws && source install/setup.bash, then drive
Terminal 5: ros2 topic hz /map (and other diagnostics above)

For Terminals 2–5: open a new host terminal, run docker ps, then docker exec -it <CONTAINER_ID> /bin/bash, then cd /ros2_ws && source install/setup.bash, then the command above.
