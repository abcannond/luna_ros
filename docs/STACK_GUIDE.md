# Stack guide

Features, design choices, launching, and troubleshooting for the Luna ROS stack.

**See also:** [HARDWARE.md](HARDWARE.md) (running on real hardware)

---

## Launching simulation

### Quick start

Inside the container (after `./run_ros_image.sh` or `dbash`):

```bash
cd /ros2_ws && source install/setup.bash
ros2 launch lunabot_2425 gz_bringup.launch.py
```

One launch runs Gazebo, RTAB-Map, Nav2, and RViz. Use the **Nav2 Goal** tool in RViz to send goals.

**Variants:**

| Goal | Command |
|------|---------|
| Gazebo only (no mapping, no Nav2) | `ros2 launch lunabot_2425 gz_bringup.launch.py launch_mapping:=false` |
| Artemis arena instead of default UCF | `ros2 launch lunabot_2425 gz_bringup.launch.py world:=artemis_arena` |

### What runs (in order)

1. **Gazebo** — Robot and simulated RealSense D455. Publishes RGB, depth, point cloud, and odometry under `camera/camera`.
2. **RTAB-Map** — SLAM from RGB + depth. Publishes `/map` (occupancy grid) and the `map`→`odom` transform.
3. **Depth processing** — Depth → 2D scan (`/scan`) and point cloud for Nav2's costmaps.
4. **Nav2** — Builds costmaps, plans paths, sends velocity commands on `/cmd_vel`.

### Manual launch (two terminals)

Use this to restart mapping without restarting the sim.

**Terminal 1 — Gazebo** (must run first):

```bash
cd /ros2_ws && source install/setup.bash
ros2 launch lunabot_2425 gz_bringup.launch.py
```

Add `world:=ucf_arena` or `world:=artemis_arena` to pick the arena.

**Terminal 2 — RTAB-Map + Nav2** (open a new shell with `dbash`):

```bash
cd /ros2_ws && source install/setup.bash
ros2 launch luna_mapping rtabmap_nav2_sim.launch.py launch_rviz:=true
```

### Optional terminals

**RViz only** (if started with `launch_rviz:=false`):

```bash
rviz2 -d $(ros2 pkg prefix luna_mapping)/share/luna_mapping/config/rtabmap_nav2.rviz
```

**Keyboard teleop:**

```bash
cd /ros2_ws && source install/setup.bash
drive
```

Controls: **i** forward, **,** back, **j** / **l** turn, **k** stop.

**Diagnostics:**

```bash
ros2 topic list
ros2 topic hz /map
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `launch_rviz:=true` | false | Start RViz with the stack. |
| `launch_nav2:=false` | true | Run only mapping; no Nav2. |
| `use_rtabmap_odom:=true` | false | Use RTAB-Map visual odometry for `odom`→`base_link`. |
| `clear_costmap_on_start:=false` | true | Disable the one-time costmap clear after Nav2 is up. |

### World selection

| World | Launch argument |
|-------|-----------------|
| UCF arena | default, or `world:=ucf_arena` |
| Artemis arena | `world:=artemis_arena` |

### Topic reference

| Topic | Description |
|-------|-------------|
| `/map` | Occupancy grid from RTAB-Map |
| `/scan` | 2D scan from depth (used by Nav2) |
| `/camera/camera/depth/color/points` | Point cloud (Nav2 voxel layer) |
| `/odom` | Odometry |
| `/cmd_vel` | Velocity commands (Nav2 or teleop) |
| `/local_costmap/costmap`, `/global_costmap/costmap` | Nav2 costmaps |
| `/plan`, `/local_plan` | Global and local path |

---

## Overview

The same pipeline runs in **simulation** (Gazebo + simulated RealSense D455) and on **hardware** (Jetson, RealSense D455, 4× Nexigo N980P). Only the source of camera data, time, and TF (transform) frames changes.

**Data flow:** Camera data goes into RTAB-Map (which builds the map) and into depth processing (which produces a 2D scan and a point cloud). Nav2 uses the map plus that scan and point cloud to build costmaps, plan paths, and send velocity commands. So: *camera → map + obstacles → Nav2 → robot.*

- **In sim:** Gazebo provides the robot and depth camera. The mapping launch publishes static TFs (e.g. `odom`→`base_link`, `base_link`→camera frames) when `sim:=true`, so the TF tree is complete without a separate sim package.
- **On hardware:** `hardware_bringup.launch.py` starts the RealSense and fiducial cams and publishes `base_link`→`camera_link`. The same mapping launch runs with `sim:=false` and `use_sim_time:=false`. Your robot driver must publish `/odom` and the `odom`→`base_link` transform.

**Why it’s set up this way:** Topic names, frame names, and Nav2/RTAB-Map params are shared between sim and hardware. That way tuning and behavior transfer to the real robot without maintaining two stacks.

---

## Pipeline and features

Each part of the stack is described below. Technical terms (TF, costmap, voxel layer, etc.) are used as in ROS 2 / Nav2; the first time something matters for “why we did it,” we spell it out briefly.

### Gazebo

**Role:** Run the robot and a simulated RealSense D455, and bridge their data into ROS 2.

Gazebo spawns the model and sensors, then the bridge publishes RGB images, depth images, `camera_info`, and (in sim) odometry on topics under the `camera/camera` namespace. That namespace matches what the RealSense driver uses on hardware, so the rest of the stack can subscribe to the same topic names in both environments.

### Frame fixers

**Role:** Make image and camera_info frame_ids consistent so RTAB-Map and Nav2 don’t mis-associate or drop data.

Gazebo (and some drivers) put sensor-specific frame_ids in the image and camera_info headers (e.g. a Gazebo frame name). RTAB-Map and Nav2 expect stable optical frame names like `camera_depth_optical_frame`. The launch runs **camera_info_fixer** and **image_frame_fixer** nodes that subscribe to the raw topics and republish to `*_fixed` topics with the correct frame_id. Downstream nodes use the `*_fixed` topics, so the TF tree and frame_ids line up.

### Depth FOV filter

**Role:** Narrow the effective depth field of view and remove the floor from obstacle data so the costmap doesn’t get a “trail” from the ground.

The filter can mask regions (e.g. to exclude fiducial camera views) and applies a **ground crop**: it zeros out the bottom fraction of the depth image (`ground_crop_bottom`). That keeps the floor out of the point cloud and thus out of the costmap voxel layer. The *filtered* depth is used for the laserscan and point cloud that feed Nav2. RTAB-Map, however, subscribes to *raw* depth so mapping isn’t over-filtered and stays reliable.

### Depth → scan and point cloud

**Role:** Turn depth into the 2D and 3D representations Nav2 expects.

- **depthimage_to_laserscan** produces `/scan` (2D slice). Nav2 uses it for the obstacle layer.
- **depth_image_proc** (point_cloud_xyzrgb) produces `/camera/camera/depth/color/points`.
- **height_filter_pointcloud** subscribes to that cloud, transforms to `base_link`, and drops points with `z` below `min_height` (default 0.02 m) or above `max_height` (default 2.0 m). It publishes `/camera/camera/depth/color/points_ground_filtered`. Nav2’s local costmap **voxel layer** uses this filtered topic.

Height-based filtering removes floor in 3D instead of cropping image rows, so low obstacles (rocks, crates) that sit above the ground plane are kept. Tune `min_height` / `max_height` in the launch (height_filter_pointcloud params) if needed.

### RTAB-Map

**Role:** Build the occupancy grid and the map→odom part of the TF tree.

RTAB-Map does SLAM from RGB and depth. It publishes `/map` (occupancy grid) and the `map`→`odom` transform. Optionally, the **rgbd_odometry** node can publish `odom`→`base_link` when you set `use_rtabmap_odom:=true`; that’s “visual odometry” from the camera. Grid and noise parameters (e.g. `Grid/RangeMin`, `NoiseFilteringMinNeighbors`, `MinClusterSize`) are tuned so the map is stable and small obstacles (e.g. rocks) are kept without clutter; see [Costmap trailing and noise](#costmap-trailing-and-noise) for how that ties into the costmap.

### Nav2

**Role:** Plan paths and send velocity commands using the map and live obstacle data.

Nav2 maintains two costmaps:

- **Global costmap:** Static layer (RTAB-Map’s `/map`) plus inflation. Used for global path planning.
- **Local costmap:** Static layer plus a **voxel layer** (fed by the point cloud) plus inflation. The voxel layer is tuned (e.g. `mark_threshold`, range limits, short persistence) and is fed from the *ground-cropped* point cloud so the floor doesn’t paint cost and we limit “trailing” when the robot turns.

**Obstacle avoidance (no collisions):** Costmaps and the DWB controller are tuned so the robot does not hit rocks or other obstacles:
- **Safety margin:** `robot_radius` is set larger than the physical robot (0.45 m) so the planner and controller keep extra clearance from obstacles.
- **Inflation:** `inflation_radius` and `cost_scaling_factor` give a clear keep-out zone around obstacles while keeping free space low cost so the planner still has passable regions.
- **Controller:** **BaseObstacle** scale is high (0.28) so the robot strongly avoids cost. Max speed is limited (0.32 m/s) so the robot has time to react and stop or deviate before hitting obstacles.
- **Vision:** Ground crop is reduced (0.06) so low rocks appear in the voxel layer; local costmap updates at 8 Hz so obstacles show up quickly. See [Tuning and change log](#tuning-and-change-log) for all values and reasons.

There is also an optional **clear costmap on start**: shortly after Nav2 is up, the launch can call the clear services once so each run starts with a clean in-memory costmap. Default is on; set `clear_costmap_on_start:=false` to disable.

### Velocity chain

**Role:** Single path from planner to robot so only one source drives the robot.

Commands flow: Nav2 → velocity_smoother → `/cmd_vel` → robot (Gazebo bridge in sim, or the controller on hardware). Only one source — either Nav2 or teleop — should publish to `/cmd_vel` at a time to avoid fighting commands.

### Zone publisher (luna_nav)

**Role:** Optional node that reports the current zone name from odometry.

The zone publisher subscribes to **`/odom`** by default (same source as Nav2; set `odom_topic` parameter to remap). Zone bounds are **configurable via parameters** (defaults match UCF/Artemis-style arena). Run: `ros2 run luna_nav zone_publisher`. To override bounds or odom topic: `--ros-args --params-file $(ros2 pkg prefix luna_nav)/share/luna_nav/config/zone_publisher.yaml`.

---

## Design decisions

These choices keep the stack consistent between sim and hardware and avoid common pitfalls.

| Decision | Reason |
|----------|--------|
| **Same topic names in sim and hardware** | One set of launch files and configs; only `sim` and `use_sim_time` (and which launch starts the cameras) change. |
| **Static TFs only when `sim:=true`** | In sim, the mapping launch can publish the full tree (odom→base_link, base_link→camera). On hardware, the robot driver and `hardware_bringup` provide those; we don’t start sim-only TF nodes. |
| **Ground-cropped depth for costmap, raw for RTAB-Map** | Cropping removes the floor from the voxel layer and reduces costmap trailing. RTAB-Map keeps raw depth so we don’t over-filter and lose map quality. |
| **Voxel layer on local costmap** | We need live obstacles (e.g. rocks) in the costmap. Tuning (mark_threshold, range, persistence, ground crop) keeps that while limiting trailing and noise. |
| **Visual odometry option** | When the robot gets stuck (e.g. on a rock), wheel odometry can keep integrating and the map/planner drift. With RTAB-Map visual odom, the camera drives `odom`→`base_link`; when the robot is stuck, the camera doesn’t move so odom stops and the map stays aligned. See [Odom and map alignment when stuck](#odom-and-map-alignment-when-stuck). |
| **Fixed timers for mapping/Nav2 in gz_bringup** | We wait a fixed delay (~25 s) then start the mapping launch so the sim and bridge are ready. No dependency on a specific “robot ready” topic, which keeps the launch simple and predictable. |

---

## Hardware and the stack

On hardware you run two terminals:

- **Terminal 1:** `hardware_bringup.launch.py` — RealSense, 4 fiducial cams, and static TF `base_link`→`camera_link`.
- **Terminal 2:** `rtabmap_nav2_hardware.launch.py` — same mapping launch as in sim, with `use_sim_time:=false` and `sim:=false`, so no sim-only TFs are started and the stack uses real time and your robot’s odometry.

Topic names and the depth/scan/point-cloud pipeline are unchanged. Full hardware steps and device setup: [HARDWARE.md](HARDWARE.md).

---

## Issues and debugging

Each subsection states the problem, cause (where it helps), and what to do. Config file names are included so you can adjust or confirm settings.

### Costmap trailing and noise

**What you see:** When the robot turns, the local costmap shows a “trail” of cost behind it. You may also see phantom obstacles or flicker from single points or floor noise.

**Why it happens:** The local costmap’s voxel layer is fed by the depth point cloud. The costmap is drawn in the map frame using the TF chain (map→odom→base_link). If that chain lags when RTAB-Map updates (e.g. during a turn), the costmap is rendered in a slightly wrong pose, so cost appears to trail. In addition, floor returns and sensor noise can add spurious cost.

**What we did:**

- **Costmap:** Set `transform_tolerance: 1.0` so the costmap doesn’t drop updates when TF is briefly late.
- **Voxel layer:** Use `mark_threshold: 2`, shorter range (`obstacle_max_range: 2.5`), and `observation_persistence: 0.2`. Feed the voxel layer from the *height-filtered* point cloud (`/camera/camera/depth/color/points_ground_filtered`) so the floor is removed in 3D (points with z &lt; min_height in base_link are dropped) and rocks/crates are kept. A small image crop (`ground_crop_bottom: 0.05`) still runs before the point cloud for the bottom strip.
- **RTAB-Map:** Set `Rtabmap/DetectionRate: 2.0` so map (and thus map→odom) updates more often when turning.
- **Noise:** In the costmap, `mark_threshold: 2` filters single-point voxels. In RTAB-Map’s grid, `Grid/RangeMin: 0.5`, `NoiseFilteringMinNeighbors: 6`, and `MinClusterSize: 10` keep the map clean while retaining real obstacles.

**Where to look:** `luna_nav/config/nav2_rtabmap_params.yaml` (local costmap, voxel); `luna_mapping/config/rtabmap_sim.yaml` (grid); ground crop is in `rtabmap_nav2_sim.launch.py` (depth_fov_filter params).

---

### Odom and map alignment when stuck

**What you see:** The robot hits a rock and gets stuck in the sim, but the map and planner still think it’s moving. Pose and costmap drift away from the true state.

**Why it happens:** Odometry is computed from wheel motion. When the body is stuck but the wheels spin (e.g. slip), the odometry keeps integrating, so the belief pose drifts.

**What to do (recommended when stuck/slip is an issue):**

1. Launch the mapping stack with `use_rtabmap_odom:=true` so RTAB-Map’s rgbd_odometry node publishes `odom`→`base_link`.
2. Use a controller config that *does not* publish `odom`→`base_link` (e.g. copy `robot_controllers_visual_odom.yaml` over `robot_controllers.yaml` in `lunabot_2425/config/`, then rebuild). Otherwise you’d have two sources for that transform.
3. Then only the camera drives `odom`→`base_link`. When the robot is stuck, the camera doesn’t move, so visual odom stops and the map stays aligned.

Other options: rely on loop closures for global map consistency (they don’t fix live drift while stuck); on hardware, use good wheel odom or visual odom and optionally fiducials. The main goal remains avoiding collisions; this is the fallback when the robot does get stuck.

---

### No map / “map frame does not exist”

**What you see:** No `/map` or the map frame is missing in RViz or TF.

**What to check:**

- Start **Gazebo first**. RTAB-Map needs camera data from the bridge; if Gazebo isn’t running, there’s no input.
- After changing code or config: run `colcon build --symlink-install --packages-select luna_mapping` (and `luna_nav` if you changed Nav2 params), then `source install/setup.bash`, and restart both Gazebo and the mapping launch.
- Confirm camera topics are publishing: e.g. `ros2 topic hz /camera/camera/color/image_raw`, `.../image_raw_fixed`, `.../depth/image_rect_raw_fixed`. If the `*_fixed` topics are 0 Hz, the frame fixer nodes may not be receiving from the bridge.
- Ensure the mapping launch (or gz_bringup with `launch_mapping:=true`) is actually started so RTAB-Map is running.

---

### depth_fov_filter crash / “array (not bytes)” error

**What you see:** The node exits with code 1 or the error “can only assign array (not bytes) to array slice.”

**Why it happens:** In ROS 2, the `data` field of a depth image can be either `bytes` or `array.array`. In-place slice assignment behaved differently for the two types, which led to the crash.

**What we did:** The node now does all slice updates in a local bytearray and sets `out.data = bytes(buf)` once before publishing, so it works for both types.

---

### Clear costmap on start

**What it is:** By default, shortly after Nav2 is up, the launch triggers a one-time clear of both local and global costmaps so each run starts with a clean in-memory costmap. RTAB-Map already starts with a fresh map (e.g. `--delete_db_on_start`); this only resets the costmaps.

**What to do:** To disable the one-time clear, pass `clear_costmap_on_start:=false` when starting the mapping launch.

---

### TF and connectivity

**Sim TF tree:** `map` → `odom` → `base_link`. RTAB-Map publishes `map`→`odom`; the mapping launch (when `sim:=true`) publishes a static `odom`→`base_link` (identity). So the tree is connected.

**“No transform” or “unconnected trees”:** Usually this means diagnostics (e.g. `tf2_echo`, `view_frames`) were run before the stack was fully up. Start Gazebo, then the mapping launch; wait until you see RTAB-Map and costmap activity, then run TF diagnostics.

**Nav2 “Localization inactive”:** With RTAB-Map SLAM we don’t run AMCL. Pose comes from the TF chain (map→odom→base_link). The “Localization inactive” indicator in the Nav2 panel is expected and can be ignored.

---

### Blocky or wrong costmap

**What you see:** The costmap looks blocky, misaligned, or wrong.

**What to check:** The costmap combines the static map with live obstacles (scan, point cloud). If the transform from `map` to `base_link` is missing or stale, those obstacles are projected into the wrong place. Run `ros2 run tf2_ros tf2_echo map base_link` and ensure the chain is publishing. Start Gazebo first, then the mapping stack. For tuning (trailing, noise), see [Costmap trailing and noise](#costmap-trailing-and-noise).

---

### Robot doesn’t move (teleop or Nav2)

**What you see:** You send a goal or press teleop keys but the robot doesn’t move.

**What to check:**

- **Teleop:** Confirm that `/cmd_vel` is published when you press keys (`ros2 topic echo /cmd_vel`). Ensure only one source (Nav2 or teleop) is publishing to `/cmd_vel`.
- **Controllers:** Run `ros2 control list_controllers -c /controller_manager` (or the model’s controller_manager). The `luna_cont` controller should be active. If not, restart the sim and wait until you see “joint_state_broadcaster active — starting luna_cont…”.

---

### Duplicate nodes / “nodes share an exact name”

**What you see:** Warnings about duplicate node names.

**What to do:** Run the RTAB-Map+Nav2 launch in only one terminal. If you start it both from gz_bringup and again manually, you get two sets of the same nodes.

---

### RViz goal tool / publisher type error

**What you see:** “Could not create publisher … rt/goal_pose with incompatible type” or similar.

**What to do:** Use only the **Nav2 Goal** tool in the RViz toolbar. Remove or don’t use the generic “Set Goal” tool, and reload the RViz config (e.g. `rtabmap_nav2.rviz`) so only the Nav2 goal tool is active.

---

### Build / workspace

- **twist_stamper not found:** Run `colcon build --symlink-install --packages-select twist_stamper`, then `source install/setup.bash`.
- **Duplicate package names / CMake errors:** Run `drun` (or `run_ros_image.sh`) from the **repo root** (the `luna_ros` directory). If the workspace mount was wrong and the layout is broken, from inside the container run: `cd /ros2_ws && rm -rf build install`, then `source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash`.

---

## Tuning and change log

Important tuning decisions and changes are recorded here so the “why” and “what” stay clear. Config file: **`luna_nav/config/nav2_rtabmap_params.yaml`** unless noted.

| What changed | Why | Value / location |
|--------------|-----|------------------|
| **Robot radius (safety margin)** | Robot was colliding with rocks; planner treated robot as too small and planned paths too close to obstacles. | `robot_radius`: 0.30 → **0.45** m in both costmaps so the planner and controller keep more clearance (no collisions). |
| **BaseObstacle scale (DWB)** | Trajectories through cost were not penalized enough; robot drove into high-cost (rock) cells. | `BaseObstacle.scale`: 0.02 → **0.28** so the controller strongly avoids cost and stays away from obstacles. |
| **Max speed** | Robot needed more time to react and stop before hitting obstacles. | `max_vel_x` / `max_speed_xy`: 0.5 → **0.32** m/s; velocity_smoother `max_velocity` matched. Slower = more time to see obstacles and deviate. |
| **Sim time (DWB)** | Planner looks further ahead so it can avoid obstacles earlier. | `sim_time`: 1.5 → **2.0** s. |
| **Local costmap update rate** | Obstacles must appear in the costmap quickly so the controller can react. | `update_frequency`: 5 → **8** Hz. |
| **Voxel observation persistence / delay** | Costmap should react faster to newly seen obstacles (e.g. rocks). | `observation_persistence`: 0.35 → **0.25**; `observation_source_delay`: 0.1 → **0.08** s. |
| **Inflation: radius and decay** | Clear keep-out zone around obstacles; free space stays passable. | Local: `inflation_radius` **0.50** m, `cost_scaling_factor` **5.0**. Global: **0.42** m, **5.0**. |
| **Ground crop** | Low rocks in the bottom of the image were cropped out and missing from the voxel layer. | `ground_crop_bottom`: 0.1 → **0.06** (launch: depth_fov_filter) so more of the scene (including low rocks) feeds the costmap. |
| **Ground crop (depth filter)** | Floor was painting cost and causing a “trail” when turning. We only crop the pipeline that feeds the voxel layer; RTAB-Map uses raw depth. | `ground_crop_bottom`: **0.1** (fixed in `rtabmap_nav2_sim.launch.py`). See [Costmap trailing and noise](#costmap-trailing-and-noise) and [STACK_REVIEW_LUNABOTICS.md](STACK_REVIEW_LUNABOTICS.md). |
| **Voxel mark_threshold, range, persistence** | Reduce phantom obstacles and costmap trailing while still seeing rocks. | `mark_threshold: 2`, `obstacle_max_range: 2.5`, short `observation_persistence` / `observation_source_delay`. Voxel fed from ground-cropped point cloud. |
| **Costmap transform_tolerance** | When map→odom lags briefly (e.g. on turn), costmap was dropping updates. | **1.0** s so updates are not dropped during short TF lag. |
| **Controller costmap_update_timeout** | Nav2 was aborting goals immediately when costmap updates lagged (e.g. depth processing). | **3.0** s (controller_server) so brief delays don't trigger abort. |
| **Progress checker** | Nav2 was aborting when robot moved slowly (e.g. turning in place or avoiding obstacles). | `required_movement_radius`: **0.3** m, `movement_time_allowance`: **20** s so slow/careful motion doesn't trigger abort. |
| **RTAB-Map DetectionRate** | Map→odom was updating too slowly when turning, contributing to cost trail. | **2.0** Hz so the map (and TF) update more often. |
| **Zone publisher: odom topic and bounds** | Use same odom source as Nav2; support different arenas without code edits. | Default `odom_topic`: **/odom**. Zone bounds configurable via params; see `luna_nav/config/zone_publisher.yaml`. |

When you change something significant, add a row here (what changed, why, and where).

---

## Where settings live

| What | File |
|------|------|
| Sim launch (Gazebo, timers, mapping) | `lunabot_2425/launch/gz_bringup.launch.py` |
| Mapping + Nav2, depth filter, fixers, clear costmap | `luna_mapping/launch/rtabmap_nav2_sim.launch.py` |
| Nav2 (costmaps, voxel, inflation, DWB critics) | `luna_nav/config/nav2_rtabmap_params.yaml` |
| RTAB-Map (grid, noise, detection rate) | `luna_mapping/config/rtabmap_sim.yaml` |
| Depth FOV filter (ground_crop_bottom, mask_regions) | `rtabmap_nav2_sim.launch.py` (depth_fov_filter params) |
| Zone publisher (odom topic, zone bounds) | `luna_nav/config/zone_publisher.yaml` |
| Controller (wheel odom on/off) | `lunabot_2425/config/robot_controllers.yaml`; `robot_controllers_visual_odom.yaml` (odom TF off) |
