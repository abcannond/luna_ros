# Stack guide

Features, design choices, launching, and troubleshooting for the Luna ROS stack.

**See also:** [HARDWARE.md](HARDWARE.md) (hardware) · [COMPETITION_SIM.md](COMPETITION_SIM.md) (full competition launch + `start_mission`) · [AUTONOMY.md](AUTONOMY.md) (zones, WFD, mission FSM; avoid duplicating that here) · [ARCHITECTURE.md](ARCHITECTURE.md) (topics, TF, YAML index)

---

## Debugging quick reference

Use this when something fails after launch. For pipeline detail and tuning, read the sections below.

### Build sanity

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

After changing Python packages, rebuild the affected package. Stale `install/` trees can be removed with `rm -rf build install log` if imports act wrong.

### TF checks

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

Expect a connected chain **`map` → `odom` → `base_link`** during mapping. If **`odom`→`base_link`** freezes while sim time advances, rgbd odometry may have stopped (registration failures). See [COMPETITION_READINESS.md](COMPETITION_READINESS.md).

### Topics

```bash
ros2 topic list
ros2 topic hz /camera/camera/color/image_raw_fixed
ros2 topic hz /odom
ros2 topic echo /current_zone --once
```

If `*_fixed` topics are silent, upstream bridge or frame fixers may not be running.

### Nav2 and mission

```bash
ros2 action list | grep navigate
ros2 service call /mission_supervisor/start_mission std_srvs/srv/Trigger {}
```

If goals abort immediately, check **`goal_frame_id`**, costmap state, and TF staleness. Mission supervisor cancels any prior goal before sending a new one to reduce preemption loops.

### Optional NDJSON debug

Set **`debug_ndjson_log:=true`** on `mission_supervisor` to subscribe to costmap, TF, and cmd_vel observers and append NDJSON to **`debug_ndjson_path`** (default under `/tmp`). Same flag exists on **`depth_fov_filter`**. Keep defaults **false** in normal runs.

### Full sim validation

Follow [COMPETITION_SIM.md](COMPETITION_SIM.md). Record pass or fail in team notes or PRs.

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
| Headless (no Gazebo GUI, stable in Docker) | `ros2 launch lunabot_2425 gz_bringup.launch.py headless:=true` |

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

**Diagnostics:** see [Debugging quick reference](#debugging-quick-reference). For a quick map/scan pulse you can still run `ros2 topic hz /map`, `ros2 topic hz /scan`, and `ros2 run tf2_tools view_frames`.

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

Nav2's local costmap **voxel layer** subscribes to the point cloud directly, using `min_obstacle_height` to filter ground returns.

### RTAB-Map

**Role:** Build the occupancy grid and the map→odom part of the TF tree.

RTAB-Map does SLAM from RGB and depth. It publishes `/map` (occupancy grid) and the `map`→`odom` transform. Optionally, the **rgbd_odometry** node can publish `odom`→`base_link` when you set `use_rtabmap_odom:=true`; that’s “visual odometry” from the camera. Grid and noise parameters (e.g. `Grid/RangeMin`, `NoiseFilteringMinNeighbors`, `MinClusterSize`) are tuned so the map is stable and small obstacles (e.g. rocks) are kept without clutter; see [Costmap trailing and noise](#costmap-trailing-and-noise) for how that ties into the costmap.

### Nav2

**Role:** Plan paths and send velocity commands using the map and live obstacle data.

Nav2 maintains two costmaps:

- **Global costmap:** Static layer (RTAB-Map’s `/map`) plus inflation. Used for global path planning.
- **Local costmap:** **Voxel layer** (point cloud) + **obstacle layer** (`/scan`) + inflation — no static layer, so RTAB-Map grid artifacts are not replayed locally. Depth is ground-cropped before Nav2; near-field min range rejects self-hits when spinning.

**Obstacle avoidance (no collisions):** Costmaps and the DWB controller are tuned so the robot does not hit rocks or other obstacles:
- **Safety margin:** `robot_radius` is set conservatively (~0.42 m) so the planner keeps clearance from obstacles.
- **Inflation:** `inflation_radius` and `cost_scaling_factor` give a clear keep-out zone around obstacles while keeping free space passable.
- **Controller:** **BaseObstacle** scale (0.50) penalizes trajectories through cost. Max speed is limited (0.18 m/s). Sim time is 3.0 s for look-ahead.
- **Vision:** Ground crop (~0.14) for Nav2 depth; `obstacle_min_range` ~0.52 m to avoid marking the robot’s own wheels/chassis. See [Tuning and change log](#tuning-and-change-log).

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

**Why it happens:** (1) **Self-hits during spin:** The depth camera is forward and pitched down; the bottom and sides of the image often see wheels, rocker, or chassis at roughly constant range. When the robot rotates in place, those returns stay at similar depth in the camera frame but sweep around in `odom`, which looks like a **ring or arc** of cost (not classic TF “slip”). (2) **TF lag:** The costmap is built using map→odom→base_link; brief lag during fast turns can smear obstacles. (3) **Floor / noise:** Ground returns and speckle add spurious cost.

**What we did:**

- **Local costmap: no static layer.** The local costmap uses only live sensor data (`voxel_layer` + `obstacle_layer` + `inflation_layer`). The global costmap still uses the static layer for path planning.
- **Reject near-field depth for obstacles:** `obstacle_min_range` ~**0.52 m** on voxel and scan layers (and `range_min` on `depthimage_to_laserscan`) so chassis/wheel returns are not marked as obstacles. Tune if you need closer obstacle detection in front of the camera.
- **Larger bottom crop on depth:** `ground_crop_bottom` ~**0.14** in `depth_fov_filter` to mask rows where wheels and near ground dominate (RTAB-Map still uses unfiltered depth on its topic).
- **Short observation persistence:** `observation_persistence: 0.1` on voxel and scan so transient marks decay quickly when the sensor moves.
- **Costmap:** `transform_tolerance: 1.0` so brief TF delay doesn’t drop updates.
- **Voxel layer:** `mark_threshold: 2`, `obstacle_max_range: 2.5`, `min_obstacle_height: 0.05`.
- **RTAB-Map:** `Rtabmap/DetectionRate: 2.0` for fresher map→odom when turning.
- **Noise (RTAB-Map grid):** `Grid/RangeMin: 0.5`, neighbor filtering, `MinClusterSize` as in `rtabmap_sim.yaml`.

**Where to look:** `luna_nav/config/nav2_rtabmap_params.yaml` (local costmap, voxel); `luna_mapping/config/rtabmap_sim.yaml` (grid); ground crop is in `rtabmap_nav2_sim.launch.py` (depth_fov_filter params).

---

### Odom and map alignment when stuck

**What you see:** The robot hits a rock and gets stuck in the sim, but the map and planner still think it’s moving. Pose and costmap drift away from the true state.

**Why it happens:** Odometry is computed from wheel motion. When the body is stuck but the wheels spin (e.g. slip), the odometry keeps integrating, so the belief pose drifts.

**What to do (recommended when stuck/slip is an issue):**

1. Launch the mapping stack with `use_rtabmap_odom:=true` so RTAB-Map’s rgbd_odometry node publishes `odom`→`base_link`.
2. Set `enable_odom_tf: false` under `luna_cont` in `lunabot_2425/config/robot_controllers.yaml` so wheel odometry does not also publish `odom`→`base_link`. The default competition sim already uses this. If you enable wheel odom TF again, you will have two sources for that transform.
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

**Sim TF tree:** `map` → `odom` → `base_link`. RTAB-Map publishes `map`→`odom`. With `use_rtabmap_odom:=true`, `rgbd_odometry` publishes `odom`→`base_link`. `competition_sim.launch.py` also publishes static `world`→`map` at the spawn pose. The tree is connected when those nodes are running.

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

### Path looks valid but wheels twist with little forward motion (Nav2)

**What you see:** RViz shows a green global path; the quad pods steer and rotate but the robot barely translates.

**Why:** DWB’s **`trans_stopped_velocity`** must be **less than `max_vel_x`**. If it is higher, every forward command counts as “translationally stopped,” so **RotateToGoal** / alignment critics dominate and the robot keeps reorienting instead of driving.

**What to do:** In **`nav2_rtabmap_params.yaml`**, under `FollowPath`, keep **`trans_stopped_velocity`** well below **`max_vel_x`** (see tuning table below). Rebuild or restart Nav2 after edits.

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

| What changed | Why | Current value |
|--------------|-----|---------------|
| **Robot radius** | Keep-out buffer around obstacles. | `robot_radius`: **0.42 m** (both costmaps). |
| **BaseObstacle scale (DWB)** | Avoid lethal / high-cost cells; balance with path following in soft inflation. | **0.95** (with PathDist / PathAlign). |
| **`trans_stopped_velocity` (DWB)** | Threshold for “translating” vs rotate-in-place; must be **< `max_vel_x`**. | **0.03 m/s** (`max_vel_x` **0.12**). |
| **Max speed** | Slower = more time to react. | `max_vel_x`: **0.12 m/s**, `max_vel_theta`: **0.4 rad/s** (DWB FollowPath). |
| **Sim time (DWB)** | Longer look-ahead for avoidance. | **3.0 s**. |
| **`min_y_velocity_threshold` (controller server)** | Noise floor for lateral motion; use ~0 for non-holonomic DWB. | **0.001** (`max_vel_y`: **0**). |
| **Voxel + scan** | Depth → obstacles; reject self-hits when spinning. | `observation_persistence` **0.1**, `obstacle_min_range` **~0.52 m**, `min_obstacle_height` **0.05**. |
| **Inflation** | Steeper falloff keeps corridors more traversable; lethal band stays at obstacles. | Local: `inflation_radius` **0.75 m**, `cost_scaling_factor` **3.0**. Global: `inflation_radius` **1.0 m**, `cost_scaling_factor` **2.2**. |
| **Global costmap** | Static map only (no live obstacle layer). | Plugins: `static_layer` + `inflation_layer`. |
| **Ground crop** | Mask wheels / near ground in depth for Nav2 only. | `ground_crop_bottom`: **0.14** (`depth_fov_filter` in `rtabmap_nav2_sim.launch.py`). |
| **RTAB-Map** | Map freshness and grid detail. | `DetectionRate: 2.0`, `Grid/DepthDecimation: 2`, `NoiseFilteringRadius: 0.15`. |
| **Frontier explorer** | Settling between goals, balanced scoring. | `settle_time_s: 4.0`, `min_frontier_size: 10`, `min_goal_distance: 0.8`. |
| **Mission supervisor** | Stationary pre-goal delay + goal retry. | `map_warmup_s` **3.0** (no rotation); auto-retry after 5 s. |

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
| Controller (wheel odom on/off) | `lunabot_2425/config/robot_controllers.yaml` (`enable_odom_tf` under `luna_cont`) |
