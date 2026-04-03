# Competition prep and SSH sequence

Operational checklist for **Artemis Arena** runs: robot placement, bring-up order, SSH usage (bandwidth), localization, mapping, and planning stack.

**Related:** [AUTONOMY.md](AUTONOMY.md), [STACK_GUIDE.md](STACK_GUIDE.md).

**Design reference (team):** [Canva — competition flow](https://www.canva.com/design/DAG4uBCdxtM/6pPivqpfjFyfts0RoROFcQ/edit) *(internal; keep in sync with this doc).*

---

## Robot arena placement

| Item | Value / note |
|------|----------------|
| **Starting zone (nominal)** | **2 m × 2 m** *(confirm against current year field diagram; rules also allow random position/orientation within the defined start region).* |
| **Robot stowed envelope** | 150 × 75 × 75 cm max (orientation per inspection); **≤ 80 kg** including navigation aids that are not on-robot. |
| **Placement** | Random position and **heading** selected at run time; **Arrow 3** defines forward for the direction wheel (N/E/S/W). |

---

## Why SSH appears in this sequence

- **KSC bandwidth scoring** rewards keeping **average link utilization ≤ 4 Mbps**; sustained SSH shells, log streaming, and extra tunnels add load.
- **Practice:** Use SSH for **Jetson bring-up, camera checks, and quick diagnostics**, then **disconnect** when the robot should rely on the **WAP ↔ MCC** link for teleop/autonomy.
- **Rule of thumb:** Close SSH **before** high-data phases (multiple video streams, bag recording to network, etc.) unless you intentionally budget bandwidth.

---

## Steps (filled outline)

### 1. Boot everything (power and network order)

1. **E-stop** accessible; batteries safe; **power meter / data logger** visible (per inspection).
2. **Jetson / compute** on; **RealSense** (or depth camera) powered; **Wi‑Fi client** on robot associated with **team WAP** (KSC: **Channel 1** in arena, correct **SSID**).
3. **WAP** on shelf per arena layout; **Ethernet** from arena drop to MCC switch; **laptops** in MCC only with allowed gear.
4. Confirm **clock / `use_sim_time`** is **false** on hardware.

### 2. Mission Control launch file (MCC laptop)

1. Source workspace: `source install/setup.bash`.
2. Launch your **MCC** entrypoint (router OK, **no** back-channel Wi‑Fi to robot).
3. Open **telemetry** views (monitor-only during autonomy); have **MCJ callouts** scripted (start/end hands-free).

### 3. Launch teleop node (MCC)

1. Start **teleop** (or game controller node) **only when** rules allow **manual control** (not during declared autonomy).
2. Verify **cmd_vel** (or your stack’s equivalent) reaches the robot and **E-stop** still cuts motion.

### 4. SSH into Jetson (short session)

Use for **configuration and checks**; keep sessions **minimal**.

```bash
ssh <user>@<jetson-hostname-or-ip>
```

Typical tasks while connected:

- Export ROS env / workspace.
- **Launch file** (see below) or systemd-managed bring-up.
- Quick `ros2 topic list`, `ros2 node list`, camera topics.

**Exit SSH** when bring-up is stable: `exit`.

### 5. Jetson launch file (on-robot)

Single orchestration launch (example name—match your repo):

- **Drivers:** RealSense → `realsense2_camera` (`realsense2_camera` package).
- **AprilTag:** `apriltag_ros` (or `apriltag_ros` pipeline) → tag poses / `tf`.
- **Mapping / odom:** RTAB-Map **visual odometry** node if used (RGB-D); mapping node if building map live.
- **Nav2:** `bt_navigator`, planners, costmaps (**SmacPlanner** global, **TEB** local—see below).
- **Teleop** optional on robot (often disabled in favor of MCC teleop).

Parameterize **`map`**, **`odom`**, **`base_link`**, and **tag poses** in YAML so arena changes do not require code edits.

### 6. Initialize cameras

1. Confirm **RGB + depth** streams (RealSense serial, USB stability).
2. Confirm **`camera_info`** and **frame_ids** (needed for AprilTag and depth).
3. If using **fiducial cameras** separate from RealSense, bring them up and verify topics for **starting-zone** tags.

### 7. Run diagnostics on mechanical components

**Before** relying on autonomy:

- **Drive:** short forward/back/strafe (if holonomic) at low speed; listen for faults.
- **Excavation / dump:** one **safe** actuator cycle **without** loading regolith if rules require clearance from arena staff.
- **E-stop:** press once; verify **motion stop** and **reset procedure** (second deliberate action per rules).

### 8. End SSH to save bandwidth

1. Exit all SSH sessions to Jetson.
2. Confirm robot still reachable from **MCC** over **team Wi‑Fi** (teleop + topics).
3. Optionally **throttle** heavy logging over Wi‑Fi during the run.

### 9. Localize (starting zone)

**Goal:** stable **`map` → `base_link`** (or fused pose) before leaving teleop-driven start maneuvers.

| Mode | Source | Role |
|------|--------|------|
| **Absolute** | **AprilTag** (`apriltag_ros`) | Known tag poses in **`map`** → global pose / correction. |
| **Relative** | **RTAB-Map visual odometry** | Smooth short-term motion between tag updates. |

Suggested routine:

1. Ensure tag map and camera extrinsics are loaded.
2. Wait for **at least one** good tag observation (size, reprojection sanity).
3. Fuse per your stack (EKF, AMCL seed, or direct `map`→`odom` update per team policy).

### 10. Move to center of starting zone (teleop)

- While still allowed **teleop in starting zone only**, drive to your **nominal center** pose (optional alignment for repeatability).
- Log **not** required for scoring; **reduces** variance for path into obstacle zone.

### 11. 360° spin (optional, teleop or autonomy)

- **Purpose:** gather **depth/lidar** coverage for local map / obstacle layer; **not** for compass (compass disallowed on robot).
- Keep **speed low**; avoid throwing regolith (dust scoring).
- If done during a **declared autonomy** segment, follow **hands-free** and **MCJ announcement** rules.

---

## Localization (summary)

| Layer | Technology | ROS 2 packages / notes |
|-------|------------|-------------------------|
| **Absolute** | **AprilTag** — known tag positions, PnP pose | **`apriltag_ros`** (or maintained fork for your distro); static transforms **tag → map**. |
| **Relative** | **RTAB-Map VIO** — RGB-D odometry between frames | **`rtabmap_odom`** / `rtabmap_slam` params; disable wheel fusion if unused. |
| **Fusion** | Optional **robot_localization** EKF | Combine wheel odom + VIO + tag pose updates with sane covariances. |

**Rules alignment:** No **GPS**; **compass** data must not feed autonomy—disable magnetometer fusion in IMU drivers if applicable. **Do not** use **walls** for mapping/navigation. **Ultrasonic** ranging is disallowed on robot per rulebook you use—do not rely on it.

---

## Mapping

| Output | Source | Package |
|--------|--------|---------|
| **3D / dense** | RealSense depth + RGB | **`realsense2_camera`** |
| **Occupancy grid** | RTAB-Map from depth (and/or Octomap if you add it) | **`rtabmap_slam`**; `/map` = `nav_msgs/OccupancyGrid` |

Use **one** consistent **`map`** frame for Nav2 and AprilTag **tag poses**.

---

## Path planning (Nav2)

| Layer | Planner | Typical use |
|-------|---------|-------------|
| **Global** | **SmacPlanner** (2D) | Feasible path on **global costmap** from `/map` + inflation. |
| **Local** | **TEB** (Timed Elastic Band) | Smooth, dynamic obstacle avoidance on **local costmap**; tune for regolith/skid. |

**Inputs:** `/map`, **`tf`**, **odom**, **scan** or **pointcloud** to local costmap (depth → laserscan or voxel layer per your stack).

**Travel autonomy (scoring):** rules expect **mapping + planning** through the obstacle zone—not a single fixed “point and shoot” heading; your **global + local** pair supports **replanning** as the map updates.

---

## One-page sequence (copy for pit clipboard)

1. Boot robot + WAP + MCC network.  
2. MCC: launch **mission control** + **teleop** (when allowed).  
3. **SSH Jetson** → launch **Jetson stack** → **cameras** → **mech diagnostics** → **exit SSH**.  
4. **Localize** (AprilTag + RTAB-Map VIO).  
5. **Teleop:** center of start (optional) → optional **360°** scan.  
6. **Declare autonomy** per MCJ script when attempting scored segments.  
7. Run end: **E-stop** ready, **logger** reading for judges, **arena removal** + HEPA per procedure.

---

*Last updated: fill in **starting zone** dimensions from the official diagram each season; this doc used **2 m × 2 m** as provided.*
