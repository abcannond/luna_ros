# Autonomy: zones, ArUco localization, and navigation

This document defines a **cohesive approach** for competition autonomy: **hard-coded arena zones**, **fiducial (ArUco) localization** in the starting area, and **Nav2** for motion. It is meant to guide implementation in this repository (`luna_ros`) and on hardware.

**Related docs:** [STACK_GUIDE.md](STACK_GUIDE.md) (launching sim and Nav2), [HARDWARE.md](HARDWARE.md) (physical deployment), [COMPETITION_PREP_SSH_SEQUENCE.md](COMPETITION_PREP_SSH_SEQUENCE.md) (arena prep, SSH, localization/planning checklist), [CODEBASE_AND_SIM_COMPETITION_CYCLE.md](CODEBASE_AND_SIM_COMPETITION_CYCLE.md) (repo organization + sim competition routine).

---

## 1. Goals

| Goal | Approach |
|------|----------|
| Know **where** the robot is in the arena layout | **Zone lookup**: robot pose \((x, y)\) tested against **predefined regions** in the global frame. |
| Obtain **trustworthy global pose** at startup (and optionally when revisiting) | **ArUco markers** in the **starting zone**, observed by onboard **fiducial cameras**, solved to pose in **`map`**. |
| Move **between** regions and execute mission phases | **High-level state machine** (or behavior tree) + **Nav2** `NavigateToPose` to waypoints per zone. |
| Prove algorithms before hardware | **Gazebo** + same topic/frame conventions as the physical robot where possible. |

**Non-goals for the primary competition stack:** ML-based place recognition; full SLAM as the *only* global reference if a **prior map + fiducials + AMCL** (or equivalent) is sufficient.

---

## 2. Coordinate frames (single source of truth)

Everything that must line up for zones + Nav2 + fiducials:

| Frame | Role |
|-------|------|
| **`map`** | Nav2 global frame; occupancy grid `/map` is defined here; **zone polygons are stored in `map`**. |
| **`odom`** | Continuous odometry (wheels, etc.); smooth short-term motion. |
| **`base_link`** | Robot base; used for “where is the robot” for zone checks (optionally footprint center). |

**TF chain:** `map` → `odom` → `base_link` must be consistent. Fiducial updates should **correct** `map` → `odom` (or feed **initial pose** to AMCL) rather than teleporting `base_link` without fusion.

**Arena vs map:** If the rulebook uses arena-fixed coordinates, define either:

- **Option A:** Build / load **`map`** so its origin matches the arena layout used for ArUco world coordinates, **or**
- **Option B:** Keep a static transform **`arena` → `map`** and define zones in `arena`, then transform points into `map` for Nav2.

Pick one convention and document it in calibration notes.

---

## 3. ArUco / fiducial localization

### Role

- **Starting zone:** Primary **global** correction: known marker poses in **`map`** (or `arena` + static TF) + PnP from camera images → **robot pose in `map`**.
- **After leaving the starting zone:** Markers may be **out of view**; localization continues via **wheel odometry** + **map-based localization** (e.g. AMCL) if using a prior map, or SLAM if building the map live.

### Repository touchpoints

- Package **`fiducial_localizer`** (`marker_localizer`): ArUco detection, pose output, TF parameters such as `world_frame`, `robot_frame`, marker pose in world.
- **Simulation:** Fiducial camera topics (e.g. under `/fid_cams/...`) and `gz_bringup` / mapping launches as described in `STACK_GUIDE.md`.

### Integration principles

1. **Calibration:** Camera intrinsics (`CameraInfo`) and **extrinsics** `camera_optical_frame` → `base_link` must be measured and stable.
2. **Marker catalog:** Dictionary, ID, and **physical size** must match the competition / field setup (wrong dictionary ⇒ no pose).
3. **Fusion:** Prefer **filter** (e.g. `robot_localization` EKF) or **AMCL initial pose** when a good detection arrives; avoid publishing unfiltered jumps to `map`→`odom` every frame without smoothing.
4. **Sanity checks:** Reject poses with bad reprojection, impossible jumps, or inconsistent height.

---

## 4. Hard-coded zone recognition

### Idea

**Zones** are **regions** in the **\(xy\)** plane of **`map`**. The robot’s position \((x, y)\) from `tf` (`map` → `base_link`) is classified by **geometric predicates**:

- **Axis-aligned rectangles:** simple bounds checks.
- **General polygons:** **point-in-polygon** (ray casting or winding number).

No learning is required: the arena layout is known and fixed for a given competition configuration.

### Data model (suggested)

- **Config file** (YAML/JSON), e.g. `config/zones.yaml`:

  - `zones`: list of `{ id, name, polygon: [[x,y], ...] }` or `bounds: { xmin, xmax, ymin, ymax }`.
  - Optional: `default_goal_pose` per zone, `allowed_behaviors`, Nav2 waypoint id.

- **Runtime node** (future package, e.g. `luna_zones` or node inside `luna_brain`):

  - Input: `tf` lookup `map` → `base_link` (timer-driven, e.g. 5–10 Hz).
  - Output: `current_zone` (string/id), optional `std_msgs/Bool` “in_start_zone”, diagnostics.

### Robustness

- **Boundary jitter:** Near edges, pose noise can flip zone ID. Use **hysteresis** (require \(N\) consecutive samples in a zone) or small **inset** of polygons for “core” vs “border.”
- **Consistency:** Use the same reference point on the robot (e.g. `base_link` projection) for all zones.

---

## 5. Mission control: navigating between zones

Zones define **where** you are; they do not replace **path planning**.

**Recommended pattern:**

1. **Finite state machine (FSM)** or **behavior tree** encodes mission phases (e.g. align in start → transit to zone B → execute task → return).
2. Each transition uses **Nav2**:
   - Publish **`NavigateToPose`** goals (or use the Nav2 action API) to **waypoints** stored per zone or per edge in the mission graph.
3. **Zone node** feeds the FSM: e.g. “entered `mining_zone`” triggers a behavior; “left `start_zone`” arms a timer or next goal.

**Frontier-based exploration** (optional, see §7) is **not** required for zone-to-zone driving if waypoints are sufficient; it can supplement mapping or recovery.

---

## 6. Simulation vs physical robot

| Aspect | Simulation | Physical |
|--------|------------|----------|
| Fiducials | Ideal images; tune detection thresholds anyway | Lighting, exposure, motion blur |
| `/map` | Often from RTAB-Map in sim | Prior map + AMCL, or SLAM + careful drift handling |
| Zones | Same YAML as target physical `map` | **Verify** polygon corners after map alignment (tape measure + fiducial origin) |

**Workflow:** Define zones in **`map`** coordinates used by Nav2; validate in Gazebo; **re-verify** once a real map frame is aligned to the arena.

---

## 7. Frontier detection (WFD) -- IMPLEMENTED

**Purpose:** Find boundaries between **known free** and **unknown** cells for exploration or mapping—not for naming competition zones. **Frontiers** are regions on the boundary between open (mapped free) space and unexplored space; moving to successive frontiers grows the map until none remain.

**Inputs:** `nav_msgs/OccupancyGrid` on `/map`, robot pose in `map` (e.g. from `tf`).

**Outputs:** Candidate exploration goals or markers for RViz.

### Wavefront Frontier Detector (WFD)

For implementation or literature alignment, use the **Wavefront Frontier Detector (WFD)** as described in:

> Anirudh Topiwala, Pranav Inani, Abhishek Kathpal, *Frontier Based Exploration for Autonomous Robot*, arXiv:1806.03581 [cs.RO], 10 Jun 2018.  
> PDF: [https://arxiv.org/pdf/1806.03581](https://arxiv.org/pdf/1806.03581) · abs: [https://arxiv.org/abs/1806.03581](https://arxiv.org/abs/1806.03581)

That work presents WFD as a frontier-based exploration strategy implemented in ROS (Gazebo and TurtleBot hardware). The algorithm expands a **wavefront** from the robot’s position over known free space to detect frontier cells efficiently—suitable for pairing with this stack’s `/map` from RTAB-Map (or any occupancy grid source).

**Note:** Earlier literature often cites **Yamauchi** (1997) for the original frontier *concept*; the reference above is the WFD-focused ROS-oriented treatment to cite for your wavefront implementation.

**Implementation:** `luna_nav/frontier_explorer.py` -- see [WFD_IMPLEMENTATION.md](WFD_IMPLEMENTATION.md) for algorithm, parameters, and tuning.

**Integration:** The `mission_supervisor` FSM enables/disables frontier exploration via the `~/enable` service. In competition mode, WFD serves as a fallback when waypoint navigation times out.

---

## 8. Zone configuration files

Zone definitions are stored as YAML in `lunabot_2425/config/zones/`:

- `ucf_zones.yaml` -- UCF Exolith Arena bounds in Gazebo world coordinates
- `artemis_zones.yaml` -- Artemis Arena (KSC) bounds

The `zone_publisher` node loads these at startup via the `zones_file` parameter. Each zone has `[x_min, x_max, y_min, y_max]` bounds and an RGBA color for RViz visualization.

---

## 9. Implementation status

- [x] Frame conventions frozen (`map`, `odom`, `base_link`).
- [x] ArUco dictionary `DICT_4X4_50` configured.
- [x] Zone YAML files created (`ucf_zones.yaml`, `artemis_zones.yaml`).
- [x] `zone_publisher` node: TF-based, YAML loading, hysteresis, MarkerArray.
- [x] `frontier_explorer` node: WFD algorithm, Nav2 integration, enable/disable service.
- [x] `mission_supervisor` FSM: competition phase transitions, dashboard publishing.
- [ ] Camera intrinsics calibration on hardware.
- [ ] Fiducial pose fusion via EKF/AMCL.
- [ ] Rosbag logging for debug.

---

## 10. Summary

**Global pose** in the arena is anchored by **ArUco** in the starting zone and maintained by **odometry + map localization** elsewhere. **Where** the robot is in the **mission layout** is determined by **hard-coded zones** in **`map`**, using **robot \((x,y)\)** from TF—not by learning. **Nav2** executes **between** those regions using **waypoints** driven by a **state machine** or behavior tree. This keeps autonomy **explainable, testable in sim, and aligned with competition operation.**
