# Codebase organization and simulation competition cycle

This document ties together **how to organize the repository** for autonomy, localization, zones, and mission logic, and **how to run simulation** so it matches at least a **basic competition-style routine** for repeatable test runs.

**See also:** [AUTONOMY_ZONES_AND_LOCALIZATION.md](AUTONOMY_ZONES_AND_LOCALIZATION.md), [COMPETITION_PREP_SSH_SEQUENCE.md](COMPETITION_PREP_SSH_SEQUENCE.md), [STACK_GUIDE.md](STACK_GUIDE.md).

---

## 1. Design goals

| Goal | Approach |
|------|----------|
| Same concepts in **sim** and **hardware** | One **`map`** frame, same topic names for sensors, Nav2, and (eventually) fiducials. |
| Clear **ownership** of configs | Arena worlds, Nav2, RTAB-Map, zones, and tag maps live in **versioned YAML** under named packages. |
| **Testable** competition phases | Sim runs support: bring-up → localize (or SLAM seed) → teleop in start → navigate → (future) zone + autonomy declarations. |
| Incremental implementation | Add **`luna_mission`** (or `luna_brain`) only when FSM/BT is ready; until then, **launch files + checklists** document the routine. |

---

## 2. Current package map (what exists today)

| Package | Responsibility |
|---------|------------------|
| **`lunabot_2425`** | URDF/xacro, Gazebo worlds/bridges, **`gz_bringup.launch.py`**, RViz configs for sim. |
| **`luna_control`** | Luna controller plugin, odometry to `/odom`. |
| **`luna_mapping`** | RTAB-Map + Nav2 sim launch, camera frame fixers, `rtabmap_nav2_sim.launch.py`. |
| **`luna_nav`** | Nav2 parameter YAML (`nav2_rtabmap_params.yaml`), bringup helpers. |
| **`fiducial_localizer`** | ArUco / marker pose (extend or parallel **AprilTag** node on hardware). |
| **`depth_to_pointcloud`** | Depth → point cloud when mapping stack is off. |

**Submodules (if present):** `luna_ros2_worlds` — Artemis / UCF SDF worlds and models.

---

## 3. Recommended layout (target as you grow)

Keep **drivers and simulation** separate from **mission logic** so Jetson and laptop launches stay obvious.

```
ros2_ws/src/
├── lunabot_2425/              # Sim + description + bridges (already)
│   ├── config/
│   │   ├── gz_bridge.config.yaml
│   │   └── zones/                         # NEW: optional zone polygons per arena
│   │       ├── artemis_zones.yaml
│   │       └── ucf_zones.yaml
│   └── launch/
│       ├── gz_bringup.launch.py
│       └── competition_cycle_sim.launch.py   # thin wrapper → Artemis + mapping
│
├── luna_mapping/              # SLAM + Nav2 sim integration (already)
├── luna_nav/                  # Nav2 params (already)
├── fiducial_localizer/        # Fiducials (already); add apriltag_ros wrapper or second node if needed
│
├── luna_localization/         # NEW (optional): EKF, static transforms, tag map loader
│   └── config/
│       └── tag_poses_map.yaml
│
└── luna_mission/              # NEW (optional): zone_classifier, mission FSM, BT
    ├── config/
    │   └── mission_phases.yaml
    └── luna_mission/
        ├── zone_classifier.py
        └── mission_supervisor.py
```

**Rules of thumb**

- **Nothing mission-specific** inside `luna_control` (stays real-time safe).
- **Zone polygons** and **tag poses** are **data** (YAML), not C++ constants.
- **One launch entry** per “hat”: sim competition test, hardware Jetson, MCC laptop (document in `STACK_GUIDE.md`).

You can **defer** `luna_mission` / `luna_localization` until needed; the folder list above is the **target shape**, not a requirement to create everything at once.

---

## 4. Configuration ownership

| Concern | Where it should live |
|---------|----------------------|
| Arena SDF / spawn pose | `luna_ros2_worlds` or `lunabot_2425/worlds` |
| ROS ↔ Gazebo bridges | `lunabot_2425/config/gz_bridge.config.yaml` |
| Nav2 (planners, costmaps, speeds) | `luna_nav/config/*.yaml` |
| RTAB-Map grid / rates | `luna_mapping/config/rtabmap_sim.yaml` (if present) + launch params |
| Zone definitions (polygons in `map`) | `lunabot_2425/config/zones/*.yaml` (recommended) |
| AprilTag / ArUco tag poses in `map` | `luna_localization/config` or `fiducial_localizer` params |
| Mission phase order / timeouts | `luna_mission/config` (future) |

---

## 5. Competition operational phases (reference)

Use the same **names** in code, docs, and MCJ scripts.

1. **INSPECT / PIT** — Not in sim; use checklist in `COMPETITION_PREP_SSH_SEQUENCE.md`.
2. **SIM BRING-UP** — Gazebo + bridges + controllers + (optional) mapping stack.
3. **LOCALIZE** — Fiducial / SLAM convergence; in sim often “wait for `/map` and stable `map`→`base_link`”.
4. **START_ZONE_MANEUVER** — Teleop: center, optional 360° (sim: `teleop_twist_keyboard` → `/cmd_vel`).
5. **TRANSIT_OBSTACLE** — Nav2 through obstacles (Travel autonomy testing).
6. **EXCAVATE** — Mechanism control (placeholder in pure nav stack).
7. **TRANSIT_TO_BERM** — Nav2 to construction side.
8. **DUMP** — Mechanism + pose in berm target.
9. **END** — Stop, log energy (hardware); sim: Ctrl+C / shutdown.

Phases **5–8** are **scorable** only with MCJ protocol on field; in sim you validate **plumbing** (Nav2, TF, costmaps, timing).

---

## 6. What simulation already provides (parity table)

| Competition need | Sim support today |
|------------------|-------------------|
| Bounded arena | `world:=ucf_arena` or `world:=artemis_arena` (`gz_bringup.launch.py`). |
| Non-GPS odometry | `/odom` from Luna controller plugin via Gazebo. |
| Depth + RGB for mapping / VIO | RealSense bridge → `/camera/camera/...`. |
| Occupancy grid + Nav2 | `launch_mapping:=true` → delayed include of `rtabmap_nav2_sim.launch.py`. |
| Teleop | `/cmd_vel` → twist_stamper → controller (see `STACK_GUIDE.md` `drive` alias). |
| Fiducial cameras (separate topics) | Bridged fiducial camera topics; full **ArUco→map** fusion may need `fiducial_localizer` launched and tuned. |
| Random start pose / heading | **Partial:** spawn pose is **fixed per world** in launch; true randomization requires launch args or small script (future). |
| AprilTag | **Hardware-first**; add Gazebo AprilTag plugin or overlay tags in sim later. |
| Zone classifier | **Not implemented**; add node + YAML when ready. |
| MCJ / hands-free gating | **Not in software**; use a **timer + honor system** in sim, or stub `mission_supervisor` later. |

---

## 7. Basic competition-style routine in simulation (commands)

These steps mirror the **checklist** in `COMPETITION_PREP_SSH_SEQUENCE.md` but for **one machine** running the stack (no SSH).

### One-liner (full competition stack with autonomy nodes)

```bash
cd /ros2_ws && source install/setup.bash
ros2 launch lunabot_2425 competition_sim.launch.py world:=ucf_arena
```

This starts Gazebo, RTAB-Map, Nav2, zone_publisher, frontier_explorer, and mission_supervisor.

To start a mission run after the stack is up (~35s):

```bash
ros2 service call /mission_supervisor/start_mission std_srvs/srv/Trigger {}
```

Equivalent without the autonomy wrapper:

```bash
ros2 launch lunabot_2425 gz_bringup.launch.py world:=artemis_arena launch_mapping:=true
```

Wait until the console shows **RTAB-Map/Nav2** starting (~25 s after sim start), then verify:

```bash
ros2 topic hz /map
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
```

### Routine (manual, matches “competition cycle” practice)

1. **Bring-up** — Done by launch (Gazebo, robot, bridges, delayed RTAB-Map + Nav2).
2. **Localize / map** — Let RTAB-Map build `/map`; drive slowly or send a small Nav2 goal so the grid expands.
3. **Teleop in start** — Second terminal, sourced:

   ```bash
   source /ros2_ws/install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
   ```

   Optional: **center** the robot in the start region; optional **slow rotate** for sensor coverage.
4. **Navigate** — RViz **Nav2 Goal** or `ros2 action send_goal` to a pose toward obstacle / construction side (test Travel-style path planning).
5. **End** — Stop teleop; Ctrl+C launch terminal when done.

### Two-terminal habit (closer to field debug)

| Terminal | Command |
|----------|---------|
| 1 | `ros2 launch lunabot_2425 gz_bringup.launch.py world:=artemis_arena launch_mapping:=false` then later `ros2 launch luna_mapping rtabmap_nav2_sim.launch.py` |
| 2 | teleop, `topic echo`, RViz |

Use **`competition_sim.launch.py`** when you want **one terminal** with the full autonomy stack for CI and quick student demos.

---

## 8. Adjustments still worth making (roadmap)

Short, ordered list:

1. **`config/zones/*.yaml`** — Placeholder polygons in `map` frame; RViz marker node optional.
2. **Launch arg `spawn_yaw:=...`** (optional) — Simulate direction wheel without code edits.
3. **`fiducial_localizer` in sim launch** — Include when ArUco topics stable; aligns with “localize in start zone” test.
4. **`luna_mission` stub** — Publishes current phase from timers for logging only (no scoring).
5. **AprilTag in Gazebo** — When sim must match Jetson pipeline exactly.

---

## 9. Testing matrix (minimum)

| Test | Pass criteria |
|------|----------------|
| Sim boots Artemis | No crash; robot visible; `/clock` advances. |
| Mapping | `/map` publishes; `map`→`odom`→`base_link` exists. |
| Teleop | Robot moves; E-stop not in sim—note for hardware. |
| Nav2 goal | Robot reaches goal without fatal planner errors. |
| Repeat | Same launch twice; comparable behavior (deterministic sim). |

---

## 10. Summary

- **Organize** by package responsibility: **sim/description**, **mapping/nav**, **fiducials**, **mission** (later).
- **Configure** zones and tags as **YAML**, not scattered launch constants.
- **Run** the basic competition **routine** in sim with **`competition_cycle_sim.launch.py`** (Artemis + RTAB-Map + Nav2), then **teleop** → **Nav2 goals** to rehearse phase order.
- **Extend** with random spawn, zone node, and AprilTag in sim as the season requires.

This gives you a **documented baseline** to start testing full runs in simulation while keeping room for hardware-specific steps (SSH, PPE, MCJ) outside the codebase.
