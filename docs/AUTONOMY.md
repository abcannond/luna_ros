# Autonomy: zones, localization, and frontier exploration

How the competition autonomy stack works: zone classification, ArUco localization, WFD frontier exploration, and mission FSM.

---

## Coordinate frames

For the full TF contract and sim vs hardware differences, read [ARCHITECTURE.md](ARCHITECTURE.md). Short version: **`map`** is the Nav2 global frame and zone YAML frame (via `world` alignment in sim). **`odom`→`base_link`** in default competition sim comes from **rgbd odometry**, not wheel TF (`enable_odom_tf` is false on `luna_cont`).

---

## Zone classification

Zones are axis-aligned rectangles in the `map` frame, defined per arena in YAML (`lunabot_2425/config/zones/`):

- `ucf_zones.yaml` — UCF Exolith Arena
- `artemis_zones.yaml` — Artemis Arena (KSC)

The `zone_publisher` node (luna_nav) looks up TF `map`→`base_link`, tests (x, y) against zone bounds, and publishes `/current_zone` (`std_msgs/String`) with hysteresis to avoid boundary jitter.

---

## ArUco localization

**Starting zone:** Known ArUco marker poses in `map` + PnP from onboard fiducial cameras → global pose correction. Uses `DICT_4X4_50`.

**After leaving:** Odometry + SLAM maintain pose. Markers are typically out of view.

**Package:** `fiducial_localizer` — multi-camera ArUco detection. Hardware params in `multi_camera_hardware.yaml`; sim params in `multi_camera_sim.yaml`.

**Integration:** Prefer filtered fusion (EKF or AMCL initial pose) over unfiltered jumps to `map`→`odom`.

---

## Frontier exploration (WFD)

**File:** `luna_nav/frontier_explorer.py`

Based on: Topiwala, Inani, Kathpal. *Frontier Based Exploration for Autonomous Robot.* [arXiv:1806.03581](https://arxiv.org/abs/1806.03581).

### Algorithm

1. Convert robot position to grid cell.
2. BFS from robot over FREE cells; any cell with an UNKNOWN neighbor is a frontier.
3. Cluster contiguous frontier cells; discard clusters < `min_frontier_size`.
4. Score each cluster: `0.4 * size - 1.0 * distance`; prefer clusters farther than `min_goal_distance`.
5. Send best centroid as Nav2 `NavigateToPose` goal.
6. After reaching goal (or failure), enter SETTLING state for `settle_time_s`, then repeat.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enabled` | `false` | Start exploring on launch |
| `min_frontier_size` | `5` | Min cells for a valid cluster |
| `rate_hz` | `1.0` | Detection loop frequency |
| `goal_tolerance` | `0.5` | Re-scan distance (m) |
| `blacklist_radius` | `1.0` | Ignore goals near previous failures (m) |
| `min_goal_distance` | `0.6` | Prefer frontiers farther than this (m) |
| `settle_time_s` | `3.0` | Pause after reaching a goal (s) |

Competition sim overrides: `min_frontier_size: 10`, `rate_hz: 0.5`, `blacklist_radius: 1.2`, `min_goal_distance: 0.8`, `settle_time_s: 4.0`.

### Services

- `~/enable` (`std_srvs/SetBool`) — enable/disable at runtime.

### Tuning

| Symptom | Fix |
|---------|-----|
| Too many tiny frontiers | Increase `min_frontier_size` |
| Robot oscillates | Increase `blacklist_radius` |
| Robot picks nearby goals | Increase `min_goal_distance` |

---

## Mission supervisor (FSM)

**File:** `luna_nav/mission_supervisor.py`

Phases: `IDLE` → `LOCALIZING` → `READY` (short stationary warmup) → `TRAVERSING_TO_EXCAVATION` → … → `COMPLETE`.

Key behaviors:
- **Map warmup:** During `READY`, the robot stays **stationary** for `map_warmup_s` (default **3 s** in `competition_sim.launch.py`) before the first Nav2 goal. There is **no** startup in-place spin (rotation caused depth/costmap trailing in sim).
- **Frontier exploration:** Enabled as fallback when waypoint navigation times out.
- **Goal retry:** If Nav2 aborts a goal, the supervisor retries after 5 s.

Start a mission:

```bash
ros2 service call /mission_supervisor/start_mission std_srvs/srv/Trigger {}
```

---

## Implementation status

- [x] Frame conventions (`map`, `odom`, `base_link`)
- [x] Zone YAML files + `zone_publisher` node
- [x] WFD `frontier_explorer` with enable/disable service
- [x] `mission_supervisor` FSM with dashboard
- [ ] Fiducial pose fusion via EKF/AMCL
- [ ] Rosbag logging
