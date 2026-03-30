# Wavefront Frontier Detector (WFD) Implementation

This document describes the Wavefront Frontier Detector implemented in `luna_nav/frontier_explorer.py`, its algorithm, parameters, and usage.

---

## Source

> Anirudh Topiwala, Pranav Inani, Abhishek Kathpal.
> *Frontier Based Exploration for Autonomous Robot.*
> arXiv:1806.03581 [cs.RO], 10 Jun 2018.
> PDF: https://arxiv.org/pdf/1806.03581
> Abstract: https://arxiv.org/abs/1806.03581

The paper presents a BFS-based approach to frontier detection on occupancy grids, implemented and validated in ROS on Gazebo and TurtleBot hardware.

---

## Algorithm

A **frontier** is a cell on the boundary between known-free space and unknown space in an occupancy grid. By driving the robot to successive frontiers, the map grows until no unknown space is reachable.

### Pseudocode (WFD, adapted from arXiv:1806.03581)

```
Input: OccupancyGrid M, robot position (rx, ry)
Output: list of frontier centroids

1. Convert (rx, ry) to grid cell (r0, c0)
2. BFS from (r0, c0) over FREE cells:
   - For each free cell (r, c), check 8-connected neighbors
   - If any neighbor is UNKNOWN (-1), mark (r, c) as FRONTIER
3. Cluster contiguous FRONTIER cells via BFS:
   - For each unvisited frontier cell, flood-fill to find its connected region
   - Discard clusters smaller than min_frontier_size
4. For each valid cluster, compute centroid in world coordinates
5. Select best frontier (nearest to robot, not blacklisted)
6. Send centroid as Nav2 NavigateToPose goal
7. On goal reached or failure, repeat from step 1
```

### Cell Classification

| OccupancyGrid value | Meaning |
|---------------------|---------|
| 0 | Free (known, traversable) |
| -1 | Unknown (not yet observed) |
| >= 50 | Occupied (obstacle) |

---

## Implementation Details

**File:** `ros2_ws/src/luna_nav/luna_nav/frontier_explorer.py`

### Subscriptions

- `/map` (`nav_msgs/OccupancyGrid`) -- the occupancy grid from RTAB-Map
- TF `map` -> `base_link` -- robot position for BFS seed

### Publications

- `/frontier_goals` (`geometry_msgs/PoseArray`) -- all detected frontier centroids
- `/frontier_markers` (`visualization_msgs/MarkerArray`) -- RViz visualization
- `/exploration_status` (`std_msgs/String`) -- current state: `disabled`, `scanning`, `exploring`, `no_frontiers_remaining`

### Services

- `~/enable` (`std_srvs/SetBool`) -- enable/disable exploration at runtime

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enabled` | `false` | Start exploring on launch |
| `map_topic` | `/map` | OccupancyGrid topic |
| `min_frontier_size` | `5` | Minimum cells for a valid frontier cluster |
| `rate_hz` | `1.0` | Detection loop frequency |
| `goal_tolerance` | `0.5` | Meters; re-scan when this close to goal |
| `blacklist_radius` | `1.0` | Meters; ignore goals near previous failures |

### Goal Selection Strategy

The current implementation selects the **nearest** frontier centroid that is not blacklisted. When a Nav2 goal fails (rejected or aborted), the frontier is added to a blacklist. If all frontiers are blacklisted, the list is cleared for a fresh attempt.

Alternative strategies (not yet implemented):
- **Largest frontier first** -- explore big unknowns before small gaps
- **Information gain** -- weight by expected map coverage
- **Cost-aware** -- factor in Nav2 path cost, not just Euclidean distance

---

## Usage

### Standalone exploration (map-building session)

```bash
ros2 launch lunabot_2425 competition_sim.launch.py world:=ucf_arena
# Wait for stack to initialize (~35s), then enable exploration:
ros2 service call /frontier_explorer/enable std_srvs/srv/SetBool "{data: true}"
```

### Competition routine (mission supervisor controls WFD)

When using `mission_supervisor`, the FSM enables frontier exploration as a fallback if waypoint-based navigation times out:

1. Mission starts in LOCALIZING
2. On timeout or success, transitions to TRAVERSING
3. If traversal times out, FSM calls `frontier_explorer/enable` with `true`
4. When exploration completes or target zone is reached, FSM disables WFD

### Disabling for competition

If the mission uses only waypoint navigation:

```bash
ros2 launch lunabot_2425 competition_sim.launch.py
# frontier_explorer starts disabled (enabled:=false)
# mission_supervisor has use_frontier_exploration:=true by default
# set to false to never use WFD:
ros2 param set /mission_supervisor use_frontier_exploration false
```

---

## Tuning Guide

| Symptom | Fix |
|---------|-----|
| Too many tiny frontiers | Increase `min_frontier_size` (e.g., 10-20) |
| Robot oscillates between two frontiers | Increase `blacklist_radius` |
| Exploration too slow | Increase `rate_hz` to 2.0 |
| Robot tries to reach unreachable frontiers | Nav2 will abort; frontier auto-blacklisted |
| Frontiers detected inside walls | Check RTAB-Map grid quality; ensure `Grid/RayTracing: true` |

---

## Design Decisions

1. **Pure Python, no external deps beyond ROS:** The BFS runs on numpy arrays converted from the OccupancyGrid. No C++ compilation needed.

2. **Enable/disable via service:** The mission FSM can toggle exploration without killing the node. This supports the competition flow where WFD is only needed during specific phases.

3. **Blacklist mechanism:** Failed goals are blacklisted to prevent infinite retries on unreachable frontiers. The blacklist is cleared when exploration is re-enabled or when all frontiers are blacklisted.

4. **Nearest-first selection:** Simpler and more predictable than information-gain metrics. Good enough for the bounded Lunabotics arena where full coverage isn't the goal.
