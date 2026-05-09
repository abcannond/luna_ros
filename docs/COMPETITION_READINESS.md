# Competition readiness assessment

Honest system-level review. This is not a pass or fail grade. It lists strengths, gaps, risks, and next steps so leads can prioritize work before field time.

---

## Localization

**Strengths.** RTAB-Map provides **`map`→`odom`** and, with **`use_rtabmap_odom`**, **`rgbd_odometry`** for **`odom`→`base_link`** and **`/odom`**. Static **`world`→`map`** at spawn aligns mission goals with the arena in sim. ArUco multi-camera node publishes **`/fiducial_pose`** for mission gating.

**Gaps.** Fiducial output is not fused into Nav2 or RTAB-Map in this tree as a single supported path. Drift and re-localization after long runs need a defined operator procedure (save map, reset odom, or manual intervention).

**Risks.** Visual odometry loss or poor depth can freeze or corrupt **`odom`→`base_link`**, which breaks Nav2 pose tracking. Sim uses rgbd odom with wheel TF disabled; hardware relies on driver odom; behavior differs if depth fails.

**Next steps.** Document recovery (pause mission, re-drive features, restart mapping) in field runbooks. If fusion is required, scope EKF or map-based correction as future work and track in issues.

---

## Perception and mapping

**Strengths.** Depth FOV filter and frame fixers normalize camera data for RTAB-Map. Competition sim launch wires the full rgbd graph.

**Gaps.** Lighting, reflectivity, and Gazebo sensor limits can differ from the arena. Depth dropouts are not automatically masked in all code paths beyond the configured filters.

**Risks.** Stale or missing depth leads to registration failures and VO stalls (linked to localization risks above).

**Next steps.** Record rosbag snippets from failed sim runs. Tune filter and RTAB-Map params against representative arena-like scenes when possible.

---

## Navigation

**Strengths.** Nav2 stack is parameterized in **`nav2_rtabmap_params.yaml`**. Costmaps consume **`/map`** and laser or scan sources as configured.

**Gaps.** Recovery behaviors may not cover every VO loss case. Controller and planner tuning is environment-specific.

**Risks.** Invalid goals, wrong frame, or stale TF produce immediate aborts or oscillation.

**Next steps.** After VO fixes, re-run full sim goals and log **`bt_navigator`** outcomes. Align **`goal_frame_id`** with documented deployment (see [DEPLOYMENT.md](DEPLOYMENT.md)).

---

## Mission FSM

**Strengths.** Phases, zones, and optional frontier fallback are centralized in **`mission_supervisor`**. Service **`/mission_supervisor/start_mission`** is the operator entry.

**Gaps.** Timeouts and retries are YAML and code defaults; not every failure mode has an automated fallback.

**Risks.** Preemption loops between mission logic and Nav2 were a known pain point; supervisor now cancels the active goal before sending a new one. Remaining edge cases need exercise in sim.

**Next steps.** Stress-test phase transitions with interrupted goals and map updates. Keep **`debug_ndjson_log`** off unless diagnosing; it adds subscriptions and file I/O.

---

## Operator workflow

**Strengths.** Single documented sim path: **`competition_sim.launch.py`**, wait, then **`start_mission`**. [COMPETITION_SIM.md](COMPETITION_SIM.md) describes timing and expectations.

**Gaps.** Hardware startup is multi-terminal; checklist in [HARDWARE.md](HARDWARE.md) must be followed in order.

**Risks.** Wrong **`use_sim_time`** or missing **`source install/setup.bash`** causes silent failures.

**Next steps.** Add competition-day dry runs with the pit sequence in [COMPETITION_SIM.md](COMPETITION_SIM.md#field-competition-checklist-pit-and-ssh).

---

## Logging and post-mortem

**Strengths.** Optional NDJSON logging exists for mission supervisor and depth FOV filter when **`debug_ndjson_log`** is true.

**Gaps.** No enforced rosbag policy in-repo for every run.

**Risks.** Without bags, VO and Nav2 failures are hard to replay.

**Next steps.** Define a minimal topic list for field runs (TF, odom, cmd_vel, costmap updates, mission state) and store bags with run IDs.

---

## Field reliability

**Strengths.** Docker and Jetson Dockerfiles exist. USB and camera notes are in [HARDWARE.md](HARDWARE.md).

**Gaps.** Network, power, thermal, and host clock sync are operator responsibilities; not continuously monitored in software here.

**Risks.** USB bandwidth and cable quality affect RealSense and extra cameras.

**Next steps.** Sign off [HARDWARE.md](HARDWARE.md) checklist after each lab session. Track Jetson image version and kernel.

---

## Cross-links to open engineering items

| Item | Where tracked |
|------|----------------|
| VO loss and stale **`odom`→`base_link`** | This doc, [STACK_GUIDE.md](STACK_GUIDE.md) (Debugging quick reference) |
| Mission and Nav2 preemption | Code: cancel-before-new-goal in **`mission_supervisor`**; validate in sim |
| Debug instrumentation | Params **`debug_ndjson_log`** default **false** on mission supervisor and depth FOV filter |
| Doc and code alignment | [ARCHITECTURE.md](ARCHITECTURE.md) (inventory + TF sections) |

---

## Unresolved or needs validation

Items below are explicit; they are not implied to work without test.

- **Verified in CI-style Docker:** `docker build` from this repo (image `luna_ros_ci`) and **`colcon build --symlink-install`** over the mounted workspace succeed; **`scripts/smoke_workspace.sh`** passes with `install` sourced. **Full Gazebo competition sim** remains a manual gate per developer GPU and display (see [CONTRIBUTING.md](CONTRIBUTING.md)).
- End-to-end **`colcon build --symlink-install`** and full sim run on every native (non-Docker) machine still depends on a correct Jazzy install.
- Long-duration mapping drift and fiducial-assisted correction on hardware.
- CI without GPU: full Gazebo smoke may stay manual; see [CONTRIBUTING.md](CONTRIBUTING.md).
