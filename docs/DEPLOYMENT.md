# Deployment profiles

One stack, different launch combinations. Pick a profile and use only the launches listed for it unless you are debugging.

## Profile: sim competition (default)

**Goal.** Full Lunabotics-style run in Gazebo with RTAB-Map, Nav2, zones, mission supervisor, optional frontier fallback.

**Launch.**

```bash
ros2 launch lunabot_2425 competition_sim.launch.py world:=ucf_arena
# or world:=artemis_arena
```

**Then.**

```bash
ros2 service call /mission_supervisor/start_mission std_srvs/srv/Trigger {}
```

**Notes.** `use_rtabmap_odom` is enabled from `gz_bringup` → mapping include. `luna_cont` has **`enable_odom_tf: false`**. Static **`world`→`map`** aligns mission goals with the spawned pose.

**Doc.** [COMPETITION_SIM.md](COMPETITION_SIM.md)

---

## Profile: sim Gazebo only (no RTAB-Map)

**Goal.** Gazebo plus cameras for visual check, or manual mapping experiments.

**Launch.**

```bash
ros2 launch lunabot_2425 gz_bringup.launch.py launch_mapping:=false world:=ucf_arena
```

**Notes.** `depth_to_pointcloud` may run when mapping is off. Do not also start the full mapping launch without understanding duplicate publishers on shared topics.

---

## Profile: hardware field mapping

**Goal.** RealSense, fiducial cams, robot odom, RTAB-Map, Nav2 on the physical robot.

**Launches (two terminals).**

1. `ros2 launch lunabot_2425 hardware_bringup.launch.py` (and device overrides as needed)
2. `ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py`

**Notes.** **`use_sim_time:=false`**, **`sim:=false`**. Base driver must publish **`/odom`** and **`odom`→`base_link`**. Checklist in [HARDWARE.md](HARDWARE.md).

---

## Profile: dev and legacy launches

Other launch files (`d455_rtabmap.launch.py`, `nav2_bringup_rtabmap.launch.py`, `rs_launch_custom.py`, arena-only under `luna_ros2_worlds`) are for **development** or one-off tests. They are listed under **Repository inventory** in [ARCHITECTURE.md](ARCHITECTURE.md). Prefer the three profiles above for team onboarding.

---

## Docker

Image build and run are described in the root [README.md](../README.md) and [Dockerfile](../Dockerfile). Jetson uses [Dockerfile.jetson](../Dockerfile.jetson).
