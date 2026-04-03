# Competition simulation routine

How to run the full Lunabotics-style autonomy stack in **Gazebo** and start the **mission supervisor** FSM. Use this if you just cloned the repo and want a single, copy-paste flow.

## Prerequisites

- ROS 2 **Jazzy** and this workspace built (typically inside the project Docker image).
- From the workspace root after build:

```bash
cd /ros2_ws && source install/setup.bash
```

## One-terminal launch

Starts **Gazebo**, **RTAB-Map**, **Nav2**, **RViz** (competition layout), **zone_publisher**, **frontier_explorer**, and **mission_supervisor**.

**UCF arena (default world in launch):**

```bash
ros2 launch lunabot_2425 competition_sim.launch.py world:=ucf_arena
```

**Artemis arena:**

```bash
ros2 launch lunabot_2425 competition_sim.launch.py world:=artemis_arena
```

Wait until Gazebo is up, the robot has spawned, controllers are active, and RTAB-Map/Nav2 have finished starting (often **~30–40 seconds** after launch; watch the terminal for errors).

## Start the mission

The supervisor begins in `IDLE`. Call:

```bash
ros2 service call /mission_supervisor/start_mission std_srvs/srv/Trigger {}
```

That begins the FSM: localization window → short **stationary** warmup → first Nav2 goal toward excavation, then construction, return, etc. Details and phases: [AUTONOMY.md](AUTONOMY.md).

**Note:** The mission no longer commands an **in-place 360° spin** at startup. That rotation was triggering depth/costmap trailing in sim; warmup is now a brief **stationary** delay only (`map_warmup_s`, default **3 s** in `competition_sim.launch.py`).

## Optional: two terminals

If you prefer Gazebo alone first, then mapping:

1. `ros2 launch lunabot_2425 gz_bringup.launch.py world:=ucf_arena`
2. In another shell: `ros2 launch luna_mapping rtabmap_nav2_sim.launch.py launch_rviz:=true`  
   (You would need to start autonomy nodes separately; the single `competition_sim.launch.py` flow above is recommended for a full routine.)

## Related docs

| Doc | Contents |
|-----|----------|
| [AUTONOMY.md](AUTONOMY.md) | Zones, frontier explorer, mission FSM phases |
| [STACK_GUIDE.md](STACK_GUIDE.md) | Stack overview, tuning, troubleshooting |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Nodes, topics, launch graph |
