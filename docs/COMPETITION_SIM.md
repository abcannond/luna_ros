# Competition simulation routine

How to run the full Lunabotics-style autonomy stack in **Gazebo** and start the **mission supervisor** FSM. Use this if you just cloned the repo and want a single, copy-paste flow.

## Prerequisites

- ROS 2 **Jazzy** and this workspace built (typically inside the project Docker image).
- From the workspace root after build:

```bash
cd /ros2_ws && source install/setup.bash
```

## Launch (this terminal blocks)

Starts **Gazebo**, **RTAB-Map**, **Nav2**, **RViz** (competition layout). **`mission_supervisor` and related autonomy nodes start only after a 30 s delay** from launch time (`competition_sim.launch.py` `TimerAction`). Until then, `/mission_supervisor/start_mission` does not exist, so `ros2 service call` will sit on **waiting for service to become available...**

Use a **second terminal** (another shell into the same container or host, with the same workspace sourced) for the service call below. Do not expect to run launch and the service call from the same prompt unless you background launch.

**UCF arena (default world in launch):**

```bash
ros2 launch lunabot_2425 competition_sim.launch.py world:=ucf_arena
```

**Artemis arena:**

```bash
ros2 launch lunabot_2425 competition_sim.launch.py world:=artemis_arena
```

Wait until you see **`Starting autonomy nodes (zone_publisher, frontier_explorer, mission_supervisor)...`** in the launch log, then **`mission_supervisor: phase=IDLE`** (often about **30–40 s** after you started launch). RTAB-Map and Nav2 are brought up earlier by `gz_bringup`; the mission node is intentionally late so the stack can settle.

### Before `start_mission` (sanity checks)

In the **second** terminal, with `source install/setup.bash`:

```bash
ros2 node list | grep mission_supervisor
ros2 service list | grep start_mission
```

You should see `/mission_supervisor` and `/mission_supervisor/start_mission`. If not, wait longer or check the launch terminal for crashes.

## Start the mission

The supervisor begins in `IDLE`. Call:

```bash
ros2 service call /mission_supervisor/start_mission std_srvs/srv/Trigger {}
```

You should get a quick response, for example `success: true` and `message: "Mission started -- LOCALIZING"`. If the client prints **`making request`** and then hangs with no result, check the **launch** terminal for a Python traceback in `mission_supervisor`.

That begins the FSM: localization window → short **stationary** warmup → first Nav2 goal toward excavation, then construction, return, etc. Details and phases: [AUTONOMY.md](AUTONOMY.md).

**Note:** The mission no longer commands an **in-place 360° spin** at startup. That rotation was triggering depth/costmap trailing in sim; warmup is now a brief **stationary** delay only (`map_warmup_s`, default **3 s** in `competition_sim.launch.py`).

## Optional: two terminals

If you prefer Gazebo alone first, then mapping:

1. `ros2 launch lunabot_2425 gz_bringup.launch.py world:=ucf_arena`
2. In another shell: `ros2 launch luna_mapping rtabmap_nav2_sim.launch.py launch_rviz:=true`  
   (You would need to start autonomy nodes separately; the single `competition_sim.launch.py` flow above is recommended for a full routine.)

## Field competition checklist (pit and SSH)

Short operational notes for **arena runs** (bandwidth, boot order). This repo uses **`hardware_bringup.launch.py`** and **`rtabmap_nav2_hardware.launch.py`** on the robot, ArUco via **`fiducial_localizer`**, and Nav2 params in **`nav2_rtabmap_params.yaml`**. Full physical setup: [HARDWARE.md](HARDWARE.md). Profiles: [DEPLOYMENT.md](DEPLOYMENT.md).

### Why SSH matters at KSC

Average link utilization is scored; long SSH sessions, log streaming, and extra tunnels add load. Use SSH on the Jetson for **bring-up and quick checks**, then **disconnect** when the robot should rely on the **WAP ↔ MCC** link.

### Pit sequence (adapt names to your runbook)

1. E-stop accessible; power and inspection items per rules.
2. Jetson on; RealSense and fiducial cams up; robot Wi‑Fi on team WAP; MCC on arena network. **`use_sim_time`** is **false** on hardware.
3. MCC: source workspace; start your mission control / monitoring stack (no back-channel Wi‑Fi to the robot if rules forbid it).
4. **SSH to Jetson** → source workspace → launch **`hardware_bringup.launch.py`**, then **`rtabmap_nav2_hardware.launch.py`** (two terminals; see [HARDWARE.md](HARDWARE.md)). Confirm cameras and `/odom` / TF.
5. **Exit SSH** when stable; verify robot still reachable from MCC.
6. Localize (fiducial + mapping policy per [AUTONOMY.md](AUTONOMY.md)); teleop in start zone if allowed; declare autonomy per MCJ when attempting scored segments.

**Design reference (team):** [Canva — competition flow](https://www.canva.com/design/DAG4uBCdxtM/6pPivqpfjFyfts0RoROFcQ/edit) *(internal; keep in sync with field procedure).*

### Clipboard (one page)

1. Boot robot, WAP, MCC network.  
2. MCC: monitoring / teleop when rules allow.  
3. SSH Jetson → hardware bringup → mapping/Nav2 → cameras OK → **exit SSH**.  
4. Localize; teleop alignment in start zone if needed.  
5. Autonomy when declared; E-stop and logger ready at end.

---

## Related docs

| Doc | Contents |
|-----|----------|
| [AUTONOMY.md](AUTONOMY.md) | Zones, frontier explorer, mission FSM phases |
| [STACK_GUIDE.md](STACK_GUIDE.md) | Launch variants, tuning, debugging |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Nodes, topics, TF, YAML index, inventory |
