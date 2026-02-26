# Stack review: Lunabotics competition fit

This document reviews the current approach to pathfinding, map creation, and obstacle detection and whether it is sound for the **Lunabotics competition** (autonomous/telerobotic excavation, regolith, berm construction, arena navigation). It also covers the **ground crop** tradeoff, **computational limits**, and **risks** so you are not blindsided later.

---

## Competition context (what autonomy must do)

- **Lunabotics:** Robot must navigate a simulated lunar surface (UCF / Artemis arena), excavate regolith (BP-1 simulant), and deposit it to build berm structures. Autonomy is highly rewarded.
- **Implications:** The robot must (1) navigate reliably between zones (starting, excavation, obstacle, construction), (2) avoid static and dynamic obstacles (rocks, berms, other robots?), (3) operate on regolith and possibly uneven or sloped terrain, and (4) do so within time and compute limits (e.g. on a Jetson).

The stack is built for: **SLAM (RTAB-Map) + occupancy grid + Nav2 costmaps + depth-based obstacles**. That fits the “navigate the arena and avoid obstacles” part. Excavation and deposition logic sit on top of this (waypoints, zone publisher, etc.).

---

## Is the ground crop a good idea?

### What it does

The **ground crop** (`ground_crop_bottom`, currently `0.1` = bottom 10% of the depth image) zeros out the bottom rows of the depth image **only for the pipeline that feeds the voxel layer** (point cloud → Nav2 local costmap). RTAB-Map and the laserscan node use **raw** depth (no crop).

- **Voxel layer** (local costmap): fed by `/camera/camera/depth/color/points`, which is built from **filtered** (ground-cropped) depth. So the floor in front of the robot is largely removed from the point cloud → less costmap “trail” when turning.
- **RTAB-Map:** raw depth → the **map** still sees the full scene, including floor. The map is used as the **static layer** for both global and local costmaps, so once something is in the map, it stays as obstacle/free.
- **Laserscan** (`/scan`): currently **raw** depth (see launch file comment: “filtered was too aggressive, no obstacles”). So the 2D scan still sees the full vertical slice. In our current **nav2_rtabmap_params** the local costmap does **not** use the scan — it uses only static layer + voxel (point cloud) + inflation. So the scan is available but not used for the costmap in the RTAB-Map config.

**Conclusion on ground crop:** It is a **reasonable tradeoff for flat arenas**: it reduces costmap trailing and floor noise in the voxel layer while the static map (from raw depth) still captures obstacles once they are observed. So we don’t “delete” obstacles from the world model; we only reduce live floor points in the voxel layer.

### Where it can pose issues

1. **Low obstacles in the cropped band**  
   A low rock or berm edge that appears **only** in the bottom 10% of the image will not be in the point cloud → the **voxel layer** will not mark it as obstacle. The robot could drive into it before that area is in the **static map**. So:
   - **Risk:** First-time exposure to a low obstacle that sits only in the cropped region.
   - **Mitigation:** The static map (RTAB-Map, raw depth) will add it as soon as it’s seen in the rest of the image or on a later frame. So the risk is mostly “first pass” and depends on obstacle height and camera pose. Keeping `ground_crop_bottom` modest (e.g. 0.1–0.15) limits the blind band.

2. **Slopes, ramps, tilted robot**  
   “Bottom of image” is not “ground” if the robot is tilted or the terrain slopes. Then we might zero out part of a ramp or the base of a berm and treat it as free in the voxel layer. Again, the static map (raw depth) will still see it, so after a few frames the static layer has it. So risk is short-term and geometry-dependent.

3. **Future: craters / uneven terrain**  
   If the competition moves to more lunar-like craters or strong uneven terrain, a **fixed image-row crop** is a hack. A more robust approach is **height-based filtering** in the point cloud (e.g. remove points below `z < base_link.z + threshold` in `base_link` or `odom`). That would require a small node that subscribes to the point cloud and filters by height before publishing to the voxel layer. The current crop is simple and good for flat floors; document it and plan for a height-based filter if the arena spec changes.

### Recommendation

- **Keep the ground crop** for the voxel pipeline; it fixes a real problem (trailing) and the static map still gets full depth.
- **Config:** `ground_crop_bottom` is fixed at 0.1 in the launch (depth_fov_filter node param). To change it you’d edit the launch or add a config file; the run is intended to use one tested configuration.
- **If the competition introduces slopes/craters:** Consider a height-based ground filter in front of the voxel layer instead of (or in addition to) the image crop.

---

## Pathfinding / map / obstacle pipeline vs competition

### What the stack does

| Component | Role | Competition fit |
|-----------|------|------------------|
| **RTAB-Map** | SLAM from RGB-D; publishes `/map` and map→odom. Uses **raw** depth. | Good: single map, no predefined arena size; loop closures and visual odom option help when stuck. |
| **Static layer** | Global and local costmaps use RTAB-Map’s `/map`. | Good: obstacles and walls once seen are in the map; no need to re-detect. |
| **Voxel layer** | Local costmap only; live point cloud (ground-cropped). | Good: rocks and new obstacles in view get cost; tuning (mark_threshold, range, persistence) limits noise and trailing. |
| **No scan in local costmap** | nav2_rtabmap_params uses only static + voxel + inflation (no LaserScan layer). | Intentional: depth→scan is published but voxel (3D) is used so we get height and avoid treating floor as obstacle. Scan could be added as an extra observation source if you want redundancy. |
| **Global costmap** | Static + inflation only (no live obstacles). | Good: global plan follows the map; local costmap handles live obstacles. |
| **Planner** | NavfnPlanner (Dijkstra), allow_unknown. | Standard and adequate; tolerance 0.5 m. |
| **Behaviors** | Spin, backup, drive_on_heading, wait. | Enough for recovery; no custom “stuck in regolith” behavior. |

So the **approach is sound** for “navigate arena, avoid obstacles, get to waypoints.” Inflation and DWB critic scales are tuned so the robot keeps clearance from rocks while free space stays clearly passable; see **[STACK_GUIDE.md § Tuning and change log](STACK_GUIDE.md#tuning-and-change-log)** for what was changed and why. The main gaps are (1) dependency on a single depth camera, (2) no explicit handling of regolith (e.g. “don’t drive here” or traction), and (3) zone and odom assumptions (see below).

### Fundamental risks (don’t get blindsided)

1. **Single depth camera**  
   If the camera is occluded, dirty, or fails, you lose both mapping and live obstacles. There is no redundant sensor for obstacle detection. For competition, have a clear “sensor health” check (e.g. topic rate, or a simple “no points in point cloud for N seconds” fallback behavior).

2. **Zone publisher and odom source**  
   **Done:** `zone_publisher` now subscribes to **`/odom`** by default (configurable via the `odom_topic` parameter). Use the same odom source as Nav2; remap in launch if your controller publishes to a different topic (e.g. `/luna_cont/odom` → `/odom`).

3. **Zone bounds**  
   **Done:** Zone bounds are **configurable via ROS parameters**. Defaults match UCF/Artemis-style arena. Override per arena with a params file: `luna_nav/config/zone_publisher.yaml` (edit the `zone.*` lists and pass `--params-file` when running the node).

4. **Odom when stuck**  
   Already covered in STACK_GUIDE: when the robot is stuck, wheel odom can keep integrating and the map/planner drift. Using RTAB-Map visual odom and turning off the controller’s odom TF is the recommended fallback. That is the right architectural choice; just ensure the rest of the stack (including zone_publisher) uses a consistent odom source.

5. **No explicit “regolith” or “no-go” layer**  
   The costmap does not distinguish “regolith” from “solid obstacle.” If the rules ever require “do not drive on regolith” or “only drive in lanes,” you’d add a semantic or cost layer (e.g. from camera or a separate map). Not a flaw for current “avoid obstacles and navigate”; just a future extension point.

---

## Computational limits and optimizations

- **Jetson:** The stack is designed to run on a Jetson with the same launch as on x86. RTAB-Map and Nav2 can be CPU/GPU heavy; depth decimation and costmap size directly affect load.

- **Current tuning:**
  - **RTAB-Map:** `Grid/DepthDecimation: 5` (for 1280×720); `Rtabmap/DetectionRate: 2.0`; no database save (`database_path: ""`). This keeps map updates at a reasonable rate without overloading.
  - **Local costmap:** 4×4 m, 0.05 m resolution (80×80 cells); update 5 Hz, publish 2 Hz. Voxel: z_resolution 0.1 m, 16 voxels (1.6 m vertical). Point cloud is the only observation source for the voxel layer.
  - **Controller:** 10 Hz. Planner 20 Hz. Smoother 20 Hz.

- **What to watch:**
  - If the Jetson struggles, first reduce **costmap update_frequency** or **resolution** (e.g. 0.1 m for local), or increase RTAB-Map **DepthDecimation** (e.g. 6 if resolution allows). Then consider lowering **controller_frequency** (e.g. 5 Hz) as a last resort.
  - **Rtabmap/DetectionRate** already at 2.0 is a balance between map freshness and CPU; going higher may not be necessary and can increase load.

- **Recommendation:** Add a short “Performance tuning” subsection in the docs (or STACK_GUIDE) that lists these levers (decimation, costmap resolution/frequency, controller frequency) and when to use them (e.g. “if CPU is pegged on Jetson, try …”). No code change required; just document so you can adjust at competition without guessing.

---

## Fool-proofing checklist

- [x] **Ground crop:** Fixed at 0.1 in launch; documented. One tested config for automated run (no launch arg).
- [x] **Odom consistency:** Zone publisher uses `/odom` by default; configurable via `odom_topic` parameter so it matches Nav2.
- [x] **Zone bounds:** Configurable via parameters; see `luna_nav/config/zone_publisher.yaml`. Override per arena with `--params-file`.
- [ ] **Sensor fallback:** Define what happens if depth or point cloud stops (e.g. stop or slow, don’t assume free space).
- [ ] **Visual odom when stuck:** Use `use_rtabmap_odom:=true` and controller config without odom TF when slip/stuck is a concern; document in runbooks.
- [ ] **Performance:** Document decimation, costmap resolution/frequency, and controller frequency as tuning knobs for Jetson; test under load before competition.

---

## Summary

- **Ground crop:** Keep it; it’s a good tradeoff for flat arenas and reduces costmap trailing. Make it configurable, fix the 10% vs 28% comment, and plan for height-based filtering if the competition moves to slopes/craters.
- **Pathfinding/map/obstacles:** The approach (RTAB-Map static map + voxel layer for live obstacles, no scan in local costmap) fits Lunabotics navigation and obstacle avoidance. Main risks are single-camera dependency, zone/odom consistency, and hardcoded zone bounds; addressing those makes the stack more fool-proof.
- **Compute:** Current settings are reasonable; document tuning levers so you can adapt on Jetson without fundamental changes.
- **Not fundamentally flawed:** The design is coherent. The items above are hardening and documentation so you are not blindsided by arena changes, odom/zone mismatch, or performance limits at competition.
