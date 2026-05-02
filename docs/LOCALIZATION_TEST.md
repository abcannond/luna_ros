# Fiducial Localization Test — Home Sandbox

Validates that the fiducial localizer produces an accurate, stable `/fiducial_pose`
in the map frame using the home sandbox arena and the physical multi-marker board
(outer tag 256 @ 0.88 m, inner tag 395 @ 0.125 m, `DICT_5X5_1000`).

This builds on the hardware smoke test (see `docs/HARDWARE_SMOKE_TEST.md` Part B),
which must pass before attempting this test.

---

## Arena and coordinate frame

```
Bird's eye view — home sandbox

    Y
    ^
    |
    |   [TAG on left wall, face pointing +X]
    |
    +-----------> X
  (0,0)
  Robot start
```

- **Map origin (0, 0, 0):** bottom-left inner corner of the sandbox frame, floor level.
- **+X:** rightward into the arena (robot forward direction).
- **+Y:** along the left wall.
- **+Z:** up.

The robot starts with `base_link` at or near the origin, facing +X.

### Tag world pose (measured)

| Parameter | Value | Notes |
|---|---|---|
| `marker_world_x` | -0.04 m | Board face is 4 cm behind the corner beam |
| `marker_world_y` | 0.4625 m | Center of 92.5 cm board, starting at Y=0 |
| `marker_world_z` | 0.92 m | Height of tag center from floor |
| `marker_world_roll_arr` | 1.5708 | Tag is vertical (wall-mounted) |
| `marker_world_pitch_arr` | 0.0 | — |
| `marker_world_yaw_arr` | 0.0 | Tag face points +X (into arena) |

These values are already set in
`ros2_ws/src/fiducial_localizer/params/multi_camera_hardware.yaml`.

---

## Prerequisites

- Hardware smoke test (Part B) passes — all four cameras stream at ~30 Hz,
  RealSense at ~15 Hz, TF tree is complete.
- The marker board is mounted on the left wall of the sandbox as described above.
- The robot starts at the map origin (bottom-left corner), facing +X.

---

## Running the test

### Step 1 — SSH to the Jetson and enter the container

```bash
ssh <user>@<jetson-ip>
./run_ros_image.sh luna_ros:jetson
cd /ros2_ws
```

### Step 2 — Build if needed

```bash
colcon build --symlink-install --packages-select fiducial_localizer lunabot_2425
source install/setup.bash
```

### Step 3 — Launch bringup with localizer and RViz

```bash
ros2 launch lunabot_2425 hardware_bringup.launch.py \
  launch_localizer:=true \
  launch_rviz:=true \
  fid_front_left_dev:=/dev/video0 \
  fid_front_right_dev:=/dev/video2 \
  fid_back_left_dev:=/dev/video4 \
  fid_back_right_dev:=/dev/video6
```

Wait for:
- Four `usb_cam` startup lines.
- After ~4 s: `multi_camera_marker_localizer` activates via lifecycle manager.
- RViz window opens.

In RViz, set **Fixed Frame** to `map`.

### Step 4 — Verify pose is publishing — Terminal 2

```bash
source install/setup.bash
ros2 topic hz /fiducial_pose        # should tick when any camera sees the tag
ros2 topic echo /fiducial_pose --once
```

With the robot at the origin facing +X, the reported pose should be close to `(0, 0, 0)`
with a yaw near 0. It will not be exact because the camera TF offsets are still
placeholder values — see Known Limitations below.

---

## Validation checks

### Check 1 — Pose is plausible at the start position

Place the robot at the map origin. Echo `/fiducial_pose`. Expected approximately:

```
position:
  x: ~0.0   (small offset due to placeholder camera TFs)
  y: ~0.0
  z: ~0.0
```

If the numbers are wildly wrong (e.g. x: 5.0), check that the tag world pose
values in the YAML match what is described above.

### Check 2 — Pose tracks correctly when the robot moves

Drive the robot 0.5 m forward along +X (toward the tag). Echo `/fiducial_pose`
again. The reported X should decrease by roughly 0.5 m (robot is now closer to
the tag wall). Y and Z should be roughly unchanged.

### Check 3 — Pose is stable when stationary

With the robot still, run:

```bash
ros2 topic echo /fiducial_pose
```

Watch 10–20 readings. `position.x` and `position.y` should vary by less than
~0.05 m between readings. Large drift means the tag is at the edge of detection
range or the camera K matrix is uncalibrated.

### Check 4 — Multiple cameras contribute

Block the camera that has the clearest line of sight to the tag. `/fiducial_pose`
should keep publishing if another camera also sees the tag. If it stops, only one
camera is detecting — investigate lighting or tag distance for the other cameras.

---

## RViz indicators

| Display | What to look for |
|---|---|
| **Fiducial Pose** (red arrow) | Appears and tracks the robot position in the map frame |
| **Camera feeds** (Image × 4) | Tag visible and annotated in at least one feed |
| **TF / RobotModel** | `base_link` appears near the origin at robot start |

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `/fiducial_pose` never publishes | Tag not detected — check tag is `DICT_5X5_1000`, IDs 256/395, well lit, within 1 m (`max_marker_distance_m`) |
| Pose is stable but offset by a fixed amount | Camera TF placeholders incorrect — measure real camera offsets on the rover and update static TFs in `hardware_bringup.launch.py` |
| Pose jumps erratically | Camera K matrix is all zeros — calibrate cameras with `ros2 run camera_calibration cameracalibrator` |
| Pose stops when one camera blocked | Other cameras are not detecting — likely lighting or distance; try moving the robot closer or improving lighting |
| `Camera N TF lookup failed` in logs | Static TF for that camera frame is not running |

---

## Known limitations

- **Camera TF offsets are placeholders.** The static transforms from `base_link`
  to each camera optical frame in `hardware_bringup.launch.py` are approximate.
  The localization will have a constant positional bias until these are measured
  from the real robot and updated. For navigation, measure the true offsets.

- **`max_marker_distance_m: 1.0`** is conservative. At competition the tag may
  need to be seen at 2–4 m. Increase this value and retest at operating range
  before competition.

- **Single tag, single group.** This test uses one tag board (`active_marker_group: 0`).
  Full arena localization may require additional boards at other zone boundaries.
