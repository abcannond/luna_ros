# Hardware smoke test — RealSense + 3 fiducial cams

Goal: on the Jetson, verify that the **RealSense D455 publishes depth** and the
**fiducial localizer publishes `/fiducial_pose`** when a single ArUco marker is
in view of any of three cameras. No RTAB-Map, no EKF, no Nav2.

Test marker:
- Dictionary: `DICT_4X4_50`
- ID: any (`marker_id: -1` — the localizer accepts all IDs in the dictionary)
- Outer black border: **0.50 m** square
- Held flat on a wall or board, well-lit, ~1–2 m from a camera.

Active cameras: `front_left`, `front_right`, `back_left`. (Edit
`ACTIVE_POSITIONS` near the top of `hardware_smoke.launch.py` if you swap
which three positions are plugged in.)

---

## 1. SSH to the Jetson and enter the container

```bash
ssh <user>@<jetson-ip>
./run_ros_image.sh luna_ros:jetson         # whichever wrapper you use
```

Inside the container:

```bash
cd /ros2_ws
```

## 2. Build the workspace (only on first run, or after edits)

```bash
colcon build --symlink-install --packages-select fiducial_localizer lunabot_2425
source install/setup.bash
```

## 3. Identify the three USB camera device nodes

```bash
v4l2-ctl --list-devices
```

Note the `/dev/video*` entry for each Nexigo (usually the lowest numbered video
node listed under each USB camera block — e.g. `/dev/video0`, `/dev/video2`,
`/dev/video4`). Plug the cameras in **before** running the launch so the device
numbering is stable. The RealSense will also appear in this list — ignore it,
the realsense2_camera driver opens it directly.

## 4. Launch the smoke stack

In **Terminal 1** (cameras + localizer):

```bash
ros2 launch lunabot_2425 hardware_smoke.launch.py \
  fid_front_left_dev:=/dev/video0 \
  fid_front_right_dev:=/dev/video2 \
  fid_back_left_dev:=/dev/video4
```

Watch for:
- `RealSense Node Is Up!` (or similar from realsense2_camera).
- Three `usb_cam` startup lines, one per camera.
- After ~4 s, `multi_camera_marker_localizer` configures and activates via
  `lifecycle_manager_fiducial_smoke`.

If a camera fails to open, the device path is wrong or the camera is busy —
unplug/replug, rerun `v4l2-ctl --list-devices`, and pass the corrected path.

## 5. Verify in a second terminal

In **Terminal 2** (in the same container):

```bash
source /ros2_ws/install/setup.bash
```

### 5a. RealSense depth and color

```bash
ros2 topic hz /camera/camera/depth/image_rect_raw     # expect ~15 Hz
ros2 topic hz /camera/camera/color/image_raw          # expect ~15 Hz
ros2 topic echo /camera/camera/depth/camera_info --once   # K matrix populated
```

#### Quick image check — rqt_image_view (lightest weight)

X11 forwarding or a local display required for all GUI options below.

```bash
ros2 run rqt_image_view rqt_image_view
```

Pick `/camera/camera/depth/image_rect_raw` and `/camera/camera/color/image_raw`
from the dropdown. Depth renders as a grayscale image — brighter = closer.

#### Full 3-D point cloud — RViz2

**On the Jetson** (with a display or `ssh -X`):
```bash
rviz2
```

**On your laptop** (same network — less load on the Jetson):
```bash
export ROS_DOMAIN_ID=<same as jetson>
rviz2
```

Inside RViz2:

1. Set **Fixed Frame** → `camera_link`.
2. Add → **Image** → topic `/camera/camera/color/image_raw` — live color feed.
3. Add → **PointCloud2** → topic `/camera/camera/depth/color/points` — aligned
   point cloud. Set **Color Transformer** to `RGB8` for a color overlay, or
   `AxisColor` to see depth by hue.
4. *(Optional)* Add → **Image** → topic `/camera/camera/depth/image_rect_raw`
   for a side-by-side depth image panel.

The point cloud topic only publishes because `align_depth.enable: true` is set
in the launch file. If the PointCloud2 display is empty, confirm the topic is
live:

```bash
ros2 topic hz /camera/camera/depth/color/points   # expect ~15 Hz
```

If the topic is missing entirely, check that the RealSense node started without
errors and that `align_depth.enable` wasn't overridden.

### 5b. Fiducial cameras

```bash
ros2 topic hz /fid_cams/front_left_camera/image_raw   # expect ~30 Hz
ros2 topic hz /fid_cams/front_right_camera/image_raw
ros2 topic hz /fid_cams/back_left_camera/image_raw
ros2 topic echo /fid_cams/front_left_camera/camera_info --once  # K populated
```

Each `camera_info` topic should show a non-zero K matrix from the camera's
calibration; if it's all zeros, the camera is publishing but uncalibrated and
the PnP solve will be garbage. Run a calibration if needed
(`ros2 run camera_calibration cameracalibrator …`).

### 5c. TF tree (sanity)

```bash
ros2 run tf2_tools view_frames
```

Check that `base_link → front_left_camera_link_optical`,
`… → front_right_camera_link_optical`, `… → back_left_camera_link_optical`,
and `base_link → camera_link` all exist.

### 5d. Fiducial pose

Hold the printed 50 cm DICT_4X4_50 id-50 marker in front of one of the active
cameras at ~1 m, square to the lens.

```bash
ros2 topic hz /fiducial_pose
ros2 topic echo /fiducial_pose --once
```

Expected:
- `hz` ticks at a few Hz while the tag is in view (drops to 0 when occluded).
- `header.frame_id == "map"`.
- `pose.pose.position` shows the rover's position relative to the marker
  (since `world_to_marker_xyz` is `[0,0,0]` in this YAML, this is just the
  rover-frame ↔ marker-frame offset).

Move the marker side-to-side: `pose.position.x/y` should change in the
expected direction. Rotate the marker: yaw should change.

If `/fiducial_pose` never appears:

| Symptom | Cause |
|---|---|
| `multi_camera_marker_localizer` log says "marker detected" but no pose | TF lookup base_link → camera frame failing — check static TFs are alive. |
| No "marker detected" log at all | Wrong dictionary, or marker too small/far. Confirm the print is `DICT_4X4_50` and try at ~1 m, well lit. |
| `Camera N TF lookup failed` warns | The `base_link → <pos>_camera_link_optical` static TF isn't running (see launch). |
| `K matrix all zeros` for that camera | usb_cam isn't loading a calibration — pose math will diverge. Calibrate the camera. |

## 6. Shut down

`Ctrl+C` in Terminal 1 stops everything. The lifecycle manager will deactivate
the localizer cleanly on shutdown.

---

## What this does NOT verify

- Wheel/visual odometry (no `/odom` source running).
- `map → odom` from the EKF.
- RTAB-Map SLAM or the Nav2 stack.
- Static-TF accuracy: the `base_link → camera_link*` transforms in
  `hardware_smoke.launch.py` are placeholders. They affect only how
  `/fiducial_pose` *interprets* a detection — depth visualization and raw
  image streams are independent of them. Once you trust the pose magnitude
  qualitatively, measure your real mount offsets and update the launch file.

For the full hardware stack (RTAB-Map + EKF + Nav2 + RViz) use
`rtabmap_nav2_hardware.launch.py` after `hardware_bringup.launch.py`; see
`docs/HARDWARE.md`.
