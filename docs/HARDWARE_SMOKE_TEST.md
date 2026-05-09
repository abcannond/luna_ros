# Hardware smoke test

Two scenarios are documented here:

- **[Part A](#part-a-laptop-single-camera-test)** — one USB camera plugged into your laptop. Quick sanity check that the camera streams and detects AprilTag 36h11 ID 38.
- **[Part B](#part-b-full-jetson-test-3-cameras--realsense)** — full robot bringup on the Jetson: 3 fiducial cameras + RealSense D455 + fiducial localizer.

---

## Test marker

- Family: **AprilTag 36h11** (`DICT_APRILTAG_36h11`)
- ID: **38**
- Side length: **0.17 m** (update `marker_size_m` if you print at a different size)
- Well-lit, held square to the lens, ~0.5–1.5 m away.

---

## Part A: Laptop single-camera test

Goal: camera streams images, overlay node draws a **green box + axes + pose** on
AprilTag 36h11 ID 38. One USB camera plugged into the laptop, everything runs
locally inside the container (cwd = `/ros2_ws`).

### A1. Find the device node

```bash
v4l2-ctl --list-devices
```

Note the `/dev/video*` for your camera (lowest-numbered node under its USB block,
e.g. `/dev/video2` for a NexiGo).

### A2. Start the camera — Terminal 1

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/test_cam \
  -p video_device:=/dev/video2 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0 \
  -p pixel_format:=mjpeg2rgb
```

Confirm it is publishing:

```bash
ros2 topic hz /test_cam/image_raw   # expect ~30 Hz
```

### A3. Run the detection overlay — Terminal 2

```bash
python3 src/helpers/fiducial_overlay_rclpy.py \
  --ros-args \
  -p image_topic:=/test_cam/image_raw \
  -p camera_info_topic:=/test_cam/camera_info \
  -p aruco_dict:=DICT_APRILTAG_36h11 \
  -p marker_id:=38 \
  -p marker_size_m:=0.17 \
  -p show_window:=true
```

A window **"Fiducial Overlay (/camera/image_annotated)"** opens:
- All detected AprilTag 36h11 tags get a white border + ID label.
- **Tag 38 gets a green box + 3-axis pose overlay.**

Hold the tag ~0.5–1 m from the camera. The axes should lock on and follow the
tag as you move it.

**No window?** (`No DISPLAY` error) — run `rqt_image_view` in a third terminal
and subscribe to `/camera/image_annotated` instead.

**Pose axes wrong scale?** — measure the actual printed side length and re-run
with the correct `-p marker_size_m:=<meters>`.

**Tag not detected?** — verify OpenCV has AprilTag support:
```bash
python3 -c "import cv2; print(cv2.aruco.DICT_APRILTAG_36h11)"
```
If this errors, install `opencv-contrib-python` (not plain `opencv-python`).

---

## Part B: Full Jetson test (4 cameras + RealSense)

Goal: on the Jetson, verify the **RealSense D455 publishes depth** and the
**fiducial localizer publishes `/fiducial_pose`** when an AprilTag 36h11 marker
(ID 38 in our test prints) is in view of any of four cameras. No RTAB-Map, no
EKF, no Nav2 — those layer on after the smoke test passes.

Active cameras: `front_left`, `front_right`, `back_left`, `back_right` at 90°
spacing. The bringup launch publishes one `usb_cam` node per position with
matching static TFs.

### B1. SSH to the Jetson and enter the container

```bash
ssh <user>@<jetson-ip>
./run_ros_image.sh luna_ros:jetson
cd /ros2_ws
```

### B2. Build (first run or after edits)

```bash
colcon build --symlink-install --packages-select fiducial_localizer lunabot_2425
source install/setup.bash
```

### B3. Identify the four USB camera device nodes

```bash
v4l2-ctl --list-devices
```

Note the `/dev/video*` for each Nexigo (lowest-numbered node per USB block,
e.g. `/dev/video0/2/4/6`). Plug cameras in before launching so device numbering
is stable. The RealSense appears too — ignore it, the driver opens it directly.

### B4. Launch the bringup with smoke-test flags — Terminal 1

The smoke test is now a flag on `hardware_bringup.launch.py`:

```bash
ros2 launch lunabot_2425 hardware_bringup.launch.py \
  launch_localizer:=true \
  launch_rviz:=true \
  fid_front_left_dev:=/dev/video0 \
  fid_front_right_dev:=/dev/video2 \
  fid_back_left_dev:=/dev/video4 \
  fid_back_right_dev:=/dev/video6
```

To use the AprilTag 36h11 preset YAML instead of the default DICT_4X4_50:

```bash
ros2 launch lunabot_2425 hardware_bringup.launch.py \
  launch_localizer:=true launch_rviz:=true \
  localizer_params:=$(ros2 pkg prefix fiducial_localizer)/share/fiducial_localizer/params/multi_camera_hardware_apriltag.yaml
```

Or override marker dictionary / size on the fly without editing YAML:

```bash
ros2 launch lunabot_2425 hardware_bringup.launch.py \
  launch_localizer:=true launch_rviz:=true \
  marker_dict:=DICT_APRILTAG_36h11 marker_size_m:=1.0
```

Watch for:
- `RealSense Node Is Up!`
- Four `usb_cam` startup lines.
- After ~4 s: `multi_camera_marker_localizer` activates via lifecycle manager.
- RViz window opens with the hardware config (see B6).

### B5. Verify — Terminal 2

```bash
source install/setup.bash
```

**RealSense:**
```bash
ros2 topic hz /camera/camera/depth/image_rect_raw   # expect ~15 Hz
ros2 topic hz /camera/camera/color/image_raw        # expect ~15 Hz
ros2 topic echo /camera/camera/depth/camera_info --once  # K matrix populated
```

**Fiducial cameras (all four):**
```bash
for c in front_left front_right back_left back_right; do
  ros2 topic hz /fid_cams/${c}_camera/image_raw     # expect ~30 Hz each
done
ros2 topic echo /fid_cams/front_left_camera/camera_info --once
```

Each `camera_info` must have a non-zero K matrix. If all zeros the camera is
uncalibrated — PnP will give garbage. Calibrate with
`ros2 run camera_calibration cameracalibrator`.

**TF tree:**
```bash
ros2 run tf2_tools view_frames
```
Expect: `base_link → {front_left,front_right,back_left,back_right}_camera_link_optical`
and `base_link → camera_link`.

**Fiducial pose:**
```bash
ros2 topic hz /fiducial_pose
ros2 topic echo /fiducial_pose --once
```

Hold the marker ~1 m from any camera. `hz` should tick; move the marker and
`pose.position` should track it.

| Symptom | Cause |
|---|---|
| "marker detected" logged but no `/fiducial_pose` | TF lookup `base_link → camera frame` failing — check static TFs. |
| No "marker detected" at all | Wrong family, or marker too small/far. Confirm dictionary matches the YAML; try ~0.5–1 m, well lit. |
| `Camera N TF lookup failed` | `base_link → <pos>_camera_link_optical` static TF not running. |
| `K matrix all zeros` | usb_cam not loading calibration — pose math will diverge. Calibrate first. |

### B6. RViz visualization (`launch_rviz:=true`)

The RViz window opened by the launch loads
`lunabot_2425/config/hardware.rviz`. It works for both the smoke test and the
full SLAM stack — displays without their topics simply render empty.

| Display | Topic | When it lights up |
|---|---|---|
| **RealSense Cloud** (PointCloud2) | `/camera/camera/depth/color/points` | Always — live D455 cloud at ~15 Hz |
| **RTAB-Map Cloud** (PointCloud2, *off by default*) | `/rtabmap/cloud_map` | When `rtabmap_nav2_hardware.launch.py` is also running. Enable in the Displays panel. |
| **Map** | `/map` | When RTAB-Map / Nav2 is publishing the occupancy grid |
| **Fiducial Pose** (PoseWithCovariance) | `/fiducial_pose` | Red arrow appears when any camera sees a marker |
| **Front/Back × Left/Right Cam** (Image × 4) | `/fid_cams/<pos>_camera/image_raw` | Always when bringup is running |
| **TF + RobotModel + Grid** | TF tree, `/robot_description` | Always |

Fixed Frame defaults to `map`. If you haven't started anything publishing
`map → odom` yet, switch it to `base_link` from the Displays panel for the
smoke test.

### B7. Shut down

`Ctrl+C` in Terminal 1. The lifecycle manager deactivates the localizer cleanly.

---

## What this does NOT verify

- Wheel/visual odometry or `/odom`.
- `map → odom` EKF fusion.
- RTAB-Map SLAM or Nav2.
- Static-TF accuracy — the `base_link → camera_link*` values in
  `hardware_bringup.launch.py` are placeholder mounts. Measure the real offsets
  before trusting `/fiducial_pose` for navigation.

For the full stack (RTAB-Map + EKF + Nav2) start the bringup without the smoke
flags, then in a second terminal:

```bash
ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py
```

The same `hardware.rviz` config will then also show `/map` and the RTAB-Map
cloud (enable it in the Displays panel). See `docs/HARDWARE.md`.
