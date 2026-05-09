#!/bin/bash
# Overlay for hardware multi-marker board: outer tag 256 (0.88m), DICT_5X5_1000.
# Pass a camera topic as the first arg, e.g.:
#   ./run_overlay_laptop.sh /fid_cams/front_left_camera/image_raw
IMAGE_TOPIC="${1:-/fid_cams/front_left_camera/image_raw}"
CAM_NS="${IMAGE_TOPIC%/image_raw}"

python3 "$(dirname "$0")/fiducial_overlay_rclpy.py" \
  --ros-args \
  -p image_topic:="${IMAGE_TOPIC}" \
  -p camera_info_topic:="${CAM_NS}/camera_info" \
  -p aruco_dict:=DICT_5X5_1000 \
  -p marker_id:=256 \
  -p marker_size_m:=0.88 \
  -p show_window:=true
