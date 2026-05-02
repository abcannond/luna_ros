#!/bin/bash
python3 "$(dirname "$0")/fiducial_overlay_rclpy.py" \
  --ros-args \
  -p image_topic:=/test_cam/image_raw \
  -p camera_info_topic:=/test_cam/camera_info \
  -p aruco_dict:=DICT_APRILTAG_36h11 \
  -p marker_id:=38 \
  -p marker_size_m:=0.5 \
  -p show_window:=true
