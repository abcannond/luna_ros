#!/bin/bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/test_cam \
  -p video_device:=/dev/video2 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0 \
  -p pixel_format:=mjpeg2rgb
