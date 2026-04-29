#!/usr/bin/env python3
"""
Hardware bringup: Intel RealSense D455 + 4× Nexigo N980P webcams (fiducial).

Uses usb_cam for the Nexigo cameras (MJPEG decode) to fit within USB 2.0 bandwidth.

Publishes topics matching simulation so the same mapping/nav stack can be used:
  - /camera/camera/color/*, /camera/camera/depth/*  (RealSense)
  - /fid_cams/front_left_camera/*, /fid_cams/front_right_camera/*, etc.

Usage (on Jetson or host with cameras attached):
  ros2 launch lunabot_2425 hardware_bringup.launch.py

Then in another terminal:
  ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    pkg_share = get_package_share_directory("lunabot_2425")
    realsense_share = get_package_share_directory("realsense2_camera")

    # Device paths for 4 Nexigo N980P (adjust for your USB layout; use v4l2-ctl --list-devices)
    declare_fl_dev = DeclareLaunchArgument(
        "fid_front_left_dev",
        default_value="/dev/video0",
        description="V4L2 device for front-left fiducial camera",
    )
    declare_fr_dev = DeclareLaunchArgument(
        "fid_front_right_dev",
        default_value="/dev/video2",
        description="V4L2 device for front-right fiducial camera",
    )
    declare_bl_dev = DeclareLaunchArgument(
        "fid_back_left_dev",
        default_value="/dev/video4",
        description="V4L2 device for back-left fiducial camera",
    )
    declare_br_dev = DeclareLaunchArgument(
        "fid_back_right_dev",
        default_value="/dev/video6",
        description="V4L2 device for back-right fiducial camera",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # ----- RealSense D455 -----
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "enable_sync": "true",
            "align_depth.enable": "true",
            # realsense2_camera >= 4.5x uses depth_profile / color_profile (not *.profile)
            "depth_module.depth_profile": "640x480x15",
            "rgb_camera.color_profile": "640x480x15",
            "camera_namespace": "camera",
        }.items(),
    )

    # Static TF: base_link -> camera_link (adjust to your physical mount)
    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_static_tf",
        arguments=[
            "0.3", "0.1", "0.65",
            "0", "0.262", "0",
            "base_link",
            "camera_link",
        ],
        output="screen",
    )

    # ----- 4× Nexigo N980P (usb_cam) -----
    # Uses usb_cam instead of v4l2_camera so MJPEG decoding is handled internally,
    # keeping USB 2.0 bandwidth at ~2-5 MB/s per camera instead of ~18 MB/s (YUYV).
    def fid_cam_node(name: str, device: LaunchConfiguration, ns: str, optical_frame: str):
        return Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name=name,
            namespace=ns,
            parameters=[
                {"video_device": device},
                {"image_width": 640},
                {"image_height": 480},
                {"framerate": 30.0},
                {"pixel_format": "mjpeg2rgb"},
                {"camera_frame_id": optical_frame},
                {"camera_name": name},
            ],
            output="screen",
        )

    front_left = fid_cam_node(
        "front_left_camera",
        LaunchConfiguration("fid_front_left_dev"),
        "fid_cams/front_left_camera",
        "front_left_camera_link_optical",
    )
    front_right = fid_cam_node(
        "front_right_camera",
        LaunchConfiguration("fid_front_right_dev"),
        "fid_cams/front_right_camera",
        "front_right_camera_link_optical",
    )
    back_left = fid_cam_node(
        "back_left_camera",
        LaunchConfiguration("fid_back_left_dev"),
        "fid_cams/back_left_camera",
        "back_left_camera_link_optical",
    )
    back_right = fid_cam_node(
        "back_right_camera",
        LaunchConfiguration("fid_back_right_dev"),
        "fid_cams/back_right_camera",
        "back_right_camera_link_optical",
    )

    # Static TF: base_link -> each fiducial camera optical frame (required for multi_camera_marker_localizer).
    # Measure and tune xyz/rpy for your robot; these are placeholders for a typical 4-cam layout.
    def static_tf_fid_cam(child_frame: str, x: str, y: str, z: str, yaw: str = "0"):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"base_to_{child_frame}",
            arguments=[x, y, z, yaw, "0", "0", "base_link", child_frame],
            output="screen",
        )

    static_tf_fid_fl = static_tf_fid_cam("front_left_camera_link_optical", "0.25", "0.20", "0.15", "0")
    static_tf_fid_fr = static_tf_fid_cam("front_right_camera_link_optical", "0.25", "-0.20", "0.15", "0")
    static_tf_fid_bl = static_tf_fid_cam("back_left_camera_link_optical", "-0.25", "0.20", "0.15", "3.14159")
    static_tf_fid_br = static_tf_fid_cam("back_right_camera_link_optical", "-0.25", "-0.20", "0.15", "3.14159")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time"),
        declare_fl_dev,
        declare_fr_dev,
        declare_bl_dev,
        declare_br_dev,
        realsense_launch,
        static_tf_camera,
        front_left,
        front_right,
        back_left,
        back_right,
        static_tf_fid_fl,
        static_tf_fid_fr,
        static_tf_fid_bl,
        static_tf_fid_br,
    ])
