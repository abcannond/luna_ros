#!/usr/bin/env python3
"""
Hardware bringup: Intel RealSense D455 + 4× Nexigo N980P webcams (fiducial).

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
        default_value="/dev/video2",
        description="V4L2 device for front-left fiducial camera",
    )
    declare_fr_dev = DeclareLaunchArgument(
        "fid_front_right_dev",
        default_value="/dev/video4",
        description="V4L2 device for front-right fiducial camera",
    )
    declare_bl_dev = DeclareLaunchArgument(
        "fid_back_left_dev",
        default_value="/dev/video6",
        description="V4L2 device for back-left fiducial camera",
    )
    declare_br_dev = DeclareLaunchArgument(
        "fid_back_right_dev",
        default_value="/dev/video8",
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
            "depth_module.profile": "640x480x15",
            "rgb_camera.profile": "640x480x15",
            "camera_namespace": "camera/camera",
        }.items(),
    )

    # Static TF: base_link -> camera_link (adjust to your physical mount)
    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_static_tf",
        arguments=[
            "0.3", "0.1", "0.65",
            "0", "0.49", "0",
            "base_link",
            "camera_link",
        ],
        output="screen",
    )

    # ----- 4× Nexigo N980P (v4l2_camera) -----
    def fid_cam_node(name: str, device: LaunchConfiguration, ns: str):
        return Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name=name,
            namespace=ns,
            parameters=[
                {"video_device": device},
                {"image_size": [640, 480]},
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
        )

    front_left = fid_cam_node(
        "front_left_camera",
        LaunchConfiguration("fid_front_left_dev"),
        "fid_cams/front_left_camera",
    )
    front_right = fid_cam_node(
        "front_right_camera",
        LaunchConfiguration("fid_front_right_dev"),
        "fid_cams/front_right_camera",
    )
    back_left = fid_cam_node(
        "back_left_camera",
        LaunchConfiguration("fid_back_left_dev"),
        "fid_cams/back_left_camera",
    )
    back_right = fid_cam_node(
        "back_right_camera",
        LaunchConfiguration("fid_back_right_dev"),
        "fid_cams/back_right_camera",
    )

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
    ])
