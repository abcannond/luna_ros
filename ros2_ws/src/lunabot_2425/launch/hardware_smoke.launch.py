#!/usr/bin/env python3
"""
Hardware smoke test: RealSense D455 + 3x fiducial cameras + fiducial localizer.

Minimum bringup to verify on the Jetson that:
  1. RealSense is publishing color and depth.
  2. Three fiducial cameras are publishing image/camera_info.
  3. The multi-camera fiducial localizer publishes /fiducial_pose when a single
     50 cm DICT_4X4_50 marker (id=50) is in view of any active camera.

Does NOT start RTAB-Map, EKF, Nav2, or RViz. See docs/HARDWARE_SMOKE_TEST.md.

Usage:
  ros2 launch lunabot_2425 hardware_smoke.launch.py \
    fid_front_left_dev:=/dev/video0 \
    fid_front_right_dev:=/dev/video2 \
    fid_back_left_dev:=/dev/video4
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Edit this tuple if you swap which 3 positions are populated. Each entry must
# match a list entry (same index) in multi_camera_hardware_test.yaml.
ACTIVE_POSITIONS = ("front_left", "front_right", "back_left")


def generate_launch_description():
    realsense_share = get_package_share_directory("realsense2_camera")
    fiducial_share = get_package_share_directory("fiducial_localizer")
    fiducial_params = os.path.join(
        fiducial_share, "params", "multi_camera_hardware_test.yaml"
    )

    declare_fl_dev = DeclareLaunchArgument(
        "fid_front_left_dev", default_value="/dev/video0",
        description="V4L2 device for front-left fiducial camera",
    )
    declare_fr_dev = DeclareLaunchArgument(
        "fid_front_right_dev", default_value="/dev/video2",
        description="V4L2 device for front-right fiducial camera",
    )
    declare_bl_dev = DeclareLaunchArgument(
        "fid_back_left_dev", default_value="/dev/video4",
        description="V4L2 device for back-left fiducial camera",
    )

    # ---------- RealSense D455 ----------
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "enable_sync": "true",
            "align_depth.enable": "true",
            "depth_module.depth_profile": "640x480x15",
            "rgb_camera.color_profile": "640x480x15",
            "camera_namespace": "camera/camera",
        }.items(),
    )

    # base_link -> camera_link (RealSense). Adjust xyz/rpy for your mount.
    static_tf_realsense = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_static_tf",
        arguments=["0.3", "0.1", "0.65", "0", "0.262", "0",
                   "base_link", "camera_link"],
        output="screen",
    )

    # ---------- Fiducial cameras (3x usb_cam, MJPEG) ----------
    cam_devices = {
        "front_left": LaunchConfiguration("fid_front_left_dev"),
        "front_right": LaunchConfiguration("fid_front_right_dev"),
        "back_left": LaunchConfiguration("fid_back_left_dev"),
    }
    # base_link -> <position>_camera_link_optical static TFs. Placeholders;
    # measure your mount and update before trusting the published pose.
    cam_tfs = {
        "front_left":  ("0.25",  "0.20", "0.15", "0"),
        "front_right": ("0.25", "-0.20", "0.15", "0"),
        "back_left":   ("-0.25", "0.20", "0.15", "3.14159"),
    }

    fid_nodes = []
    static_tf_nodes = []
    for pos in ACTIVE_POSITIONS:
        optical_frame = f"{pos}_camera_link_optical"
        node_name = f"{pos}_camera"
        fid_nodes.append(Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name=node_name,
            namespace=f"fid_cams/{node_name}",
            parameters=[{
                "video_device": cam_devices[pos],
                "image_width": 640,
                "image_height": 480,
                "framerate": 30.0,
                "pixel_format": "mjpeg2rgb",
                "camera_frame_id": optical_frame,
                "camera_name": node_name,
            }],
            output="screen",
        ))
        x, y, z, yaw = cam_tfs[pos]
        static_tf_nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"base_to_{optical_frame}",
            arguments=[x, y, z, yaw, "0", "0", "base_link", optical_frame],
            output="screen",
        ))

    # ---------- Fiducial localizer (LifecycleNode) + lifecycle manager ----------
    fiducial_localizer = Node(
        package="fiducial_localizer",
        executable="multi_camera_marker_localizer",
        name="multi_camera_marker_localizer",
        output="screen",
        parameters=[fiducial_params, {"use_sim_time": False}],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_fiducial_smoke",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["multi_camera_marker_localizer"],
        }],
    )

    # Delay the localizer + lifecycle manager so cameras have time to advertise
    # camera_info before the localizer starts looking for it.
    delayed_fiducial = TimerAction(
        period=4.0,
        actions=[fiducial_localizer, lifecycle_manager],
    )

    return LaunchDescription([
        declare_fl_dev,
        declare_fr_dev,
        declare_bl_dev,
        realsense_launch,
        static_tf_realsense,
        *fid_nodes,
        *static_tf_nodes,
        delayed_fiducial,
    ])
