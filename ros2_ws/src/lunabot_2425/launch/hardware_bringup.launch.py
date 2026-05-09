#!/usr/bin/env python3
"""
Hardware bringup: Intel RealSense D455 + 4× Nexigo N980P webcams.

Publishes topics matching simulation so the same mapping/nav stack can run.
Optional flags add the fiducial localizer and an RViz instance for hardware
visualization (smoke testing, marker debugging, RTAB-Map verification).

Examples:

  # Bare bringup — cameras + RealSense only.
  ros2 launch lunabot_2425 hardware_bringup.launch.py

  # Smoke test — same bringup plus the fiducial localizer and RViz.
  ros2 launch lunabot_2425 hardware_bringup.launch.py \\
    launch_localizer:=true launch_rviz:=true

  # AprilTag 36h11 preset instead of the default DICT_4X4_50:
  ros2 launch lunabot_2425 hardware_bringup.launch.py \\
    launch_localizer:=true launch_rviz:=true \\
    localizer_params:=$(ros2 pkg prefix fiducial_localizer)/share/fiducial_localizer/params/multi_camera_hardware_apriltag.yaml

  # Override marker dictionary / size on the fly:
  ros2 launch lunabot_2425 hardware_bringup.launch.py launch_localizer:=true \\
    marker_dict:=DICT_APRILTAG_36h11 marker_size_m:=1.0

For the full SLAM + Nav2 stack, run this without flags then:
  ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _localizer_param_overrides(context):
    """Build the per-launch parameter overrides dict, omitting unset keys."""
    overrides = {"use_sim_time": False}
    md = LaunchConfiguration("marker_dict").perform(context)
    mid = LaunchConfiguration("marker_id").perform(context)
    msz = LaunchConfiguration("marker_size_m").perform(context)
    if md:
        overrides["aruco_dict"] = md
    if mid:
        overrides["marker_id"] = int(mid)
    if msz:
        overrides["marker_size_m"] = float(msz)
    return overrides


def _build_localizer_actions(context):
    """Build localizer + lifecycle manager nodes with resolved param overrides."""
    params_file = LaunchConfiguration("localizer_params").perform(context)
    overrides = _localizer_param_overrides(context)

    localizer = Node(
        package="fiducial_localizer",
        executable="multi_camera_marker_localizer",
        name="multi_camera_marker_localizer",
        output="screen",
        parameters=[params_file, overrides],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_fiducial_hw",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["multi_camera_marker_localizer"],
            "bond_timeout": 0.0,
        }],
    )
    # Cameras need ~4 s to advertise camera_info before the localizer activates.
    return [TimerAction(period=4.0, actions=[localizer, lifecycle_manager])]


def generate_launch_description():
    pkg_share = get_package_share_directory("lunabot_2425")
    realsense_share = get_package_share_directory("realsense2_camera")
    fiducial_share = get_package_share_directory("fiducial_localizer")

    default_localizer_params = os.path.join(
        fiducial_share, "params", "multi_camera_hardware.yaml"
    )
    rviz_config = os.path.join(pkg_share, "config", "hardware.rviz")

    # ---------- Launch arguments ----------
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
    declare_br_dev = DeclareLaunchArgument(
        "fid_back_right_dev", default_value="/dev/video6",
        description="V4L2 device for back-right fiducial camera",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use sim time",
    )
    declare_launch_localizer = DeclareLaunchArgument(
        "launch_localizer", default_value="false",
        description="Start multi_camera_marker_localizer + lifecycle manager",
    )
    declare_launch_rviz = DeclareLaunchArgument(
        "launch_rviz", default_value="false",
        description="Open RViz with the hardware visualization config",
    )
    declare_localizer_params = DeclareLaunchArgument(
        "localizer_params", default_value=default_localizer_params,
        description="Path to fiducial localizer YAML preset",
    )
    declare_marker_dict = DeclareLaunchArgument(
        "marker_dict", default_value="",
        description="Override aruco_dict (e.g. DICT_APRILTAG_36h11). Empty = use YAML.",
    )
    declare_marker_id = DeclareLaunchArgument(
        "marker_id", default_value="",
        description="Override marker_id. -1 = accept any. Empty = use YAML.",
    )
    declare_marker_size_m = DeclareLaunchArgument(
        "marker_size_m", default_value="",
        description="Override marker side length in meters. Empty = use YAML.",
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
            "camera_namespace": "camera",
        }.items(),
    )

    # base_link -> camera_link (RealSense). Adjust xyz/rpy for the real mount.
    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_static_tf",
        arguments=["0.3", "0.1", "0.65", "0", "0.262", "0",
                   "base_link", "camera_link"],
        output="screen",
    )

    # ---------- 4× Nexigo N980P (usb_cam) ----------
    cam_info_dir = os.path.join(pkg_share, "config", "camera_info")

    def fid_cam_node(name, device, ns, optical_frame):
        calib_url = f"file://{cam_info_dir}/{name}.yaml"
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
                {"camera_info_url": calib_url},
            ],
            output="screen",
        )

    cams = [
        fid_cam_node("front_left_camera",
                     LaunchConfiguration("fid_front_left_dev"),
                     "fid_cams/front_left_camera",
                     "front_left_camera_link_optical"),
        fid_cam_node("front_right_camera",
                     LaunchConfiguration("fid_front_right_dev"),
                     "fid_cams/front_right_camera",
                     "front_right_camera_link_optical"),
        fid_cam_node("back_left_camera",
                     LaunchConfiguration("fid_back_left_dev"),
                     "fid_cams/back_left_camera",
                     "back_left_camera_link_optical"),
        fid_cam_node("back_right_camera",
                     LaunchConfiguration("fid_back_right_dev"),
                     "fid_cams/back_right_camera",
                     "back_right_camera_link_optical"),
    ]

    # base_link -> each fiducial camera optical frame. Placeholder mounts —
    # measure your real offsets and update before trusting /fiducial_pose.
    def static_tf_fid_cam(child, x, y, z, yaw="0"):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"base_to_{child}",
            arguments=[x, y, z, yaw, "0", "0", "base_link", child],
            output="screen",
        )

    static_tfs = [
        static_tf_fid_cam("front_left_camera_link_optical",  "0.25",  "0.20", "0.15", "0"),
        static_tf_fid_cam("front_right_camera_link_optical", "0.25", "-0.20", "0.15", "0"),
        static_tf_fid_cam("back_left_camera_link_optical",  "-0.25",  "0.20", "0.15", "3.14159"),
        static_tf_fid_cam("back_right_camera_link_optical", "-0.25", "-0.20", "0.15", "3.14159"),
    ]

    # ---------- Optional: fiducial localizer (gated on launch_localizer) ----------
    localizer_actions = OpaqueFunction(
        function=lambda ctx: _build_localizer_actions(ctx),
        condition=IfCondition(LaunchConfiguration("launch_localizer")),
    )

    # ---------- Optional: RViz (gated on launch_rviz) ----------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_hardware",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_fl_dev,
        declare_fr_dev,
        declare_bl_dev,
        declare_br_dev,
        declare_launch_localizer,
        declare_launch_rviz,
        declare_localizer_params,
        declare_marker_dict,
        declare_marker_id,
        declare_marker_size_m,
        realsense_launch,
        static_tf_camera,
        *cams,
        *static_tfs,
        localizer_actions,
        rviz_node,
    ])
