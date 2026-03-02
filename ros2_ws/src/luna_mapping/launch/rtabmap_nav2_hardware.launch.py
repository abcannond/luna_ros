#!/usr/bin/env python3
"""
RTAB-Map + Nav2 for hardware (RealSense D455 + 4× fiducial cams).

Launch this after hardware_bringup.launch.py. Uses the same pipeline as sim
but with use_sim_time:=false and no Gazebo static TFs (odom must come from the robot).
Optionally starts the multi-camera fiducial localizer so all 4 cams are used for pose.

Usage:
  Terminal 1: ros2 launch lunabot_2425 hardware_bringup.launch.py
  Terminal 2: ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py
  Optional:    ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py launch_fiducial:=false  # skip fiducial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    luna_mapping_dir = get_package_share_directory("luna_mapping")
    fiducial_dir = get_package_share_directory("fiducial_localizer")
    fiducial_hardware_params = os.path.join(fiducial_dir, "params", "multi_camera_hardware.yaml")

    launch_fiducial = LaunchConfiguration("launch_fiducial", default="true")

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_fiducial",
            default_value="true",
            description="If true, run multi-camera fiducial localizer (all 4 cams) for pose.",
        ),
        DeclareLaunchArgument(
            "use_rtabmap_odom",
            default_value="true",
            description="Use RTAB-Map visual odometry (set false if robot driver publishes /odom).",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(luna_mapping_dir, "launch", "rtabmap_nav2_sim.launch.py")
            ),
            launch_arguments={
                "use_sim_time": "false",
                "sim": "false",
                "use_rtabmap_odom": LaunchConfiguration("use_rtabmap_odom", default="true"),
                "launch_rviz": "true",
                "launch_nav2": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fiducial_dir, "launch", "multi_camera_fiducial.launch.py")
            ),
            condition=IfCondition(launch_fiducial),
            launch_arguments={
                "params_file": fiducial_hardware_params,
                "use_sim_time": "false",
            }.items(),
        ),
    ])
