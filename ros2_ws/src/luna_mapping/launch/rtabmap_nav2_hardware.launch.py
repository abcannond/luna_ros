#!/usr/bin/env python3
"""
RTAB-Map + Nav2 for hardware (RealSense D455 + optional fiducial cams).

Launch this after hardware_bringup.launch.py. Uses the same pipeline as sim
but with use_sim_time:=false and no Gazebo static TFs (odom must come from the robot).

Usage:
  Terminal 1: ros2 launch lunabot_2425 hardware_bringup.launch.py
  Terminal 2: ros2 launch luna_mapping rtabmap_nav2_hardware.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    luna_mapping_dir = get_package_share_directory("luna_mapping")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(luna_mapping_dir, "launch", "rtabmap_nav2_sim.launch.py")
            ),
            launch_arguments={
                "use_sim_time": "false",
                "sim": "false",
                "launch_rviz": "true",
                "launch_nav2": "true",
            }.items(),
        ),
    ])
