#!/usr/bin/env python3
"""
Competition-style simulation: Artemis arena + RTAB-Map + Nav2 in one command.

Use this to rehearse the basic field routine (bring-up → map/odom → teleop → Nav2 goals)
without passing world:=... every time.

Equivalent:
  ros2 launch lunabot_2425 gz_bringup.launch.py world:=artemis_arena launch_mapping:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_2425')
    gz_bringup = os.path.join(pkg_share, 'launch', 'gz_bringup.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_bringup),
            launch_arguments={
                'world': 'artemis_arena',
                'launch_mapping': 'true',
            }.items(),
        ),
    ])
