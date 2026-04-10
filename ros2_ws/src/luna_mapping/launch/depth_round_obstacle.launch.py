#!/usr/bin/env python3
"""Optional 23–24-style depth blob (crater-like) detector; default off."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='16-bit depth image (mm), aligned to color if available',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/aligned_depth_to_color/camera_info',
            description='CameraInfo matching the depth image',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='false',
            description='Publish /luna/depth_round_obstacle/debug/image',
        ),
        Node(
            package='luna_mapping',
            executable='depth_round_obstacle_detector',
            name='depth_round_obstacle_detector',
            output='screen',
            parameters=[{
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            }],
        ),
    ])
