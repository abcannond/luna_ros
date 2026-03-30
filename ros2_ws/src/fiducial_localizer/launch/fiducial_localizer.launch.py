from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description='Path to a YAML with node parameters'),

        Node(
            package='fiducial_localizer',
            executable='marker_localizer',
            name='marker_localizer',
            output='screen',
            parameters=[LaunchConfiguration('params')] if LaunchConfiguration('params') else []
        )
    ])
