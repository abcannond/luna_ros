from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    autonomous = Node(
        package="luna_nav",
        executable="autonomy_manager",
        name="autonomy_manager",
        output="screen",
        arguments=[LaunchConfiguration('auto_mode')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_mode',
            default_value='1',
            description='Auto mode selector (1=full auto, 2=drive only, etc.)'
        ),
        autonomous,
    ])