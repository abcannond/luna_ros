from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    autonomous = Node(
        package="luna_nav",
        executable="autonomy_manager",
        name="autonomy_manager",
        output="screen",
    )

    return LaunchDescription([

        autonomous,
    ])