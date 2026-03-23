"""Launch joy_node + teleop_twist_joy for controller teleop. Holonomic + deadman."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lunabot_2425")
    config_path = os.path.join(pkg_share, "config", "joy_teleop.yaml")

    declare_joy_echo = DeclareLaunchArgument(
        "joy_echo",
        default_value="true",
        description="If true, run joy_echo node to print human-readable input feedback",
    )
    joy_echo_arg = LaunchConfiguration("joy_echo", default="true")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[config_path, {"use_sim_time": True}],
        remappings=[("cmd_vel", "/cmd_vel")],
        output="screen",
    )

    joy_echo_node = Node(
        package="luna_mapping",
        executable="joy_echo",
        name="joy_echo",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(joy_echo_arg),
    )

    return LaunchDescription([
        declare_joy_echo,
        joy_node,
        teleop_node,
        joy_echo_node,
    ])
