"""teleop_nav_gate + teleop_twist_keyboard → /teleop_cmd_vel_raw (arm via /teleop_nav_gate/arm)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lunabot_2425")
    gate_config_path = os.path.join(pkg_share, "config", "teleop_nav_gate.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Kept for compatibility; keyboard/gate use wall time for responsive input in sim.",
    )

    teleop_nav_gate = Node(
        package="lunabot_2425",
        executable="teleop_nav_gate",
        name="teleop_nav_gate",
        parameters=[gate_config_path, {"use_sim_time": False}],
        output="screen",
    )

    teleop_keyboard = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        parameters=[{"use_sim_time": False}],
        remappings=[
            ("cmd_vel", "/teleop_cmd_vel_raw"),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        teleop_nav_gate,
        teleop_keyboard,
    ])
