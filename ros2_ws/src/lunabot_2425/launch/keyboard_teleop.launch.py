"""teleop_nav_gate + teleop_twist_keyboard → /teleop_cmd_vel_raw (arm via /teleop_nav_gate/arm)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lunabot_2425")
    _ = pkg_share  # keep for symmetry with other launches

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Kept for compatibility; keyboard/gate use wall time for responsive input in sim.",
    )

    # teleop_nav_gate params (inlined; was config/teleop_nav_gate.yaml)
    teleop_nav_gate_params = {
        "start_button": 7,
        "back_button": 6,
        "teleop_cmd_vel_in": "/teleop_cmd_vel_raw",
        "cmd_vel_out": "/cmd_vel",
        "teleop_active_out": "/teleop_active",
        "nav2_lifecycle_nodes": [
            "controller_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "smoother_server",
            "velocity_smoother",
        ],
        "nav2_manager": "/lifecycle_manager_navigation/manage_nodes",
        "use_sim_time": False,
    }

    teleop_nav_gate = Node(
        package="lunabot_2425",
        executable="teleop_nav_gate",
        name="teleop_nav_gate",
        parameters=[teleop_nav_gate_params],
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
