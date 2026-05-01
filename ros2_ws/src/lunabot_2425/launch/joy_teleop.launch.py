"""Launch joy_node + teleop_twist_joy + teleop_nav_gate.

teleop_twist_joy publishes to /teleop_cmd_vel_raw; teleop_nav_gate forwards to /cmd_vel when
teleop is armed with Start (see teleop_nav_gate.yaml). Nav2 publishes /cmd_vel when teleop is off.

Optional: linkage_joy:=true starts luna_linkage_joy (CAN linkages via LT/LB on /joy). Requires can0 on the robot.
"""
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
    gate_config_path = os.path.join(pkg_share, "config", "teleop_nav_gate.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Kept for launch compatibility; joy/teleop/gate use wall time so input stays responsive in sim.",
    )

    declare_joy_echo = DeclareLaunchArgument(
        "joy_echo",
        default_value="true",
        description="If true, run joy_echo node to print human-readable input feedback",
    )
    joy_echo_arg = LaunchConfiguration("joy_echo", default="true")

    declare_linkage_joy = DeclareLaunchArgument(
        "linkage_joy",
        default_value="false",
        description="If true, run luna_linkage_joy (CAN linkage teleop; needs real can0).",
    )
    linkage_joy_arg = LaunchConfiguration("linkage_joy", default="false")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"use_sim_time": False}],
        remappings=[("joy", "/joy")],
        output="screen",
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[config_path, {"use_sim_time": False}],
        remappings=[
            ("joy", "/joy"),
            ("cmd_vel", "/teleop_cmd_vel_raw"),
            ("/cmd_vel", "/teleop_cmd_vel_raw"),
        ],
        output="screen",
    )

    teleop_nav_gate = Node(
        package="lunabot_2425",
        executable="teleop_nav_gate",
        name="teleop_nav_gate",
        parameters=[gate_config_path, {"use_sim_time": False}],
        output="screen",
    )

    joy_echo_node = Node(
        package="luna_mapping",
        executable="joy_echo",
        name="joy_echo",
        parameters=[{"use_sim_time": False}],
        output="screen",
        condition=IfCondition(joy_echo_arg),
    )

    linkage_joy_node = Node(
        package="luna_control",
        executable="luna_linkage_joy",
        name="luna_linkage_joy",
        parameters=[{"use_sim_time": False}],
        output="screen",
        condition=IfCondition(linkage_joy_arg),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_joy_echo,
        declare_linkage_joy,
        joy_node,
        teleop_node,
        teleop_nav_gate,
        joy_echo_node,
        linkage_joy_node,
    ])
