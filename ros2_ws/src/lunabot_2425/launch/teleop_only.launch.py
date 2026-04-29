"""Joy teleop + teleop_nav_gate only (Nav2 optional — lifecycle calls no-op if nodes absent).

Typical hardware:
  ros2 launch lunabot_2425 teleop_only.launch.py use_sim_time:=false"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("lunabot_2425")
    joy_launch = os.path.join(pkg, "launch", "joy_teleop.launch.py")

    declare_use_sim = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="false on real robot; true in simulation",
    )
    declare_joy_echo = DeclareLaunchArgument(
        "joy_echo",
        default_value="true",
        description="Print controller events via joy_echo",
    )

    return LaunchDescription([
        declare_use_sim,
        declare_joy_echo,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "joy_echo": LaunchConfiguration("joy_echo"),
            }.items(),
        ),
    ])
