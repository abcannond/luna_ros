"""Launch multi-camera fiducial localizer (4 wheel-pod cameras)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("fiducial_localizer")
    default_params = os.path.join(pkg, "params", "multi_camera_sim.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to YAML parameters",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        Node(
            package="fiducial_localizer",
            executable="multi_camera_marker_localizer",
            name="multi_camera_marker_localizer",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
