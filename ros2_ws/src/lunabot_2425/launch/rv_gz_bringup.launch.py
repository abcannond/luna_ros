import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    package_name = "lunabot_2425"
    pkg_share = get_package_share_directory(package_name)

    xacro_file = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    robot_desc = xacro.process_file(xacro_file).toxml()

    world_file = os.path.join(pkg_share, "worlds", "arena_b.world")

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"),
            "launch", "gz_sim.launch.py"
        )),
        launch_arguments={
            'gz_args': f"-r {world_file}",
            'on_exit_shutdown': 'True'
        }.items(),
    )

    gz_create_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "mooncake"],
        output="screen",
    )

    gz_param_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            "config_file": os.path.join(pkg_share, "config", "gz_bridge.config.yaml")
        }],
        output='screen'
    )

    gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["luna_cont", "joint_broad"],
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/cmd_vel_stamped'),
        ],
    )

    return LaunchDescription([
        rsp_node,
        twist_stamper,
        gz_sim,
        gz_create_robot,
        gz_param_bridge,
        gz_image_bridge,
        controller_spawner,
    ])
