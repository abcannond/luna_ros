import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = "lunabot_2425"

    # Paths
    description_path = os.path.join(
        get_package_share_directory(package_name),
        "description",
        "robot.urdf"  # use the generated URDF
    )

    world_file = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "arena_b.world"
    )

    # Load URDF XML
    with open(description_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # Gazebo
    gz_sim_source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py"
    ))

    gz_sim = IncludeLaunchDescription(
        gz_sim_source,
        launch_arguments={
            'gz_args': ["-r", world_file],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # Spawn robot in Gazebo from URDF
    gz_create_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-file", description_path, "-name", "mooncake"],
        output="screen",
    )

    # Parameter bridge
    gz_param_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {"config_file": os.path.join(
                get_package_share_directory(package_name),
                "config",
                "gz_bridge.config.yaml",
            )}
        ],
        output='screen'
    )

    # Image bridge
    gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    # Controller spawner
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["luna_cont", "joint_broad"],
    )

    # Twist stamper
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in','/luna_cont/cmd_vel_unstamped'),
                    ('/cmd_vel_out','/luna_cont/cmd_vel')],
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
