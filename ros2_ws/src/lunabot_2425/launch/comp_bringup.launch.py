import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = "lunabot_2425"
    pkg_share = get_package_share_directory(package_name)

    robot_description = ParameterValue (
        Command([
        'xacro ',
        os.path.join(pkg_share, 'description', 'robot.urdf.xacro'),
        ' use_sim:=false'
        ])
    )

    # robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
    )

    # controller manager
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(pkg_share, 'config', 'robot_controllers.yaml'),
        ],
        output='screen',
    )

    # twist stamper
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        parameters=[{'use_sim_time': False, 'frame_id': 'base_link'}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/cmd_vel_stamped'),
        ],
        output='screen',
    )

    # controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen',
    )

    luna_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'luna_cont',
            '--controller-manager', '/controller_manager'
        ],
        output='screen',
    )

    # spawn joint state broadcaster
    spawn_jsb = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='Spawning joint_state_broadcaster...'),
            joint_state_broadcaster_spawner,
        ]
    )

    # spawn luna_cont last
    spawn_luna_cont_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg='joint_state_broadcaster active — starting luna_cont...'),
                luna_controller_spawner,
            ]
        )
    )

    return LaunchDescription([
        rsp,
        ros2_control_node,
        twist_stamper,
        spawn_jsb,
        spawn_luna_cont_after_jsb,
    ])