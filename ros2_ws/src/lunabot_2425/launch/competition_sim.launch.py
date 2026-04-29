#!/usr/bin/env python3
"""
Competition simulation launch file -- single entry point.

Brings up the full stack for a Lunabotics competition-style run in Gazebo:
  1. gz_bringup (Gazebo + robot + controllers + RTAB-Map + Nav2 + RViz)
  2. zone_publisher (arena zone classification + RViz markers)
  3. frontier_explorer (WFD-based autonomous exploration, initially disabled)
  4. mission_supervisor (FSM for competition phases)

Usage:
  ros2 launch lunabot_2425 competition_sim.launch.py world:=ucf_arena
  ros2 launch lunabot_2425 competition_sim.launch.py world:=artemis_arena

To start a mission after the stack is up:
  ros2 service call /mission_supervisor/start_mission std_srvs/srv/Trigger {}

See docs/COMPETITION_SIM.md for the full routine. Startup uses a short stationary warmup only (no in-place spin).
"""

import math
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SPAWN_POSES = {
    'artemis_arena': {'x': -2.750, 'y': -1.750, 'z': 0.0, 'yaw': math.pi / 2},
    # Must match gz_bringup UCF ros_gz_sim create -x/-y/-Y (map = world at this pose).
    'ucf_arena':     {'x': -3.800, 'y':  3.100, 'z': 0.0, 'yaw': -math.pi / 2},
}


def _resolve_zones_file(context, *args, **kwargs):
    """Select the zones YAML matching the world argument."""
    world = context.launch_configurations.get('world', 'ucf_arena')
    lunabot_share = get_package_share_directory('lunabot_2425')
    zones_map = {
        'ucf_arena': os.path.join(lunabot_share, 'config', 'zones', 'ucf_zones.yaml'),
        'artemis_arena': os.path.join(lunabot_share, 'config', 'zones', 'artemis_zones.yaml'),
    }
    zones_file = zones_map.get(world, zones_map['ucf_arena'])
    luna_nav_share = get_package_share_directory('luna_nav')
    mission_config = os.path.join(luna_nav_share, 'config', 'mission_phases.yaml')
    launch_autonomy = LaunchConfiguration('launch_autonomy')

    spawn = SPAWN_POSES.get(world, SPAWN_POSES['ucf_arena'])
    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments=[
            '--x', str(spawn['x']),
            '--y', str(spawn['y']),
            '--z', str(spawn['z']),
            '--yaw', str(spawn['yaw']),
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'map',
        ],
        parameters=[{'use_sim_time': True}],
    )

    zone_publisher = Node(
        package='luna_nav',
        executable='zone_publisher',
        name='zone_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'zones_file': zones_file,
            'rate_hz': 5.0,
            'hysteresis_n': 3,
        }],
    )

    frontier_explorer = Node(
        package='luna_nav',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'enabled': False,
            'min_frontier_size': 10,
            'rate_hz': 0.5,
            'goal_tolerance': 0.5,
            'blacklist_radius': 1.2,
            'min_goal_distance': 0.8,
            'settle_time_s': 4.0,
        }],
    )

    mission_supervisor = Node(
        package='luna_nav',
        executable='mission_supervisor',
        name='mission_supervisor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': mission_config,
            'zones_file': zones_file,
            'goal_frame_id': 'world',
            'rate_hz': 2.0,
            'use_frontier_exploration': True,
            # Stationary settle only (seconds) before first goal — no rotation (see mission_supervisor).
            'map_warmup_s': 3.0,
        }],
    )

    return [
        LogInfo(msg=f'Competition sim: world={world}, zones={zones_file}'),
        world_to_map_tf,
        TimerAction(
            period=30.0,
            condition=IfCondition(launch_autonomy),
            actions=[
                LogInfo(msg='Starting autonomy nodes (zone_publisher, frontier_explorer, mission_supervisor)...'),
                zone_publisher,
                frontier_explorer,
                mission_supervisor,
            ],
        ),
    ]


def generate_launch_description():
    lunabot_share = get_package_share_directory('lunabot_2425')
    luna_mapping_share = get_package_share_directory('luna_mapping')
    competition_rviz = os.path.join(
        luna_mapping_share, 'config', 'competition_sim.rviz'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='ucf_arena',
        description='Arena: ucf_arena or artemis_arena',
    )
    declare_launch_autonomy = DeclareLaunchArgument(
        'launch_autonomy',
        default_value='false',
        description='Launch mission_supervisor/frontier_explorer stack (disabled by default).',
    )

    gz_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lunabot_share, 'launch', 'gz_bringup.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'launch_mapping': 'true',
            'rviz_config': competition_rviz,
        }.items(),
    )

    return LaunchDescription([
        declare_world,
        declare_launch_autonomy,
        gz_bringup,
        OpaqueFunction(function=_resolve_zones_file),
    ])
