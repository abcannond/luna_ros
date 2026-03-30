#!/usr/bin/env python3
"""
Integrated launch file for RTAB-Map + Nav2 with simulated RealSense D455.

This launch file integrates:
- RTAB-Map for SLAM and visual odometry
- Occupancy grid generation from depth camera
- Nav2 for navigation and path planning
- Depth-to-LaserScan for obstacle detection
- Depth-to-PointCloud for 3D obstacle avoidance

Usage:
  ros2 launch luna_mapping rtabmap_nav2_sim.launch.py

This should be launched AFTER gz_bringup.launch.py is running.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    luna_mapping_dir = get_package_share_directory('luna_mapping')
    luna_nav_dir = get_package_share_directory('luna_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rtabmap_odom = LaunchConfiguration('use_rtabmap_odom')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    launch_rviz = LaunchConfiguration('launch_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')
    declare_use_rtabmap_odom = DeclareLaunchArgument(
        'use_rtabmap_odom', default_value='false',
        description='Use RTAB-Map visual odometry instead of wheel/Gazebo odom')
    declare_launch_nav2 = DeclareLaunchArgument(
        'launch_nav2', default_value='true',
        description='Launch Nav2 stack')
    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz', default_value='false',
        description='Launch RViz')
    declare_sim = DeclareLaunchArgument(
        'sim', default_value='true',
        description='If true, publish sim-only Gazebo<->ROS frame aliases. Set false for hardware.')
    sim = LaunchConfiguration('sim', default='true')
    declare_clear_costmap_on_start = DeclareLaunchArgument(
        'clear_costmap_on_start', default_value='true',
        description='Clear local and global costmaps once after Nav2 is up.')
    clear_costmap_on_start = LaunchConfiguration('clear_costmap_on_start', default='true')

    # ============================================================
    # RTAB-Map SLAM
    # ============================================================
    rtabmap_params_file = os.path.join(luna_mapping_dir, 'config', 'rtabmap_sim.yaml')

    # RTAB-Map subscribes to frame-fixed topics so frame_ids match the URDF TF tree
    rtabmap_remappings = [
        ('rgb/image', '/camera/camera/color/image_raw_fixed'),
        ('rgb/camera_info', '/camera/camera/color/camera_info_fixed'),
        ('depth/image', '/camera/camera/depth/image_rect_raw_fixed'),
        ('depth/camera_info', '/camera/camera/depth/camera_info_fixed'),
        ('odom', '/odom'),
        ('map', '/map'),
        ('grid_map', '/map'),
    ]

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {'use_sim_time': use_sim_time},
            {'subscribe_depth': True},
            {'subscribe_rgb': True},
            {'subscribe_odom_info': False},
            {'frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'map_frame_id': 'map'},
            {'approx_sync': True},
            {'approx_sync_max_interval': 0.02},
            {'qos_image': 2},
            {'qos_camera_info': 2},
            # Occupancy grid from depth
            {'Grid/FromDepth': 'true'},
            {'Grid/RayTracing': 'true'},
            {'Grid/CellSize': '0.05'},
            {'Grid/RangeMax': '5.0'},
            {'Grid/RangeMin': '0.4'},
            {'Grid/MaxGroundHeight': '0.15'},
            {'Grid/MaxObstacleHeight': '2.0'},
            {'Grid/NormalsSegmentation': 'true'},
            {'Grid/MaxGroundAngle': '30'},
            {'Grid/FlatObstacleDetected': 'false'},
            {'Grid/FootprintLength': '0.65'},
            {'Grid/FootprintWidth': '0.53'},
            {'Grid/FootprintHeight': '0.40'},
            {'Rtabmap/DetectionRate': '2.0'},
        ],
        remappings=rtabmap_remappings,
        arguments=['--delete_db_on_start']
    )

    # RTAB-Map visual odometry (optional - for when wheel odom is unreliable)
    rtabmap_odom = Node(
        condition=IfCondition(use_rtabmap_odom),
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {'use_sim_time': use_sim_time},
            {'frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'publish_tf': True},
            {'wait_for_transform': 0.2},
            {'approx_sync': True},
            {'approx_sync_max_interval': 0.02},
            {'qos': 2},
            {'Odom/Strategy': '0'},
            {'Odom/GuessMotion': 'true'},
            {'Odom/ResetCountdown': '0'},
            {'Vis/FeatureType': '8'},
            {'Vis/MaxFeatures': '1500'},
            {'Vis/MinInliers': '15'},
            {'Vis/CorType': '0'},
        ],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw_fixed'),
            ('rgb/camera_info', '/camera/camera/color/camera_info_fixed'),
            ('depth/image', '/camera/camera/depth/image_rect_raw_fixed'),
            ('depth/camera_info', '/camera/camera/depth/camera_info_fixed'),
        ]
    )

    # ============================================================
    # Depth visualization (colorized for RViz)
    # ============================================================
    depth_to_rgb = Node(
        package='luna_mapping',
        executable='depth_to_rgb',
        name='depth_to_rgb',
        output='log',
        parameters=[{
            'input_topic': '/camera/camera/depth/image_rect_raw',
            'output_topic': '/camera/camera/depth/image_colorized',
            'max_range': 5.0,
        }]
    )

    # ============================================================
    # Depth FOV filter (ground crop + mask regions for costmap)
    # ============================================================
    depth_fov_filter = Node(
        package='luna_mapping',
        executable='depth_fov_filter',
        name='depth_fov_filter',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/depth/image_rect_raw',
            'output_topic': '/camera/camera/depth/image_rect_raw_filtered',
            'mask_regions': '',
            'ground_crop_bottom': 0.05,
        }]
    )

    # ============================================================
    # Frame ID fixers
    # Gazebo bridge uses internal frame names (e.g. mooncake/base_link/d455_rgb_camera).
    # RTAB-Map and Nav2 need standard optical frame names that exist in the URDF TF tree.
    # These nodes republish with corrected frame_id on *_fixed topics.
    # ============================================================
    camera_info_fixer_color = Node(
        package='luna_mapping',
        executable='camera_info_fixer',
        name='camera_info_fixer_color',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/color/camera_info',
            'output_topic': '/camera/camera/color/camera_info_fixed',
            'output_frame_id': 'camera_color_optical_frame',
        }]
    )

    camera_info_fixer_depth = Node(
        package='luna_mapping',
        executable='camera_info_fixer',
        name='camera_info_fixer_depth',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/depth/camera_info',
            'output_topic': '/camera/camera/depth/camera_info_fixed',
            'output_frame_id': 'camera_depth_optical_frame',
        }]
    )

    image_frame_fixer_rgb = Node(
        package='luna_mapping',
        executable='image_frame_fixer',
        name='image_frame_fixer_rgb',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/color/image_raw',
            'output_topic': '/camera/camera/color/image_raw_fixed',
            'output_frame_id': 'camera_color_optical_frame',
        }]
    )

    # Raw depth -> fixed frame for RTAB-Map (unfiltered for best map quality)
    image_frame_fixer_depth_rtabmap = Node(
        package='luna_mapping',
        executable='image_frame_fixer',
        name='image_frame_fixer_depth_rtabmap',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/depth/image_rect_raw',
            'output_topic': '/camera/camera/depth/image_rect_raw_fixed',
            'output_frame_id': 'camera_depth_optical_frame',
        }]
    )

    # Filtered depth -> fixed frame for laserscan (ground-cropped)
    image_frame_fixer_depth_laserscan = Node(
        package='luna_mapping',
        executable='image_frame_fixer',
        name='image_frame_fixer_depth_laserscan',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/depth/image_rect_raw_filtered',
            'output_topic': '/camera/camera/depth/image_rect_raw_fixed_for_scan',
            'output_frame_id': 'camera_depth_optical_frame',
        }]
    )

    # Filtered depth -> color optical frame for point cloud alignment
    image_frame_fixer_depth_pointcloud = Node(
        package='luna_mapping',
        executable='image_frame_fixer',
        name='image_frame_fixer_depth_pointcloud',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/depth/image_rect_raw_filtered',
            'output_topic': '/camera/camera/depth/image_rect_raw_aligned',
            'output_frame_id': 'camera_color_optical_frame',
        }]
    )

    # ============================================================
    # Depth to LaserScan (2D obstacle detection)
    # Uses FILTERED depth so ground returns don't appear as obstacles.
    # ============================================================
    depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_height': 40,
            'scan_time': 0.033,
            'range_min': 0.4,
            'range_max': 4.0,
            'output_frame_id': 'camera_depth_optical_frame',
        }],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw_fixed_for_scan'),
            ('depth_camera_info', '/camera/camera/depth/camera_info_fixed'),
            ('scan', '/scan'),
        ]
    )

    # Depth to PointCloud2 (3D voxel layer input)
    # Uses filtered depth aligned to color frame for proper XYZRGB point cloud
    depth_to_pointcloud = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='point_cloud_xyzrgb',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image_rect_color', '/camera/camera/color/image_raw_fixed'),
            ('rgb/camera_info', '/camera/camera/color/camera_info_fixed'),
            ('depth_registered/image_rect', '/camera/camera/depth/image_rect_raw_aligned'),
            ('points', '/camera/camera/depth/color/points'),
        ]
    )

    # Height-based ground filter: remove points below min_height in base_link.
    # Keeps rocks and crates (above ground), avoids blind band from image crop.
    height_filter_pointcloud = Node(
        package='luna_mapping',
        executable='height_filter_pointcloud',
        name='height_filter_pointcloud',
        output='log',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/camera/camera/depth/color/points',
            'output_topic': '/camera/camera/depth/color/points_ground_filtered',
            'target_frame': 'base_link',
            'min_height': 0.02,
            'max_height': 2.0,
        }],
    )

    # ============================================================
    # Nav2 stack
    # ============================================================
    nav2_params_file = os.path.join(luna_nav_dir, 'config', 'nav2_rtabmap_params.yaml')

    nav2_nodes = GroupAction(
        condition=IfCondition(launch_nav2),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[('cmd_vel', 'cmd_vel_nav')]
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[('cmd_vel', 'cmd_vel_nav')]
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel'),
                ]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'smoother_server',
                        'velocity_smoother',
                    ]
                }]
            ),
        ]
    )

    # ============================================================
    # RViz
    # ============================================================
    rviz_config = os.path.join(luna_mapping_dir, 'config', 'rtabmap_nav2_with_fid_cams.rviz')

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ============================================================
    # Static transforms (sim-only: Gazebo frame aliases)
    #
    # NOTE: odom->base_link is NOT published here.
    # In sim, LunaController publishes dynamic odom->base_link from wheel odom.
    # On hardware, the robot driver provides it.
    #
    # RSP (robot_state_publisher) already publishes the full URDF chain:
    #   base_link -> depth_camera_link -> camera_color_optical_frame
    #   base_link -> depth_camera_link -> camera_depth_optical_frame
    #   base_link -> depth_camera_link -> camera_link
    # These sim-only TFs are ONLY for Gazebo-internal frame name aliases
    # that appear in un-fixed topics (not used by the pipeline, but avoids
    # "frame not found" warnings in rviz/diagnostics).
    # ============================================================
    static_tf_camera_depth_frame = Node(
        condition=IfCondition(sim),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_depth_frame',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'camera_depth_optical_frame',
            'camera_depth_frame'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rtabmap_odom,
        declare_launch_nav2,
        declare_launch_rviz,
        declare_sim,
        declare_clear_costmap_on_start,

        # Sim-only TF alias (RSP provides the actual camera chain from URDF)
        static_tf_camera_depth_frame,

        # Depth processing
        depth_to_rgb,
        depth_fov_filter,

        # Frame fixers
        camera_info_fixer_color,
        camera_info_fixer_depth,
        image_frame_fixer_rgb,
        image_frame_fixer_depth_rtabmap,
        image_frame_fixer_depth_laserscan,
        image_frame_fixer_depth_pointcloud,

        # RTAB-Map
        rtabmap_slam,
        rtabmap_odom,

        # Depth to scan + point cloud + height filter
        depth_to_laserscan,
        depth_to_pointcloud,
        height_filter_pointcloud,

        # Nav2
        nav2_nodes,

        # One-time costmap clear after Nav2 initializes
        TimerAction(
            period=18.0,
            actions=[
                ExecuteProcess(
                    name='clear_costmap_once',
                    cmd=[
                        'bash', '-c',
                        'sleep 2; '
                        'for i in $(seq 1 15); do '
                        'ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" 2>/dev/null && '
                        'ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" 2>/dev/null && break; '
                        'sleep 2; done'
                    ],
                    output='log',
                )
            ],
            condition=IfCondition(
                PythonExpression(["'", launch_nav2, "' == 'true' and '", clear_costmap_on_start, "' == 'true'"])
            ),
        ),

        rviz_node,
    ])
