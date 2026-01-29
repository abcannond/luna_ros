#!/usr/bin/env python3
"""
Integrated launch file for RTAB-Map + Nav2 with simulated RealSense D455.

This launch file integrates:
- RTAB-Map for SLAM and visual odometry
- Occupancy grid generation from depth camera
- Nav2 for navigation and path planning
- Depth-to-LaserScan for obstacle detection
- Depth-to-PointCloud for 3D obstacle avoidance
- Optional fiducial marker localization

Usage:
  ros2 launch luna_mapping rtabmap_nav2_sim.launch.py

This should be launched AFTER gz_bringup.launch.py is running.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    # Package directories
    luna_mapping_dir = get_package_share_directory('luna_mapping')
    luna_nav_dir = get_package_share_directory('luna_nav')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rtabmap_odom = LaunchConfiguration('use_rtabmap_odom')
    use_fiducial_odom = LaunchConfiguration('use_fiducial_odom')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_use_rtabmap_odom = DeclareLaunchArgument(
        'use_rtabmap_odom',
        default_value='false',
        description='Use RTAB-Map visual odometry (set false if using Gazebo odom)'
    )
    
    declare_use_fiducial_odom = DeclareLaunchArgument(
        'use_fiducial_odom',
        default_value='false',
        description='Use fiducial marker localization'
    )
    
    declare_launch_nav2 = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='Launch Nav2 stack'
    )
    
    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz (set false if already running from gz_bringup)'
    )
    
    # ============================================================
    # RTAB-Map configuration
    # ============================================================
    rtabmap_params_file = os.path.join(luna_mapping_dir, 'config', 'rtabmap_sim.yaml')
    
    # Topic remappings for RTAB-Map
    # Gazebo bridges to /camera/camera/... (RealSense format)
    # Use fixed camera_info topics with correct frame_id
    rtabmap_remappings = [
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info_fixed'),
        ('depth/image', '/camera/camera/depth/image_rect_raw'),
        ('depth/camera_info', '/camera/camera/depth/camera_info_fixed'),
        ('odom', '/odom'),
        ('map', '/map'),
        ('grid_map', '/map'),
    ]
    
    # RTAB-Map SLAM node
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
            {'qos_image': 2},
            {'qos_camera_info': 2},
            # Grid map (occupancy grid) parameters
            {'Grid/FromDepth': 'true'},
            {'Grid/RayTracing': 'true'},
            {'Grid/CellSize': '0.05'},
            {'Grid/RangeMax': '5.0'},
            {'Grid/RangeMin': '0.3'},
            {'Grid/MaxGroundHeight': '0.05'},
            {'Grid/MaxObstacleHeight': '2.0'},
            # Publish occupancy grid
            {'Rtabmap/DetectionRate': '1.0'},
        ],
        remappings=rtabmap_remappings,
        arguments=['--delete_db_on_start']
    )
    
    # RTAB-Map visual odometry (optional, disabled by default since Gazebo provides odom)
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
            {'approx_sync': True},
            {'qos': 2},
        ],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info_fixed'),
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
            ('depth/camera_info', '/camera/camera/depth/camera_info_fixed'),
        ]
    )
    
    # ============================================================
    # Camera info and image frame_id fixers
    # ============================================================
    # Gazebo's ros_gz_bridge sets camera_info and image frame_id to Gazebo sensor names
    # (e.g., "mooncake/base_link/d455_rgb_camera"), but RTAB-Map expects
    # standard ROS frame names (e.g., "camera_color_optical_frame").
    # These nodes republish messages with corrected frame_id.
    
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
    
    # Fix RGB image frame_id (for point cloud alignment)
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
    
    # Fix depth image frame_id for laserscan (uses depth optical frame)
    image_frame_fixer_depth_laserscan = Node(
        package='luna_mapping',
        executable='image_frame_fixer',
        name='image_frame_fixer_depth_laserscan',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/depth/image_rect_raw',
            'output_topic': '/camera/camera/depth/image_rect_raw_fixed',
            'output_frame_id': 'camera_depth_optical_frame',
        }]
    )
    
    # Fix depth image frame_id to match RGB (for point cloud alignment)
    # depth_image_proc expects both images to have the same frame_id when aligning
    image_frame_fixer_depth_pointcloud = Node(
        package='luna_mapping',
        executable='image_frame_fixer',
        name='image_frame_fixer_depth_pointcloud',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/depth/image_rect_raw',
            'output_topic': '/camera/camera/depth/image_rect_raw_aligned',
            'output_frame_id': 'camera_color_optical_frame',  # Match RGB for alignment
        }]
    )
    
    # ============================================================
    # Depth image processing
    # ============================================================
    
    # Depth to LaserScan for Nav2 obstacle layer
    # Output: /scan (2D "fake lidar" scan derived from depth)
    depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_height': 10,  # Number of rows to use from depth image
            'scan_time': 0.033,
            'range_min': 0.3,
            'range_max': 5.0,
            'output_frame_id': 'camera_depth_optical_frame',
        }],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw_fixed'),  # Use fixed frame_id
            ('depth_camera_info', '/camera/camera/depth/camera_info_fixed'),
            ('scan', '/scan'),  # Standard Nav2 input topic
        ]
    )
    
    # Depth to PointCloud2 for voxel layer (if not using Gazebo's point cloud)
    # Output: /camera/camera/depth/color/points (aligned to color)
    # Uses fixed image topics with matching frame_ids
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
    
    # ============================================================
    # Fiducial localization (optional)
    # ============================================================
    fiducial_localizer = Node(
        condition=IfCondition(use_fiducial_odom),
        package='fiducial_localizer',
        executable='marker_localizer',
        name='marker_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/fid_cams/front_left_camera/image_raw',
            'camera_info_topic': '/fid_cams/front_left_camera/camera_info',
            'aruco_dict': 'DICT_4X4_50',
            'marker_size_m': 0.16,
            'world_frame': 'map',
            'robot_frame': 'base_link',
        }],
    )
    
    # ============================================================
    # Nav2 stack
    # ============================================================
    nav2_params_file = os.path.join(luna_nav_dir, 'config', 'nav2_rtabmap_params.yaml')
    
    # Nav2 lifecycle nodes
    nav2_nodes = GroupAction(
        condition=IfCondition(launch_nav2),
        actions=[
            # Controller server (outputs to cmd_vel_nav for velocity smoother)
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[('cmd_vel', 'cmd_vel_nav')]
            ),
            
            # Planner server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),
            
            # Behavior server (outputs to cmd_vel_nav for velocity smoother)
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[('cmd_vel', 'cmd_vel_nav')]
            ),
            
            # BT Navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),
            
            # Waypoint follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),
            
            # Smoother server
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),
            
            # Velocity smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),  # Input from Nav2
                    ('cmd_vel_smoothed', 'cmd_vel'),  # Output to robot base
                ]
            ),
            
            # Lifecycle manager
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
    # RViz (optional)
    # ============================================================
    rviz_config = os.path.join(luna_mapping_dir, 'config', 'rtabmap_nav2.rviz')
    
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
    # Static transforms for camera frames and odometry
    # ============================================================
    
    # Initial odom -> base_link transform (identity, Gazebo will override once it starts)
    # This ensures the TF tree is connected immediately
    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=[
            '0', '0', '0',  # x, y, z (identity - robot starts at origin)
            '0', '0', '0',  # roll, pitch, yaw (identity)
            'odom',  # parent frame
            'base_link'  # child frame
        ],
        output='screen'
    )
    
    # Transform from base_link to Gazebo RGB camera frame
    # Position/orientation from URDF: depth_camera_joint (0.3, 0.1, 0.65, rpy=0,1,0)
    # Plus optical frame rotation (-1.5708, 0, -1.5708)
    static_tf_gazebo_rgb = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_gazebo_rgb',
        arguments=[
            '0.3', '0.1', '0.65',  # x, y, z (from URDF: depth_camera_joint)
            '-1.5708', '1', '-1.5708',  # roll, pitch, yaw (URDF + optical rotation)
            'base_link',  # parent frame
            'mooncake/base_link/d455_rgb_camera'  # Gazebo frame (from camera_info)
        ],
        output='screen'
    )
    
    # Transform from base_link to Gazebo depth camera frame
    static_tf_gazebo_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_gazebo_depth',
        arguments=[
            '0.3', '0.1', '0.65',  # x, y, z (from URDF)
            '-1.5708', '1', '-1.5708',  # roll, pitch, yaw (URDF + optical rotation)
            'base_link',  # parent frame
            'mooncake/base_link/d455_depth_camera'  # Gazebo frame
        ],
        output='screen'
    )
    
    # Also ensure camera_link exists (from URDF)
    static_tf_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_link',
        arguments=[
            '0.3', '0.1', '0.65',  # x, y, z (from URDF: depth_camera_joint)
            '0', '1', '0',  # roll, pitch, yaw (from URDF)
            'base_link',  # parent frame
            'camera_link'  # child frame
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_use_rtabmap_odom,
        declare_use_fiducial_odom,
        declare_launch_nav2,
        declare_launch_rviz,
        
        # Static TF transforms (must come first - RTAB-Map needs these immediately)
        static_tf_odom_base,  # Initial odom->base_link (Gazebo will override)
        static_tf_gazebo_rgb,
        static_tf_gazebo_depth,
        static_tf_camera_link,
        
        # Camera info and image fixers (must come before RTAB-Map and depth processing)
        camera_info_fixer_color,
        camera_info_fixer_depth,
        image_frame_fixer_rgb,
        image_frame_fixer_depth_laserscan,
        image_frame_fixer_depth_pointcloud,
        
        # RTAB-Map
        rtabmap_slam,
        rtabmap_odom,
        
        # Depth processing
        depth_to_laserscan,
        depth_to_pointcloud,
        
        # Fiducial localization
        fiducial_localizer,
        
        # Nav2
        nav2_nodes,
        
        # RViz
        rviz_node,
    ])
