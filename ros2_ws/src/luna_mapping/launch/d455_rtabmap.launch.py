# luna_mapping/launch/d455_rtabmap.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # --- RealSense D455 ---
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'enable_rgbd': 'true',
            'align_depth.enable': 'true',
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
        }.items()
    )

    # --- Static TF base_link -> camera_link ---
    # TODO: update the transform to match your physical camera mount.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_tf',
        arguments=[
            '0.20', '0.0', '0.30',   # x y z (meters)
            '0', '0', '0',           # roll pitch yaw (radians)
            'base_link',
            'camera_link'
        ]
    )

    # --- RTAB-Map visual SLAM + mapping ---
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            ])
        ),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'frame_id': 'base_link',
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'subscribe_rgb': 'true',
            'subscribe_depth': 'true',
            'approx_sync': 'true',
            'qos': '2',
            'rviz': 'true',
        }.items()
    )

    return LaunchDescription([
        realsense_launch,
        static_tf,
        rtabmap_launch,
    ])
