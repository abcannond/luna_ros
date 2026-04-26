import os
import math

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
from launch.conditions import LaunchConfigurationEquals, IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    package_name = "lunabot_2425"
    try:
        new_world_package = "luna_ros2_worlds"
        worlds_pkg_share = get_package_share_directory(new_world_package)
        models_path = os.path.join(worlds_pkg_share, "models")
        worlds_path = os.path.join(worlds_pkg_share, "worlds")
        pkg_path = worlds_pkg_share
        has_worlds_pkg = True
    except PackageNotFoundError:
        new_world_package = package_name
        pkg_share = get_package_share_directory(package_name)
        models_path = os.path.join(pkg_share, "stl")
        worlds_path = os.path.join(pkg_share, "worlds")
        pkg_path = pkg_share
        has_worlds_pkg = False

    # Arena selection: pass world:=ucf_arena or world:=artemis_arena on the launch command
    declare_world = DeclareLaunchArgument(
        "world",
        default_value="ucf_arena",
        description="Arena: ucf_arena (default) or artemis_arena",
    )
    world_name = LaunchConfiguration("world", default="ucf_arena")

    # When true (default), start RTAB-Map + Nav2 + RViz after Gazebo is up (single-command sim).
    declare_launch_mapping = DeclareLaunchArgument(
        "launch_mapping",
        default_value="true",
        description="If true, launch RTAB-Map, Nav2, and RViz after ~18 s (single terminal). If false, Gazebo + fid cam RViz only.",
    )
    launch_mapping = LaunchConfiguration("launch_mapping", default="true")
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value="",
        description="Optional RViz config path for rtabmap_nav2_sim (empty = use its default).",
    )
    rviz_config = LaunchConfiguration("rviz_config", default="")
    # gz_args as Substitutions so launch arg is applied when using luna_ros2_worlds
    if has_worlds_pkg:
        gz_args_parts = [
            TextSubstitution(text="-r " + worlds_path + "/"),
            world_name,
            TextSubstitution(text=".sdf"),
        ]
    else:
        gz_args_parts = [
            TextSubstitution(text="-r " + os.path.join(worlds_path, "empty.world")),
        ]

    # --- RSP ---
    rsp_source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package_name),
        "launch",
        "rsp.launch.py"
    ))

    rsp = IncludeLaunchDescription(
        rsp_source,
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # --- GZ SIM ---
    gz_sim_source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py"
    ))

    gz_sim_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{models_path}:{worlds_path}:{pkg_path}"
    )

    gz_sim = IncludeLaunchDescription(
        gz_sim_source,
        launch_arguments={
            "gz_args": gz_args_parts,
            "on_exit_shutdown": "True",
        }.items(),
    )

    # UCF arena spawn
    # NOTE: spawn a bit deeper into the corner so the initial pose is not
    # hugging the start-zone poles. This gives Nav2 more free space to plan
    # a path before interacting with the pole cost.
    gz_create_robot_ucf = Node(
        package="ros_gz_sim",
        executable="create",
        condition=LaunchConfigurationEquals("world", "ucf_arena"),
        arguments=[
            "-topic", "robot_description",
            "-name", "mooncake",
            "-x", "-3.80",
            "-y", "3.10",
            "-z", "0.1",
            "-R", "0",
            "-P", "0",
            "-Y", str(-math.pi/2),
        ],
        output="screen",
    )
    # Artemis arena spawn (exactly from artemis_arenas branch)
    gz_create_robot_artemis = Node(
        package="ros_gz_sim",
        executable="create",
        condition=LaunchConfigurationEquals("world", "artemis_arena"),
        arguments=[
            "-topic", "robot_description",
            "-name", "mooncake",
            "-x", "-2.750",
            "-y", "-1.750",
            "-z", "0.1",
            "-R", "0",
            "-P", "0",
            "-Y", str(math.pi/2),
        ],
        output="screen",
    )
    # Delay spawn so Gazebo has time to load the world
    gz_create_robot_delayed = TimerAction(
        period=10.0,
        actions=[gz_create_robot_ucf, gz_create_robot_artemis],
    )

    # --- GZ BRIDGES ---
    gz_param_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            "config_file": os.path.join(
                get_package_share_directory(package_name),
                "config",
                "gz_bridge.config.yaml",
            )
        }],
        output='screen',
    )

    gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    # Relay /cmd_vel (Twist) -> /cmd_vel_stamped (TwistStamped). LunaController subscribes to
    # /cmd_vel_stamped via robot_controllers.yaml command_topic so we don't depend on gz_ros2_control namespace.
    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        parameters=[{"use_sim_time": True, "frame_id": "base_link"}],
        remappings=[
            ("cmd_vel_in", "/cmd_vel"),
            ("cmd_vel_out", "/cmd_vel_stamped"),
        ],
        output="screen",
    )
    # --- CONTROLLER SPAWNERS (correct order) ---

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    luna_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "luna_cont",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    # Spawn joint_state_broadcaster AFTER the robot is spawned (either arena)
    spawn_jsb_after_robot_ucf = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_create_robot_ucf,
            on_exit=[
                LogInfo(msg="Robot spawned — waiting for gz_ros_control to initialise..."),
                TimerAction(
                    period=5.0,
                    actions=[
                        LogInfo(msg="Starting joint_state_broadcaster..."),
                        joint_state_broadcaster_spawner,
                    ],
                ),
            ]
        )
    )
    spawn_jsb_after_robot_artemis = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_create_robot_artemis,
            on_exit=[
                LogInfo(msg="Robot spawned — waiting for gz_ros_control to initialise..."),
                TimerAction(
                    period=5.0,
                    actions=[
                        LogInfo(msg="Starting joint_state_broadcaster..."),
                        joint_state_broadcaster_spawner,
                    ],
                ),
            ]
        )
    )

    # Spawn luna_cont ONLY AFTER joint_state_broadcaster has finished activating
    spawn_luna_cont_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg="joint_state_broadcaster active — starting luna_cont..."),
                luna_controller_spawner,
            ]
        )
    )

    # depth_to_pointcloud: only when mapping is NOT active (mapping launch has its own
    # frame-fixed + ground-filtered point cloud pipeline via depth_image_proc).
    # Running both publishes conflicting data on /camera/camera/depth/color/points.
    depth_to_pointcloud_node = Node(
        package='depth_to_pointcloud',
        executable='depth_to_pointcloud_node',
        name='depth_to_pointcloud_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(launch_mapping),
    )

    start_depth_to_pointcloud_ucf = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_create_robot_ucf,
            on_exit=[
                LogInfo(msg="Robot spawned — starting depth_to_pointcloud node..."),
                depth_to_pointcloud_node
            ]
        )
    )
    start_depth_to_pointcloud_artemis = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_create_robot_artemis,
            on_exit=[
                LogInfo(msg="Robot spawned — starting depth_to_pointcloud node..."),
                depth_to_pointcloud_node
            ]
        )
    )


    # --- RVIZ (only when NOT launching mapping: depth + fid cams view) ---
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "depth_and_fid_cams_view.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    rviz_delayed = TimerAction(period=5.0, actions=[rviz_node])

    # --- RTAB-Map + Nav2 (when launch_mapping true: single-command sim) ---
    rtabmap_nav2_source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("luna_mapping"),
            "launch",
            "rtabmap_nav2_sim.launch.py",
        )
    )
    rtabmap_nav2_include = IncludeLaunchDescription(
        rtabmap_nav2_source,
        launch_arguments={
            "use_sim_time": "true",
            "launch_rviz": "true",
            "launch_nav2": "true",
        }.items(),
    )
    # Fixed delay: wait for sim to build and robot to spawn (25 s), then start RTAB-Map/Nav2
    sim_ready_msg = LogInfo(msg="Sim is ready! Starting RTAB-Map/Nav2...")
    rtabmap_nav2_delayed = TimerAction(
        period=25.0,
        actions=[sim_ready_msg, rtabmap_nav2_include],
        condition=IfCondition(launch_mapping),
    )
    nav2_up_msg = TimerAction(
        period=37.0,
        actions=[LogInfo(msg="Nav2 is up!")],
        condition=IfCondition(launch_mapping),
    )

    return LaunchDescription([
        declare_world,
        declare_launch_mapping,
        declare_rviz_config,
        rsp,
        gz_sim_resource,
        gz_sim,
        gz_create_robot_delayed,
        twist_stamper,
        spawn_jsb_after_robot_ucf,
        spawn_jsb_after_robot_artemis,
        spawn_luna_cont_after_jsb,
        gz_param_bridge,
        gz_image_bridge,
        start_depth_to_pointcloud_ucf,
        start_depth_to_pointcloud_artemis,
        rtabmap_nav2_delayed,
        nav2_up_msg,
        # RViz (depth + fid cams) only when mapping is disabled
        TimerAction(period=5.0, actions=[rviz_node], condition=UnlessCondition(launch_mapping)),
    ])