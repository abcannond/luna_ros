import os
import math

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    package_name = "lunabot_2425"
    try:
        new_world_package = "luna_ros2_worlds"
        worlds_pkg_share = get_package_share_directory(new_world_package)
        models_path = os.path.join(worlds_pkg_share, "models")
        worlds_path = os.path.join(worlds_pkg_share, "worlds")
        pkg_path = worlds_pkg_share
        gz_world_file = os.path.join(worlds_path, "ucf_arena.sdf")
    except PackageNotFoundError:
        new_world_package = package_name
        pkg_share = get_package_share_directory(package_name)
        models_path = os.path.join(pkg_share, "stl")
        worlds_path = os.path.join(pkg_share, "worlds")
        pkg_path = pkg_share
        gz_world_file = os.path.join(worlds_path, "empty.world")

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
            "gz_args": f"-r {gz_world_file}",
            "on_exit_shutdown": "True",
        }.items(),
    )

    # # --- SPAWN ROBOT ---
    # gz_create_robot = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     arguments=[
    #         "-topic", "robot_description",
    #         "-name", "mooncake",
    #         "-x", "-3",
    #         "-y", "-3",
    #         "-z", "0.3",
    #     ],
    #     output="screen",
    # )

    gz_create_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "mooncake",
            "-x", "-3.75",
            "-y", "2.75",
            "-z", "0.1",
            "-R", "0",           # roll
            "-P", "0",           # pitch
            "-Y", str(-math.pi/2) # yaw about Z
        ],
        output="screen",
    )
    # Delay spawn so Gazebo has time to load the world; otherwise create keeps
    # "Requesting list of world names" and the robot/controllers never start.
    gz_create_robot_delayed = TimerAction(period=10.0, actions=[gz_create_robot])

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

    # Spawn joint_state_broadcaster AFTER the robot is spawned
    spawn_jsb_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_create_robot,
            on_exit=[
                LogInfo(msg="Robot spawned — starting joint_state_broadcaster..."),
                joint_state_broadcaster_spawner,
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

    depth_to_pointcloud_node = Node(
        package='depth_to_pointcloud',          # your package name
        executable='depth_to_pointcloud_node',  # node executable
        name='depth_to_pointcloud_node',
        output='screen',
        parameters=[{'use_sim_time': True}],     # optional if you want sim time
        remappings=[                             # optional, remap your topics
            # ('/input_depth', '/camera/depth/image_raw'),
            # ('/output_points', '/camera/depth/points'),
        ]
    )

    # Start depth_to_pointcloud AFTER the robot is spawned
    start_depth_to_pointcloud = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_create_robot,
            on_exit=[
                LogInfo(msg="Robot spawned — starting depth_to_pointcloud node..."),
                depth_to_pointcloud_node
            ]
        )
    )


    # --- RVIZ ---
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        "rviz",
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
    # Start RViz after sim and /clock are running so it uses sim time for TF (avoids TF_OLD_DATA).
    rviz_delayed = TimerAction(period=5.0, actions=[rviz_node])

    return LaunchDescription([
        rsp,
        gz_sim_resource,
        gz_sim,
        gz_create_robot_delayed,
        twist_stamper,
        spawn_jsb_after_robot,
        spawn_luna_cont_after_jsb,
        gz_param_bridge,
        gz_image_bridge,
        start_depth_to_pointcloud,
        rviz_delayed,
    ])


#old launch file with timers in case we need to bring her back. 

# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import (
#     IncludeLaunchDescription,
#     SetEnvironmentVariable,
#     RegisterEventHandler,
#     LogInfo,
# )
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.event_handlers import OnProcessExit
# from launch_ros.actions import Node


# def generate_launch_description():

#     package_name = "lunabot_2425"
#     new_world_package = "luna_ros2_worlds"

#     # --- RSP ---
#     rsp_source = PythonLaunchDescriptionSource(os.path.join(
#         get_package_share_directory(package_name),
#         "launch",
#         "rsp.launch.py"
#     ))

#     rsp = IncludeLaunchDescription(
#         rsp_source,
#         launch_arguments={"use_sim_time": "true"}.items(),
#     )

#     # --- GZ SIM ---
#     gz_sim_source = PythonLaunchDescriptionSource(os.path.join(
#         get_package_share_directory("ros_gz_sim"),
#         "launch",
#         "gz_sim.launch.py"
#     ))

#     models_path = os.path.join(get_package_share_directory(new_world_package), "models")
#     worlds_path = os.path.join(get_package_share_directory(new_world_package), "worlds")
#     pkg_path = get_package_share_directory(new_world_package)

#     gz_sim_resource = SetEnvironmentVariable(
#         name='GZ_SIM_RESOURCE_PATH',
#         value=f"{models_path}:{worlds_path}:{pkg_path}"
#     )

#     gz_world_file = os.path.join(worlds_path, "ucf_arena.sdf")

#     gz_sim = IncludeLaunchDescription(
#         gz_sim_source,
#         launch_arguments={
#             "gz_args": f"-r {gz_world_file}",
#             "on_exit_shutdown": "True",
#         }.items(),
#     )

#     # --- SPAWN ROBOT ---
#     gz_create_robot = Node(
#         package="ros_gz_sim",
#         executable="create",
#         arguments=[
#             "-topic", "robot_description",
#             "-name", "mooncake",
#             "-x", "-3",
#             "-y", "-3",
#             "-z", "0.3",
#         ],
#         output="screen",
#     )

#     # --- GZ BRIDGES ---
#     gz_param_bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         parameters=[{
#             "config_file": os.path.join(
#                 get_package_share_directory(package_name),
#                 "config",
#                 "gz_bridge.config.yaml",
#             )
#         }],
#         output='screen',
#     )

#     twist_stamper = Node(
#         package='twist_stamper',
#         executable='twist_stamper',
#         parameters=[{'use_sim_time': True}],
#         remappings=[
#             ('/cmd_vel_in', '/luna_cont/cmd_vel_unstamped'),
#             ('/cmd_vel_out', '/luna_cont/cmd_vel'),
#         ],
#     )

#     gz_image_bridge = Node(
#         package="ros_gz_image",
#         executable="image_bridge",
#         arguments=["/camera/image_raw"]
#     )

#     # --- CONTROLLER SPAWNERS (correct order) ---

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster",
#             "--controller-manager", "/controller_manager"
#         ],
#         output="screen"
#     )

#     luna_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "luna_cont",
#             "--controller-manager", "/controller_manager"
#         ],
#         output="screen"
#     )

#     # Spawn controllers AFTER the robot is spawned into Gazebo
#     spawn_controllers_after_robot = RegisterEventHandler(
#         OnProcessExit(
#             target_action=gz_create_robot,
#             on_exit=[
#                 LogInfo(msg="Robot spawned — starting joint_state_broadcaster..."),
#                 joint_state_broadcaster_spawner,

#                 LogInfo(msg="joint_state_broadcaster active — starting luna_cont..."),
#                 luna_controller_spawner,
#             ]
#         )
#     )

#     # --- RVIZ ---
#     rviz_config_file = os.path.join(
#         get_package_share_directory(package_name),
#         "rviz",
#         "depth_and_fid_cams_view.rviz"
#     )

#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         arguments=["-d", rviz_config_file],
#         output="screen",
#     )

#     return LaunchDescription([
#         rsp,
#         twist_stamper,
#         gz_sim_resource,
#         gz_sim,
#         gz_create_robot,
#         spawn_controllers_after_robot,
#         gz_param_bridge,
#         gz_image_bridge,
#         rviz_node,
#     ])