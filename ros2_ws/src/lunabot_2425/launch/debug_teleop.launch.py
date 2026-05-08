from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    teleop_twist_joy_params = {
        "axis_linear": {"x": 1, "y": 0},
        "axis_angular": {"yaw": 3},
        "scale_linear": {"x": 0.5, "y": 0.5},
        "scale_angular": {"yaw": 0.5},
        "require_enable_button": False,
        "enable_button": 0,
        "enable_turbo_button": -1,
        "use_sim_time": False,
    }

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[teleop_twist_joy_params],
        remappings=[
            ("cmd_vel", "/cmd_vel"),
        ],
        output="screen",
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])