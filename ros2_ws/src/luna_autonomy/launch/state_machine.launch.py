#!/usr/bin/env python3
"""
Launch the Luna autonomy state machine node.

-----------------------------------------------------------------------------
TODO (launch file): This file is a stub. Purpose is to document what is left.
-----------------------------------------------------------------------------
TODO: State machine is intentionally non-functional until BOOT, Nav2, and
      hardware interfaces are implemented in state_machine.py.
TODO: Ensure mapping + Nav2 are running BEFORE starting this (e.g. run
      lunabot_2425 + luna_mapping rtabmap_nav2 first). Otherwise BOOT fails.
TODO: Decide whether to include this in a top-level "full autonomy" launch
      (e.g. hardware_bringup + rtabmap_nav2 + state_machine) or keep separate.
TODO: If using sim time, pass use_sim_time:=true and ensure /clock is published.
TODO: Add remappings when state machine subscribes/publishes (e.g. /cmd_vel,
      /goal_pose, /current_zone) once those are defined in code.
TODO: Consider launching state_machine with a delay after Nav2 so BOOT's
      "Nav2 readiness" check has time to pass.
TODO: Document required launch order in COMMANDS.md or LAUNCH_COMMANDS.md.
-----------------------------------------------------------------------------
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # -------------------------------------------------------------------------
    # TODO: Declare launch arguments if needed, e.g.:
    #   use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    #   Then pass {'use_sim_time': use_sim_time} in parameters below.
    # TODO: Add argument for log level or config file path if desired.
    # -------------------------------------------------------------------------

    state_machine_node = Node(
        package="luna_autonomy",
        executable="state_machine",
        name="state_machine",
        output="screen",
        # ---------------------------------------------------------------------
        # TODO: Add parameters once state_machine reads params (e.g. timeouts,
        #       excavation/berm targets, satisfaction threshold).
        # parameters=[{}],
        # TODO: Add remappings when topics/actions are defined in state_machine,
        #       e.g. remappings=[('/cmd_vel', '/luna_cont/cmd_vel')],
        # ---------------------------------------------------------------------
    )

    return LaunchDescription([
        # TODO: Add any DeclareLaunchArgument here.
        state_machine_node,
    ])
