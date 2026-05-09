# quad_swerve_controller

This package provides an older ROS 2 `ros2_control` controller implementation for the Luna drivetrain.

**Current bringup does not load it.** The active stack uses **`luna_control`** (`LunaController` as `luna_cont` in `lunabot_2425/config/robot_controllers.yaml`).

Keep this package only if you still need the code for reference, comparison, or a future migration. If the team standardizes on `luna_control` only, this tree can be removed after a maintainer sign-off.
