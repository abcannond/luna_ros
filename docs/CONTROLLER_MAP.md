**Overview**
This is an outline of how teleop works, this includes:
1) how controller inputs are read
2) how sim is controlled


**How Files Work Together**
1) how controller inputs are read
 *Files*
 -``joy_teleop.launch.py``
   - sets up /joy publisher and joy_echo
   - driver node handles hardware inputs
 - ``joy_echo.py``
   - subscribes to /joy and prints values
 - setup.py

2) sim control
 *Files*
 - ``joy_teleop.launch.py``
   - starts teleop_nav_gate
 - ``teleop_twist_joy``
   - subscribes to /joy and publishes /teleop_cmd_vel_raw
 - ``teleop_nav_gate``
   - waits for "start" from controller
   - makes /teleop_cmd_vel_raw -> /cmd_vel (when armed)
 - ``gz_bringup.launch.py``
   - launches sim
   - launches twist_stamper
 - ``twist_stamper.py``
   - makes /cmd_vel -> /cmd_vel_stamped
 - ``robot_controllers.yaml``
   - config luna_cont so command_topic is /cmd_vel_stamped
   - luna_cont allows for robot to move in sim
 - ``gz_bridge.config.yaml``
   - bridges gazebo and RViz
