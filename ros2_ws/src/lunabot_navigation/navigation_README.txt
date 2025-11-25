SETUP
    1. Make sure you are in the right location
    cd ~/Desktop/MQP/luna_ros/ros2_ws/src/lunabot_navigation

    2. Make it executable
    chmod +x mission_manager.py navigate_to_coordinate.py nav_helpers.py


RUN DOCKER SIMUMULATION
    docker build -t run_luna_sim:latest .
    ./run_ros_image.sh run_luna_sim:latest

BUILD
    cd /ros2_ws
    colcon build --packages-select lunabot_navigation
    source install/setup.bash
    ros2 run lunabot_navigation mission_manager