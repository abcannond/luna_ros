# Luna ROS — desktop/sim image (aligned with origin/master + RViz/mesh-friendly libs).
# Base image already includes core ROS 2 Jazzy perception stacks; we add sim, Nav2,
# RTAB-Map debians, build tools, and OpenGL/Mesa bits so RViz renders materials reliably.

FROM ros:jazzy-perception-noble

SHELL ["/bin/bash", "-c"]

# Install required ROS, build, GUI, and mesh-rendering dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-depth-image-proc \
    ros-jazzy-cv-bridge \
    libeigen3-dev \
    libopencv-dev \
    libgl1 \
    libglx0 \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-vulkan-drivers \
    libxcb-xinerama0 \
    libxcb-xfixes0 \
    libxcb-shape0 \
    libxcb-randr0 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-xtest0 \
    libx11-xcb1 \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5core5a \
    mesa-utils \
    dos2unix \
    libassimp5 \
    ros-jazzy-rviz2 \
    ros-jazzy-realsense2-camera \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-rtabmap-launch \
    ros-jazzy-rtabmap-slam \
    ros-jazzy-rtabmap-odom \
    ros-jazzy-rtabmap-rviz-plugins \
    ros-jazzy-tf2-tools \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-depthimage-to-laserscan \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-v4l2-camera \
    && rm -rf /var/lib/apt/lists/*

# Shell QoL (same as master)
RUN echo "alias src='source install/setup.bash'" >> /etc/bash.bashrc && \
    echo "alias srcr='source /opt/ros/jazzy/setup.bash'" >> /etc/bash.bashrc && \
    echo "alias cdr='cd /ros2_ws'" >> /etc/bash.bashrc && \
    echo "alias bld='colcon build --symlink-install'" >> /etc/bash.bashrc && \
    echo "alias drive='PYTHONUNBUFFERED=1 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel -p use_sim_time:=true'" >> /etc/bash.bashrc && \
    echo "alias sim='ros2 launch lunabot_2425 gz_bringup.launch.py'" >> /etc/bash.bashrc

WORKDIR /ros2_ws

COPY entrypoint.sh /entrypoint.sh
RUN dos2unix /entrypoint.sh && chmod +x /entrypoint.sh

# Default Gazebo resource path when workspace is bind-mounted at /ros2_ws (gz_bringup also sets this at launch).
ENV GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share:/ros2_ws/src/luna_ros2_worlds/models

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
