FROM ros:jazzy-perception-noble

SHELL ["/bin/bash", "-c"]

# Install required ROS and GUI dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-depth-image-proc \
    ros-jazzy-cv-bridge \
    libeigen3-dev \
    libopencv-dev \
    python3-colcon-common-extensions \
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
    dos2unix && \
    rm -rf /var/lib/apt/lists/*


RUN echo "alias src='source install/setup.bash'" >> /etc/bash.bashrc
RUN echo "alias srcr='source /opt/ros/jazzy/setup.bash'" >> /etc/bash.bashrc
RUN echo "alias cdr='cd /ros2_ws'" >> /etc/bash.bashrc
RUN echo "alias bld='colcon build --symlink-install'" >> /etc/bash.bashrc
RUN echo "alias drive='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/luna_cont/cmd_vel_unstamped'" >> /etc/bash.bashrc
RUN echo "alias sim='ros2 launch lunabot_2425 gz_bringup.launch.py'" >> /etc/bash.bashrc

WORKDIR /ros2_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Gazebo resources
# ENV GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:/ros2_ws/src/"
ENV GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share:/ros2_ws/src/luna_ros2_worlds/models

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
