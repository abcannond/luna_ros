FROM ros:jazzy-perception-noble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-depth-image-proc \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5core5a \
    libx11-xcb1 \
    libxcb-xinerama0 \
    libxcb-xfixes0 \
    libxcb-shape0 \
    libxcb-randr0 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-xtest0 \
    mesa-utils \
    dos2unix \
    ros-jazzy-realsense2-camera \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-rtabmap-launch \
    ros-jazzy-rtabmap-slam \
    ros-jazzy-rtabmap-rviz-plugins \
    ros-jazzy-tf2-tools \
    \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*


# Set working directory
WORKDIR /ros2_ws

# Entry point will handle sourcing ROS and workspace
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Environment for Gazebo resources
ENV GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:/ros2_ws/src/"

# Use entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command: open bash
CMD ["bash"]
