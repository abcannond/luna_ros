FROM ros:jazzy-perception-noble

SHELL ["/bin/bash", "-c"]

# Install required ROS and system packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
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
