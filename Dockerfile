FROM ros:jazzy-perception-noble

SHELL ["/bin/bash", "-c"]

# Install required ROS and GUI dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-depth-image-proc \
    # OpenGL + Vulkan (Intel/AMD/NVIDIA support)
    libgl1 \
    libglx0 \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-vulkan-drivers \
    # X11 + Qt deps
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


WORKDIR /ros2_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Gazebo resources
ENV GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:/ros2_ws/src/"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
