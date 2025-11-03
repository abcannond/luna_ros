# FROM ros:jazzy-perception-noble

# SHELL ["/bin/bash", "-c"]

# RUN apt-get update && apt-get upgrade -y

# RUN apt-get install -y \
# ros-jazzy-ros-gz \
# ros-jazzy-ros2-control \
# ros-jazzy-ros2-controllers \
# ros-jazzy-gz-ros2-control \
# libqt5gui5 \
# libqt5widgets5 \
# libqt5core5a \
# libx11-xcb1 \
# libxcb-xinerama0 \
# libxcb-xfixes0 \
# libxcb-shape0 \
# libxcb-randr0 \
# libxcb-image0 \
# libxcb-keysyms1 \
# libxcb-xtest0 \
# mesa-utils

# RUN mkdir /ros2_ws
# WORKDIR /ros2_ws
# COPY ./ros2_ws/ .

# RUN source /opt/ros/jazzy/setup.bash \
# && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
# && colcon build --symlink-install

# WORKDIR /

# COPY entrypoint.sh /
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT [ "/entrypoint.sh" ]

# ENV GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:/ros2_ws/src/"

# CMD ["ros2", "launch", "lunabot_2425", "gz_bringup.launch.py"]

# Base ROS 2 Jazzy image
# FROM ros:jazzy-perception-noble

# SHELL ["/bin/bash", "-c"]

# # Update and install dependencies
# RUN apt-get update && apt-get upgrade -y && \
#     apt-get install -y \
#         ros-jazzy-ros-gz \
#         ros-jazzy-ros2-control \
#         ros-jazzy-ros2-controllers \
#         ros-jazzy-gz-ros2-control \
#         libqt5gui5 \
#         libqt5widgets5 \
#         libqt5core5a \
#         libx11-xcb1 \
#         libxcb-xinerama0 \
#         libxcb-xfixes0 \
#         libxcb-shape0 \
#         libxcb-randr0 \
#         libxcb-image0 \
#         libxcb-keysyms1 \
#         libxcb-xtest0 \
#         mesa-utils \
#         python3-colcon-common-extensions \
#         python3-rosdep \
#         python3-vcstool \
#         git \
#         nano \
#         curl \
#     && rm -rf /var/lib/apt/lists/*

# # Create workspace
# RUN mkdir -p /ros2_ws/src
# WORKDIR /ros2_ws

# # Copy your package source code into the container
# COPY ./ros2_ws/src ./src

# # Install dependencies and build with symlink (so edits in src reflect immediately)
# RUN source /opt/ros/jazzy/setup.bash && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src --rosdistro jazzy -y && \
#     colcon build --symlink-install

# # Set Gazebo resource path to include your models
# ENV GZ_SIM_RESOURCE_PATH="/ros2_ws/src:${GZ_SIM_RESOURCE_PATH}"

# # Copy entrypoint
# COPY entrypoint.sh /
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]

# # Default launch
# CMD ["ros2", "launch", "lunabot_2425", "gz_bringup.launch.py"]

# Use ROS 2 Jazzy base image


# Use ROS Jazzy with perception stack as base
FROM ros:jazzy-perception-noble

SHELL ["/bin/bash", "-c"]

# Update and install required packages
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
        git \
        curl \
        python3-pip

# Create workspace
RUN mkdir -p /ros2_ws
WORKDIR /ros2_ws

# Copy the entire workspace including all subfolders
COPY ./ros2_ws /ros2_ws

# Install dependencies and build
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y && \
    colcon build --symlink-install

# Set entrypoint
WORKDIR /
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Ensure Gazebo can find models and worlds inside the workspace
ENV GZ_SIM_RESOURCE_PATH="/ros2_ws/src/lunabot_2425/models:/ros2_ws/src/lunabot_2425/worlds:${GZ_SIM_RESOURCE_PATH}"

# Default launch command
CMD ["ros2", "launch", "lunabot_2425", "gz_bringup.launch.py"]
