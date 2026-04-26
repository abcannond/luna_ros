# Installation & Setup

## Overview

All ROS 2 commands run inside a Docker container. Git and Docker commands run on the host.

| Where | What |
|-------|------|
| **Host** (repo root: `luna_ros`) | Git, Docker build/run |
| **Inside container** | `colcon build`, `ros2 launch`, etc. |

---

## Optional: Shell aliases

Add to `~/.bashrc` (then `source ~/.bashrc`):

```bash
alias dbld='docker build -t luna_ros:latest .'
alias dbld_nc='docker build --no-cache -t luna_ros:latest .'
alias drun='./run_ros_image.sh luna_ros:latest'
alias dbash='docker exec -it $(docker ps -q -f "ancestor=luna_ros:latest") /bin/bash'
alias gitmod='git submodule update --init --recursive'
```

---

## First time setup

On the host, from any directory:

```bash
git clone git@github.com:abcannond/luna_ros.git
cd luna_ros
git submodule update --init --recursive
docker build -t luna_ros:latest .
./run_ros_image.sh luna_ros:latest
```

Inside the container:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## After `git pull`

Inside the container (rebuild changed packages so configs and launch files are current):

```bash
cd /ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select luna_mapping luna_nav lunabot_2425 luna_control fiducial_localizer depth_to_pointcloud
source install/setup.bash
```

---

## Jetson (ARM64)

```bash
docker build -f Dockerfile.jetson -t luna_ros:jetson .
./run_ros_image.sh luna_ros:jetson
```

See [docs/HARDWARE.md](docs/HARDWARE.md) for hardware-specific steps.
