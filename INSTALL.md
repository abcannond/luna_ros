# Installation & Setup

## Overview

This file contains everything required to install, build, and update the luna_ros
environment. Runtime launch commands are intentionally excluded.

---

## Where You Run Things

Host (repo root: luna_ros):

- Git
- Docker

Inside Docker container:

- ROS commands
- Either:
  - the shell from run_ros_image.sh, or
  - a new terminal via dbash or:
  docker exec -it  /bin/bash

---

## Optional: Shell Aliases

(Run once per machine — avoids typing long commands)

Add these to ~/.bashrc (nano ~/.bashrc, paste at end, save), then run:
  source ~/.bashrc

Aliases:
  alias dbld='docker build -t luna_ros:latest .'
  alias dbld_nc='docker build --no-cache -t luna_ros:latest .'
  alias drun='./run_ros_image.sh luna_ros:latest'
  alias dbash='docker exec -it $(docker ps -q -f "ancestor=luna_ros:latest") /bin/bash'
  alias gitmod='git submodule update --init --recursive'

After that, you can use:
  dbld      build
  dbld_nc  clean build
  drun      run container
  dbash     new shell in container
  gitmod    update submodules

---

## FIRST TIME

(Once per machine or fresh clone)

Host, repo root:

  git clone [git@github.com](mailto:git@github.com):abcannond/luna_ros.git
  cd luna_ros
  gitmod
  dbld
  drun

---

## Inside Container

(Once per clone or when packages change)

  cd /ros2_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install
  source install/setup.bash

You can ignore the quad_swerve_controller "not found" message.

---

## AFTER git pull

(Required — otherwise RTAB-Map / Nav2 may use old config)

Inside container:

  cd /ros2_ws && source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --packages-select luna_mapping luna_nav lunabot_2425
  source install/setup.bash