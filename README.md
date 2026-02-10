Luna ROS — Simulation and Navigation Stack
==========================================

This repo runs a ROS 2 Jazzy simulation of the WPI Lunabotics robot in Gazebo Harmonic, with RTAB-Map for mapping and Nav2 for navigation. Everything runs inside Docker so you get a consistent environment (tested on Ubuntu 24.04).

What it does
------------

1. Gazebo simulates the robot, a depth camera (RealSense D455), and four fiducial (wheel-pod) cameras. You can run either the UCF arena or the Artemis arena via a launch argument (`world:=ucf_arena` or `world:=artemis_arena`).

2. RTAB-Map uses the depth camera images to build a map (SLAM) and publish an occupancy grid on /map.

3. Depth data is turned into a 2D laser-style scan (/scan) and a 3D point cloud so Nav2 can see obstacles.

4. Nav2 uses the map and the scan/point cloud to plan paths and send velocity commands so the robot can drive to a goal you click in RViz.

5. Fiducial localization (on by default) uses all four wheel-pod cameras to detect ArUco markers and publish robot pose in the map frame, so at least one marker is visible for continuous pose updates throughout the run.

In short: Gazebo sends camera data into RTAB-Map (which builds the map) and into depth processing (which makes obstacles visible to Nav2). Nav2 then plans and drives the robot. Fiducial localization (4 cameras) runs by default for pose estimation from markers.

Quick start
-----------

First time: clone the repo, init submodules, build the Docker image, then start the container. See LAUNCH_COMMANDS.md for the exact commands.

Every run:
  1. Start the container if it's not running.
  2. Terminal 1: Build the workspace once if needed, then launch Gazebo.
  3. Terminal 2: In a new terminal, exec into the same container and launch RTAB-Map + Nav2.
  4. Optionally open RViz and use the Nav2 Goal tool to send a navigation goal.

The full step-by-step (what to run in each terminal, what you should see, and troubleshooting) is in LAUNCH_COMMANDS.md. For a minimal list of commands only, see Commands to run and what.txt.

How to run the container
------------------------

From the repo root, after building the image:

  ./run_ros_image.sh luna_ros:latest

That script sets up X11 so GUIs (Gazebo, RViz) show up on your machine. For Terminals 2 and on, you open new host terminals and run:

  docker exec -it <CONTAINER_ID> /bin/bash

to get another shell in the same container. Use docker ps to see the container ID.

Docs in this repo
-----------------

  LAUNCH_COMMANDS.md   — Full guide: how it works, what to run in each terminal, options, troubleshooting.
  Commands to run and what.txt   — Short command list and topic reference.
  Dockerfile   — Defines the Docker image (ROS 2 Jazzy, Gazebo, Nav2, RTAB-Map, etc.).

Requirements
------------

  Docker
  Linux with X11 (e.g. Ubuntu 24.04)
  Clone, submodules, and docker build as described in LAUNCH_COMMANDS.md
