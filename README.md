
# Run Lunabotics robot simulation in Docker

This repo is meant to describe the Docker image(s) that are required to run a ROS2 Jazzy / Gazebo Harmonic simulation of the WPI Lunabotics 24-25 drivetrain.
It includes a Dockerfile to build, and a shell script to run that includes all of the necessary options to successfully run on Linux (tested on Ubuntu 24.04). 
The script allows GUI apps to run locally via Docker.

## How to run

Run the container once you've built an image by using 
`./run_ros_image.sh [image_id]`.
This will set up the X host properly, allowing Docker to present a visible GUI.
