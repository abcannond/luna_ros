#!/bin/bash
# Git commands to push RTAB-Map + Nav2 integration to fiducial_shit branch

cd /home/piotr/luna_ros/ros2_ws/luna_ros

# Check current branch
echo "Current branch:"
git branch --show-current

# Show what will be committed
echo ""
echo "Files to be committed:"
git status --short

# Add all changes (modified and new files)
echo ""
echo "Adding all changes..."
git add .

# Commit with descriptive message
echo ""
echo "Committing changes..."
git commit -m "Add RTAB-Map + Nav2 costmap integration with depth camera

- Integrated RTAB-Map SLAM for occupancy grid generation from RealSense D455
- Added Nav2 navigation stack with depth-based costmaps
- Created depth-to-laserscan and depth-to-pointcloud converters
- Added camera_info and image frame_id fixers for Gazebo compatibility
- Updated URDF with proper optical frames for cameras
- Added static TF transforms for odom->base_link and camera frames
- Created comprehensive launch file: rtabmap_nav2_sim.launch.py
- Updated topic names to match existing conventions (/camera/camera/...)
- Added Nav2 configuration for depth-based obstacle detection
- Created RViz config and documentation files"

# Push to remote branch
echo ""
echo "Pushing to origin/fiducial_shit..."
git push origin fiducial_shit

echo ""
echo "Done! Changes pushed to fiducial_shit branch."
