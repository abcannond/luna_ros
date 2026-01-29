# Launch Commands for RTAB-Map + Nav2 Integration

## Prerequisites

**First time setup (if not done):**
```bash
# Build Docker image
cd /home/piotr/luna_ros/ros2_ws/luna_ros
docker build -t luna_ros:latest .

# Start container
./run_ros_image.sh luna_ros:latest
```

---

## Terminal 1: Gazebo Simulation

**Purpose**: Start the Gazebo simulation with robot and sensors

```bash
# Inside Docker container
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Launch Gazebo simulation
ros2 launch lunabot_2425 gz_bringup.launch.py
```

**What you should see:**
- Gazebo GUI opens with robot in world
- Robot spawned (check for "Robot spawned" message)
- Camera topics being bridged
- No errors about missing packages

**Keep this terminal running!**

---

## Terminal 2: RTAB-Map + Nav2 Stack

**Purpose**: Launch RTAB-Map SLAM, depth processing, and Nav2 navigation

```bash
# Open new terminal and exec into container
docker exec -it <CONTAINER_ID> /bin/bash

# Source workspace
cd /ros2_ws
source install/setup.bash

# Launch RTAB-Map + Nav2
ros2 launch luna_mapping rtabmap_nav2_sim.launch.py
```

**What you should see:**
- RTAB-Map processing frames: `rtabmap (X): Rate=1.00s...`
- No "Could not convert rgb/depth msgs" errors
- Nav2 nodes starting up
- Camera info fixers running
- Depth processing nodes active

**Keep this terminal running!**

---

## Terminal 3: Monitoring/Diagnostics (Optional)

**Purpose**: Monitor topics and verify everything is working

```bash
# Open new terminal and exec into container
docker exec -it <CONTAINER_ID> /bin/bash
source /ros2_ws/install/setup.bash

# Check all topics
ros2 topic list

# Monitor key topics
ros2 topic hz /map                          # Occupancy grid
ros2 topic hz /scan                         # LaserScan
ros2 topic hz /camera/camera/depth/color/points  # Point cloud
ros2 topic hz /local_costmap/costmap        # Local costmap
ros2 topic hz /global_costmap/costmap       # Global costmap
ros2 topic hz /odom                          # Odometry

# Check TF tree
ros2 run tf2_tools view_frames

# Verify transforms
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# Check node status
ros2 node list
ros2 node info /rtabmap
```

---

## Terminal 4: RViz Visualization (Optional)

**Purpose**: Visualize maps, costmaps, and sensor data

```bash
# Open new terminal and exec into container
docker exec -it <CONTAINER_ID> /bin/bash
source /ros2_ws/install/setup.bash

# Launch RViz with RTAB-Map config
rviz2 -d $(ros2 pkg prefix luna_mapping)/share/luna_mapping/config/rtabmap_nav2.rviz
```

**Or launch RViz manually and add displays:**
```bash
rviz2
```

**Then add these displays:**
1. **Map** → Topic: `/map`
2. **Local Costmap** → Topic: `/local_costmap/costmap`
3. **Global Costmap** → Topic: `/global_costmap/costmap`
4. **PointCloud2** → Topic: `/camera/camera/depth/color/points`
5. **LaserScan** → Topic: `/scan`
6. **RGB Image** → Topic: `/camera/camera/color/image_raw_fixed`
7. **RobotModel** → Description Topic: `/robot_description`
8. **TF** → Show all frames

---

## Terminal 5: Teleop/Control (Optional)

**Purpose**: Manually drive the robot for testing

```bash
# Open new terminal and exec into container
docker exec -it <CONTAINER_ID> /bin/bash
source /ros2_ws/install/setup.bash

# Use the drive alias (sends to /cmd_vel)
drive

# Or manually:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

**Controls:**
- `i` - Forward
- `,` - Backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `q`/`z` - Increase/decrease linear speed
- `w`/`x` - Increase/decrease angular speed

---

## Quick Launch Script (All-in-One)

Create a convenience script to launch everything:

```bash
# Create launch script
cat > /ros2_ws/launch_all.sh << 'EOF'
#!/bin/bash
set -e

cd /ros2_ws
source install/setup.bash

echo "=========================================="
echo "Launching RTAB-Map + Nav2 Integration"
echo "=========================================="
echo ""
echo "Make sure Gazebo is running in Terminal 1 first!"
echo "Press Enter to continue..."
read

echo "Starting RTAB-Map + Nav2..."
ros2 launch luna_mapping rtabmap_nav2_sim.launch.py
EOF

chmod +x /ros2_ws/launch_all.sh
```

**Usage:**
```bash
./launch_all.sh
```

---

## Launch Arguments (Customization)

### RTAB-Map + Nav2 Launch Options:

```bash
# Use RTAB-Map visual odometry instead of Gazebo odom
ros2 launch luna_mapping rtabmap_nav2_sim.launch.py use_rtabmap_odom:=true

# Enable fiducial marker localization
ros2 launch luna_mapping rtabmap_nav2_sim.launch.py use_fiducial_odom:=true

# Launch RViz automatically
ros2 launch luna_mapping rtabmap_nav2_sim.launch.py launch_rviz:=true

# Disable Nav2 (mapping only)
ros2 launch luna_mapping rtabmap_nav2_sim.launch.py launch_nav2:=false

# Combine options
ros2 launch luna_mapping rtabmap_nav2_sim.launch.py \
    use_rtabmap_odom:=true \
    launch_rviz:=true
```

---

## Troubleshooting Commands

### If something isn't working:

```bash
# Check if topics exist
ros2 topic list

# Check topic rates
ros2 topic hz <topic_name>

# Check if nodes are running
ros2 node list

# Check node info
ros2 node info <node_name>

# Check TF frames
ros2 run tf2_tools view_frames

# View logs
ros2 topic echo /rosout | grep ERROR

# Restart a specific node (if needed)
# Stop launch, fix issue, restart
```

---

## Complete Launch Sequence

**Recommended order:**

1. **Terminal 1**: Start Gazebo (wait for full startup)
2. **Terminal 2**: Start RTAB-Map + Nav2
3. **Terminal 3**: Monitor topics (optional)
4. **Terminal 4**: Launch RViz (optional)
5. **Terminal 5**: Teleop control (optional)

---

## Quick Reference

| Terminal | Command | Purpose |
|----------|---------|---------|
| 1 | `ros2 launch lunabot_2425 gz_bringup.launch.py` | Gazebo simulation |
| 2 | `ros2 launch luna_mapping rtabmap_nav2_sim.launch.py` | RTAB-Map + Nav2 |
| 3 | `ros2 topic hz /map` | Monitoring |
| 4 | `rviz2` | Visualization |
| 5 | `drive` | Teleop control |

---

## Finding Container ID

```bash
# List running containers
docker ps

# Use container ID or name
docker exec -it <CONTAINER_ID> /bin/bash
# OR
docker exec -it <container_name> /bin/bash
```
