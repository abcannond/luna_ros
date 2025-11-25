#!/usr/bin/env python3
"""
Navigation helper functions.
All utility functions for navigation live here.
"""
from geometry_msgs.msg import PoseStamped
import math

def create_goal_pose(x, y, navigator, theta=0.0, frame_id='map'):
    """
    Create a goal pose for Nav2.
    
    Args:
        x: X coordinate in meters
        y: Y coordinate in meters
        navigator: BasicNavigator instance (for clock)
        theta: Orientation in radians (default: 0.0)
        frame_id: Reference frame (default: 'map')
        
    Returns:
        PoseStamped: Goal pose ready for Nav2
    """
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = frame_id
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    
    # Convert theta to quaternion (yaw only)
    goal_pose.pose.orientation.w = math.cos(theta / 2)
    goal_pose.pose.orientation.z = math.sin(theta / 2)
    
    return goal_pose

def distance_between_points(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def is_position_reached(current_x, current_y, goal_x, goal_y, tolerance=0.1):
    """Check if robot is within tolerance of goal"""
    dist = distance_between_points(current_x, current_y, goal_x, goal_y)
    return dist < tolerance

# Add more helper functions as needed:
# - angle_between_points()
# - normalize_angle()
# - create_waypoint_list()
# etc.