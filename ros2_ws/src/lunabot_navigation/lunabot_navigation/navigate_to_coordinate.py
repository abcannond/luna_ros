#!/usr/bin/env python3
"""
Module for navigating to coordinates.
Called by mission_manager or other high-level planners.
Uses Nav2's prebuilt navigation stack.
"""
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class NavigateToCoordinate:
    def __init__(self):
        self.navigator = BasicNavigator()
        
    def go_to(self, x, y, theta=0.0, frame_id='map'):
        """
        Navigate to (x, y) coordinates.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters  
            theta: Orientation in radians (default: 0.0)
            frame_id: Reference frame (default: 'map')
            
        Returns:
            bool: True if navigation succeeded, False otherwise
        """
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0  # No rotation for now
        
        # Send to Nav2 (prebuilt!)
        self.navigator.goToPose(goal_pose)
        
        # Wait for completion (prebuilt!)
        while not self.navigator.isTaskComplete():
            pass
            
        # Return result (prebuilt!)
        return self.navigator.getResult()

# Can be used as a module
def navigate_to_coordinate(x, y):
    """Simple function interface for quick use"""
    nav = NavigateToCoordinate()
    return nav.go_to(x, y)