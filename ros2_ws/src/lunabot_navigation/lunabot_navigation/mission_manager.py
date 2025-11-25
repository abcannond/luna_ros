#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from lunabot_navigation.nav_helpers import create_goal_pose
import time

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.navigator = BasicNavigator()
        self.state = "IDLE"
        self.timer = self.create_timer(1.0, self.run)
        self.get_logger().info('Mission Manager started!')
        
    def run(self):
        if self.state == "IDLE":
            self.get_logger().info('IDLE - Starting in 1 second...')
            time.sleep(1)
            self.state = "SIM_GO_TO_COORDINATE"
            
        elif self.state == "SIM_GO_TO_COORDINATE":
            self.get_logger().info('Navigating to (2.0, 2.0)')
            goal = create_goal_pose(2.0, 2.0, self.navigator)

            self.navigator.goToPose(goal) 
            
            while not self.navigator.isTaskComplete():
                time.sleep(1)
                
            self.get_logger().info('Reached goal!' if self.navigator.getResult() else 'Failed!')
            self.state = "COMPLETE"
        
        elif self.state == "COMPLETE":
            self.get_logger().info('Done')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()