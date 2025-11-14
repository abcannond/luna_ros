#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PosePrint(Node):
    def __init__(self):
        super().__init__('pose_print')
        self.create_subscription(PoseStamped, '/jetson/localizer_robot_pose', self.cb, 10)

    def cb(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        print(f"[{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}] "
              f"world->robot: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
              f"quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})")

def main():
    rclpy.init()
    node = PosePrint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
