#!/usr/bin/env python3
"""
Relay /luna_cont/odom -> /odom and broadcast odom->base_link TF.

LunaController publishes to ~/odom which resolves to /luna_cont/odom inside
gz_ros2_control. RTAB-Map and Nav2 expect /odom, and TF must be broadcast in
the normal ROS2 context so use_sim_time is handled correctly.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomRelay(Node):

    def __init__(self):
        super().__init__('odom_relay')
        self.declare_parameter('source_topic', '/luna_cont/odom')
        source = self.get_parameter('source_topic').value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._broadcaster = TransformBroadcaster(self)
        self._pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Odometry, source, self._cb, qos)
        self.get_logger().info(f'odom_relay: {source} -> /odom + TF')

    def _cb(self, msg: Odometry):
        self._pub.publish(msg)

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
