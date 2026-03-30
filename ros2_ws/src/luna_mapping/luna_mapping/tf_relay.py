#!/usr/bin/env python3
"""
TF Relay node to ensure odom -> base_link transform exists.
Gazebo might publish TF with namespace prefixes, this node ensures
the standard ROS frames are connected.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_ros


class TfRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        
        self.declare_parameter('source_frame', 'odom')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('publish_rate', 30.0)
        
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to periodically check and republish transform
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.get_logger().info(
            f'TF Relay: Relaying {self.source_frame} -> {self.target_frame}'
        )
    
    def timer_callback(self):
        try:
            # Try to get the transform
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Republish with standard frame names
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.source_frame
            t.child_frame_id = self.target_frame
            t.transform = transform.transform
            
            self.tf_broadcaster.sendTransform(t)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            # Transform doesn't exist yet or error - that's ok, will retry
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error in tf_relay: {e}')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
