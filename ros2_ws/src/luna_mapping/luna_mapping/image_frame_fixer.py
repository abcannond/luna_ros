#!/usr/bin/env python3
"""
Republishes image messages with corrected frame_id.
Fixes frame_id mismatches between RGB and depth images for point cloud generation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageFrameFixer(Node):
    def __init__(self):
        super().__init__('image_frame_fixer')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('output_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('output_frame_id', 'camera_depth_optical_frame')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        
        # Create publisher and subscriber
        self.publisher_ = self.create_publisher(Image, output_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info(
            f'ImageFrameFixer: Republishing {input_topic} -> {output_topic} '
            f'with frame_id={self.output_frame_id}'
        )
    
    def image_callback(self, msg):
        # Create new message with corrected frame_id
        fixed_msg = Image()
        fixed_msg.header = msg.header
        fixed_msg.header.frame_id = self.output_frame_id
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.encoding = msg.encoding
        fixed_msg.is_bigendian = msg.is_bigendian
        fixed_msg.step = msg.step
        fixed_msg.data = msg.data
        
        self.publisher_.publish(fixed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageFrameFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error in image_frame_fixer: {e}')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
