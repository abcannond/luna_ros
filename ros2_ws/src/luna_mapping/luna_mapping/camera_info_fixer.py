#!/usr/bin/env python3
"""
Republishes camera_info messages with corrected frame_id.
Gazebo's ros_gz_bridge sets frame_id to Gazebo sensor names (e.g., mooncake/base_link/d455_rgb_camera),
but RTAB-Map expects standard ROS frame names (e.g., camera_color_optical_frame).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoFixer(Node):
    def __init__(self):
        super().__init__('camera_info_fixer')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_frame_id', 'camera_color_optical_frame')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        
        # Create publisher and subscriber
        self.publisher_ = self.create_publisher(CameraInfo, output_topic, 10)
        self.subscription = self.create_subscription(
            CameraInfo,
            input_topic,
            self.camera_info_callback,
            10
        )
        
        self.get_logger().info(
            f'CameraInfoFixer: Republishing {input_topic} -> {output_topic} '
            f'with frame_id={self.output_frame_id}'
        )
    
    def camera_info_callback(self, msg):
        # Create new message with corrected frame_id
        fixed_msg = CameraInfo()
        fixed_msg.header = msg.header
        fixed_msg.header.frame_id = self.output_frame_id
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.distortion_model = msg.distortion_model
        fixed_msg.d = msg.d
        fixed_msg.k = msg.k
        fixed_msg.r = msg.r
        fixed_msg.p = msg.p
        fixed_msg.binning_x = msg.binning_x
        fixed_msg.binning_y = msg.binning_y
        fixed_msg.roi = msg.roi
        
        self.publisher_.publish(fixed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error in camera_info_fixer: {e}')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore shutdown errors - launch handles cleanup


if __name__ == '__main__':
    main()
