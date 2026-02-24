#!/usr/bin/env python3
"""
Converts depth image to colorized RGB for RViz display.
Depth (32FC1 or 16UC1) -> RGB8 with jet colormap.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthToRgbNode(Node):
    def __init__(self):
        super().__init__('depth_to_rgb')
        self.declare_parameter('input_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('output_topic', '/camera/camera/depth/image_colorized')
        self.declare_parameter('max_range', 5.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.max_range = self.get_parameter('max_range').value

        # Match ros_gz_bridge QoS (Reliable)
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, output_topic, qos)
        self.sub = self.create_subscription(Image, input_topic, self.cb, qos)

        self.get_logger().info(f'DepthToRgb: {input_topic} -> {output_topic}')

    def cb(self, msg: Image):
        try:
            if msg.encoding in ('32FC1',):
                arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                arr = np.nan_to_num(arr, nan=0.0, posinf=0.0)
                arr = np.clip(arr, 0, self.max_range)
                norm = (arr / self.max_range * 255).astype(np.uint8)
            elif msg.encoding in ('16UC1', 'mono16'):
                arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                max_val = min(65535, int(self.max_range * 1000))  # mm
                arr = np.clip(arr, 0, max_val)
                norm = (arr.astype(np.float32) / max_val * 255).astype(np.uint8)
            else:
                self.get_logger().warn_once(f'Unsupported encoding: {msg.encoding}')
                return

            colorized = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
            colorized = cv2.cvtColor(colorized, cv2.COLOR_BGR2RGB)

            out = self.bridge.cv2_to_imgmsg(colorized, encoding='rgb8')
            out.header = msg.header
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn_once(f'DepthToRgb error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthToRgbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
