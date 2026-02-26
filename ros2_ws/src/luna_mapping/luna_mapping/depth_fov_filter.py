#!/usr/bin/env python3
"""
Static mask for depth image: zero out fixed pixel regions.
Use for fiducial cameras and other self-obstacles that are static in camera view.
Format: "x,y,w,h;x,y,w,h" in normalized coords (0-1). x,y = top-left, w,h = size.
Example: "0.75,0,0.25,0.35" = right 25%, top 35% (one fiducial).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def parse_mask_regions(s: str):
    """Parse 'x,y,w,h;x,y,w,h' into list of (x, y, w, h) normalized 0-1."""
    if not s or not s.strip():
        return []
    regions = []
    for part in s.split(';'):
        part = part.strip()
        if not part:
            continue
        try:
            vals = [float(x.strip()) for x in part.split(',')]
            if len(vals) >= 4:
                regions.append((vals[0], vals[1], vals[2], vals[3]))
        except (ValueError, IndexError):
            pass
    return regions


class DepthFovFilter(Node):
    def __init__(self):
        super().__init__('depth_fov_filter')

        self.declare_parameter('input_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('output_topic', '/camera/camera/depth/image_rect_raw_filtered')
        self.declare_parameter('mask_regions', '')
        self.declare_parameter('ground_crop_bottom', 0.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        mask_str = str(self.get_parameter('mask_regions').value or '')
        self.mask_regions = parse_mask_regions(mask_str)
        try:
            self.ground_crop_bottom = max(0.0, min(1.0, float(self.get_parameter('ground_crop_bottom').value)))
        except (TypeError, ValueError, AttributeError):
            self.ground_crop_bottom = 0.0

        self.publisher_ = self.create_publisher(Image, output_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'DepthFovFilter: {input_topic} -> {output_topic}, '
            f'mask_regions={len(self.mask_regions)}, ground_crop={self.ground_crop_bottom}'
        )

    def image_callback(self, msg: Image):
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step

        # Work in a bytearray to avoid array.array slice-assignment issues; assign once at end
        buf = bytearray(msg.data)
        h, w = msg.height, msg.width

        if msg.encoding in ('16UC1', 'mono16'):
            bpp = 2
        elif msg.encoding in ('32FC1',):
            bpp = 4
        else:
            self.get_logger().warn(f'Unhandled encoding {msg.encoding}')
            return

        if msg.step <= 0 or len(buf) < h * msg.step:
            return

        bottom_rows = int(h * self.ground_crop_bottom) if self.ground_crop_bottom > 0 else 0

        for row in range(h):
            base = row * msg.step
            # Ground crop: zero bottom fraction of image
            if row >= h - bottom_rows and bottom_rows > 0:
                buf[base : base + msg.step] = b'\x00' * msg.step
                continue
            # Static mask regions
            for (nx, ny, nw, nh) in self.mask_regions:
                y0 = int(ny * h)
                y1 = int((ny + nh) * h)
                if row < y0 or row >= y1:
                    continue
                x0 = int(nx * w)
                x1 = int((nx + nw) * w)
                x0 = max(0, min(x0, w))
                x1 = max(0, min(x1, w))
                if x0 >= x1:
                    continue
                left_byte = x0 * bpp
                right_byte = x1 * bpp
                buf[base + left_byte : base + right_byte] = b'\x00' * (right_byte - left_byte)

        out.data = bytes(buf)
        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFovFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
