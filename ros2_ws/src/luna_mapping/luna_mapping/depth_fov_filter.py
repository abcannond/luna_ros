#!/usr/bin/env python3
"""
Static mask for depth image: zero out fixed pixel regions.
- crop_left / crop_right: zero out this fraction of width on left/right (horizontal FOV crop).
- mask_regions: "x,y,w,h;..." for fiducials/self-obstacles.
- ground_crop_bottom: zero out this fraction of height at bottom.
"""

import rclpy
import json
import time
import urllib.request
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
        self.declare_parameter('crop_left', 0.0)
        self.declare_parameter('crop_right', 0.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        mask_str = str(self.get_parameter('mask_regions').value or '')
        self.mask_regions = parse_mask_regions(mask_str)
        try:
            self.ground_crop_bottom = max(0.0, min(1.0, float(self.get_parameter('ground_crop_bottom').value)))
        except (TypeError, ValueError, AttributeError):
            self.ground_crop_bottom = 0.0
        try:
            self.crop_left = max(0.0, min(0.5, float(self.get_parameter('crop_left').value)))
        except (TypeError, ValueError, AttributeError):
            self.crop_left = 0.0
        try:
            self.crop_right = max(0.0, min(0.5, float(self.get_parameter('crop_right').value)))
        except (TypeError, ValueError, AttributeError):
            self.crop_right = 0.0

        self.publisher_ = self.create_publisher(Image, output_topic, 10)
        self._debug_log_path = '/home/piotr/luna_ros/.cursor/debug.log'
        self._frame_count = 0
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'DepthFovFilter: {input_topic} -> {output_topic}, '
            f'crop_left={self.crop_left}, crop_right={self.crop_right}, '
            f'mask_regions={len(self.mask_regions)}, ground_crop={self.ground_crop_bottom}'
        )

    def image_callback(self, msg: Image):
        self._frame_count += 1
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

        do_sample_log = (self._frame_count % 30 == 0)
        pre_bottom_nonzero = 0
        pre_mid_nonzero = 0
        sample_pixels = 0
        sample_stride = 16

        bottom_rows = int(h * self.ground_crop_bottom) if self.ground_crop_bottom > 0 else 0
        left_cols = int(w * self.crop_left) if self.crop_left > 0 else 0
        right_cols = int(w * self.crop_right) if self.crop_right > 0 else 0
        if left_cols + right_cols > w:
            right_cols = max(0, w - left_cols)

        for row in range(h):
            base = row * msg.step
            if do_sample_log and row % sample_stride == 0:
                in_bottom = row >= max(0, h - max(bottom_rows, 1))
                in_mid = (h // 3) <= row < (2 * h // 3)
                if in_bottom or in_mid:
                    for col in range(0, w, sample_stride):
                        px = buf[base + (col * bpp): base + ((col + 1) * bpp)]
                        nz = 1 if any(px) else 0
                        sample_pixels += 1
                        if in_bottom:
                            pre_bottom_nonzero += nz
                        if in_mid:
                            pre_mid_nonzero += nz
            # Ground crop: zero bottom fraction of image
            if row >= h - bottom_rows and bottom_rows > 0:
                buf[base : base + msg.step] = b'\x00' * msg.step
                continue
            # Horizontal FOV crop: zero left and right columns
            if left_cols > 0:
                buf[base : base + left_cols * bpp] = b'\x00' * (left_cols * bpp)
            if right_cols > 0:
                right_byte = (w - right_cols) * bpp
                buf[base + right_byte : base + msg.step] = b'\x00' * (right_cols * bpp)
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

        if do_sample_log and sample_pixels > 0:
            post_bottom_nonzero = 0
            post_mid_nonzero = 0
            for row in range(0, h, sample_stride):
                base = row * msg.step
                in_bottom = row >= max(0, h - max(bottom_rows, 1))
                in_mid = (h // 3) <= row < (2 * h // 3)
                if in_bottom or in_mid:
                    for col in range(0, w, sample_stride):
                        px = out.data[base + (col * bpp): base + ((col + 1) * bpp)]
                        nz = 1 if any(px) else 0
                        if in_bottom:
                            post_bottom_nonzero += nz
                        if in_mid:
                            post_mid_nonzero += nz
            # #region agent log
            self._debug_log(
                hypothesis_id='H2',
                location='depth_fov_filter.py:image_callback',
                message='Depth sample occupancy before/after filter',
                data={
                    'frame': self._frame_count,
                    'encoding': msg.encoding,
                    'ground_crop_bottom': self.ground_crop_bottom,
                    'pre_bottom_nonzero': pre_bottom_nonzero,
                    'post_bottom_nonzero': post_bottom_nonzero,
                    'pre_mid_nonzero': pre_mid_nonzero,
                    'post_mid_nonzero': post_mid_nonzero,
                },
            )
            # #endregion

    def _debug_log(self, hypothesis_id: str, location: str, message: str, data: dict):
        try:
            payload = {
                'id': f'log_{int(time.time() * 1000)}_{hypothesis_id}',
                'timestamp': int(time.time() * 1000),
                'runId': 'pre-fix',
                'hypothesisId': hypothesis_id,
                'location': location,
                'message': message,
                'data': data,
            }
            try:
                # #region agent log
                req = urllib.request.Request(
                    'http://127.0.0.1:7243/ingest/118513d0-d9fa-4d38-9e11-6db013dbe340',
                    data=json.dumps(payload).encode('utf-8'),
                    headers={'Content-Type': 'application/json'},
                    method='POST',
                )
                urllib.request.urlopen(req, timeout=0.15).read()
                # #endregion
            except Exception:
                pass
            with open(self._debug_log_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(payload, separators=(',', ':')) + '\n')
        except Exception:
            pass


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
