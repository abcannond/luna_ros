#!/usr/bin/env python3
"""
Depth-based crater detector for Lunabotics.

Craters appear in the depth image as regions where depth is significantly
larger than the surrounding floor (camera looks into the depression).
Deep craters may also produce NaN/invalid clusters.

Algorithm:
  1. Estimate local floor depth via large-kernel median filter.
  2. Mark pixels where depth exceeds floor estimate by threshold.
  3. Cluster into round regions; filter by area and circularity.
  4. Project crater rim to 3D on the floor plane; publish PointCloud2.

Output /craters/points feeds into Nav2 voxel_layer with clearing=false
so crater markings persist in the local costmap.
"""

import math
import struct
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthCraterDetector(Node):

    def __init__(self):
        super().__init__('depth_crater_detector')

        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info_fixed')
        self.declare_parameter('depth_threshold_m', 0.10)
        self.declare_parameter('floor_kernel', 41)
        self.declare_parameter('min_crater_pixels', 200)
        self.declare_parameter('max_crater_pixels', 5000)
        self.declare_parameter('min_roundness', 0.45)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('height_crop_bottom', 0.20)
        self.declare_parameter('processing_rate_hz', 2.0)
        self.declare_parameter('publish_debug_image', False)

        self._bridge = CvBridge()
        self._fx = self._fy = self._cx = self._cy = None
        self._frame_id = ''
        self._have_info = False
        self._last_proc = 0.0

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._info_cb, 10)
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self._depth_cb, qos)

        self._cloud_pub = self.create_publisher(PointCloud2, '/craters/points', 10)
        self._debug_pub = self.create_publisher(Image, '/craters/debug/image', 2)

        self.get_logger().info('depth_crater_detector: ready')

    def _info_cb(self, msg: CameraInfo):
        self._fx = float(msg.k[0])
        self._fy = float(msg.k[4])
        self._cx = float(msg.k[2])
        self._cy = float(msg.k[5])
        self._frame_id = msg.header.frame_id
        self._have_info = True

    def _depth_cb(self, msg: Image):
        if not self._have_info:
            return

        rate = float(self.get_parameter('processing_rate_hz').value)
        now = time.monotonic()
        if now - self._last_proc < (1.0 / rate):
            return
        self._last_proc = now

        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        depth = raw.astype(np.float32)
        if raw.dtype == np.uint16:
            depth /= 1000.0

        h, w = depth.shape
        max_range = float(self.get_parameter('max_range_m').value)
        crop_rows = int(h * float(self.get_parameter('height_crop_bottom').value))

        valid = (depth > 0.1) & (depth < max_range)
        if crop_rows > 0:
            valid[h - crop_rows:, :] = False
        depth_valid = np.where(valid, depth, 0.0).astype(np.float32)

        k = int(self.get_parameter('floor_kernel').value)
        if k % 2 == 0:
            k += 1
        floor_est = cv2.medianBlur(depth_valid, k)

        threshold = float(self.get_parameter('depth_threshold_m').value)
        crater_mask = ((depth_valid - floor_est) > threshold) & valid

        kernel = np.ones((5, 5), np.uint8)
        crater_u8 = crater_mask.astype(np.uint8)
        crater_u8 = cv2.morphologyEx(crater_u8, cv2.MORPH_CLOSE, kernel)
        crater_u8 = cv2.morphologyEx(crater_u8, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            crater_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_px = int(self.get_parameter('min_crater_pixels').value)
        max_px = int(self.get_parameter('max_crater_pixels').value)
        min_round = float(self.get_parameter('min_roundness').value)

        debug = bool(self.get_parameter('publish_debug_image').value)
        bgr = None
        if debug:
            norm = cv2.normalize(depth_valid, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            bgr = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)

        points = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (min_px <= area <= max_px):
                continue
            perim = cv2.arcLength(cnt, True)
            if perim < 1.0:
                continue
            roundness = (4.0 * math.pi * area) / (perim ** 2)
            if roundness < min_round:
                continue

            M = cv2.moments(cnt)
            if M['m00'] < 1.0:
                continue
            cx_px = int(M['m10'] / M['m00'])
            cy_px = int(M['m01'] / M['m00'])

            # Use floor depth (not crater depth) so points land on the floor plane
            z = float(floor_est[cy_px, cx_px])
            if z < 0.1:
                continue

            # Center point
            points.append(self._deproject(cx_px, cy_px, z))

            # Ring of 12 points around the crater circumference for better costmap coverage
            (ex, ey), er = cv2.minEnclosingCircle(cnt)
            for deg in range(0, 360, 30):
                rad = math.radians(deg)
                rx = int(ex + er * math.cos(rad))
                ry = int(ey + er * math.sin(rad))
                rx = max(0, min(rx, w - 1))
                ry = max(0, min(ry, h - 1))
                zr = float(floor_est[ry, rx])
                if zr > 0.1:
                    points.append(self._deproject(rx, ry, zr))

            if bgr is not None:
                cv2.drawContours(bgr, [cnt], 0, (0, 0, 255), 2)
                cv2.circle(bgr, (cx_px, cy_px), 4, (255, 0, 0), -1)

        if points:
            self._cloud_pub.publish(self._make_cloud(points, msg.header))

        if bgr is not None:
            try:
                dbg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
                dbg.header = msg.header
                self._debug_pub.publish(dbg)
            except CvBridgeError:
                pass

    def _deproject(self, u: int, v: int, z: float):
        x = (u - self._cx) * z / self._fx
        y = (v - self._cy) * z / self._fy
        return (float(x), float(y), float(z))

    def _make_cloud(self, points, header):
        cloud = PointCloud2()
        cloud.header = header
        cloud.header.frame_id = self._frame_id or header.frame_id
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.is_dense = True
        data = bytearray()
        for x, y, z in points:
            data += struct.pack('fff', x, y, z)
        cloud.data = bytes(data)
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = DepthCraterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
