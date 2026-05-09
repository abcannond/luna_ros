#!/usr/bin/env python3
"""
ROS 2 port of 23–24 depth_imaging_package/depth_publisher.py obstacle branch.

Uses aligned-depth style processing: Canny on smoothed depth, contour filter,
convex hull + area + roundness, then deproject hull center using CameraInfo.

Publishes Float32MultiArray [x, y, z, radius_m] in the depth camera optical frame
(same layout as legacy /realsense/depth/obstacle for compatibility).
"""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray


def deproject(u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float):
    if z <= 0.0 or not math.isfinite(z):
        return None
    x = (float(u) - cx) * z / fx
    y = (float(v) - cy) * z / fy
    return (x, y, z)


class DepthRoundObstacleDetector(Node):
    def __init__(self):
        super().__init__('depth_round_obstacle_detector')

        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('min_depth_difference', 10.0)
        self.declare_parameter('canny_threshold1', 10.0)
        self.declare_parameter('canny_threshold2', 50.0)
        self.declare_parameter('min_hull_area', 4000.0)
        self.declare_parameter('max_hull_area', 27000.0)
        self.declare_parameter('min_roundness', 0.7)
        self.declare_parameter('width_crop_fraction', 0.05)
        self.declare_parameter('height_crop_fraction', 0.3)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('processing_rate_hz', 3.0)

        self._bridge = CvBridge()
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None
        self._frame_id = ''
        self._have_info = False
        self._warned_dtype = False
        self._last_proc = 0.0

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._pub = self.create_publisher(Float32MultiArray, '/luna/depth_round_obstacle', 10)
        self._pub_point = self.create_publisher(PointStamped, '/luna/depth_round_obstacle/point', 10)
        self._pub_debug = self.create_publisher(Image, '/luna/depth_round_obstacle/debug/image', 2)

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._info_cb,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self._depth_cb,
            qos,
        )

        self.get_logger().info(
            'depth_round_obstacle_detector: 23–24-style round blob detection; '
            'output /luna/depth_round_obstacle + optional debug image'
        )

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
        if rate > 0:
            now = time.monotonic()
            if now - self._last_proc < (1.0 / rate):
                return
            self._last_proc = now

        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        if depth.dtype != np.uint16 and not getattr(self, '_warned_dtype', False):
            self.get_logger().warn(
                f'Expected uint16 depth, got {depth.dtype}; values assumed millimeters'
            )
            self._warned_dtype = True

        h, w = depth.shape[:2]
        ch = int(h * float(self.get_parameter('height_crop_fraction').value))
        cw = int(w * float(self.get_parameter('width_crop_fraction').value))
        if ch <= 0 or cw <= 0 or 2 * cw >= w or ch >= h:
            return

        d_roi = depth[0 : h - ch, cw : w - cw]
        u_off, v_off = cw, 0

        if d_roi.size == 0:
            return

        work = d_roi.astype(np.float32)
        mx = float(np.max(work)) if np.max(work) > 0 else 1.0
        smoothed = cv2.GaussianBlur(work, (3, 3), 0)
        sm8 = (smoothed / mx * 255.0).astype(np.uint8)

        t1 = float(self.get_parameter('canny_threshold1').value)
        t2 = float(self.get_parameter('canny_threshold2').value)
        edges = cv2.Canny(sm8, int(t1), int(t2))
        kernel = np.ones((7, 7), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_dd = float(self.get_parameter('min_depth_difference').value)
        amin = float(self.get_parameter('min_hull_area').value)
        amax = float(self.get_parameter('max_hull_area').value)
        min_round = float(self.get_parameter('min_roundness').value)
        publish_debug = bool(self.get_parameter('publish_debug_image').value)

        bgr = None
        if publish_debug:
            norm = cv2.normalize(sm8, None, 0, 255, cv2.NORM_MINMAX)
            bgr = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)

        best = None
        for contour in contours:
            if len(contour) < 3:
                continue
            mask = np.zeros_like(sm8, dtype=np.uint8)
            cv2.drawContours(mask, [contour], 0, 255, thickness=cv2.FILLED)
            avg_depth = float(np.mean(sm8[mask > 0]))
            py, px = int(contour[0][0][1]), int(contour[0][0][0])
            if py >= sm8.shape[0] or px >= sm8.shape[1]:
                continue
            if abs(avg_depth - float(sm8[py, px])) <= min_dd:
                continue

            hull = cv2.convexHull(contour)
            area_h = cv2.contourArea(hull)
            perim = cv2.arcLength(hull, True)
            if perim <= 1e-6:
                continue
            roundness = (4.0 * math.pi * area_h) / (perim ** 2)
            if not (amin <= area_h <= amax and roundness >= min_round):
                continue

            (cx_loc, cy_loc), radius_px = cv2.minEnclosingCircle(hull)
            cx_loc, cy_loc = int(cx_loc), int(cy_loc)
            u_full = cx_loc + u_off
            v_full = cy_loc + v_off

            z_m = float(depth[v_full, u_full]) / 1000.0
            pt0 = deproject(u_full, v_full, z_m, self._fx, self._fy, self._cx, self._cy)
            if pt0 is None:
                continue

            if cx_loc + int(radius_px) < d_roi.shape[1]:
                s, t = cy_loc, cx_loc + int(radius_px)
            elif cx_loc - int(radius_px) >= 0:
                s, t = cy_loc, cx_loc - int(radius_px)
            else:
                s, t = cy_loc, cx_loc
            u_edge = t + u_off
            v_edge = s + v_off
            z_e = float(depth[v_edge, u_edge]) / 1000.0
            pt1 = deproject(u_edge, v_edge, z_e, self._fx, self._fy, self._cx, self._cy)
            if pt1 is None:
                continue
            rad_m = abs(pt0[0] - pt1[0])
            best = (pt0[0], pt0[1], pt0[2], rad_m, hull, (cx_loc, cy_loc), int(radius_px))
            break

        if best is not None:
            x0, y0, z0, rad_m, hull, center, radius_px = best
            out = Float32MultiArray()
            out.data = [float(x0), float(y0), float(z0), float(rad_m)]
            self._pub.publish(out)

            ps = PointStamped()
            ps.header = msg.header
            ps.header.frame_id = self._frame_id or msg.header.frame_id
            ps.point.x = x0
            ps.point.y = y0
            ps.point.z = z0
            self._pub_point.publish(ps)

            if bgr is not None:
                cv2.drawContours(bgr, [hull], 0, (255, 0, 0), 2)
                cv2.circle(bgr, center, radius_px, (0, 0, 255), 2)
        elif publish_debug and bgr is not None:
            pass

        if publish_debug and bgr is not None:
            try:
                dbg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
                dbg.header = msg.header
                dbg.header.frame_id = self._frame_id or msg.header.frame_id
                self._pub_debug.publish(dbg)
            except CvBridgeError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = DepthRoundObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
