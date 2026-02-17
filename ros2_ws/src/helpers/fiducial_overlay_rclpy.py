#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

def _rotz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[ c,-s,0],
                     [ s, c,0],
                     [ 0, 0,1]], dtype=float)

class FiducialOverlay(Node):
    def __init__(self):
        super().__init__('fiducial_overlay')
        # params
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size_m', 0.16)
        self.declare_parameter('show_window', True)
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)

        # ArUco dictionary
        dict_name = self.get_parameter('aruco_dict').value
        if not hasattr(cv2.aruco, dict_name):
            self.get_logger().warn(f"Invalid aruco_dict {dict_name}, using DICT_4X4_50")
            dict_name = 'DICT_4X4_50'
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.detect = lambda img: self.detector.detectMarkers(img)
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detect = lambda img: cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)

        self.bridge = CvBridge()
        self.have_cam_info = (self.fx > 0 and self.fy > 0)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.sub_img = self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.sub_info = self.create_subscription(CameraInfo, self.camera_info_topic, self.caminfo_cb, 10)
        self.pub_overlay = self.create_publisher(Image, '/camera/image_annotated', 10)

        self.get_logger().info("fiducial_overlay ready: drawing boxes/axes on /camera/image_annotated")

    def caminfo_cb(self, msg: CameraInfo):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.have_cam_info = True

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        corners, ids, _ = self.detect(frame)
        if ids is not None and len(ids) > 0:
            ids = ids.flatten()
            # draw all markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # highlight target marker id (green)
            for i, mid in enumerate(ids):
                if int(mid) == self.marker_id:
                    pts = corners[i].astype(int).reshape(-1, 2)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                    # draw axes if intrinsics available
                    if self.have_cam_info:
                        s = self.marker_size
                        obj_pts = np.array([[-s/2,  s/2, 0.0],
                                            [ s/2,  s/2, 0.0],
                                            [ s/2, -s/2, 0.0],
                                            [-s/2, -s/2, 0.0]], dtype=np.float32)
                        img_pts = corners[i].reshape(-1, 2).astype(np.float32)
                        K = np.array([[self.fx, 0, self.cx],
                                      [0, self.fy, self.cy],
                                      [0, 0, 1]], dtype=np.float64)
                        dist = np.zeros((5,1), dtype=np.float64)
                        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        if ok and hasattr(cv2, "drawFrameAxes"):
                            try:
                                cv2.drawFrameAxes(frame, K, dist, rvec, tvec, s*0.5)
                            except Exception:
                                pass
                    break  # only one target id highlighted

        # publish overlay
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.pub_overlay.publish(out)

        # optional on-screen window
        if self.show_window:
            try:
                cv2.imshow("Fiducial Overlay (/camera/image_annotated)", frame)
                cv2.waitKey(1)
            except Exception:
                # likely no DISPLAY; disable after first failure
                self.show_window = False
                self.get_logger().warn("No GUI display available; disabling window output.")

def main():
    rclpy.init()
    node = FiducialOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.show_window:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
