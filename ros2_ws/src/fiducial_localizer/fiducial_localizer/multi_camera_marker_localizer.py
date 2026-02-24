#!/usr/bin/env python3
"""
Multi-camera fiducial localizer: subscribes to all 4 wheel-pod cameras,
detects ArUco markers in any camera, and publishes robot pose in world frame.
Uses TF for each camera's pose w.r.t. base_link so at least one marker is
visible for continuous localization.
"""
import math
import time
from functools import partial

from typing import List, Optional

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from tf2_ros import TransformException

from .marker_localizer import (
    rotz,
    quat_from_R,
    tvec_R_to_T,
    invert_T,
)


class MultiCameraMarkerLocalizer(Node):
    def __init__(self):
        super().__init__("multi_camera_marker_localizer")

        # ---------- params ----------
        self.declare_parameter("image_topics", [
            "/fid_cams/front_left_camera/image_raw",
            "/fid_cams/front_right_camera/image_raw",
            "/fid_cams/back_left_camera/image_raw",
            "/fid_cams/back_right_camera/image_raw",
        ])
        self.declare_parameter("camera_info_topics", [
            "/fid_cams/front_left_camera/camera_info",
            "/fid_cams/front_right_camera/camera_info",
            "/fid_cams/back_left_camera/camera_info",
            "/fid_cams/back_right_camera/camera_info",
        ])
        self.declare_parameter("camera_frames", [
            "front_left_camera_link_optical",
            "front_right_camera_link_optical",
            "back_left_camera_link_optical",
            "back_right_camera_link_optical",
        ])
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("marker_id", -1)
        self.declare_parameter("marker_size_m", 0.16)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("world_to_marker_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("world_to_marker_yaw", 0.0)
        self.declare_parameter("republish_map_odom_rate_hz", 10.0)
        self.declare_parameter("max_map_odom_translation_jump_m", 0.5)
        self.declare_parameter("max_map_odom_rotation_jump_rad", 0.5)

        image_topics = self.get_parameter("image_topics").get_parameter_value().string_array_value
        camera_info_topics = self.get_parameter("camera_info_topics").get_parameter_value().string_array_value
        camera_frames = self.get_parameter("camera_frames").get_parameter_value().string_array_value
        n = min(len(image_topics), len(camera_info_topics), len(camera_frames))
        self.image_topics = list(image_topics)[:n]
        self.camera_info_topics = list(camera_info_topics)[:n]
        self.camera_frames = list(camera_frames)[:n]

        self.marker_id = self.get_parameter("marker_id").get_parameter_value().integer_value
        self.marker_size = self.get_parameter("marker_size_m").get_parameter_value().double_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.robot_frame = self.get_parameter("robot_frame").get_parameter_value().string_value
        self.world_to_marker_xyz = np.array(
            self.get_parameter("world_to_marker_xyz").get_parameter_value().double_array_value
        )
        self.world_to_marker_yaw = self.get_parameter("world_to_marker_yaw").get_parameter_value().double_value
        self.republish_rate_hz = self.get_parameter("republish_map_odom_rate_hz").get_parameter_value().double_value
        self.max_translation_jump = self.get_parameter("max_map_odom_translation_jump_m").get_parameter_value().double_value
        self.max_rotation_jump = self.get_parameter("max_map_odom_rotation_jump_rad").get_parameter_value().double_value
        # Last map->odom for republishing when no marker seen (keeps TF tree connected)
        self._last_tf_map_odom: Optional[TransformStamped] = None
        # ArUco
        dict_name = self.get_parameter("aruco_dict").get_parameter_value().string_value
        if not hasattr(cv2, "aruco"):
            self.get_logger().error("OpenCV built without aruco. Cannot localize.")
        if not hasattr(cv2.aruco, dict_name):
            self.get_logger().warn(f"Bad aruco_dict {dict_name}, falling back to DICT_4X4_50")
            dict_name = "DICT_4X4_50"
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.detect = lambda img: self.detector.detectMarkers(img)
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detect = lambda img: cv2.aruco.detectMarkers(
                img, self.aruco_dict, parameters=self.aruco_params
            )

        self.cam_info: List[Optional[tuple]] = [None] * n
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, "/fiducial_pose", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        R_wm = rotz(self.world_to_marker_yaw)
        self.T_w_m = tvec_R_to_T(self.world_to_marker_xyz, R_wm)
        self.publish_static_marker()

        # Image: best-effort to match gz bridge image topics
        qos_image = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_camera_info = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Reentrant group so sensor callbacks run in thread pool (avoids rclpy msg conversion issues in some setups)
        sensor_cb_group = ReentrantCallbackGroup()

        for i in range(n):
            self.create_subscription(
                Image,
                self.image_topics[i],
                partial(self.image_cb, i),
                qos_image,
                callback_group=sensor_cb_group,
            )
            self.create_subscription(
                CameraInfo,
                self.camera_info_topics[i],
                partial(self.camera_info_cb, i),
                qos_camera_info,
                callback_group=sensor_cb_group,
            )

        self.last_log_per_cam: List[float] = [0.0] * n

        if self.republish_rate_hz > 0:
            period = 1.0 / self.republish_rate_hz
            self.create_timer(period, self._republish_map_odom_cb)

        self.get_logger().info(
            f"[fiducial] READY: {n} cameras. Topics: {self.image_topics[0]} ... "
            "(waiting for camera_info then looking for ArUco; will log when marker detected)"
        )

    def publish_static_marker(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.world_frame
        tf.child_frame_id = "marker"
        tf.transform.translation.x = float(self.world_to_marker_xyz[0])
        tf.transform.translation.y = float(self.world_to_marker_xyz[1])
        tf.transform.translation.z = float(self.world_to_marker_xyz[2])
        qw = math.cos(self.world_to_marker_yaw / 2)
        qz = math.sin(self.world_to_marker_yaw / 2)  # yaw-only quat
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)
        self.static_tf_broadcaster.sendTransform(tf)

    def camera_info_cb(self, camera_idx: int, msg: CameraInfo) -> None:
        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        self.cam_info[camera_idx] = (fx, fy, cx, cy)

    def image_cb(self, camera_idx: int, msg: Image):
        now = time.time()
        if camera_idx >= len(self.cam_info) or self.cam_info[camera_idx] is None:
            if now - self.last_log_per_cam[camera_idx] > 3.0:
                self.get_logger().warn(
                    f"Camera {camera_idx} ({self.camera_frames[camera_idx]}): waiting for camera_info"
                )
                self.last_log_per_cam[camera_idx] = now
            return

        fx, fy, cx, cy = self.cam_info[camera_idx]
        if fx <= 0 or fy <= 0:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            if now - self.last_log_per_cam[camera_idx] > 2.0:
                self.get_logger().warn(f"Camera {camera_idx} cv_bridge: {e}")
                self.last_log_per_cam[camera_idx] = now
            return

        corners, ids, _ = self.detect(cv_img)
        if ids is None or len(ids) == 0:
            return

        ids_list = ids.flatten().tolist()
        use_idx = 0
        if self.marker_id >= 0:
            found = False
            for i, mid in enumerate(ids_list):
                if int(mid) == int(self.marker_id):
                    use_idx = i
                    found = True
                    break
            if not found:
                return

        s = self.marker_size
        obj_pts = np.array([
            [-s / 2, s / 2, 0.0],
            [s / 2, s / 2, 0.0],
            [s / 2, -s / 2, 0.0],
            [-s / 2, -s / 2, 0.0],
        ], dtype=np.float32)
        img_pts = corners[use_idx].reshape(-1, 2).astype(np.float32)
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        dist = np.zeros((5, 1), dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            return

        if now - self.last_log_per_cam[camera_idx] > 2.0:
            self.get_logger().info(
                f"Camera {camera_idx} ({self.camera_frames[camera_idx]}): marker detected"
            )
            self.last_log_per_cam[camera_idx] = now

        R_cm, _ = cv2.Rodrigues(rvec)
        t_cm = tvec.reshape(3)
        T_c_m = tvec_R_to_T(t_cm, R_cm)
        T_m_c = invert_T(T_c_m)

        # TF: base_link -> camera_optical at msg stamp
        try:
            t = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.camera_frames[camera_idx],
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as ex:
            if now - self.last_log_per_cam[camera_idx] > 2.0:
                self.get_logger().warn(
                    f"Camera {camera_idx} TF lookup failed: {ex}"
                )
                self.last_log_per_cam[camera_idx] = now
            return

        # T_base_cam from TF (translation + rotation)
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        q = t.transform.rotation
        R = self._quat_to_rot(q)
        T_base_cam = tvec_R_to_T(np.array([tx, ty, tz]), R)
        T_cam_base = invert_T(T_base_cam)

        # world -> robot: T_w_m @ T_m_c @ T_c_base
        T_w_r = self.T_w_m @ T_m_c @ T_cam_base

        # Publish pose
        p = PoseStamped()
        p.header = Header()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = self.world_frame
        p.pose.position.x = float(T_w_r[0, 3])
        p.pose.position.y = float(T_w_r[1, 3])
        p.pose.position.z = float(T_w_r[2, 3])
        qq = quat_from_R(T_w_r[:3, :3])
        p.pose.orientation.x = float(qq[0])
        p.pose.orientation.y = float(qq[1])
        p.pose.orientation.z = float(qq[2])
        p.pose.orientation.w = float(qq[3])
        self.pose_pub.publish(p)

        # Broadcast TF map -> odom (not map -> base_link) so the tree stays map -> odom -> base_link.
        # map -> base_link = T_w_r; we need map -> odom = T_w_r @ inv(T_odom_base).
        try:
            t_odom_base = self.tf_buffer.lookup_transform(
                "odom",
                self.robot_frame,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1),
            )
            T_odom_base = self._transform_to_matrix(t_odom_base)
            T_map_odom = T_w_r @ invert_T(T_odom_base)
            if self._last_tf_map_odom is not None:
                tx_last = self._last_tf_map_odom.transform.translation
                trans_jump = np.sqrt(
                    (T_map_odom[0, 3] - tx_last.x) ** 2
                    + (T_map_odom[1, 3] - tx_last.y) ** 2
                    + (T_map_odom[2, 3] - tx_last.z) ** 2
                )
                R_last = self._quat_to_rot(self._last_tf_map_odom.transform.rotation)
                R_rel = T_map_odom[:3, :3] @ R_last.T
                trace_r = np.trace(R_rel)
                rot_jump = math.acos(np.clip((trace_r - 1) / 2, -1, 1))
                if trans_jump > self.max_translation_jump or rot_jump > self.max_rotation_jump:
                    if now - self.last_log_per_cam[camera_idx] > 2.0:
                        self.get_logger().warn(
                            f"Fiducial: rejecting map->odom jump trans={trans_jump:.2f}m rot={rot_jump:.2f}rad"
                        )
                        self.last_log_per_cam[camera_idx] = now
                    pass
                else:
                    tf_map_odom = TransformStamped()
                    tf_map_odom.header.stamp = p.header.stamp
                    tf_map_odom.header.frame_id = self.world_frame
                    tf_map_odom.child_frame_id = "odom"
                    tf_map_odom.transform.translation.x = float(T_map_odom[0, 3])
                    tf_map_odom.transform.translation.y = float(T_map_odom[1, 3])
                    tf_map_odom.transform.translation.z = float(T_map_odom[2, 3])
                    qo = quat_from_R(T_map_odom[:3, :3])
                    tf_map_odom.transform.rotation.x = float(qo[0])
                    tf_map_odom.transform.rotation.y = float(qo[1])
                    tf_map_odom.transform.rotation.z = float(qo[2])
                    tf_map_odom.transform.rotation.w = float(qo[3])
                    self.tf_broadcaster.sendTransform(tf_map_odom)
                    self._last_tf_map_odom = tf_map_odom
            else:
                tf_map_odom = TransformStamped()
                tf_map_odom.header.stamp = p.header.stamp
                tf_map_odom.header.frame_id = self.world_frame
                tf_map_odom.child_frame_id = "odom"
                tf_map_odom.transform.translation.x = float(T_map_odom[0, 3])
                tf_map_odom.transform.translation.y = float(T_map_odom[1, 3])
                tf_map_odom.transform.translation.z = float(T_map_odom[2, 3])
                qo = quat_from_R(T_map_odom[:3, :3])
                tf_map_odom.transform.rotation.x = float(qo[0])
                tf_map_odom.transform.rotation.y = float(qo[1])
                tf_map_odom.transform.rotation.z = float(qo[2])
                tf_map_odom.transform.rotation.w = float(qo[3])
                self.tf_broadcaster.sendTransform(tf_map_odom)
                self._last_tf_map_odom = tf_map_odom
        except TransformException:
            pass  # odom->base_link not available yet; skip this TF update

        if now - self.last_log_per_cam[camera_idx] > 1.0:
            self.get_logger().info(
                f"pose from camera {camera_idx} ({self.camera_frames[camera_idx]}): "
                f"xyz=({p.pose.position.x:.3f}, {p.pose.position.y:.3f}, {p.pose.position.z:.3f})"
            )
            self.last_log_per_cam[camera_idx] = now

    def _republish_map_odom_cb(self) -> None:
        """Republish last map->odom or identity so Nav2 has map->odom->base_link."""
        stamp = self.get_clock().now().to_msg()
        if self._last_tf_map_odom is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._last_tf_map_odom.header.frame_id
            tf.child_frame_id = self._last_tf_map_odom.child_frame_id
            tf.transform = self._last_tf_map_odom.transform
            self.tf_broadcaster.sendTransform(tf)
        else:
            # Fallback: publish identity map->odom until first marker seen (connects TF tree for Nav2)
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.world_frame
            tf.child_frame_id = "odom"
            tf.transform.translation.x = 0.0
            tf.transform.translation.y = 0.0
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = 0.0
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)

    def _transform_to_matrix(self, t: TransformStamped) -> np.ndarray:
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        q = t.transform.rotation
        R = self._quat_to_rot(q)
        return tvec_R_to_T(np.array([tx, ty, tz]), R)

    @staticmethod
    def _quat_to_rot(q) -> np.ndarray:
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ], dtype=float)


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraMarkerLocalizer()
    try:
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
