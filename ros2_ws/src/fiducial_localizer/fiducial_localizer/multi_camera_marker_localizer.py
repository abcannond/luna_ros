#!/usr/bin/env python3
"""
Multi-camera fiducial localizer: subscribes to all 4 wheel-pod cameras,
detects ArUco markers in any camera, and publishes robot pose in world frame.
Uses TF for each camera's pose w.r.t. base_link so at least one marker is
visible for continuous localization.
"""
import math
import time
import json
import urllib.request
from functools import partial

from typing import List, Optional

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from tf2_ros import TransformException

from .marker_localizer import (
    rotz,
    rot_rpy,
    quat_from_euler,
    quat_from_R,
    tvec_R_to_T,
    invert_T,
)


class MultiCameraMarkerLocalizer(LifecycleNode):
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
        # Covariance published on /fiducial_pose. Position σ grows with tag
        # distance d (m) so the EKF down-weights long-range detections.
        self.declare_parameter("position_stddev", 0.05)
        self.declare_parameter("yaw_stddev", 0.02)
        self.declare_parameter("range_scale_per_m", 0.02)
        # Reject overly far detections that are noisy/ambiguous in startup geometry.
        # <=0 disables range gating.
        self.declare_parameter("max_marker_distance_m", 0.0)
        # Startup stabilization: require a short run of consistent detections
        # before publishing to avoid visible spawn jitter from early noisy solves.
        self.declare_parameter("startup_stability_seconds", 8.0)
        self.declare_parameter("startup_stable_xy_thresh_m", 0.03)
        self.declare_parameter("startup_stable_yaw_thresh_rad", 0.06)
        self.declare_parameter("startup_stable_required_frames", 4)
        # Multi-marker embedded tag params
        self.declare_parameter("outer_marker_ids", [-1])
        self.declare_parameter("inner_marker_ids", [-1])
        self.declare_parameter("outer_marker_size_m", 0.88)
        self.declare_parameter("inner_marker_size_m", 0.126)
        # Runtime stability control:
        # -1 => auto-lock to first observed marker group during this run.
        # >=0 => force that marker group index and ignore others.
        self.declare_parameter("active_marker_group", -1)
        self.declare_parameter("marker_world_x", [0.0])
        self.declare_parameter("marker_world_y", [0.0])
        self.declare_parameter("marker_world_z", [0.0])
        self.declare_parameter("marker_world_yaw_arr", [0.0])
        # Full SDF roll/pitch/yaw per marker (overrides yaw_arr if non-empty).
        # Required for vertical wall markers because face normal must be modeled.
        self.declare_parameter("marker_world_roll_arr", [0.0])
        self.declare_parameter("marker_world_pitch_arr", [0.0])

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
        self.position_stddev = self.get_parameter("position_stddev").get_parameter_value().double_value
        self.yaw_stddev = self.get_parameter("yaw_stddev").get_parameter_value().double_value
        self.range_scale_per_m = self.get_parameter("range_scale_per_m").get_parameter_value().double_value
        self.max_marker_distance_m = (
            self.get_parameter("max_marker_distance_m").get_parameter_value().double_value
        )
        self.startup_stability_seconds = (
            self.get_parameter("startup_stability_seconds").get_parameter_value().double_value
        )
        self.startup_stable_xy_thresh_m = (
            self.get_parameter("startup_stable_xy_thresh_m").get_parameter_value().double_value
        )
        self.startup_stable_yaw_thresh_rad = (
            self.get_parameter("startup_stable_yaw_thresh_rad").get_parameter_value().double_value
        )
        self.startup_stable_required_frames = int(
            self.get_parameter("startup_stable_required_frames").get_parameter_value().integer_value
        )
        self.active_marker_group = int(
            self.get_parameter("active_marker_group").get_parameter_value().integer_value
        )
        self._locked_marker_group: Optional[int] = None
        # Build multi-marker lookup (inner/outer embedded tag pairs)
        outer_ids = list(self.get_parameter("outer_marker_ids").get_parameter_value().integer_array_value)
        inner_ids = list(self.get_parameter("inner_marker_ids").get_parameter_value().integer_array_value)
        outer_size = self.get_parameter("outer_marker_size_m").get_parameter_value().double_value
        inner_size = self.get_parameter("inner_marker_size_m").get_parameter_value().double_value
        world_x = list(self.get_parameter("marker_world_x").get_parameter_value().double_array_value)
        world_y = list(self.get_parameter("marker_world_y").get_parameter_value().double_array_value)
        world_z = list(self.get_parameter("marker_world_z").get_parameter_value().double_array_value)
        world_yaw_arr = list(self.get_parameter("marker_world_yaw_arr").get_parameter_value().double_array_value)
        world_roll_arr = list(self.get_parameter("marker_world_roll_arr").get_parameter_value().double_array_value)
        world_pitch_arr = list(self.get_parameter("marker_world_pitch_arr").get_parameter_value().double_array_value)
        self.multi_marker_mode = bool(outer_ids and outer_ids[0] >= 0)
        self.marker_info: dict = {}  # id -> (size, T_w_m, group_idx, is_inner)
        if self.multi_marker_mode:
            n_groups = min(len(outer_ids), len(world_x))
            self._group_T_w_m = []
            self._group_rpy = []  # (roll, pitch, yaw) per group, for static TF debug
            for gi in range(n_groups):
                xyz = np.array([
                    world_x[gi],
                    world_y[gi],
                    world_z[gi] if gi < len(world_z) else 0.0,
                ])
                yaw = world_yaw_arr[gi] if gi < len(world_yaw_arr) else 0.0
                roll = world_roll_arr[gi] if gi < len(world_roll_arr) else 0.0
                pitch = world_pitch_arr[gi] if gi < len(world_pitch_arr) else 0.0
                # SDF / Gazebo convention: extrinsic XYZ. Wall-mounted ArUco markers
                # have roll=π/2 (face vertical); ignoring this puts the robot at marker height.
                T = tvec_R_to_T(xyz, rot_rpy(roll, pitch, yaw))
                self._group_T_w_m.append(T)
                self._group_rpy.append((roll, pitch, yaw))
                oid = outer_ids[gi]
                iid = inner_ids[gi] if gi < len(inner_ids) else -1
                if oid >= 0:
                    self.marker_info[oid] = (outer_size, T, gi, False)
                if iid >= 0:
                    self.marker_info[iid] = (inner_size, T, gi, True)
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
        # LifecyclePublisher: publish() is a no-op when the node is not in ACTIVE state,
        # so nav2_lifecycle_manager's deactivate stops fiducial output cleanly.
        self.pose_pub = self.create_lifecycle_publisher(PoseWithCovarianceStamped, "/fiducial_pose", 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        if not self.multi_marker_mode:
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
        self._startup_time_sec = time.time()
        self._prev_pose_xyz: Optional[np.ndarray] = None
        self._prev_pose_t_sec: Optional[float] = None
        self._prev_pose_yaw: Optional[float] = None
        self._prev_msg_stamp_sec: Optional[float] = None
        self._startup_candidate_xyz: Optional[np.ndarray] = None
        self._startup_candidate_yaw: Optional[float] = None
        self._startup_stable_count: int = 0

        self.get_logger().info(
            f"[fiducial] READY: {n} cameras. Topics: {self.image_topics[0]} ... "
            "(waiting for camera_info then looking for ArUco; will log when marker detected)"
        )

    def _debug_log(self, run_id: str, hypothesis_id: str, location: str, message: str, data: dict) -> None:
        # region agent log
        try:
            payload = {
                "sessionId": "b744e7",
                "runId": run_id,
                "hypothesisId": hypothesis_id,
                "location": location,
                "message": message,
                "data": data,
                "timestamp": int(time.time() * 1000),
            }
            line = json.dumps(payload, ensure_ascii=True) + "\n"
            wrote = False
            for log_path in (
                "/home/piotr/MQP/.cursor/debug-b744e7.log",
                "/ros2_ws/.cursor/debug-b744e7.log",
            ):
                try:
                    with open(log_path, "a", encoding="utf-8") as f:
                        f.write(line)
                    wrote = True
                    break
                except Exception:
                    pass
            if not wrote:
                req = urllib.request.Request(
                    "http://127.0.0.1:7675/ingest/58c2fc9f-05f3-4116-b66e-0ca0a500d1ce",
                    data=json.dumps(payload, ensure_ascii=True).encode("utf-8"),
                    headers={
                        "Content-Type": "application/json",
                        "X-Debug-Session-Id": "b744e7",
                    },
                    method="POST",
                )
                urllib.request.urlopen(req, timeout=0.2).read(1)
        except Exception:
            pass
        # endregion agent log

    def publish_static_marker(self):
        stamp = self.get_clock().now().to_msg()
        if self.multi_marker_mode:
            tfs = []
            for gi, (T_w_m, rpy) in enumerate(zip(self._group_T_w_m, self._group_rpy)):
                roll, pitch, yaw = rpy
                qx, qy, qz, qw = quat_from_euler(roll, pitch, yaw)
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = self.world_frame
                tf.child_frame_id = f"marker_{gi}"
                tf.transform.translation.x = float(T_w_m[0, 3])
                tf.transform.translation.y = float(T_w_m[1, 3])
                tf.transform.translation.z = float(T_w_m[2, 3])
                tf.transform.rotation.x = float(qx)
                tf.transform.rotation.y = float(qy)
                tf.transform.rotation.z = float(qz)
                tf.transform.rotation.w = float(qw)
                tfs.append(tf)
            self.static_tf_broadcaster.sendTransform(tfs)
        else:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.world_frame
            tf.child_frame_id = "marker"
            tf.transform.translation.x = float(self.world_to_marker_xyz[0])
            tf.transform.translation.y = float(self.world_to_marker_xyz[1])
            tf.transform.translation.z = float(self.world_to_marker_xyz[2])
            qw = math.cos(self.world_to_marker_yaw / 2)
            qz = math.sin(self.world_to_marker_yaw / 2)
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
        selected_gidx = -1
        selected_marker_id = -1
        selected_is_inner = False
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

        ids_flat = [int(x) for x in ids.flatten().tolist()]

        if self.multi_marker_mode:
            # Inner marker (small, precise) preferred over outer (large, far range) per tag group
            group_best: dict = {}
            for j, mid in enumerate(ids_flat):
                if mid not in self.marker_info:
                    continue
                size, T_w_m, gidx, is_inner = self.marker_info[mid]
                if gidx in group_best and not is_inner:
                    continue  # already have inner for this group
                group_best[gidx] = (j, size, T_w_m, is_inner)
            if not group_best:
                return
            selected_gidx = next(iter(group_best.keys()))
            if self.active_marker_group >= 0:
                if self.active_marker_group not in group_best:
                    # region agent log
                    self.get_logger().info(
                        f"[DBG_FID] skip frame: forced active_marker_group={self.active_marker_group} "
                        f"not present; groups_seen={sorted(group_best.keys())}"
                    )
                    # endregion agent log
                    return
                selected_gidx = self.active_marker_group
            else:
                if self._locked_marker_group is None:
                    self._locked_marker_group = selected_gidx
                    # region agent log
                    self.get_logger().info(
                        f"[DBG_FID] auto-locked marker group: {self._locked_marker_group}"
                    )
                    # endregion agent log
                if self._locked_marker_group in group_best:
                    selected_gidx = self._locked_marker_group
                else:
                    # region agent log
                    self.get_logger().info(
                        f"[DBG_FID] skip frame: locked_marker_group={self._locked_marker_group} "
                        f"not present; groups_seen={sorted(group_best.keys())}"
                    )
                    # endregion agent log
                    return
            use_idx, use_size, use_T_w_m, selected_is_inner = group_best[selected_gidx]
            selected_marker_id = ids_flat[use_idx]
            # region agent log
            self._debug_log(
                run_id="pre-fix",
                hypothesis_id="H1_H2",
                location="multi_camera_marker_localizer.py:image_cb:multi_select",
                message="Selected marker group for pose solve",
                data={
                    "camera_idx": camera_idx,
                    "camera_frame": self.camera_frames[camera_idx],
                    "ids_seen": ids_flat,
                    "selected_marker_id": selected_marker_id,
                    "selected_group_idx": selected_gidx,
                    "selected_is_inner": bool(selected_is_inner),
                    "selected_marker_world_xyz": [
                        float(use_T_w_m[0, 3]),
                        float(use_T_w_m[1, 3]),
                        float(use_T_w_m[2, 3]),
                    ],
                },
            )
            # endregion agent log
        else:
            use_idx = 0
            if self.marker_id >= 0:
                found = False
                for i, mid in enumerate(ids_flat):
                    if mid == self.marker_id:
                        use_idx = i
                        found = True
                        break
                if not found:
                    return
            use_size = self.marker_size
            use_T_w_m = self.T_w_m
            selected_marker_id = ids_flat[use_idx]

        s = use_size
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

        # PnP residual gate: bounds pose noise from corner-detection errors.
        proj_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
        reproj_err = float(
            np.mean(np.linalg.norm(proj_pts.reshape(-1, 2) - img_pts, axis=1))
        )
        if reproj_err > 2.0:
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
        d = float(np.linalg.norm(t_cm))
        # region agent log
        self._debug_log(
            run_id="pre-fix",
            hypothesis_id="H3",
            location="multi_camera_marker_localizer.py:image_cb:pnp",
            message="PnP output in camera frame",
            data={
                "camera_idx": camera_idx,
                "t_cm": [float(t_cm[0]), float(t_cm[1]), float(t_cm[2])],
                "marker_distance_m": d,
                "rvec": [float(rvec[0, 0]), float(rvec[1, 0]), float(rvec[2, 0])],
            },
        )
        # endregion agent log
        if self.max_marker_distance_m > 0.0 and d > self.max_marker_distance_m:
            # region agent log
            self._debug_log(
                run_id="post-fix",
                hypothesis_id="H7",
                location="multi_camera_marker_localizer.py:image_cb:distance_gate",
                message="Rejected marker beyond max range gate",
                data={
                    "camera_idx": camera_idx,
                    "marker_id": selected_marker_id,
                    "group_idx": selected_gidx,
                    "distance_m": d,
                    "max_marker_distance_m": float(self.max_marker_distance_m),
                },
            )
            # endregion agent log
            return

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
        # region agent log
        self._debug_log(
            run_id="pre-fix",
            hypothesis_id="H2_H4",
            location="multi_camera_marker_localizer.py:image_cb:tf_lookup",
            message="TF lookup used for camera-to-base chain",
            data={
                "camera_idx": camera_idx,
                "robot_frame": self.robot_frame,
                "camera_frame": self.camera_frames[camera_idx],
                "msg_stamp_sec": int(msg.header.stamp.sec),
                "msg_stamp_nanosec": int(msg.header.stamp.nanosec),
                "tf_stamp_sec": int(t.header.stamp.sec),
                "tf_stamp_nanosec": int(t.header.stamp.nanosec),
                "T_base_cam_xyz": [float(tx), float(ty), float(tz)],
                "T_base_cam_quat_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
            },
        )
        # endregion agent log

        # world -> robot: T_w_m @ T_m_c @ T_c_base
        T_w_r = use_T_w_m @ T_m_c @ T_cam_base
        # region agent log
        T_w_r_alt = use_T_w_m @ T_m_c @ T_base_cam
        self._debug_log(
            run_id="pre-fix",
            hypothesis_id="H6",
            location="multi_camera_marker_localizer.py:image_cb:chain_compare",
            message="Compare pose chain using inverted vs direct TF",
            data={
                "camera_idx": camera_idx,
                "pose_world_xyz_inverted_tf": [float(T_w_r[0, 3]), float(T_w_r[1, 3]), float(T_w_r[2, 3])],
                "pose_world_xyz_direct_tf": [float(T_w_r_alt[0, 3]), float(T_w_r_alt[1, 3]), float(T_w_r_alt[2, 3])],
            },
        )
        # endregion agent log

        # Distance to detected marker (camera frame). EKF down-weights long range.
        sigma_xy = self.position_stddev + self.range_scale_per_m * d
        sigma_yaw = self.yaw_stddev + self.range_scale_per_m * d / 4.0
        var_xy = sigma_xy * sigma_xy
        var_yaw = sigma_yaw * sigma_yaw

        p = PoseWithCovarianceStamped()
        p.header = Header()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = self.world_frame
        p.pose.pose.position.x = float(T_w_r[0, 3])
        p.pose.pose.position.y = float(T_w_r[1, 3])
        p.pose.pose.position.z = float(T_w_r[2, 3])
        qq = quat_from_R(T_w_r[:3, :3])
        p.pose.pose.orientation.x = float(qq[0])
        p.pose.pose.orientation.y = float(qq[1])
        p.pose.pose.orientation.z = float(qq[2])
        p.pose.pose.orientation.w = float(qq[3])
        # Diagonal: x, y, z, roll, pitch, yaw. Disabled axes get 1e6 so the EKF ignores them.
        cov = list(p.pose.covariance)
        cov[0] = var_xy
        cov[7] = var_xy
        cov[14] = 1e6
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = var_yaw
        p.pose.covariance = cov
        now_pub = time.time()
        pose_xyz = np.array(
            [p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z],
            dtype=float,
        )
        yaw_now = self._yaw_from_rot(T_w_r[:3, :3])
        startup_age = now_pub - self._startup_time_sec
        should_publish = True
        if startup_age < self.startup_stability_seconds:
            if self._startup_candidate_xyz is None or self._startup_candidate_yaw is None:
                self._startup_candidate_xyz = pose_xyz.copy()
                self._startup_candidate_yaw = yaw_now
                self._startup_stable_count = 1
            else:
                dxy = float(np.linalg.norm((pose_xyz - self._startup_candidate_xyz)[:2]))
                dyaw = float(math.atan2(
                    math.sin(yaw_now - self._startup_candidate_yaw),
                    math.cos(yaw_now - self._startup_candidate_yaw),
                ))
                if (
                    dxy <= self.startup_stable_xy_thresh_m
                    and abs(dyaw) <= self.startup_stable_yaw_thresh_rad
                ):
                    self._startup_stable_count += 1
                else:
                    self._startup_stable_count = 1
                    self._startup_candidate_xyz = pose_xyz.copy()
                    self._startup_candidate_yaw = yaw_now
                if self._startup_stable_count < self.startup_stable_required_frames:
                    should_publish = False
                    # region agent log
                    self._debug_log(
                        run_id="post-fix",
                        hypothesis_id="H12",
                        location="multi_camera_marker_localizer.py:image_cb:startup_gate",
                        message="Suppressed unstable startup fiducial pose",
                        data={
                            "camera_idx": camera_idx,
                            "startup_age_s": float(startup_age),
                            "stable_count": int(self._startup_stable_count),
                            "required_frames": int(self.startup_stable_required_frames),
                            "candidate_xyz": [
                                float(self._startup_candidate_xyz[0]),
                                float(self._startup_candidate_xyz[1]),
                                float(self._startup_candidate_xyz[2]),
                            ],
                            "pose_world_xyz": [float(pose_xyz[0]), float(pose_xyz[1]), float(pose_xyz[2])],
                        },
                    )
                    # endregion agent log
        # Physical-plausibility gate: rover cannot translate >0.6 m/s nor rotate
        # >0.6 rad/s, so a jump of that magnitude is a misdetection, not a move.
        if should_publish and self._prev_pose_xyz is not None and self._prev_pose_t_sec is not None:
            dt_chk = now_pub - self._prev_pose_t_sec
            if dt_chk > 0.0:
                speed_chk = float(
                    np.linalg.norm((pose_xyz - self._prev_pose_xyz)[:2]) / dt_chk
                )
                if speed_chk > 0.6:
                    should_publish = False
                elif self._prev_pose_yaw is not None:
                    dyaw_chk = float(math.atan2(
                        math.sin(yaw_now - self._prev_pose_yaw),
                        math.cos(yaw_now - self._prev_pose_yaw),
                    ))
                    if abs(dyaw_chk / dt_chk) > 0.6:
                        should_publish = False
        if should_publish:
            self.pose_pub.publish(p)
        msg_stamp_sec = float(msg.header.stamp.sec) + (float(msg.header.stamp.nanosec) * 1e-9)
        if self._prev_msg_stamp_sec is not None and msg_stamp_sec <= self._prev_msg_stamp_sec:
            # region agent log
            self._debug_log(
                run_id="post-fix",
                hypothesis_id="H11",
                location="multi_camera_marker_localizer.py:image_cb:stamp_order",
                message="Non-monotonic image timestamp seen in fiducial callback",
                data={
                    "camera_idx": camera_idx,
                    "msg_stamp_sec": msg_stamp_sec,
                    "prev_msg_stamp_sec": float(self._prev_msg_stamp_sec),
                },
            )
            # endregion agent log
        self._prev_msg_stamp_sec = msg_stamp_sec
        if self._prev_pose_xyz is not None and self._prev_pose_t_sec is not None:
            dt = now_pub - self._prev_pose_t_sec
            if dt > 0.0:
                delta = pose_xyz - self._prev_pose_xyz
                speed = float(np.linalg.norm(delta) / dt)
                if speed > 0.6:
                    # region agent log
                    self._debug_log(
                        run_id="post-fix",
                        hypothesis_id="H8",
                        location="multi_camera_marker_localizer.py:image_cb:pose_jump",
                        message="Large inter-frame fiducial pose jump",
                        data={
                            "camera_idx": camera_idx,
                            "dt_s": float(dt),
                            "delta_xyz": [float(delta[0]), float(delta[1]), float(delta[2])],
                            "speed_mps": speed,
                            "current_pose_xyz": [float(pose_xyz[0]), float(pose_xyz[1]), float(pose_xyz[2])],
                        },
                    )
                    # endregion agent log
                if self._prev_pose_yaw is not None:
                    dyaw = float(math.atan2(
                        math.sin(yaw_now - self._prev_pose_yaw),
                        math.cos(yaw_now - self._prev_pose_yaw),
                    ))
                    yaw_rate = abs(dyaw / dt)
                    if yaw_rate > 0.6:
                        # region agent log
                        self._debug_log(
                            run_id="post-fix",
                            hypothesis_id="H10",
                            location="multi_camera_marker_localizer.py:image_cb:yaw_jump",
                            message="Large inter-frame fiducial yaw jump",
                            data={
                                "camera_idx": camera_idx,
                                "dt_s": float(dt),
                                "dyaw_rad": dyaw,
                                "yaw_rate_rps": yaw_rate,
                                "yaw_now_rad": yaw_now,
                            },
                        )
                        # endregion agent log
        if should_publish:
            self._prev_pose_xyz = pose_xyz
            self._prev_pose_t_sec = now_pub
            self._prev_pose_yaw = yaw_now
        if now_pub - self._startup_time_sec < 8.0:
            # region agent log
            self._debug_log(
                run_id="post-fix",
                hypothesis_id="H9",
                location="multi_camera_marker_localizer.py:image_cb:startup_publish",
                message="Startup fiducial pose sample",
                data={
                    "camera_idx": camera_idx,
                    "marker_id": selected_marker_id,
                    "group_idx": selected_gidx,
                    "distance_m": d,
                    "pose_world_xyz": [float(pose_xyz[0]), float(pose_xyz[1]), float(pose_xyz[2])],
                    "pose_world_yaw_rad": yaw_now,
                    "world_frame": self.world_frame,
                },
            )
            # endregion agent log
        # region agent log
        self._debug_log(
            run_id="pre-fix",
            hypothesis_id="H1_H2_H5",
            location="multi_camera_marker_localizer.py:image_cb:pose_publish",
            message="Published fiducial pose candidate",
            data={
                "camera_idx": camera_idx,
                "pose_world_xyz": [float(p.pose.pose.position.x), float(p.pose.pose.position.y), float(p.pose.pose.position.z)],
                "sigma_xy": float(sigma_xy),
                "sigma_yaw": float(sigma_yaw),
                "cov_x": float(cov[0]),
                "cov_y": float(cov[7]),
                "cov_yaw": float(cov[35]),
            },
        )
        # endregion agent log

        if now - self.last_log_per_cam[camera_idx] > 1.0:
            self.get_logger().info(
                f"pose from camera {camera_idx} ({self.camera_frames[camera_idx]}): "
                f"xyz=({p.pose.pose.position.x:.3f}, {p.pose.pose.position.y:.3f}, "
                f"{p.pose.pose.position.z:.3f}) σxy={sigma_xy:.3f} d={d:.2f}m"
            )
            # region agent log
            self.get_logger().info(
                "[DBG_FID] "
                f"cam={camera_idx} marker_id={selected_marker_id} group={selected_gidx} "
                f"is_inner={int(bool(selected_is_inner))} "
                f"marker_w=({use_T_w_m[0,3]:.3f},{use_T_w_m[1,3]:.3f},{use_T_w_m[2,3]:.3f}) "
                f"t_cm=({t_cm[0]:.3f},{t_cm[1]:.3f},{t_cm[2]:.3f}) "
                f"t_base_cam=({tx:.3f},{ty:.3f},{tz:.3f}) "
                f"pose_w=({p.pose.pose.position.x:.3f},{p.pose.pose.position.y:.3f},{p.pose.pose.position.z:.3f})"
            )
            # endregion agent log
            self.last_log_per_cam[camera_idx] = now

    # ---------- Lifecycle transitions ----------
    # Construction is done in __init__; transitions just gate publishing
    # so nav2_lifecycle_manager can report active/inactive cleanly.
    def on_configure(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

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

    @staticmethod
    def _yaw_from_rot(R: np.ndarray) -> float:
        return float(math.atan2(R[1, 0], R[0, 0]))


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
