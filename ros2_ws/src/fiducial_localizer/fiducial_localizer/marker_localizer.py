#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import time


def rotz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=float)


def quat_from_euler(roll, pitch, yaw):
    cr = math.cos(roll/2); sr = math.sin(roll/2)
    cp = math.cos(pitch/2); sp = math.sin(pitch/2)
    cy = math.cos(yaw/2); sy = math.sin(yaw/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx, qy, qz, qw)


def quat_from_R(R):
    # R -> 4x4 -> quat
    M = np.eye(4)
    M[:3, :3] = R
    m00, m01, m02 = M[0, 0], M[0, 1], M[0, 2]
    m10, m11, m12 = M[1, 0], M[1, 1], M[1, 2]
    m20, m21, m22 = M[2, 0], M[2, 1], M[2, 2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    return (qx, qy, qz, qw)


def tvec_R_to_T(t, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def invert_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


class MarkerLocalizer(Node):
    def __init__(self):
        super().__init__('marker_localizer')

        # ---------- params ----------
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', -1)  # -1 = take first one
        self.declare_parameter('marker_size_m', 0.16)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('robot_frame', 'robot')
        self.declare_parameter('world_to_marker_xyz', [0.0, 0.63, 0.0])
        self.declare_parameter('world_to_marker_yaw', 0.0)
        self.declare_parameter('camera_to_robot_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('camera_to_robot_yaw', 0.0)
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size_m').get_parameter_value().double_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.world_to_marker_xyz = np.array(self.get_parameter('world_to_marker_xyz').get_parameter_value().double_array_value)
        self.world_to_marker_yaw = self.get_parameter('world_to_marker_yaw').get_parameter_value().double_value
        self.camera_to_robot_xyz = np.array(self.get_parameter('camera_to_robot_xyz').get_parameter_value().double_array_value)
        self.camera_to_robot_yaw = self.get_parameter('camera_to_robot_yaw').get_parameter_value().double_value
        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value

        # aruco
        dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        if not hasattr(cv2, 'aruco'):
            self.get_logger().error("OpenCV was built without aruco. Cannot localize.")
        if not hasattr(cv2.aruco, dict_name):
            self.get_logger().warn(f"Bad aruco_dict {dict_name}, falling back to DICT_4X4_50")
            dict_name = 'DICT_4X4_50'
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.detect = lambda img: self.detector.detectMarkers(img)
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detect = lambda img: cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)

        # pubs / tfs
        self.pose_pub = self.create_publisher(PoseStamped, '/jetson/localizer_robot_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.bridge = CvBridge()

        # camera model built from params / camera_info
        self.have_cam_info = False
        self.last_log = 0.0

        # subs (NOTE: image = sensor-data QoS to match v4l2_camera)
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)

        # precompute world->marker, camera->robot
        R_wm = rotz(self.world_to_marker_yaw)
        self.T_w_m = tvec_R_to_T(self.world_to_marker_xyz, R_wm)
        R_cr = rotz(self.camera_to_robot_yaw)
        self.T_c_r = tvec_R_to_T(self.camera_to_robot_xyz, R_cr)

        # publish static world->marker once
        self.publish_static_marker()

        self.get_logger().info("marker_localizer: READY")

    def publish_static_marker(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.world_frame
        tf.child_frame_id = 'marker'
        tf.transform.translation.x = float(self.world_to_marker_xyz[0])
        tf.transform.translation.y = float(self.world_to_marker_xyz[1])
        tf.transform.translation.z = float(self.world_to_marker_xyz[2])
        q = quat_from_euler(0.0, 0.0, self.world_to_marker_yaw)
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.static_tf_broadcaster.sendTransform(tf)

    def camera_info_cb(self, msg: CameraInfo):
        # v4l2_camera is publishing good intrinsics already, your output showed that
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.have_cam_info = True
        self.get_logger().info(f"[caminfo] fx={self.fx} fy={self.fy} cx={self.cx} cy={self.cy}")

    def image_cb(self, msg: Image):
        now = time.time()
        if not self.have_cam_info and (self.fx <= 0 or self.fy <= 0):
            if now - self.last_log > 2.0:
                self.get_logger().warn("waiting for camera info / intrinsics...")
                self.last_log = now
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            if now - self.last_log > 2.0:
                self.get_logger().warn(f"cv_bridge fail: {e}")
                self.last_log = now
            return

        corners, ids, _ = self.detect(cv_img)
        if ids is None or len(ids) == 0:
            if now - self.last_log > 2.0:
                self.get_logger().info("no markers in image")
                self.last_log = now
            return

        ids_list = ids.flatten().tolist()
        self.get_logger().info(f"detected ids: {ids_list}")

        # pick wanted marker or first
        use_idx = 0
        if self.marker_id >= 0:
            # look for it
            found = False
            for i, mid in enumerate(ids_list):
                if int(mid) == int(self.marker_id):
                    use_idx = i
                    found = True
                    break
            if not found:
                if now - self.last_log > 2.0:
                    self.get_logger().info(f"marker {self.marker_id} not in frame")
                    self.last_log = now
                return

        s = self.marker_size
        obj_pts = np.array([
            [-s/2,  s/2, 0.0],
            [ s/2,  s/2, 0.0],
            [ s/2, -s/2, 0.0],
            [-s/2, -s/2, 0.0]
        ], dtype=np.float32)

        img_pts = corners[use_idx].reshape(-1, 2).astype(np.float32)

        K = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]], dtype=np.float64)
        dist = np.zeros((5, 1), dtype=np.float64)

        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            if now - self.last_log > 2.0:
                self.get_logger().warn("solvePnP failed")
                self.last_log = now
            return

        R_cm, _ = cv2.Rodrigues(rvec)
        t_cm = tvec.reshape(3)
        T_c_m = tvec_R_to_T(t_cm, R_cm)
        T_m_c = invert_T(T_c_m)

        T_w_r = self.T_w_m @ T_m_c @ self.T_c_r

        # publish pose
        p = PoseStamped()
        p.header = Header()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = self.world_frame
        p.pose.position.x = float(T_w_r[0, 3])
        p.pose.position.y = float(T_w_r[1, 3])
        p.pose.position.z = float(T_w_r[2, 3])
        q = quat_from_R(T_w_r[:3, :3])
        p.pose.orientation.x = float(q[0])
        p.pose.orientation.y = float(q[1])
        p.pose.orientation.z = float(q[2])
        p.pose.orientation.w = float(q[3])
        self.pose_pub.publish(p)

        # broadcast TF
        tf = TransformStamped()
        tf.header.stamp = p.header.stamp
        tf.header.frame_id = self.world_frame
        tf.child_frame_id = self.robot_frame
        tf.transform.translation.x = p.pose.position.x
        tf.transform.translation.y = p.pose.position.y
        tf.transform.translation.z = p.pose.position.z
        tf.transform.rotation.x = p.pose.orientation.x
        tf.transform.rotation.y = p.pose.orientation.y
        tf.transform.rotation.z = p.pose.orientation.z
        tf.transform.rotation.w = p.pose.orientation.w
        self.tf_broadcaster.sendTransform(tf)

        self.get_logger().info(
            f"PUBLISHED pose: xyz=({p.pose.position.x:.3f}, {p.pose.position.y:.3f}, {p.pose.position.z:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
