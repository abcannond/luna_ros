#!/usr/bin/env python3
"""
Subscribes to /fiducial_active_cameras (Int32MultiArray). Any camera that sees
a fiducial is active. Subscribes to all 4 image and camera_info topics; republishes
only active cameras to /fiducial_cameras/active_<i>/image and .../camera_info
(i in 0..3). Reduces display bandwidth by only forwarding cameras that see a marker.
"""
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray


class FiducialCameraRepublisher(Node):
    def __init__(self):
        super().__init__("fiducial_camera_republisher")

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

        image_topics = self.get_parameter("image_topics").get_parameter_value().string_array_value
        camera_info_topics = self.get_parameter("camera_info_topics").get_parameter_value().string_array_value
        n = min(len(image_topics), len(camera_info_topics))
        self.image_topics = list(image_topics)[:n]
        self.camera_info_topics = list(camera_info_topics)[:n]

        self._active_set = set()
        self._latest_image = [None] * n
        self._latest_camera_info = [None] * n

        self._image_pubs = []
        self._camera_info_pubs = []
        for i in range(n):
            self._image_pubs.append(
                self.create_publisher(Image, f"/fiducial_cameras/active_{i}/image", 10)
            )
            self._camera_info_pubs.append(
                self.create_publisher(
                    CameraInfo, f"/fiducial_cameras/active_{i}/camera_info", 10
                )
            )

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
        for i in range(n):
            self.create_subscription(
                Image,
                self.image_topics[i],
                partial(self._image_cb, i),
                qos_image,
            )
            self.create_subscription(
                CameraInfo,
                self.camera_info_topics[i],
                partial(self._camera_info_cb, i),
                qos_camera_info,
            )
        self.create_subscription(
            Int32MultiArray,
            "/fiducial_active_cameras",
            self._active_cb,
            QoSProfile(depth=10),
        )

        self.get_logger().info(
            "fiducial_camera_republisher: READY (republishes any camera that sees "
            "a marker to /fiducial_cameras/active_<0..3>/image)"
        )

    def _active_cb(self, msg: Int32MultiArray):
        self._active_set = set(int(x) for x in msg.data if 0 <= x < len(self.image_topics))
        for idx in self._active_set:
            if self._latest_image[idx] is not None:
                self._image_pubs[idx].publish(self._latest_image[idx])
            if self._latest_camera_info[idx] is not None:
                self._camera_info_pubs[idx].publish(self._latest_camera_info[idx])

    def _image_cb(self, idx: int, msg: Image):
        self._latest_image[idx] = msg
        if idx in self._active_set:
            self._image_pubs[idx].publish(msg)

    def _camera_info_cb(self, idx: int, msg: CameraInfo):
        self._latest_camera_info[idx] = msg
        if idx in self._active_set:
            self._camera_info_pubs[idx].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FiducialCameraRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
