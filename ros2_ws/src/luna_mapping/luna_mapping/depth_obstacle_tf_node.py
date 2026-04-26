#!/usr/bin/env python3
"""
Transform /luna/depth_round_obstacle camera-frame hits into map frame via TF.

ROS 2 counterpart to 23–24 depth_imaging_package/obstacle_localizer.py, without
hardcoded RealSense extrinsics (uses tf2 + CameraInfo frame_id).
"""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class DepthObstacleTfNode(Node):
    def __init__(self):
        super().__init__('depth_obstacle_tf_node')

        self.declare_parameter('obstacle_array_topic', '/luna/depth_round_obstacle')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('output_point_topic', '/luna/depth_round_obstacle/map/point')
        self.declare_parameter('publish_legacy_xyr', True)
        self.declare_parameter('legacy_topic', '/luna/map_obstacle_xyr')

        self._map_frame = self.get_parameter('map_frame').value
        self._obstacle_frame = ''
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._pub_point = self.create_publisher(
            PointStamped, self.get_parameter('output_point_topic').value, 10
        )
        self._pub_legacy = None
        if bool(self.get_parameter('publish_legacy_xyr').value):
            self._pub_legacy = self.create_publisher(
                Float32MultiArray, self.get_parameter('legacy_topic').value, 10
            )

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._info_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter('obstacle_array_topic').value,
            self._obstacle_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'depth_obstacle_tf_node: {self.get_parameter("obstacle_array_topic").value} '
            f'+ camera_info -> {self._map_frame}'
        )

    def _info_cb(self, msg: CameraInfo) -> None:
        if msg.header.frame_id:
            self._obstacle_frame = msg.header.frame_id

    def _lookup_to_map(self) -> tuple:
        if not self._obstacle_frame:
            return None, None
        try:
            t = self.tf_buffer.lookup_transform(
                self._map_frame,
                self._obstacle_frame,
                Time(),
                Duration(seconds=0.2),
            )
            return t, None
        except TransformException:
            pass
        try:
            t = self.tf_buffer.lookup_transform(
                self._map_frame,
                self._obstacle_frame,
                Time(),
                Duration(seconds=0.05),
            )
            return t, None
        except TransformException as ex:
            return None, ex

    def _obstacle_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            return
        if not self._obstacle_frame:
            return

        t, err = self._lookup_to_map()
        if t is None:
            if err is not None:
                self.get_logger().warn_throttle(
                    5.0, f'TF {self._map_frame}<-{self._obstacle_frame}: {err}'
                )
            return

        ps = PointStamped()
        ps.header.frame_id = self._obstacle_frame
        ps.header.stamp = t.header.stamp
        ps.point.x = float(msg.data[0])
        ps.point.y = float(msg.data[1])
        ps.point.z = float(msg.data[2])
        rad_m = float(msg.data[3])

        try:
            out = do_transform_point(ps, t)
        except Exception as ex:
            self.get_logger().warn_throttle(5.0, f'do_transform_point: {ex}')
            return

        self._pub_point.publish(out)

        if self._pub_legacy is not None:
            legacy = Float32MultiArray()
            legacy.data = [float(out.point.x), float(out.point.y), rad_m]
            self._pub_legacy.publish(legacy)


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
