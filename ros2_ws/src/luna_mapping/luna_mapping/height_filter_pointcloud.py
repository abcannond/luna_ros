#!/usr/bin/env python3
"""
Height-based ground filter for PointCloud2.

Subscribes to a point cloud, transforms it to base_link, and removes points
below min_height (and optionally above max_height). Publishes the filtered cloud
in base_link. This avoids the blind band of image-row cropping and keeps
low obstacles (rocks, crates) that sit above the ground plane.

Parameters:
  - input_topic: PointCloud2 topic (e.g. /camera/camera/depth/color/points)
  - output_topic: Filtered PointCloud2 (e.g. .../points_ground_filtered)
  - target_frame: Frame for height filter (default base_link)
  - min_height: Keep points with z >= this in target_frame (default 0.02 m)
  - max_height: Keep points with z <= this (default 2.0 m)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import numpy as np

# PointCloud2 field types
DATATYPE_FLOAT32 = 7


def transform_to_matrix(trans) -> np.ndarray:
    """Build 4x4 homogeneous transform from geometry_msgs/Transform."""
    q = trans.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    tx = trans.transform.translation.x
    ty = trans.transform.translation.y
    tz = trans.transform.translation.z
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float32)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def read_xyz_fast(msg: PointCloud2) -> np.ndarray:
    """Read x,y,z assuming point_step and fixed offsets. Returns (N,3)."""
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    ox = oy = oz = 0
    for f in msg.fields:
        if f.name == 'x':
            ox = f.offset
        elif f.name == 'y':
            oy = f.offset
        elif f.name == 'z':
            oz = f.offset
    # All points are float32 x,y,z (possibly with padding); read as structured array
    dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
    # Actually we can't assume contiguous x,y,z. So iterate by point.
    out = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        out[i, 0] = np.frombuffer(data[base + ox : base + ox + 4], dtype=np.float32)[0]
        out[i, 1] = np.frombuffer(data[base + oy : base + oy + 4], dtype=np.float32)[0]
        out[i, 2] = np.frombuffer(data[base + oz : base + oz + 4], dtype=np.float32)[0]
    return out


def create_cloud_xyz(header: Header, points_xyz: np.ndarray) -> PointCloud2:
    """Create a PointCloud2 with only x,y,z float32 fields."""
    n = points_xyz.shape[0]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x', offset=0, datatype=DATATYPE_FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=DATATYPE_FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=DATATYPE_FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.data = points_xyz.astype(np.float32).tobytes()
    return msg


class HeightFilterPointcloud(Node):
    def __init__(self):
        super().__init__('height_filter_pointcloud')

        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('input_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('output_topic', '/camera/camera/depth/color/points_ground_filtered')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('min_height', 0.02)
        self.declare_parameter('max_height', 2.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cb,
            qos_profile_sensor_data,
        )

        # Heartbeat: publish empty cloud every 1 s so costmap observation buffer gets updates
        # even when input is slow or TF temporarily unavailable
        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat_cb)

        self.get_logger().info(
            f'HeightFilterPointcloud: {self.input_topic} -> {self.output_topic} '
            f'(frame={self.target_frame}, z in [{self.min_height}, {self.max_height}])'
        )

    def cb(self, msg: PointCloud2):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.15),
            )
        except TransformException:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.05),
                )
            except TransformException as ex:
                self.get_logger().warn_throttle(2.0, f'TF lookup failed: {ex}')
                # Still publish empty cloud so costmap sees an update
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = self.target_frame
                self.pub.publish(create_cloud_xyz(header, np.zeros((0, 3), dtype=np.float32)))
                return

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.target_frame

        T = transform_to_matrix(trans)
        xyz = read_xyz_fast(msg)
        if xyz.shape[0] == 0:
            self.pub.publish(create_cloud_xyz(header, np.zeros((0, 3), dtype=np.float32)))
            return
        # Drop NaN/inf (invalid depth)
        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]
        if xyz.shape[0] == 0:
            self.pub.publish(create_cloud_xyz(header, np.zeros((0, 3), dtype=np.float32)))
            return

        # Homogeneous transform to target frame
        ones = np.ones((xyz.shape[0], 1), dtype=np.float32)
        p = np.hstack([xyz, ones])
        p_tf = (T @ p.T).T
        z = p_tf[:, 2]

        mask = (z >= self.min_height) & (z <= self.max_height)
        if not np.any(mask):
            self.pub.publish(create_cloud_xyz(header, np.zeros((0, 3), dtype=np.float32)))
            return

        out_xyz = p_tf[mask, :3]
        out_msg = create_cloud_xyz(header, out_xyz)
        self.pub.publish(out_msg)

    def _heartbeat_cb(self):
        """Publish empty cloud so costmap always receives updates within expected_update_rate."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        self.pub.publish(create_cloud_xyz(header, np.zeros((0, 3), dtype=np.float32)))


def main(args=None):
    rclpy.init(args=args)
    node = HeightFilterPointcloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
