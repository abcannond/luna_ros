#!/usr/bin/env python3
"""Publish arena grid cell label (G/E/border) and optional RViz center markers."""

import os
from typing import Any, Dict, Set

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from luna_nav.arena_grid import (
    ArenaGrid,
    cell_center,
    get_coarse_zone_name,
    load_arena_config,
    zone_codes_at_point,
)


def _rgba(r: float, g: float, b: float, a: float = 0.9) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


def _colors_for_codes(codes: Set[str]) -> ColorRGBA:
    if len(codes) >= 2:
        return _rgba(1.0, 0.55, 0.1, 0.95)
    if codes == {"s"}:
        return _rgba(0.2, 0.8, 0.3, 0.9)
    if codes == {"e"}:
        return _rgba(0.85, 0.75, 0.2, 0.9)
    if codes == {"o"}:
        return _rgba(0.9, 0.25, 0.25, 0.9)
    if codes == {"c"}:
        return _rgba(0.35, 0.45, 0.95, 0.9)
    return _rgba(0.5, 0.5, 0.5, 0.7)


class ArenaGridPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("arena_grid_publisher")
        default_yaml = os.path.join(
            get_package_share_directory("luna_nav"), "config", "arena_zones.yaml"
        )
        self.declare_parameter("arena_zones_file", default_yaml)
        self.declare_parameter("odom_topic", "/luna_cont/odom")
        self.declare_parameter("cell_topic", "arena_grid_cell")
        self.declare_parameter("marker_topic", "arena_grid_markers")
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_rate_hz", 1.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("publish_always", True)
        self.declare_parameter("outside_arena_label", "outside_arena")
        self.declare_parameter("outside_named_label", "outside")
        self.declare_parameter("marker_scale", 0.12)

        ypath = str(self.get_parameter("arena_zones_file").value)
        ypath = ypath if ypath else "/nonexistent"
        self._data: Dict[str, Any] = load_arena_config(ypath)
        self._grid = ArenaGrid.from_config(self._data)
        self._origin_tol = float(self._data.get("origin_tolerance_m", 0.01))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        odom_topic = str(self.get_parameter("odom_topic").value)
        self._sub = self.create_subscription(Odometry, odom_topic, self._on_odom, qos)

        cell_topic = str(self.get_parameter("cell_topic").value)
        self._cell_pub = self.create_publisher(String, cell_topic, 10)

        self._marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self._publish_markers = bool(self.get_parameter("publish_markers").value)
        mhz = float(self.get_parameter("marker_rate_hz").value)
        period = 1.0 / max(0.1, mhz)
        if self._publish_markers:
            self._marker_timer = self.create_timer(period, self._publish_marker_array)
        else:
            self._marker_timer = None

        self._last_label: str | None = None
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publish_always = bool(self.get_parameter("publish_always").value)
        self._outside_arena = str(self.get_parameter("outside_arena_label").value)
        self._outside_named = str(self.get_parameter("outside_named_label").value)

        self.get_logger().info(
            f"Arena grid {self._grid.ncols}x{self._grid.nrows} cell_size={self._grid.cell_size_m} m, "
            f"odom={odom_topic} -> '{cell_topic}'"
        )

    def _label_for_pose(self, x: float, y: float) -> str:
        from luna_nav.arena_grid import label_for_pose

        return label_for_pose(
            x,
            y,
            self._data,
            self._grid,
            outside_arena_label=self._outside_arena,
            outside_named_label=self._outside_named,
        )

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        label = self._label_for_pose(x, y)
        changed = label != self._last_label
        if changed:
            coarse = get_coarse_zone_name(x, y, self._data)
            self.get_logger().info(
                f"grid cell ({x:.2f}, {y:.2f}) coarse='{coarse}' -> \"{label}\""
            )
            self._last_label = label
        if self._publish_always or changed:
            out = String()
            out.data = label
            self._cell_pub.publish(out)

    def _publish_marker_array(self) -> None:
        arena = self._grid.arena
        s = self._grid.cell_size_m
        arr = MarkerArray()
        scale = float(self.get_parameter("marker_scale").value)

        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self._frame_id
        clear.ns = "arena_grid_cells"
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        mid = 1
        for ri in range(self._grid.nrows):
            for ci in range(self._grid.ncols):
                center = cell_center(ci, ri, arena, s)
                if center is None:
                    continue
                cx, cy = center
                codes = zone_codes_at_point(cx, cy, self._data, self._origin_tol)
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = self._frame_id
                m.ns = "arena_grid_cells"
                m.id = mid
                mid += 1
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = cx
                m.pose.position.y = cy
                m.pose.position.z = 0.05
                m.pose.orientation.w = 1.0
                m.scale.x = scale
                m.scale.y = scale
                m.scale.z = scale
                m.color = _colors_for_codes(codes)
                m.lifetime.sec = 0
                arr.markers.append(m)

        self._marker_pub.publish(arr)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = ArenaGridPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
