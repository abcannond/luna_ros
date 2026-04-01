#!/usr/bin/env python3
"""
Zone Publisher: subscribes to odom (x,y in meters), maps position to a named zone,
and publishes the zone string to /current_zone. Zone bounds come from
config/arena_zones.yaml (shared with arena_grid_publisher).
"""

import os
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from luna_nav.arena_grid import get_coarse_zone_name, load_arena_config


class ZonePublisherNode(Node):
    """Subscribes to odom; publishes current zone to /current_zone on every message."""

    def __init__(self) -> None:
        super().__init__("zone_publisher")

        default_yaml = os.path.join(
            get_package_share_directory("luna_nav"), "config", "arena_zones.yaml"
        )
        self.declare_parameter("arena_zones_file", default_yaml)
        self.declare_parameter("odom_topic", "/luna_cont/odom")

        ypath = str(self.get_parameter("arena_zones_file").value)
        self._data: Dict[str, Any] = load_arena_config(ypath)

        self._zone_pub = self.create_publisher(String, "current_zone", 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        odom_topic = str(self.get_parameter("odom_topic").value)
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos
        )
        self._last_zone: str | None = None

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        zone = get_coarse_zone_name(x, y, self._data)
        if zone != self._last_zone:
            self.get_logger().info(f"zone: ({x:.2f}, {y:.2f}) -> \"{zone}\"")
            self._last_zone = zone
        out = String()
        out.data = zone
        self._zone_pub.publish(out)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = ZonePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
