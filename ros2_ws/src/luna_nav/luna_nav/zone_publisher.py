#!/usr/bin/env python3
"""
Zone Publisher: subscribes to odom (x,y in meters), maps position to a named zone,
and publishes the zone string to /current_zone. Zone bounds are rectangular; edit
ZONE_CONFIG to match your arena.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# -----------------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------------

ODOM_TOPIC = '/luna_cont/odom'

# Zone rectangles in meters: (x_min, x_max, y_min, y_max). Checked in order; first match wins.
ZONE_CONFIG = {
    'starting_zone': (0.0, 2.0, 0.0, 2.0),
    'excavation_zone': (0.0, 2.5, 0.0, 11.0),
    'obstacle_zone_main': (4.38, 6.38, 0.0, 11.0),
    'obstacle_zone_exclude': (4.38, 6.88, 0.0, 1.5),  # hole in obstacle zone
    'construction_zone': (7.0, 12.0, 0.0, 11.0),
}

# (0,0) and points within this distance count as starting zone (avoids "outside bounds" at origin).
ORIGIN_TOLERANCE_M = 0.01


# -----------------------------------------------------------------------------
# Zone logic
# -----------------------------------------------------------------------------

def point_in_rectangle(x, y, x_min, x_max, y_min, y_max):
    """True if (x, y) is inside the axis-aligned rectangle."""
    return x_min <= x <= x_max and y_min <= y <= y_max


def get_zone_name(x, y):
    """Return zone string for (x, y) in meters. Priority: origin → starting → excavation → obstacle → construction → outside bounds."""
    if abs(x) <= ORIGIN_TOLERANCE_M and abs(y) <= ORIGIN_TOLERANCE_M:
        return 'starting zone'
    if point_in_rectangle(x, y, *ZONE_CONFIG['starting_zone']):
        return 'starting zone'
    if point_in_rectangle(x, y, *ZONE_CONFIG['excavation_zone']):
        return 'excavation zone'
    if point_in_rectangle(x, y, *ZONE_CONFIG['obstacle_zone_main']) and not point_in_rectangle(
        x, y, *ZONE_CONFIG['obstacle_zone_exclude']
    ):
        return 'obstacle zone'
    if point_in_rectangle(x, y, *ZONE_CONFIG['construction_zone']):
        return 'construction zone'
    return 'outside bounds'


# -----------------------------------------------------------------------------
# Node
# -----------------------------------------------------------------------------

class ZonePublisherNode(Node):
    """Subscribes to odom; publishes current zone to /current_zone on every message."""

    def __init__(self):
        super().__init__('zone_publisher')
        self._zone_pub = self.create_publisher(String, 'current_zone', 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self._on_odom, qos
        )
        self._last_zone = None

    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        zone = get_zone_name(x, y)
        if zone != self._last_zone:
            self.get_logger().info(f"zone: ({x:.2f}, {y:.2f}) -> \"{zone}\"")
            self._last_zone = zone
        out = String()
        out.data = zone
        self._zone_pub.publish(out)


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ZonePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
