#!/usr/bin/env python3
"""
Zone Publisher: subscribes to odom (x,y in meters), maps position to a named zone,
and publishes the zone string to /current_zone. Zone bounds are rectangular.

Odom topic and zone bounds are configurable via ROS parameters so the same node
works with controller odom, RTAB-Map visual odom, or different arenas (UCF, Artemis).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# Default zone bounds in meters (x_min, x_max, y_min, y_max). Override via parameters.
DEFAULT_ZONE_BOUNDS = {
    'starting_zone': [0.0, 2.0, 0.0, 2.0],
    'excavation_zone': [0.0, 2.5, 0.0, 11.0],
    'obstacle_zone_main': [4.38, 6.38, 0.0, 11.0],
    'obstacle_zone_exclude': [4.38, 6.88, 0.0, 1.5],
    'construction_zone': [7.0, 12.0, 0.0, 11.0],
}


def point_in_rectangle(x, y, x_min, x_max, y_min, y_max):
    """True if (x, y) is inside the axis-aligned rectangle."""
    return x_min <= x <= x_max and y_min <= y <= y_max


def get_zone_name(x, y, zone_config, origin_tolerance):
    """Return zone string for (x, y). Priority: origin → starting → excavation → obstacle → construction → outside bounds."""
    if abs(x) <= origin_tolerance and abs(y) <= origin_tolerance:
        return 'starting zone'
    s = zone_config['starting_zone']
    if point_in_rectangle(x, y, s[0], s[1], s[2], s[3]):
        return 'starting zone'
    e = zone_config['excavation_zone']
    if point_in_rectangle(x, y, e[0], e[1], e[2], e[3]):
        return 'excavation zone'
    om = zone_config['obstacle_zone_main']
    oe = zone_config['obstacle_zone_exclude']
    if point_in_rectangle(x, y, om[0], om[1], om[2], om[3]) and not point_in_rectangle(
        x, y, oe[0], oe[1], oe[2], oe[3]
    ):
        return 'obstacle zone'
    c = zone_config['construction_zone']
    if point_in_rectangle(x, y, c[0], c[1], c[2], c[3]):
        return 'construction zone'
    return 'outside bounds'


def _parse_bounds(param_value, default):
    """Parse parameter to list of 4 floats. Accepts list or string 'x_min,x_max,y_min,y_max'."""
    if param_value is None:
        return default
    if isinstance(param_value, (list, tuple)) and len(param_value) >= 4:
        return [float(param_value[0]), float(param_value[1]), float(param_value[2]), float(param_value[3])]
    if isinstance(param_value, str):
        parts = [p.strip() for p in param_value.split(',')]
        if len(parts) >= 4:
            return [float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])]
    return default


class ZonePublisherNode(Node):
    """Subscribes to odom; publishes current zone to /current_zone on every message."""

    def __init__(self):
        super().__init__('zone_publisher')

        # Odom topic: use /odom by default so it works with controller or RTAB-Map visual odom (remap as needed).
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('origin_tolerance_m', 0.01)
        for key in DEFAULT_ZONE_BOUNDS:
            self.declare_parameter(f'zone.{key}', DEFAULT_ZONE_BOUNDS[key])

        odom_topic = self.get_parameter('odom_topic').value
        if isinstance(odom_topic, str):
            odom_topic = odom_topic.strip() or '/odom'
        else:
            odom_topic = '/odom'

        self._zone_config = {}
        for key in DEFAULT_ZONE_BOUNDS:
            default = DEFAULT_ZONE_BOUNDS[key]
            val = self.get_parameter(f'zone.{key}').value
            self._zone_config[key] = _parse_bounds(val, default)

        self._origin_tolerance = float(self.get_parameter('origin_tolerance_m').value)

        self._zone_pub = self.create_publisher(String, 'current_zone', 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos
        )
        self._last_zone = None
        self.get_logger().info(
            f'zone_publisher: odom_topic={odom_topic}, zones from params (edit YAML or launch to override)'
        )

    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        zone = get_zone_name(x, y, self._zone_config, self._origin_tolerance)
        if zone != self._last_zone:
            self.get_logger().info(f"zone: ({x:.2f}, {y:.2f}) -> \"{zone}\"")
            self._last_zone = zone
        out = String()
        out.data = zone
        self._zone_pub.publish(out)


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
