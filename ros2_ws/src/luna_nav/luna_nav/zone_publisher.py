#!/usr/bin/env python3
"""
Zone Publisher: classifies robot position into named arena zones.

Loads zone rectangles from a YAML file, looks up the robot pose via TF
(map -> base_link), and publishes the current zone name plus RViz
MarkerArray overlays for each zone boundary.

Parameters:
  zones_file    -- path to YAML with zone definitions (see config/zones/)
  rate_hz       -- TF lookup / publish rate (default 5.0)
  hysteresis_n  -- consecutive samples in a new zone before switching (default 3)
"""

import json

import rclpy
import yaml
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as DurationMsg

import tf2_ros


def _point_in_rect(x, y, bounds):
    return bounds[0] <= x <= bounds[1] and bounds[2] <= y <= bounds[3]


class ZonePublisherNode(Node):

    def __init__(self):
        super().__init__('zone_publisher')

        self.declare_parameter('zones_file', '')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('hysteresis_n', 3)

        zones_file = self.get_parameter('zones_file').value
        rate = float(self.get_parameter('rate_hz').value or 5.0)
        self._hysteresis_n = int(self.get_parameter('hysteresis_n').value or 3)

        self._zones = {}
        self._zone_colors = {}
        self._frame_id = 'map'
        self._waypoints = {}

        if zones_file:
            self._load_yaml(zones_file)
        else:
            self.get_logger().warn(
                'No zones_file parameter set; using empty zone config. '
                'Set zones_file:=<path> to load arena zones.'
            )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._zone_pub = self.create_publisher(String, '/current_zone', 10)
        self._dashboard_pub = self.create_publisher(String, '/zone_dashboard', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/zone_markers', 10)

        self._current_zone = 'unknown'
        self._candidate_zone = 'unknown'
        self._candidate_count = 0

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self._marker_timer = self.create_timer(2.0, self._publish_markers)

        self.get_logger().info(
            f'zone_publisher: zones_file={zones_file}, '
            f'rate={rate} Hz, hysteresis={self._hysteresis_n}'
        )

    def _load_yaml(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load zones YAML: {e}')
            return

        self._frame_id = data.get('frame_id', 'map')
        zones_data = data.get('zones', {})
        for name, info in zones_data.items():
            bounds = info.get('bounds', [])
            if len(bounds) == 4:
                self._zones[name] = [float(b) for b in bounds]
                color = info.get('color', [0.5, 0.5, 0.5, 0.2])
                self._zones[name] = [float(b) for b in bounds]
                self._zone_colors[name] = [float(c) for c in color]
            else:
                self.get_logger().warn(f'Zone "{name}" has invalid bounds: {bounds}')

        self._waypoints = data.get('waypoints', {})
        self.get_logger().info(
            f'Loaded {len(self._zones)} zones from {path}: '
            f'{list(self._zones.keys())}'
        )

    def _classify(self, x, y):
        """Return zone name for (x,y). Priority order matches YAML key order."""
        for name, bounds in self._zones.items():
            if _point_in_rect(x, y, bounds):
                return name
        return 'outside_bounds'

    def _tick(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._frame_id, 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        raw_zone = self._classify(x, y)

        if raw_zone == self._candidate_zone:
            self._candidate_count += 1
        else:
            self._candidate_zone = raw_zone
            self._candidate_count = 1

        if (self._candidate_count >= self._hysteresis_n
                and self._candidate_zone != self._current_zone):
            old = self._current_zone
            self._current_zone = self._candidate_zone
            self.get_logger().info(
                f'Zone transition: "{old}" -> "{self._current_zone}" '
                f'at ({x:.2f}, {y:.2f})'
            )

        zone_msg = String()
        zone_msg.data = self._current_zone
        self._zone_pub.publish(zone_msg)

        dashboard = {
            'zone': self._current_zone,
            'x': round(x, 3),
            'y': round(y, 3),
            'candidate': self._candidate_zone,
            'candidate_count': self._candidate_count,
        }
        dash_msg = String()
        dash_msg.data = json.dumps(dashboard)
        self._dashboard_pub.publish(dash_msg)

    def _publish_markers(self):
        if not self._zones:
            return

        ma = MarkerArray()
        for i, (name, bounds) in enumerate(self._zones.items()):
            xmin, xmax, ymin, ymax = bounds
            cx = (xmin + xmax) / 2.0
            cy = (ymin + ymax) / 2.0
            sx = xmax - xmin
            sy = ymax - ymin

            m = Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'zone_bounds'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.01
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = 0.02
            color = self._zone_colors.get(name, [0.5, 0.5, 0.5, 0.2])
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = color[3]
            m.lifetime = DurationMsg(sec=5, nanosec=0)
            ma.markers.append(m)

            label = Marker()
            label.header.frame_id = self._frame_id
            label.header.stamp = m.header.stamp
            label.ns = 'zone_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = cx
            label.pose.position.y = cy
            label.pose.position.z = 0.5
            label.scale.z = 0.4
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.9
            display_name = name.replace('_', ' ').title()
            if name == self._current_zone:
                display_name = f'>> {display_name} <<'
            label.text = display_name
            label.lifetime = DurationMsg(sec=5, nanosec=0)
            ma.markers.append(label)

        self._marker_pub.publish(ma)


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
