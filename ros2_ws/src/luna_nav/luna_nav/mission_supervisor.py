#!/usr/bin/env python3
"""
Mission Supervisor: competition-style FSM for Lunabotics autonomous operations.

Coordinates the full mission cycle:
  IDLE -> LOCALIZING -> READY -> TRAVERSING_TO_EXCAVATION -> IN_EXCAVATION_ZONE
  -> TRAVERSING_TO_CONSTRUCTION -> IN_CONSTRUCTION_ZONE -> RETURNING -> COMPLETE

Integrates with:
  - zone_publisher (/current_zone)
  - fiducial_localizer (/fiducial_pose)
  - frontier_explorer (/exploration_status, ~/enable service)
  - Nav2 (NavigateToPose action)

Protocol reference:
  2025 Lunabotics Guidebook Sections 5-6
  - Travel Automation (Sec 6.2.3): Start -> obstacle field traversal
  - Excavation & Dump Automation (Sec 6.2.1): Excavate + dump in construction zone
  - Full Autonomy (Sec 6.2.4): Complete cycle without human intervention

Parameters loaded from mission_phases.yaml.
"""

from enum import Enum, auto
import json
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration as DurationMsg

from nav2_msgs.action import NavigateToPose

import tf2_ros


class MissionPhase(Enum):
    IDLE = auto()
    LOCALIZING = auto()
    READY = auto()
    TRAVERSING_TO_EXCAVATION = auto()
    EXPLORING = auto()
    IN_EXCAVATION_ZONE = auto()
    TRAVERSING_TO_CONSTRUCTION = auto()
    IN_CONSTRUCTION_ZONE = auto()
    RETURNING = auto()
    MISSION_COMPLETE = auto()


class MissionSupervisorNode(Node):

    def __init__(self):
        super().__init__('mission_supervisor')
        cb_group = ReentrantCallbackGroup()

        self.declare_parameter('config_file', '')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('localize_timeout_s', 30.0)
        self.declare_parameter('phase_timeout_s', 120.0)
        self.declare_parameter('excavation_dwell_s', 10.0)
        self.declare_parameter('construction_dwell_s', 10.0)
        self.declare_parameter('use_frontier_exploration', True)

        config_file = self.get_parameter('config_file').value
        rate = float(self.get_parameter('rate_hz').value or 2.0)
        self._localize_timeout = float(
            self.get_parameter('localize_timeout_s').value or 30.0
        )
        self._phase_timeout = float(
            self.get_parameter('phase_timeout_s').value or 120.0
        )
        self._excavation_dwell = float(
            self.get_parameter('excavation_dwell_s').value or 10.0
        )
        self._construction_dwell = float(
            self.get_parameter('construction_dwell_s').value or 10.0
        )
        self._use_frontier = bool(
            self.get_parameter('use_frontier_exploration').value
        )

        self._waypoints = {}
        if config_file:
            self._load_config(config_file)

        self._phase = MissionPhase.IDLE
        self._phase_start_time = time.monotonic()
        self._current_zone = 'unknown'
        self._fiducial_received = False
        self._exploration_status = 'disabled'
        self._nav_goal_active = False
        self._mission_start_time = None

        self._zone_sub = self.create_subscription(
            String, '/current_zone', self._zone_cb, 10
        )
        self._fiducial_sub = self.create_subscription(
            PoseStamped, '/fiducial_pose', self._fiducial_cb, 10
        )
        self._exploration_sub = self.create_subscription(
            String, '/exploration_status', self._exploration_cb, 10
        )

        self._state_pub = self.create_publisher(String, '/mission_state', 10)
        self._dashboard_pub = self.create_publisher(
            String, '/mission_dashboard', 10
        )
        self._status_marker_pub = self.create_publisher(
            Marker, '/mission_status_marker', 10
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=cb_group,
        )

        self._start_srv = self.create_service(
            Trigger, '~/start_mission', self._start_mission_cb
        )
        self._abort_srv = self.create_service(
            Trigger, '~/abort_mission', self._abort_mission_cb
        )

        self._frontier_enable_client = self.create_client(
            SetBool, '/frontier_explorer/enable'
        )

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self._marker_timer = self.create_timer(1.0, self._publish_status_marker)

        self.get_logger().info(
            f'mission_supervisor: phase=IDLE, use_frontier={self._use_frontier}'
        )

    def _load_config(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load mission config: {e}')
            return

        self._waypoints = data.get('waypoints', {})
        self._localize_timeout = data.get(
            'localize_timeout_s', self._localize_timeout
        )
        self._phase_timeout = data.get(
            'phase_timeout_s', self._phase_timeout
        )
        self._excavation_dwell = data.get(
            'excavation_dwell_s', self._excavation_dwell
        )
        self._construction_dwell = data.get(
            'construction_dwell_s', self._construction_dwell
        )
        self.get_logger().info(f'Loaded mission config from {path}')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _zone_cb(self, msg):
        self._current_zone = msg.data

    def _fiducial_cb(self, msg):
        self._fiducial_received = True

    def _exploration_cb(self, msg):
        self._exploration_status = msg.data

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _start_mission_cb(self, request, response):
        if self._phase != MissionPhase.IDLE:
            response.success = False
            response.message = f'Cannot start: currently in {self._phase.name}'
            return response
        self._transition(MissionPhase.LOCALIZING)
        self._mission_start_time = time.monotonic()
        response.success = True
        response.message = 'Mission started -- LOCALIZING'
        self.get_logger().info(response.message)
        return response

    def _abort_mission_cb(self, request, response):
        self._set_frontier_enabled(False)
        self._transition(MissionPhase.IDLE)
        self._nav_goal_active = False
        response.success = True
        response.message = 'Mission aborted'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Main FSM tick
    # ------------------------------------------------------------------

    def _tick(self):
        elapsed = time.monotonic() - self._phase_start_time

        if self._phase == MissionPhase.IDLE:
            pass

        elif self._phase == MissionPhase.LOCALIZING:
            if self._fiducial_received:
                self.get_logger().info('Fiducial pose received -- READY')
                self._transition(MissionPhase.READY)
            elif elapsed > self._localize_timeout:
                self.get_logger().warn(
                    'Localization timeout; proceeding without fiducial.'
                )
                self._transition(MissionPhase.READY)

        elif self._phase == MissionPhase.READY:
            wp = self._waypoints.get('excavation_entry')
            if wp:
                self._send_nav_goal(wp)
            else:
                self.get_logger().warn('No excavation_entry waypoint; using WFD')
            self._transition(MissionPhase.TRAVERSING_TO_EXCAVATION)

        elif self._phase == MissionPhase.TRAVERSING_TO_EXCAVATION:
            if self._current_zone == 'excavation_zone':
                self._transition(MissionPhase.IN_EXCAVATION_ZONE)
            elif elapsed > self._phase_timeout and self._use_frontier:
                self.get_logger().info(
                    'Traversal timeout -- switching to frontier exploration'
                )
                self._set_frontier_enabled(True)
                self._transition(MissionPhase.EXPLORING)

        elif self._phase == MissionPhase.EXPLORING:
            if self._exploration_status == 'no_frontiers_remaining':
                self._set_frontier_enabled(False)
                wp = self._waypoints.get('excavation_entry')
                if wp:
                    self._send_nav_goal(wp)
                self._transition(MissionPhase.TRAVERSING_TO_EXCAVATION)
            elif self._current_zone == 'excavation_zone':
                self._set_frontier_enabled(False)
                self._transition(MissionPhase.IN_EXCAVATION_ZONE)

        elif self._phase == MissionPhase.IN_EXCAVATION_ZONE:
            if elapsed > self._excavation_dwell:
                self.get_logger().info('Excavation dwell complete; heading to construction')
                wp = self._waypoints.get('construction_entry')
                if wp:
                    self._send_nav_goal(wp)
                self._transition(MissionPhase.TRAVERSING_TO_CONSTRUCTION)

        elif self._phase == MissionPhase.TRAVERSING_TO_CONSTRUCTION:
            if self._current_zone == 'construction_zone':
                self._transition(MissionPhase.IN_CONSTRUCTION_ZONE)

        elif self._phase == MissionPhase.IN_CONSTRUCTION_ZONE:
            if elapsed > self._construction_dwell:
                self.get_logger().info('Construction dwell complete; returning')
                wp = self._waypoints.get('return_to_start')
                if wp:
                    self._send_nav_goal(wp)
                self._transition(MissionPhase.RETURNING)

        elif self._phase == MissionPhase.RETURNING:
            if self._current_zone == 'starting_zone':
                self._transition(MissionPhase.MISSION_COMPLETE)

        elif self._phase == MissionPhase.MISSION_COMPLETE:
            pass

        self._publish_dashboard()
        state_msg = String()
        state_msg.data = self._phase.name
        self._state_pub.publish(state_msg)

    def _transition(self, new_phase):
        old = self._phase
        self._phase = new_phase
        self._phase_start_time = time.monotonic()
        self.get_logger().info(
            f'Phase transition: {old.name} -> {new_phase.name}'
        )

    # ------------------------------------------------------------------
    # Nav2 goal sending
    # ------------------------------------------------------------------

    def _send_nav_goal(self, wp):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp[0])
        goal.pose.pose.position.y = float(wp[1])
        if len(wp) > 2:
            import math
            yaw = float(wp[2])
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending Nav2 goal: ({wp[0]:.2f}, {wp[1]:.2f})'
        )
        self._nav_client.send_goal_async(goal)
        self._nav_goal_active = True

    def _set_frontier_enabled(self, enabled: bool):
        if not self._frontier_enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Frontier enable service not available')
            return
        req = SetBool.Request()
        req.data = enabled
        self._frontier_enable_client.call_async(req)

    # ------------------------------------------------------------------
    # Dashboard / visualization
    # ------------------------------------------------------------------

    def _publish_dashboard(self):
        total_elapsed = 0.0
        if self._mission_start_time:
            total_elapsed = time.monotonic() - self._mission_start_time
        phase_elapsed = time.monotonic() - self._phase_start_time

        dashboard = {
            'phase': self._phase.name,
            'zone': self._current_zone,
            'fiducial_received': self._fiducial_received,
            'exploration': self._exploration_status,
            'mission_elapsed_s': round(total_elapsed, 1),
            'phase_elapsed_s': round(phase_elapsed, 1),
        }
        msg = String()
        msg.data = json.dumps(dashboard)
        self._dashboard_pub.publish(msg)

    def _publish_status_marker(self):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mission_status'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.z = 1.2
        m.scale.z = 0.3

        phase_name = self._phase.name.replace('_', ' ').title()
        total = 0.0
        if self._mission_start_time:
            total = time.monotonic() - self._mission_start_time
        m.text = f'{phase_name}\nZone: {self._current_zone}\nT: {total:.0f}s'

        if self._phase == MissionPhase.MISSION_COMPLETE:
            m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
        elif self._phase == MissionPhase.IDLE:
            m.color.r, m.color.g, m.color.b = 0.5, 0.5, 0.5
        elif 'TRAVERSING' in self._phase.name or self._phase == MissionPhase.EXPLORING:
            m.color.r, m.color.g, m.color.b = 1.0, 0.8, 0.0
        else:
            m.color.r, m.color.g, m.color.b = 0.3, 0.7, 1.0
        m.color.a = 0.95
        m.lifetime = DurationMsg(sec=2, nanosec=0)

        self._status_marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
