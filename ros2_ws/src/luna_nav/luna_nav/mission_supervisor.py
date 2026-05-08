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

Parameters loaded from mission_phases.yaml and optionally the arena zones YAML.
"""

from enum import Enum, auto
import json
import math

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration as DurationMsg

from nav2_msgs.action import NavigateToPose


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
        self.declare_parameter('zones_file', '')
        self.declare_parameter('goal_frame_id', 'world')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('localize_timeout_s', 30.0)
        self.declare_parameter('phase_timeout_s', 120.0)
        self.declare_parameter('excavation_dwell_s', 15.0)
        self.declare_parameter('construction_dwell_s', 20.0)
        self.declare_parameter('use_frontier_exploration', True)
        # Stationary pre-goal delay (s); no in-place rotation (avoids costmap artifacts).
        self.declare_parameter('map_warmup_s', 3.0)
        # Excavation: seconds to wait after lowering bucket before driving, and drive speed/duration
        self.declare_parameter('excavation_lower_delay_s', 2.0)
        self.declare_parameter('excavation_drive_speed_ms', 0.15)
        self.declare_parameter('excavation_drive_s', 5.0)
        # Dump: seconds for lift to rise before backing up, then backup speed/duration
        self.declare_parameter('dump_lift_delay_s', 2.0)
        self.declare_parameter('dump_backup_speed_ms', 0.10)
        self.declare_parameter('dump_backup_s', 6.0)

        config_file = self.get_parameter('config_file').value
        zones_file = self.get_parameter('zones_file').value
        self._goal_frame = self.get_parameter('goal_frame_id').value or 'world'
        rate = float(self.get_parameter('rate_hz').value or 2.0)
        self._localize_timeout = float(self.get_parameter('localize_timeout_s').value or 30.0)
        self._phase_timeout = float(self.get_parameter('phase_timeout_s').value or 120.0)
        self._excavation_dwell = float(self.get_parameter('excavation_dwell_s').value or 15.0)
        self._construction_dwell = float(self.get_parameter('construction_dwell_s').value or 20.0)
        self._use_frontier = bool(self.get_parameter('use_frontier_exploration').value)
        self._map_warmup = float(self.get_parameter('map_warmup_s').value or 3.0)
        self._excavation_lower_delay = float(self.get_parameter('excavation_lower_delay_s').value)
        self._excavation_drive_speed = float(self.get_parameter('excavation_drive_speed_ms').value)
        self._excavation_drive_s = float(self.get_parameter('excavation_drive_s').value)
        self._dump_lift_delay = float(self.get_parameter('dump_lift_delay_s').value)
        self._dump_backup_speed = float(self.get_parameter('dump_backup_speed_ms').value)
        self._dump_backup_s = float(self.get_parameter('dump_backup_s').value)

        self._waypoints = {}
        if config_file:
            self._load_config(config_file)
        if zones_file:
            self._load_zones_waypoints(zones_file)

        self._phase = MissionPhase.IDLE
        self._phase_start_time = self.get_clock().now()
        self._linaks_called = False       # guards single service call per dwell phase
        self._excavation_driving = False  # True while publishing forward cmd_vel during scoop
        self._excavation_raised = False   # True once bucket raise has been commanded
        self._dump_backing = False        # True while publishing reverse cmd_vel during dump
        self._dump_retracted = False      # True once lift retract has been commanded
        self._current_zone = 'unknown'
        self._fiducial_received = False
        self._exploration_status = 'disabled'
        self._nav_goal_active = False
        self._nav_goal_handle = None
        self._nav_goal_failed = False
        self._nav_goal_fail_time = None
        self._mission_start_time = None
        self._goal_retry_delay_s = 5.0
        self._ready_last_log_second = -1
        self._nav2_not_ready_ticks = 0

        self._zone_sub = self.create_subscription(
            String, '/current_zone', self._zone_cb, 10)
        self._fiducial_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/fiducial_pose', self._fiducial_cb, 10)
        self._exploration_sub = self.create_subscription(
            String, '/exploration_status', self._exploration_cb, 10)

        self._state_pub = self.create_publisher(String, '/mission_state', 10)
        self._dashboard_pub = self.create_publisher(String, '/mission_dashboard', 10)
        self._status_marker_pub = self.create_publisher(Marker, '/mission_status_marker', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=cb_group)

        self._start_srv = self.create_service(
            Trigger, '~/start_mission', self._start_mission_cb, callback_group=cb_group)
        self._abort_srv = self.create_service(
            Trigger, '~/abort_mission', self._abort_mission_cb, callback_group=cb_group)

        self._frontier_enable_client = self.create_client(SetBool, '/frontier_explorer/enable')
        # shoulder (0x80) + wrist (0xC8): both sides of 4-bar, move together to lower/raise scoop
        self._shoulder_extend  = self.create_client(Trigger, '/luna_linaks_node/shoulder/extend')
        self._shoulder_retract = self.create_client(Trigger, '/luna_linaks_node/shoulder/retract')
        self._wrist_extend     = self.create_client(Trigger, '/luna_linaks_node/wrist/extend')
        self._wrist_retract    = self.create_client(Trigger, '/luna_linaks_node/wrist/retract')
        # lift (0xF6): scissor lift extends for gravity dump; retract not commanded (unknown if it retracts)
        self._lift_extend      = self.create_client(Trigger, '/luna_linaks_node/lift/extend')
        self._linaks_stop_all  = self.create_client(Trigger, '/luna_linaks_node/stop_all')

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self._marker_timer = self.create_timer(1.0, self._publish_status_marker)

        self.get_logger().info(
            f'mission_supervisor: phase=IDLE, use_frontier={self._use_frontier}, '
            f'goal_frame={self._goal_frame}, waypoints={list(self._waypoints.keys())}'
        )

    def _load_config(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load mission config: {e}')
            return
        self._waypoints = data.get('waypoints', {})
        self._localize_timeout = data.get('localize_timeout_s', self._localize_timeout)
        self._phase_timeout = data.get('phase_timeout_s', self._phase_timeout)
        self._excavation_dwell = data.get('excavation_dwell_s', self._excavation_dwell)
        self._construction_dwell = data.get('construction_dwell_s', self._construction_dwell)
        self.get_logger().info(f'Loaded mission config from {path}')

    def _load_zones_waypoints(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load zones waypoints: {e}')
            return
        zone_waypoints = data.get('waypoints', {})
        if zone_waypoints:
            self._waypoints.update(zone_waypoints)
            self.get_logger().info(
                f'Loaded arena waypoints from {path}: {list(zone_waypoints.keys())}')

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
        self._mission_start_time = self.get_clock().now()
        response.success = True
        response.message = 'Mission started -- LOCALIZING'
        self.get_logger().info(response.message)
        return response

    def _abort_mission_cb(self, request, response):
        self._set_frontier_enabled(False)
        self._cancel_nav_goal()
        self._call_linaks(self._linaks_stop_all, 'stop_all')
        self._transition(MissionPhase.IDLE)
        self._nav_goal_active = False
        response.success = True
        response.message = 'Mission aborted'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Main FSM tick
    # ------------------------------------------------------------------

    def _phase_elapsed_s(self):
        return (self.get_clock().now() - self._phase_start_time).nanoseconds / 1e9

    def _tick(self):
        elapsed = self._phase_elapsed_s()

        if self._phase == MissionPhase.IDLE:
            pass

        elif self._phase == MissionPhase.LOCALIZING:
            if self._fiducial_received:
                self.get_logger().info('Fiducial pose received -- READY')
                self._transition(MissionPhase.READY)
            elif elapsed > self._localize_timeout:
                self.get_logger().warn('Localization timeout; proceeding without fiducial.')
                self._transition(MissionPhase.READY)

        elif self._phase == MissionPhase.READY:
            # Stationary delay before first Nav2 goal. No in-place rotation: rotation
            # + depth camera caused costmap trailing and self-hit noise.
            if elapsed < self._map_warmup:
                self._cmd_vel_pub.publish(Twist())
                now_s = int(elapsed)
                if now_s != self._ready_last_log_second:
                    self._ready_last_log_second = now_s
                    self.get_logger().info(
                        f'READY warmup {elapsed:.1f}/{self._map_warmup:.1f}s')
                return
            wp = self._waypoints.get('excavation_entry')
            if wp:
                if self._send_nav_goal(wp):
                    self._transition(MissionPhase.TRAVERSING_TO_EXCAVATION)
                else:
                    self.get_logger().warn('Nav2 not ready; staying in READY -- will retry')
            else:
                self.get_logger().warn('No excavation_entry waypoint; enable frontier or add waypoints')
                self._transition(MissionPhase.TRAVERSING_TO_EXCAVATION)

        elif self._phase == MissionPhase.TRAVERSING_TO_EXCAVATION:
            if self._current_zone == 'excavation_zone':
                self.get_logger().info('Entered excavation zone')
                self._transition(MissionPhase.IN_EXCAVATION_ZONE)
            elif self._maybe_retry_goal('excavation_entry'):
                pass
            elif elapsed > self._phase_timeout:
                if self._use_frontier:
                    self.get_logger().info('Traversal timeout -- switching to frontier exploration')
                    self._set_frontier_enabled(True)
                    self._transition(MissionPhase.EXPLORING)
                else:
                    self.get_logger().warn('Traversal timeout -- skipping to excavation dwell')
                    self._transition(MissionPhase.IN_EXCAVATION_ZONE)

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
            elif elapsed > self._phase_timeout:
                self.get_logger().warn('Exploration timeout -- forcing excavation')
                self._set_frontier_enabled(False)
                wp = self._waypoints.get('excavation_entry')
                if wp:
                    self._send_nav_goal(wp)
                self._transition(MissionPhase.TRAVERSING_TO_EXCAVATION)

        elif self._phase == MissionPhase.IN_EXCAVATION_ZONE:
            # Step 1 (t=0): extend both sides of 4-bar simultaneously to lower scoop
            if not self._linaks_called:
                self._linaks_called = True
                self._call_linaks(self._shoulder_extend, 'shoulder/extend')
                self._call_linaks(self._wrist_extend,    'wrist/extend')
                self.get_logger().info('Excavation: lowering 4-bar scoop')

            # Step 2 (t=lower_delay): drive forward to scoop
            drive_start = self._excavation_lower_delay
            drive_end   = drive_start + self._excavation_drive_s
            if drive_start <= elapsed < drive_end:
                if not self._excavation_driving:
                    self._excavation_driving = True
                    self.get_logger().info('Excavation: driving forward to scoop')
                fwd = Twist()
                fwd.linear.x = self._excavation_drive_speed
                self._cmd_vel_pub.publish(fwd)

            # Step 3 (t=drive_end): stop driving and raise bucket
            elif elapsed >= drive_end and not self._excavation_raised:
                self._excavation_raised = True
                self._cmd_vel_pub.publish(Twist())  # stop
                self._call_linaks(self._shoulder_retract, 'shoulder/retract')
                self._call_linaks(self._wrist_retract,    'wrist/retract')
                self.get_logger().info('Excavation: raising 4-bar scoop')

            if elapsed > self._excavation_dwell:
                self.get_logger().info('Excavation dwell complete; heading to construction')
                self._cmd_vel_pub.publish(Twist())
                wp = self._waypoints.get('construction_entry')
                if wp:
                    self._send_nav_goal(wp)
                self._transition(MissionPhase.TRAVERSING_TO_CONSTRUCTION)

        elif self._phase == MissionPhase.TRAVERSING_TO_CONSTRUCTION:
            if self._current_zone == 'construction_zone':
                self.get_logger().info('Entered construction zone')
                self._transition(MissionPhase.IN_CONSTRUCTION_ZONE)
            elif self._maybe_retry_goal('construction_entry'):
                pass
            elif elapsed > self._phase_timeout:
                self.get_logger().warn('Construction traversal timeout -- forcing construction dwell')
                self._transition(MissionPhase.IN_CONSTRUCTION_ZONE)

        elif self._phase == MissionPhase.IN_CONSTRUCTION_ZONE:
            # Step 1 (t=0): extend scissor lift — gravity starts the dump
            if not self._linaks_called:
                self._linaks_called = True
                self._call_linaks(self._lift_extend, 'lift/extend')
                self.get_logger().info('Dump: extending scissor lift')

            # Step 2 (t=lift_delay): reverse slowly to spread regolith along berm
            backup_start = self._dump_lift_delay
            backup_end   = backup_start + self._dump_backup_s
            if backup_start <= elapsed < backup_end:
                if not self._dump_backing:
                    self._dump_backing = True
                    self.get_logger().info('Dump: backing up to spread berm')
                rev = Twist()
                rev.linear.x = -abs(self._dump_backup_speed)
                self._cmd_vel_pub.publish(rev)

            # Step 3 (t=backup_end): stop reversing; lift stays extended (no retract — one-way mechanism)
            elif elapsed >= backup_end and not self._dump_retracted:
                self._dump_retracted = True
                self._cmd_vel_pub.publish(Twist())  # stop
                self.get_logger().info('Dump: backup complete, lift remains extended')

            if elapsed > self._construction_dwell:
                self.get_logger().info('Construction dwell complete; returning')
                self._cmd_vel_pub.publish(Twist())
                wp = self._waypoints.get('return_to_start')
                if wp:
                    self._send_nav_goal(wp)
                self._transition(MissionPhase.RETURNING)

        elif self._phase == MissionPhase.RETURNING:
            if self._current_zone == 'starting_zone':
                self.get_logger().info('Returned to starting zone -- MISSION COMPLETE')
                self._transition(MissionPhase.MISSION_COMPLETE)
            elif self._maybe_retry_goal('return_to_start'):
                pass
            elif elapsed > self._phase_timeout:
                self.get_logger().warn('Return timeout -- marking mission complete anyway')
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
        self._phase_start_time = self.get_clock().now()
        self._linaks_called = False
        self.get_logger().info(f'Phase transition: {old.name} -> {new_phase.name}')

    # ------------------------------------------------------------------
    # Nav2 goal retry
    # ------------------------------------------------------------------

    def _maybe_retry_goal(self, waypoint_key: str) -> bool:
        if not self._nav_goal_failed:
            return False
        if self._nav_goal_fail_time is None:
            return False
        cooldown = (self.get_clock().now() - self._nav_goal_fail_time).nanoseconds / 1e9
        if cooldown < self._goal_retry_delay_s:
            return False
        wp = self._waypoints.get(waypoint_key)
        if wp:
            self.get_logger().info(
                f'Retrying Nav2 goal for "{waypoint_key}" after {cooldown:.1f}s cooldown')
            self._nav_goal_failed = False
            self._send_nav_goal(wp)
        return True

    # ------------------------------------------------------------------
    # Nav2 goal sending
    # ------------------------------------------------------------------

    def _send_nav_goal(self, wp):
        """Send a NavigateToPose goal. Returns True if the async send was started."""
        # Non-blocking check: wait_for_server() would deadlock the executor from a timer callback.
        if not self._nav_client.server_is_ready():
            self._nav2_not_ready_ticks += 1
            if self._nav2_not_ready_ticks == 1 or self._nav2_not_ready_ticks % 20 == 0:
                self.get_logger().warn('Nav2 navigate_to_pose not ready; will retry on next tick')
            return False
        self._nav2_not_ready_ticks = 0

        self._cancel_nav_goal()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp[0])
        goal.pose.pose.position.y = float(wp[1])
        if len(wp) > 2:
            yaw = float(wp[2])
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending Nav2 goal ({self._goal_frame}): ({wp[0]:.2f}, {wp[1]:.2f})')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        self._nav_goal_active = True
        return True

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected (planner/controller may be inactive)')
            self._nav_goal_active = False
            self._nav_goal_failed = True
            self._nav_goal_fail_time = self.get_clock().now()
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        self._nav_goal_active = False
        self._nav_goal_handle = None
        try:
            result = future.result()
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info('Nav2 goal reached')
                self._nav_goal_failed = False
            else:
                self.get_logger().warn(f'Nav2 goal ended with status {result.status}')
                self._nav_goal_failed = True
                self._nav_goal_fail_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().warn(f'Nav2 goal result error: {e}')
            self._nav_goal_failed = True
            self._nav_goal_fail_time = self.get_clock().now()

    def _cancel_nav_goal(self):
        if self._nav_goal_handle is not None:
            try:
                self._nav_goal_handle.cancel_goal_async()
                self.get_logger().info('Cancelled active Nav2 goal')
            except Exception:
                pass
            self._nav_goal_handle = None
        self._nav_goal_active = False

    def _call_linaks(self, client, name: str):
        if not client.service_is_ready():
            self.get_logger().warn(f'linaks/{name} service not available (node running?)')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'linaks/{name}: {f.result().message}' if f.result() else f'linaks/{name}: no response'
            )
        )

    def _set_frontier_enabled(self, enabled: bool):
        if not self._frontier_enable_client.service_is_ready():
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
            total_elapsed = (self.get_clock().now() - self._mission_start_time).nanoseconds / 1e9
        dashboard = {
            'phase': self._phase.name,
            'zone': self._current_zone,
            'fiducial_received': self._fiducial_received,
            'exploration': self._exploration_status,
            'mission_elapsed_s': round(total_elapsed, 1),
            'phase_elapsed_s': round(self._phase_elapsed_s(), 1),
            'nav_goal_active': self._nav_goal_active,
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
            total = (self.get_clock().now() - self._mission_start_time).nanoseconds / 1e9
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
