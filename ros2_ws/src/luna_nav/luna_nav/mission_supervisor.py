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
import time
import urllib.request

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
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
        self.declare_parameter('zones_file', '')
        self.declare_parameter('goal_frame_id', 'world')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('localize_timeout_s', 30.0)
        self.declare_parameter('phase_timeout_s', 120.0)
        self.declare_parameter('excavation_dwell_s', 10.0)
        self.declare_parameter('construction_dwell_s', 10.0)
        self.declare_parameter('use_frontier_exploration', True)
        # Stationary pre-goal delay (s); no in-place rotation (avoids costmap artifacts).
        self.declare_parameter('map_warmup_s', 3.0)
        # Optional NDJSON debug (file + ingest). Off by default; enables extra subs when true.
        self.declare_parameter('debug_ndjson_log', False)
        self.declare_parameter('debug_ndjson_path', '')

        config_file = self.get_parameter('config_file').value
        zones_file = self.get_parameter('zones_file').value
        self._goal_frame = self.get_parameter('goal_frame_id').value or 'world'
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
        self._map_warmup = float(
            self.get_parameter('map_warmup_s').value or 8.0
        )
        self._debug_ndjson_log = bool(
            self.get_parameter('debug_ndjson_log').value
        )
        _p = self.get_parameter('debug_ndjson_path').value
        self._debug_ndjson_path = (
            str(_p).strip()
            if str(_p).strip()
            else '/tmp/luna_mission_supervisor.ndjson'
        )

        self._waypoints = {}
        if config_file:
            self._load_config(config_file)
        if zones_file:
            self._load_zones_waypoints(zones_file)

        self._phase = MissionPhase.IDLE
        self._phase_start_time = self.get_clock().now()
        self._current_zone = 'unknown'
        self._fiducial_received = False
        self._exploration_status = 'disabled'
        self._nav_goal_active = False
        self._nav_goal_handle = None
        self._nav_goal_failed = False
        self._nav_goal_fail_time = None
        self._last_sent_waypoint_key = None
        self._mission_start_time = None
        self._goal_retry_delay_s = 5.0
        self._ready_last_log_second = -1
        self._costmap_log_counter = 0
        self._robot_radius_m = 0.42
        self._last_map_odom = None
        self._cmd_summary_count = 0
        self._cmd_summary_abs_ang_max = 0.0
        self._cmd_summary_lin_avg = 0.0
        self._cmd_summary_lin_abs_avg = 0.0
        self._last_cmd_summary_t = time.time()
        self._nav2_not_ready_ticks = 0

        self._debug_log_path = '/home/piotr/luna_ros/.cursor/debug.log'

        self._zone_sub = self.create_subscription(
            String, '/current_zone', self._zone_cb, 10
        )
        self._fiducial_sub = self.create_subscription(
            PoseStamped, '/fiducial_pose', self._fiducial_cb, 10
        )
        self._exploration_sub = self.create_subscription(
            String, '/exploration_status', self._exploration_cb, 10
        )
        if self._debug_ndjson_log:
            self._local_costmap_sub = self.create_subscription(
                OccupancyGrid,
                '/local_costmap/costmap',
                self._local_costmap_cb,
                10,
            )
            self._tf_sub = self.create_subscription(
                TFMessage, '/tf', self._tf_cb, 50
            )
            self._cmd_vel_observer_sub = self.create_subscription(
                Twist, '/cmd_vel', self._cmd_vel_observer_cb, 10
            )

        self._state_pub = self.create_publisher(String, '/mission_state', 10)
        self._dashboard_pub = self.create_publisher(
            String, '/mission_dashboard', 10
        )
        self._status_marker_pub = self.create_publisher(
            Marker, '/mission_status_marker', 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=cb_group,
        )

        # Services on the same Reentrant group as the action client so executor work
        # for the client does not starve service callbacks. (Timer stays default group.)
        self._start_srv = self.create_service(
            Trigger, '~/start_mission', self._start_mission_cb,
            callback_group=cb_group,
        )
        self._abort_srv = self.create_service(
            Trigger, '~/abort_mission', self._abort_mission_cb,
            callback_group=cb_group,
        )

        self._frontier_enable_client = self.create_client(
            SetBool, '/frontier_explorer/enable'
        )

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

    def _load_zones_waypoints(self, path):
        """Load arena-specific waypoints from the zones YAML (overrides defaults)."""
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
                f'Loaded arena waypoints from {path}: {list(zone_waypoints.keys())}'
            )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _zone_cb(self, msg):
        self._current_zone = msg.data

    def _fiducial_cb(self, msg):
        self._fiducial_received = True

    def _exploration_cb(self, msg):
        self._exploration_status = msg.data

    def _cmd_vel_observer_cb(self, msg):
        ax = abs(float(msg.angular.z))
        lx = float(msg.linear.x)
        self._cmd_summary_count += 1
        self._cmd_summary_abs_ang_max = max(self._cmd_summary_abs_ang_max, ax)
        n = float(self._cmd_summary_count)
        self._cmd_summary_lin_avg += (lx - self._cmd_summary_lin_avg) / n
        self._cmd_summary_lin_abs_avg += (abs(lx) - self._cmd_summary_lin_abs_avg) / n
        now_t = time.time()
        if now_t - self._last_cmd_summary_t >= 1.0 and self._cmd_summary_count > 0:
            # #region agent log
            self._debug_log(
                hypothesis_id='H7',
                location='mission_supervisor.py:_cmd_vel_observer_cb',
                message='cmd_vel 1s summary',
                data={
                    'phase': self._phase.name,
                    'samples': int(self._cmd_summary_count),
                    'lin_avg': round(self._cmd_summary_lin_avg, 4),
                    'lin_abs_avg': round(self._cmd_summary_lin_abs_avg, 4),
                    'ang_abs_max': round(self._cmd_summary_abs_ang_max, 4),
                },
            )
            # #endregion
            self._cmd_summary_count = 0
            self._cmd_summary_abs_ang_max = 0.0
            self._cmd_summary_lin_avg = 0.0
            self._cmd_summary_lin_abs_avg = 0.0
            self._last_cmd_summary_t = now_t

        if abs(msg.angular.z) < 0.2 and abs(msg.linear.x) < 0.1:
            return
        # #region agent log
        self._debug_log(
            hypothesis_id='H3',
            location='mission_supervisor.py:_cmd_vel_observer_cb',
            message='Observed non-trivial /cmd_vel',
            data={
                'phase': self._phase.name,
                'linear_x': round(float(msg.linear.x), 4),
                'angular_z': round(float(msg.angular.z), 4),
            },
        )
        # #endregion

    def _local_costmap_cb(self, msg: OccupancyGrid):
        self._costmap_log_counter += 1
        if self._costmap_log_counter % 10 != 0:
            return
        w = int(msg.info.width)
        h = int(msg.info.height)
        res = float(msg.info.resolution) if msg.info.resolution > 0 else 0.05
        if w <= 0 or h <= 0:
            return
        cx = w // 2
        cy = h // 2
        rr = max(1, int(self._robot_radius_m / res))
        hi_threshold = 80
        lethal_threshold = 95
        near_hi_count = 0
        near_total = 0
        nearest_lethal_m = None
        data = msg.data
        for y in range(h):
            dy = y - cy
            for x in range(w):
                dx = x - cx
                idx = y * w + x
                c = int(data[idx])
                if c < 0:
                    continue
                r2 = (dx * dx) + (dy * dy)
                if r2 <= (rr * rr):
                    near_total += 1
                    if c >= hi_threshold:
                        near_hi_count += 1
                if c >= lethal_threshold:
                    d = math.sqrt(r2) * res
                    if nearest_lethal_m is None or d < nearest_lethal_m:
                        nearest_lethal_m = d
        near_hi_ratio = (near_hi_count / near_total) if near_total > 0 else 0.0
        # #region agent log
        self._debug_log(
            hypothesis_id='H5',
            location='mission_supervisor.py:_local_costmap_cb',
            message='Local costmap risk snapshot near robot',
            data={
                'phase': self._phase.name,
                'resolution_m': round(res, 4),
                'robot_radius_m': self._robot_radius_m,
                'near_hi_ratio': round(near_hi_ratio, 4),
                'nearest_lethal_m': None if nearest_lethal_m is None else round(nearest_lethal_m, 3),
                'costmap_w': w,
                'costmap_h': h,
            },
        )
        # #endregion

    def _tf_cb(self, msg: TFMessage):
        for t in msg.transforms:
            if t.header.frame_id != 'map' or t.child_frame_id != 'odom':
                continue
            tx = float(t.transform.translation.x)
            ty = float(t.transform.translation.y)
            qz = float(t.transform.rotation.z)
            qw = float(t.transform.rotation.w)
            yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
            stamp_s = float(t.header.stamp.sec) + (float(t.header.stamp.nanosec) / 1e9)
            if self._last_map_odom is None:
                self._last_map_odom = (stamp_s, tx, ty, yaw)
                return
            prev_t, prev_x, prev_y, prev_yaw = self._last_map_odom
            dt = max(1e-6, stamp_s - prev_t)
            jump_m = math.hypot(tx - prev_x, ty - prev_y)
            yaw_jump = abs(yaw - prev_yaw)
            if yaw_jump > math.pi:
                yaw_jump = abs((2.0 * math.pi) - yaw_jump)
            lin_rate = jump_m / dt
            yaw_rate = yaw_jump / dt
            self._last_map_odom = (stamp_s, tx, ty, yaw)
            # only log notable motion/jumps to keep volume low
            if lin_rate > 0.25 or yaw_rate > 0.35:
                # #region agent log
                self._debug_log(
                    hypothesis_id='H6',
                    location='mission_supervisor.py:_tf_cb',
                    message='map->odom transform jump/rate',
                    data={
                        'phase': self._phase.name,
                        'dt_s': round(dt, 4),
                        'jump_m': round(jump_m, 4),
                        'yaw_jump_rad': round(yaw_jump, 4),
                        'lin_rate_mps': round(lin_rate, 4),
                        'yaw_rate_radps': round(yaw_rate, 4),
                    },
                )
                # #endregion
            return

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
        # #region agent log
        self._debug_log(
            hypothesis_id='H1',
            location='mission_supervisor.py:_start_mission_cb',
            message='start_mission accepted',
            data={
                'map_warmup_s': float(self._map_warmup),
                'use_frontier': bool(self._use_frontier),
            },
        )
        # #endregion
        response.success = True
        response.message = 'Mission started -- LOCALIZING'
        self.get_logger().info(response.message)
        return response

    def _abort_mission_cb(self, request, response):
        self._set_frontier_enabled(False)
        self._cancel_nav_goal()
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
                self.get_logger().warn(
                    'Localization timeout; proceeding without fiducial.'
                )
                self._transition(MissionPhase.READY)

        elif self._phase == MissionPhase.READY:
            # Optional stationary delay before the first Nav2 goal. We intentionally
            # do NOT rotate here: in-place spin + depth camera caused costmap trailing
            # and self-hit noise in the local costmap.
            if elapsed < self._map_warmup:
                self._cmd_vel_pub.publish(Twist())
                now_s = int(elapsed)
                if now_s != self._ready_last_log_second:
                    self._ready_last_log_second = now_s
                    # #region agent log
                    self._debug_log(
                        hypothesis_id='H1',
                        location='mission_supervisor.py:_tick_READY',
                        message='READY stationary warmup tick',
                        data={
                            'elapsed_s': round(elapsed, 3),
                            'map_warmup_s': float(self._map_warmup),
                        },
                    )
                    # #endregion
                return
            # Do not publish Twist() here: Nav2 will command /cmd_vel; extra zeros race
            # with the controller and twist_stamper.
            wp = self._waypoints.get('excavation_entry')
            if wp:
                if self._send_nav_goal(wp):
                    self._transition(MissionPhase.TRAVERSING_TO_EXCAVATION)
                else:
                    self.get_logger().warn(
                        'Nav2 navigate_to_pose not ready; staying in READY — will retry'
                    )
            else:
                self.get_logger().warn(
                    'No excavation_entry waypoint; enable frontier or add waypoints'
                )
                self._transition(MissionPhase.TRAVERSING_TO_EXCAVATION)

        elif self._phase == MissionPhase.TRAVERSING_TO_EXCAVATION:
            if self._current_zone == 'excavation_zone':
                self.get_logger().info('Entered excavation zone')
                self._transition(MissionPhase.IN_EXCAVATION_ZONE)
            elif self._maybe_retry_goal('excavation_entry'):
                pass
            elif elapsed > self._phase_timeout:
                if self._use_frontier:
                    self.get_logger().info(
                        'Traversal timeout -- switching to frontier exploration'
                    )
                    self._set_frontier_enabled(True)
                    self._transition(MissionPhase.EXPLORING)
                else:
                    self.get_logger().warn(
                        'Traversal timeout -- skipping to excavation dwell'
                    )
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
            if elapsed > self._excavation_dwell:
                self.get_logger().info('Excavation dwell complete; heading to construction')
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
                self.get_logger().warn(
                    'Construction traversal timeout -- forcing construction dwell'
                )
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
                self.get_logger().info('Returned to starting zone -- MISSION COMPLETE')
                self._transition(MissionPhase.MISSION_COMPLETE)
            elif self._maybe_retry_goal('return_to_start'):
                pass
            elif elapsed > self._phase_timeout:
                self.get_logger().warn(
                    'Return timeout -- marking mission complete anyway'
                )
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
        self.get_logger().info(
            f'Phase transition: {old.name} -> {new_phase.name}'
        )

    # ------------------------------------------------------------------
    # Nav2 goal retry
    # ------------------------------------------------------------------

    def _maybe_retry_goal(self, waypoint_key: str) -> bool:
        """If the last Nav2 goal failed and enough cooldown has passed, resend it.

        Returns True if a retry was dispatched (caller should skip other logic).
        """
        if not self._nav_goal_failed:
            return False
        if self._nav_goal_fail_time is None:
            return False
        cooldown = (
            self.get_clock().now() - self._nav_goal_fail_time
        ).nanoseconds / 1e9
        if cooldown < self._goal_retry_delay_s:
            return False
        wp = self._waypoints.get(waypoint_key)
        if wp:
            # #region agent log
            self._debug_log(
                hypothesis_id='H11',
                location='mission_supervisor.py:_maybe_retry_goal',
                message='Retrying failed Nav2 goal after cooldown',
                data={
                    'phase': self._phase.name,
                    'waypoint_key': waypoint_key,
                    'cooldown_s': round(float(cooldown), 3),
                    'nav_goal_active': bool(self._nav_goal_active),
                },
            )
            # #endregion
            self.get_logger().info(
                f'Retrying Nav2 goal for "{waypoint_key}" '
                f'after {cooldown:.1f}s cooldown'
            )
            self._nav_goal_failed = False
            self._send_nav_goal(wp)
        return True

    # ------------------------------------------------------------------
    # Nav2 goal sending
    # ------------------------------------------------------------------

    def _send_nav_goal(self, wp):
        """Send a NavigateToPose goal. Returns True if the async send was started."""
        # Do not call wait_for_server() from the timer callback: it spins the node and
        # can deadlock the executor so ~/start_mission never completes. Non-blocking
        # check; READY phase retries when this returns False.
        if not self._nav_client.server_is_ready():
            self._nav2_not_ready_ticks += 1
            if self._nav2_not_ready_ticks == 1 or self._nav2_not_ready_ticks % 20 == 0:
                self.get_logger().warn(
                    'Nav2 navigate_to_pose not ready yet; will retry on next tick'
                )
            # #region agent log
            self._debug_log(
                hypothesis_id='H8',
                location='mission_supervisor.py:_send_nav_goal',
                message='Nav2 action server unavailable (non-blocking check)',
                data={'phase': self._phase.name},
            )
            # #endregion
            return False
        self._nav2_not_ready_ticks = 0

        # Avoid stacking NavigateToPose goals (reduces preemption / status-6 abort loops).
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
            f'Sending Nav2 goal ({self._goal_frame}): ({wp[0]:.2f}, {wp[1]:.2f})'
        )
        # #region agent log
        self._debug_log(
            hypothesis_id='H8',
            location='mission_supervisor.py:_send_nav_goal',
            message='Sending Nav2 goal',
            data={
                'phase': self._phase.name,
                'goal_frame': self._goal_frame,
                'x': round(float(wp[0]), 3),
                'y': round(float(wp[1]), 3),
                'yaw_set': round(float(wp[2]), 3) if len(wp) > 2 else None,
                'current_zone': self._current_zone,
                'nav_goal_active_before_send': bool(self._nav_goal_active),
            },
        )
        # #endregion
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
            # #region agent log
            self._debug_log(
                hypothesis_id='H8',
                location='mission_supervisor.py:_goal_response_cb',
                message='Nav2 goal rejected',
                data={'phase': self._phase.name},
            )
            # #endregion
            return
        # #region agent log
        self._debug_log(
            hypothesis_id='H8',
            location='mission_supervisor.py:_goal_response_cb',
            message='Nav2 goal accepted',
            data={'phase': self._phase.name},
        )
        # #endregion
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
                nav_result = getattr(result, 'result', None)
                err_code = None
                err_msg = None
                if nav_result is not None:
                    err_code = int(getattr(nav_result, 'error_code', 0))
                    err_msg = str(getattr(nav_result, 'error_msg', ''))
                # #region agent log
                self._debug_log(
                    hypothesis_id='H4',
                    location='mission_supervisor.py:_goal_result_cb',
                    message='Nav2 goal non-success result',
                    data={
                        'status': int(result.status),
                        'phase': self._phase.name,
                        'current_zone': self._current_zone,
                        'error_code': err_code,
                        'error_msg': err_msg,
                    },
                )
                # #endregion
        except Exception as e:
            self.get_logger().warn(f'Nav2 goal result error: {e}')
            self._nav_goal_failed = True
            self._nav_goal_fail_time = self.get_clock().now()
            # #region agent log
            self._debug_log(
                hypothesis_id='H12',
                location='mission_supervisor.py:_goal_result_cb',
                message='Exception while handling Nav2 goal result',
                data={'phase': self._phase.name, 'exception': str(e)},
            )
            # #endregion

    def _cancel_nav_goal(self):
        if self._nav_goal_handle is not None:
            try:
                # #region agent log
                self._debug_log(
                    hypothesis_id='H10',
                    location='mission_supervisor.py:_cancel_nav_goal',
                    message='Cancelling active Nav2 goal from mission supervisor',
                    data={'phase': self._phase.name, 'current_zone': self._current_zone},
                )
                # #endregion
                self._nav_goal_handle.cancel_goal_async()
                self.get_logger().info('Cancelled active Nav2 goal')
            except Exception:
                pass
            self._nav_goal_handle = None
        self._nav_goal_active = False

    def _set_frontier_enabled(self, enabled: bool):
        # Same nested-spin hazard as wait_for_server when called from _tick.
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
            total_elapsed = (
                self.get_clock().now() - self._mission_start_time
            ).nanoseconds / 1e9
        phase_elapsed = self._phase_elapsed_s()

        dashboard = {
            'phase': self._phase.name,
            'zone': self._current_zone,
            'fiducial_received': self._fiducial_received,
            'exploration': self._exploration_status,
            'mission_elapsed_s': round(total_elapsed, 1),
            'phase_elapsed_s': round(phase_elapsed, 1),
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
            total = (
                self.get_clock().now() - self._mission_start_time
            ).nanoseconds / 1e9
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

    def _debug_log(self, hypothesis_id: str, location: str, message: str, data: dict):
        if not self._debug_ndjson_log:
            return
        try:
            payload = {
                'id': f'log_{int(time.time() * 1000)}_{hypothesis_id}',
                'timestamp': int(time.time() * 1000),
                'runId': 'pre-fix',
                'hypothesisId': hypothesis_id,
                'location': location,
                'message': message,
                'data': data,
            }
            try:
                # #region agent log
                req = urllib.request.Request(
                    'http://127.0.0.1:7243/ingest/118513d0-d9fa-4d38-9e11-6db013dbe340',
                    data=json.dumps(payload).encode('utf-8'),
                    headers={'Content-Type': 'application/json'},
                    method='POST',
                )
                urllib.request.urlopen(req, timeout=0.15).read()
                # #endregion
            except Exception:
                pass
            with open(self._debug_ndjson_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(payload, separators=(',', ':')) + '\n')
        except Exception:
            pass


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
