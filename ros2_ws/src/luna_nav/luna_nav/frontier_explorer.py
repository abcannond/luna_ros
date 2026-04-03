#!/usr/bin/env python3
"""
Wavefront Frontier Detector (WFD) for autonomous exploration.

Implements the algorithm from:
  Topiwala, Inani, Kathpal -- "Frontier Based Exploration for Autonomous Robot"
  arXiv:1806.03581  (https://arxiv.org/abs/1806.03581)

The node subscribes to an OccupancyGrid (/map) and the robot pose via TF,
detects frontier cells using BFS from the robot position, clusters them,
and sends the best frontier centroid to Nav2 as a NavigateToPose goal.

Goal selection balances distance and information gain (frontier size) to
avoid zig-zagging between tiny nearby frontiers while still preferring
frontiers the robot can reach efficiently.

Parameters:
  enabled             -- bool, start exploring immediately (default false)
  map_topic           -- OccupancyGrid topic (default /map)
  min_frontier_size   -- minimum cells for a valid frontier cluster (default 8)
  rate_hz             -- detection loop rate (default 0.5)
  goal_tolerance      -- meters; re-scan when within this of a goal (default 0.5)
  blacklist_radius    -- meters; ignore frontiers near failed goals (default 1.0)
  min_goal_distance   -- meters; reject frontiers closer than this (default 0.6)
  settle_time_s       -- seconds to pause after reaching a goal before re-scanning
"""

from collections import deque
from enum import Enum, auto
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration as DurationMsg

from nav2_msgs.action import NavigateToPose

import tf2_ros


FREE = 0
UNKNOWN = -1
OCCUPIED_THRESH = 50


class ExplorationState(Enum):
    IDLE = auto()
    SCANNING = auto()
    NAVIGATING = auto()
    SETTLING = auto()
    COMPLETE = auto()


class FrontierExplorerNode(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        cb_group = ReentrantCallbackGroup()

        self.declare_parameter('enabled', False)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('rate_hz', 0.5)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('blacklist_radius', 1.0)
        self.declare_parameter('min_goal_distance', 0.6)
        self.declare_parameter('settle_time_s', 3.0)

        self._enabled = self.get_parameter('enabled').value
        map_topic = self.get_parameter('map_topic').value or '/map'
        self._min_frontier = int(self.get_parameter('min_frontier_size').value or 8)
        rate = float(self.get_parameter('rate_hz').value or 0.5)
        self._goal_tol = float(self.get_parameter('goal_tolerance').value or 0.5)
        self._blacklist_radius = float(
            self.get_parameter('blacklist_radius').value or 1.0
        )
        self._min_goal_dist = float(
            self.get_parameter('min_goal_distance').value or 0.6
        )
        self._settle_time = float(
            self.get_parameter('settle_time_s').value or 3.0
        )

        self._state = ExplorationState.IDLE
        self._latest_map = None
        self._blacklisted_goals = []
        self._current_goal = None
        self._current_goal_handle = None
        self._settle_start = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_cb, 5
        )

        self._goals_pub = self.create_publisher(PoseArray, '/frontier_goals', 10)
        self._markers_pub = self.create_publisher(
            MarkerArray, '/frontier_markers', 10
        )
        self._status_pub = self.create_publisher(String, '/exploration_status', 10)

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=cb_group,
        )

        self._enable_srv = self.create_service(
            SetBool, '~/enable', self._enable_cb
        )

        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'frontier_explorer: enabled={self._enabled}, '
            f'min_frontier={self._min_frontier}, rate={rate} Hz, '
            f'min_goal_dist={self._min_goal_dist}m, settle={self._settle_time}s'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _enable_cb(self, request, response):
        self._enabled = request.data
        if self._enabled:
            self._state = ExplorationState.SCANNING
            self._blacklisted_goals.clear()
            response.message = 'Exploration enabled'
        else:
            self._cancel_nav_goal()
            self._state = ExplorationState.IDLE
            response.message = 'Exploration disabled'
        self.get_logger().info(response.message)
        response.success = True
        return response

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def _tick(self):
        self._publish_status()

        if not self._enabled:
            return

        if self._latest_map is None:
            return

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            return

        if self._state == ExplorationState.IDLE:
            self._state = ExplorationState.SCANNING

        if self._state == ExplorationState.NAVIGATING:
            if self._current_goal is not None:
                dx = robot_xy[0] - self._current_goal[0]
                dy = robot_xy[1] - self._current_goal[1]
                if math.hypot(dx, dy) < self._goal_tol:
                    self.get_logger().info('Reached frontier goal, settling...')
                    self._state = ExplorationState.SETTLING
                    self._settle_start = self.get_clock().now()

        if self._state == ExplorationState.SETTLING:
            if self._settle_start is None:
                self._settle_start = self.get_clock().now()
            elapsed = (self.get_clock().now() - self._settle_start).nanoseconds / 1e9
            if elapsed >= self._settle_time:
                self.get_logger().info('Settle complete, re-scanning.')
                self._state = ExplorationState.SCANNING

        if self._state == ExplorationState.SCANNING:
            frontiers = self._detect_frontiers(robot_xy)
            self._publish_frontier_markers(frontiers, robot_xy)

            if not frontiers:
                self.get_logger().info('No frontiers remaining -- exploration complete.')
                self._state = ExplorationState.COMPLETE
                return

            best = self._select_frontier(frontiers, robot_xy)
            if best is None:
                self.get_logger().warn('All frontiers blacklisted; clearing blacklist.')
                self._blacklisted_goals.clear()
                return

            self._send_nav_goal(best)
            self._current_goal = best
            self._state = ExplorationState.NAVIGATING

    # ------------------------------------------------------------------
    # WFD core (arXiv:1806.03581 Algorithm 1)
    # ------------------------------------------------------------------

    def _detect_frontiers(self, robot_xy):
        grid = self._latest_map
        info = grid.info
        data = np.array(grid.data, dtype=np.int8).reshape(
            (info.height, info.width)
        )
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        rc = int((robot_xy[1] - oy) / res)
        cc = int((robot_xy[0] - ox) / res)
        rc = max(0, min(rc, info.height - 1))
        cc = max(0, min(cc, info.width - 1))

        visited = np.zeros_like(data, dtype=np.bool_)
        frontier_flags = np.zeros_like(data, dtype=np.bool_)

        queue = deque()
        queue.append((rc, cc))
        visited[rc, cc] = True

        while queue:
            r, c = queue.popleft()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if nr < 0 or nr >= info.height or nc < 0 or nc >= info.width:
                        continue
                    if visited[nr, nc]:
                        continue
                    val = int(data[nr, nc])
                    if val == FREE:
                        visited[nr, nc] = True
                        queue.append((nr, nc))
                    elif val == UNKNOWN:
                        if not frontier_flags[r, c] and int(data[r, c]) == FREE:
                            frontier_flags[r, c] = True

        clustered = np.zeros_like(data, dtype=np.bool_)
        frontiers = []
        rows, cols = np.where(frontier_flags)

        for idx in range(len(rows)):
            r0, c0 = int(rows[idx]), int(cols[idx])
            if clustered[r0, c0]:
                continue
            cluster = []
            cq = deque()
            cq.append((r0, c0))
            clustered[r0, c0] = True
            while cq:
                r, c = cq.popleft()
                cluster.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < info.height and 0 <= nc < info.width:
                            if frontier_flags[nr, nc] and not clustered[nr, nc]:
                                clustered[nr, nc] = True
                                cq.append((nr, nc))

            if len(cluster) >= self._min_frontier:
                cy = np.mean([p[0] for p in cluster]) * res + oy
                cx = np.mean([p[1] for p in cluster]) * res + ox
                frontiers.append((cx, cy, len(cluster)))

        return frontiers

    def _select_frontier(self, frontiers, robot_xy):
        """Pick the best frontier balancing proximity and information gain.

        Uses a weighted score:  score = size_weight * size - dist_weight * distance
        This avoids zig-zagging to tiny nearby frontiers while still preferring
        reachable targets. Frontiers closer than min_goal_distance are skipped
        to prevent the robot from chasing trivially small movements.
        """
        best = None
        best_score = -float('inf')
        for fx, fy, size in frontiers:
            if self._is_blacklisted(fx, fy):
                continue
            d = math.hypot(fx - robot_xy[0], fy - robot_xy[1])
            if d < self._min_goal_dist:
                continue
            score = 0.4 * size - 1.0 * d
            if score > best_score:
                best_score = score
                best = (fx, fy)
        return best

    def _is_blacklisted(self, x, y):
        for bx, by in self._blacklisted_goals:
            if math.hypot(x - bx, y - by) < self._blacklist_radius:
                return True
        return False

    # ------------------------------------------------------------------
    # Nav2 integration
    # ------------------------------------------------------------------

    def _send_nav_goal(self, xy):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = xy[0]
        goal.pose.pose.position.y = xy[1]
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending frontier goal: ({xy[0]:.2f}, {xy[1]:.2f})'
        )
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Frontier goal rejected by Nav2')
            if self._current_goal:
                self._blacklisted_goals.append(self._current_goal)
            self._current_goal_handle = None
            self._state = ExplorationState.SCANNING
            return
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        self._current_goal_handle = None
        try:
            result = future.result()
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info('Frontier goal reached')
            else:
                self.get_logger().warn(
                    f'Frontier goal ended with status {result.status}'
                )
                if self._current_goal:
                    self._blacklisted_goals.append(self._current_goal)
        except Exception as e:
            self.get_logger().warn(f'Frontier goal result error: {e}')
            if self._current_goal:
                self._blacklisted_goals.append(self._current_goal)
        self._state = ExplorationState.SETTLING
        self._settle_start = self.get_clock().now()

    def _cancel_nav_goal(self):
        if self._current_goal_handle is not None:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._current_goal_handle = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_robot_xy(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _publish_status(self):
        msg = String()
        if not self._enabled:
            msg.data = 'disabled'
        elif self._state == ExplorationState.COMPLETE:
            msg.data = 'no_frontiers_remaining'
        elif self._state == ExplorationState.NAVIGATING:
            msg.data = 'exploring'
        elif self._state == ExplorationState.SETTLING:
            msg.data = 'settling'
        elif self._state == ExplorationState.SCANNING:
            msg.data = 'scanning'
        else:
            msg.data = 'idle'
        self._status_pub.publish(msg)

    def _publish_frontier_markers(self, frontiers, robot_xy):
        ma = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        for i, (fx, fy, size) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontier_centroids'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = fx
            m.pose.position.y = fy
            m.pose.position.z = 0.1
            scale = min(0.8, max(0.2, size / 20.0))
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.8
            m.color.a = 0.7
            m.lifetime = DurationMsg(sec=5, nanosec=0)
            ma.markers.append(m)

        if self._current_goal:
            g = Marker()
            g.header.frame_id = 'map'
            g.header.stamp = self.get_clock().now().to_msg()
            g.ns = 'current_frontier_goal'
            g.id = 0
            g.type = Marker.ARROW
            g.action = Marker.ADD
            g.pose.position.x = self._current_goal[0]
            g.pose.position.y = self._current_goal[1]
            g.pose.position.z = 0.5
            g.scale.x = 0.6
            g.scale.y = 0.15
            g.scale.z = 0.15
            g.color.r = 0.0
            g.color.g = 1.0
            g.color.b = 0.0
            g.color.a = 0.9
            g.lifetime = DurationMsg(sec=5, nanosec=0)
            ma.markers.append(g)

        goals_msg = PoseArray()
        goals_msg.header.frame_id = 'map'
        goals_msg.header.stamp = self.get_clock().now().to_msg()
        for fx, fy, _ in frontiers:
            p = Pose()
            p.position.x = fx
            p.position.y = fy
            p.orientation.w = 1.0
            goals_msg.poses.append(p)
        self._goals_pub.publish(goals_msg)
        self._markers_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
