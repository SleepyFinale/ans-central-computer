#!/usr/bin/env python3
"""
Multi-robot frontier exploration coordinator.

Subscribes to the merged global map, detects frontiers, and assigns
exploration waypoints to multiple robots via their Nav2 action servers.

Each planning cycle:
  1. Detect frontier regions on the merged occupancy grid.
  2. For every idle robot, pick the best unassigned frontier using a
     utility function that balances proximity vs information gain.
  3. Send a NavigateToPose goal in the world frame; Nav2 uses TF to
     convert it into the robot's local frame.

Handles the no-overlap case transparently: the merged map already
contains all robot maps (placed side-by-side by map_merge), so each
robot sees frontiers on its own portion and explores independently
until map overlap is detected.
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class Frontier:
    centroid_world: Tuple[float, float]  # (x, y) in world frame (metres)
    size: int                            # number of frontier cells
    size_m: float                        # size * resolution (metres)
    cells: int = 0                       # alias for size


@dataclass
class RobotState:
    name: str
    action_client: ActionClient = None
    goal_handle: object = None
    goal_position: Optional[Tuple[float, float]] = None
    last_goal_time: float = 0.0
    goal_active: bool = False
    position: Optional[Tuple[float, float]] = None  # current (x,y) in world
    goals_reached: int = 0
    goals_failed: int = 0
    blacklist: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# Frontier detection
# ---------------------------------------------------------------------------

FREE = 0
UNKNOWN = -1
OCCUPIED_THRESH = 50  # cells >= this are considered occupied


def _label_frontiers_8c(mask: np.ndarray):
    """Connected-component labelling for frontier regions (8-connectivity).

    Returns (labelled_array, n_labels) similar to scipy.ndimage.label.
    """
    h, w = mask.shape
    labels = np.zeros_like(mask, dtype=np.int32)
    current = 0

    neighbours = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1),
    ]

    for y in range(h):
        for x in range(w):
            if not mask[y, x] or labels[y, x] != 0:
                continue

            current += 1
            labels[y, x] = current
            q = deque([(y, x)])

            while q:
                cy, cx = q.popleft()
                for dy, dx in neighbours:
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < h and 0 <= nx < w:
                        if mask[ny, nx] and labels[ny, nx] == 0:
                            labels[ny, nx] = current
                            q.append((ny, nx))

    return labels, current


def detect_frontiers(
    map_data: List[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    min_size_m: float,
) -> List[Frontier]:
    """Find frontier regions on an OccupancyGrid.

    A *frontier cell* is a free cell (value 0) that has at least one
    unknown neighbour (value -1) in 4-connectivity.  Adjacent frontier
    cells (8-connectivity) are clustered into frontier regions.
    """
    grid = np.array(map_data, dtype=np.int8).reshape((height, width))

    free = grid == FREE
    unknown = grid == UNKNOWN

    # frontier = free cells with at least one unknown 4-neighbour
    padded_unk = np.pad(unknown, 1, constant_values=False)
    has_unk_neighbour = (
        padded_unk[:-2, 1:-1]  # above
        | padded_unk[2:, 1:-1]  # below
        | padded_unk[1:-1, :-2]  # left
        | padded_unk[1:-1, 2:]   # right
    )
    frontier_mask = free & has_unk_neighbour

    if not np.any(frontier_mask):
        return []

    # cluster with 8-connectivity (pure NumPy implementation)
    labelled, n_labels = _label_frontiers_8c(frontier_mask)

    min_cells = max(1, int(min_size_m / resolution))
    frontiers: List[Frontier] = []
    for label_id in range(1, n_labels + 1):
        ys, xs = np.where(labelled == label_id)
        n = len(ys)
        if n < min_cells:
            continue
        cy = float(np.mean(ys))
        cx = float(np.mean(xs))
        wx = origin_x + (cx + 0.5) * resolution
        wy = origin_y + (cy + 0.5) * resolution
        frontiers.append(Frontier(
            centroid_world=(wx, wy),
            size=n,
            size_m=n * resolution,
            cells=n,
        ))

    return frontiers


# ---------------------------------------------------------------------------
# Assignment
# ---------------------------------------------------------------------------

def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def assign_frontiers(
    robots: Dict[str, RobotState],
    frontiers: List[Frontier],
    potential_scale: float,
    gain_scale: float,
    nearby_penalty_dist: float = 2.0,
) -> Dict[str, int]:
    """Assign one frontier to each idle robot.

    Returns {robot_name: frontier_index}.

    Utility for (robot, frontier):
        gain_scale * frontier.size_m  -  potential_scale * distance
        minus a penalty if another robot already targets a nearby frontier.
    """
    # robots that need a new goal
    idle_robots = {
        name: rs for name, rs in robots.items()
        if not rs.goal_active and rs.position is not None
    }
    if not idle_robots or not frontiers:
        return {}

    # already-assigned goal positions (from robots that ARE active)
    active_goals = [
        rs.goal_position
        for rs in robots.values()
        if rs.goal_active and rs.goal_position is not None
    ]

    # build utility matrix: idle_robots × frontiers
    robot_list = list(idle_robots.keys())
    utilities = np.full((len(robot_list), len(frontiers)), -np.inf)

    for ri, rname in enumerate(robot_list):
        rpos = idle_robots[rname].position
        blacklist = idle_robots[rname].blacklist
        for fi, fr in enumerate(frontiers):
            # skip blacklisted centroids
            if any(_dist(fr.centroid_world, bl) < 0.5 for bl in blacklist):
                continue

            dist = _dist(rpos, fr.centroid_world)
            gain = gain_scale * fr.size_m
            cost = potential_scale * dist
            penalty = 0.0
            for ag in active_goals:
                ad = _dist(fr.centroid_world, ag)
                if ad < nearby_penalty_dist:
                    penalty += (nearby_penalty_dist - ad) * 2.0
            utilities[ri, fi] = gain - cost - penalty

    # greedy assignment: highest utility first, no double-assignment
    assignments: Dict[str, int] = {}
    assigned_f = set()
    assigned_r = set()

    flat = np.argsort(utilities.ravel())[::-1]
    for idx in flat:
        ri = int(idx // len(frontiers))
        fi = int(idx % len(frontiers))
        if ri in assigned_r or fi in assigned_f:
            continue
        if utilities[ri, fi] <= -1e9:
            continue
        assignments[robot_list[ri]] = fi
        assigned_r.add(ri)
        assigned_f.add(fi)
        if len(assigned_r) == len(robot_list):
            break

    return assignments


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class MultiRobotExplorer(Node):

    def __init__(self):
        super().__init__('multi_robot_explorer')

        # -- parameters --
        self.declare_parameter('robot_names', ['blinky', 'pinky'])
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('explore_frequency', 0.33)
        self.declare_parameter('min_frontier_size', 0.15)
        self.declare_parameter('potential_scale', 3.0)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('progress_timeout', 30.0)
        self.declare_parameter('nearby_penalty_dist', 2.0)
        self.declare_parameter('visualize', True)

        self.robot_names: List[str] = (
            self.get_parameter('robot_names').value)
        map_topic = self.get_parameter('map_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        freq = self.get_parameter('explore_frequency').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.potential_scale = self.get_parameter('potential_scale').value
        self.gain_scale = self.get_parameter('gain_scale').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.nearby_penalty_dist = (
            self.get_parameter('nearby_penalty_dist').value)
        self.visualize = self.get_parameter('visualize').value

        # -- state --
        self.robots: Dict[str, RobotState] = {}
        self.latest_map: Optional[OccupancyGrid] = None
        self.exploration_complete = False
        self.goal_pubs: Dict[str, object] = {}

        # -- TF --
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -- map subscription --
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, map_qos)

        # -- visualisation --
        if self.visualize:
            self.marker_pub = self.create_publisher(
                MarkerArray, 'explore/frontiers', 10)

        # -- per-robot setup --
        for name in self.robot_names:
            rs = RobotState(name=name)
            self.robots[name] = rs
            # Publish PoseStamped goals on /<robot>/goal_pose.
            # These are bridged to /goal_pose on each robot's domain.
            self.goal_pubs[name] = self.create_publisher(
                PoseStamped, f'/{name}/goal_pose', 10)

        self._logged_waiting_for_map = False

        # -- planning timer --
        period = 1.0 / freq if freq > 0 else 3.0
        self.plan_timer = self.create_timer(period, self._plan_tick)

        self.get_logger().info(
            f'Multi-robot explorer started: robots={self.robot_names}, '
            f'map_topic={map_topic}, world_frame={self.world_frame}, '
            f'freq={freq:.2f} Hz')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    # -----------------------------------------------------------------------
    # Main planning loop
    # -----------------------------------------------------------------------

    def _plan_tick(self):
        if self.latest_map is None:
            if not self._logged_waiting_for_map:
                self.get_logger().info(
                    'Waiting for merged map on configured topic...')
                self._logged_waiting_for_map = True
            return

        # update robot positions from TF
        self._update_robot_positions()

        # check for stale goals (progress timeout)
        now = self.get_clock().now().nanoseconds / 1e9
        for rs in self.robots.values():
            if (rs.goal_active and rs.last_goal_time > 0
                    and (now - rs.last_goal_time) > self.progress_timeout):
                self.get_logger().warn(
                    f'[{rs.name}] Goal timed out after '
                    f'{self.progress_timeout:.0f}s, cancelling')
                self._cancel_goal(rs)
                if rs.goal_position:
                    rs.blacklist.append(rs.goal_position)

        # detect frontiers
        m = self.latest_map
        frontiers = detect_frontiers(
            m.data, m.info.width, m.info.height,
            m.info.resolution,
            m.info.origin.position.x,
            m.info.origin.position.y,
            self.min_frontier_size,
        )

        if not frontiers:
            if not self.exploration_complete:
                self.get_logger().info(
                    'No frontiers remaining — exploration complete!')
                self.exploration_complete = True
            return

        if self.exploration_complete:
            self.get_logger().info('New frontiers appeared — resuming exploration')
        self.exploration_complete = False

        n_idle = sum(
            1 for rs in self.robots.values()
            if not rs.goal_active and rs.position is not None)
        self.get_logger().debug(
            f'{len(frontiers)} frontiers, {n_idle} idle robot(s)')

        # publish visualisation
        if self.visualize:
            self._publish_frontier_markers(frontiers)

        # assign frontiers to idle robots
        assignments = assign_frontiers(
            self.robots, frontiers,
            self.potential_scale, self.gain_scale,
            self.nearby_penalty_dist,
        )

        for rname, fi in assignments.items():
            fr = frontiers[fi]
            self._send_goal(self.robots[rname], fr)

    # -----------------------------------------------------------------------
    # TF helpers
    # -----------------------------------------------------------------------

    def _update_robot_positions(self):
        for rs in self.robots.values():
            base_frame = f'{rs.name}/base_footprint'
            try:
                t = self.tf_buffer.lookup_transform(
                    self.world_frame, base_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                rs.position = (
                    t.transform.translation.x,
                    t.transform.translation.y,
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass  # keep last known position

    # -----------------------------------------------------------------------
    # Nav2 goal management
    # -----------------------------------------------------------------------

    def _send_goal(self, rs: RobotState, frontier: Frontier):
        goal_pub = self.goal_pubs.get(rs.name)
        if goal_pub is None:
            self.get_logger().warn(f'[{rs.name}] No goal publisher available')
            rs.goal_active = False
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.world_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = frontier.centroid_world[0]
        goal_msg.pose.position.y = frontier.centroid_world[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.get_logger().info(
            f'[{rs.name}] Publishing goal_pose ({frontier.centroid_world[0]:.2f}, '
            f'{frontier.centroid_world[1]:.2f}) — frontier size '
            f'{frontier.size_m:.2f}m ({frontier.size} cells)')

        goal_pub.publish(goal_msg)

        rs.goal_position = frontier.centroid_world
        rs.last_goal_time = self.get_clock().now().nanoseconds / 1e9
        rs.goal_active = True

    def _goal_response_callback(self, future, rs: RobotState):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{rs.name}] Goal rejected by Nav2')
            rs.goal_active = False
            return

        rs.goal_handle = goal_handle
        self.get_logger().debug(f'[{rs.name}] Goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, r=rs: self._goal_result_callback(f, r))

    def _goal_result_callback(self, future, rs: RobotState):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            rs.goals_reached += 1
            self.get_logger().info(
                f'[{rs.name}] Goal reached '
                f'(total: {rs.goals_reached})')
        elif status == GoalStatus.STATUS_ABORTED:
            rs.goals_failed += 1
            self.get_logger().warn(
                f'[{rs.name}] Goal aborted — blacklisting')
            if rs.goal_position:
                rs.blacklist.append(rs.goal_position)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{rs.name}] Goal cancelled')
        else:
            self.get_logger().info(
                f'[{rs.name}] Goal finished with status {status}')

        rs.goal_active = False
        rs.goal_handle = None

    def _cancel_goal(self, rs: RobotState):
        if rs.goal_handle is not None:
            try:
                rs.goal_handle.cancel_goal_async()
            except Exception:
                pass
        rs.goal_active = False
        rs.goal_handle = None

    # -----------------------------------------------------------------------
    # Visualisation
    # -----------------------------------------------------------------------

    def _publish_frontier_markers(self, frontiers: List[Frontier]):
        ma = MarkerArray()

        # delete old markers
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        del_marker.header.frame_id = self.world_frame
        del_marker.ns = 'frontiers'
        ma.markers.append(del_marker)

        for i, fr in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = fr.centroid_world[0]
            m.pose.position.y = fr.centroid_world[1]
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            scale = max(0.15, min(0.6, fr.size_m * 0.3))
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale * 0.5
            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 0.2
            m.color.a = 0.8
            m.lifetime.sec = 5
            ma.markers.append(m)

        # mark assigned goals in a different colour
        idx = len(frontiers) + 2
        for rs in self.robots.values():
            if rs.goal_active and rs.goal_position:
                gm = Marker()
                gm.header.frame_id = self.world_frame
                gm.header.stamp = self.get_clock().now().to_msg()
                gm.ns = 'goals'
                gm.id = idx
                idx += 1
                gm.type = Marker.ARROW
                gm.action = Marker.ADD
                gm.pose.position.x = rs.goal_position[0]
                gm.pose.position.y = rs.goal_position[1]
                gm.pose.position.z = 0.3
                gm.pose.orientation.w = 0.707
                gm.pose.orientation.x = 0.0
                gm.pose.orientation.y = 0.707
                gm.pose.orientation.z = 0.0
                gm.scale.x = 0.4
                gm.scale.y = 0.12
                gm.scale.z = 0.12
                gm.color.r = 1.0
                gm.color.g = 0.3
                gm.color.b = 0.0
                gm.color.a = 1.0
                gm.lifetime.sec = 5
                ma.markers.append(gm)

        self.marker_pub.publish(ma)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # cancel all active goals
        for rs in node.robots.values():
            if rs.goal_active:
                node._cancel_goal(rs)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
