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
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException

from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String as StringMsg
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from central_explorer_event_logger import ExplorerEventLogger


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class Frontier:
    centroid_world: Tuple[float, float]  # (x, y) in world frame (metres)
    size: int                            # number of frontier cells
    size_m: float                        # size * resolution (metres)
    cells: int = 0                       # alias for size
    indices: Optional[np.ndarray] = None  # (N, 2) array of (y, x) cell indices


@dataclass
class RobotState:
    name: str
    action_client: ActionClient = None
    goal_handle: object = None
    goal_position: Optional[Tuple[float, float]] = None
    last_goal_time: float = 0.0
    goal_active: bool = False
    goal_pending: bool = False
    goal_status: str = 'none'
    position: Optional[Tuple[float, float]] = None  # current (x,y) in world
    goals_reached: int = 0
    goals_failed: int = 0
    blacklist: list = field(default_factory=list)
    server_unavailable_logged: bool = False
    last_distance_remaining: Optional[float] = None
    last_progress_distance: Optional[float] = None
    last_progress_time: float = 0.0
    active_goal_sent_time: float = 0.0
    last_pose_when_goal_sent: Optional[Tuple[float, float]] = None
    initial_position: Optional[Tuple[float, float]] = None
    initial_orientation_w: float = 1.0
    returning_home: bool = False
    # Track behaviour around repeated / stagnant goals. last_goal_world stores
    # the most recent goal position in world coordinates (x, y).
    last_goal_world: Optional[Tuple[float, float]] = None
    repeat_goal_count: int = 0
    # Short history of recent goals and associated frontier sizes for
    # stagnation detection. Each entry is (x, y, frontier_size_m).
    recent_goals: deque = field(default_factory=lambda: deque(maxlen=10))
    last_retarget_time: float = 0.0


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

    # Interpret min_size_m as a characteristic linear size and require at
    # least the corresponding area in cells. This better matches the
    # cluster-size semantics used in costmap-based frontier search.
    min_cells = max(1, int((min_size_m / resolution) ** 2))
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
        indices = np.stack((ys, xs), axis=1)
        frontiers.append(Frontier(
            centroid_world=(wx, wy),
            size=n,
            size_m=n * resolution,
            cells=n,
            indices=indices,
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
    min_goal_separation: float = 0.25,
    blacklist_radius: float = 0.5,
    utility_penalty_fn: Optional[Callable[[str, Frontier], float]] = None,
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
            if any(_dist(fr.centroid_world, bl) < blacklist_radius for bl in blacklist):
                continue

            dist = _dist(rpos, fr.centroid_world)
            # avoid goals that are effectively already within Nav2's goal
            # tolerance around the robot; these lead to immediate "success"
            # with no real motion
            if dist < min_goal_separation:
                continue
            gain = gain_scale * fr.size_m
            cost = potential_scale * dist
            penalty = 0.0
            for ag in active_goals:
                ad = _dist(fr.centroid_world, ag)
                if ad < nearby_penalty_dist:
                    penalty += (nearby_penalty_dist - ad) * 2.0
            if utility_penalty_fn is not None:
                penalty += float(utility_penalty_fn(rname, fr))
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
        # New parameters to control local-vs-global behaviour.
        # mode:
        #   - 'auto'        : start in local-per-robot mode and switch to global
        #                     once the merge_state reports MERGED.
        #   - 'local_only'  : always use per-robot local maps.
        #   - 'global_only' : always use the merged global map (back-compat).
        self.declare_parameter('mode', 'auto')
        # Topic where a helper node (or map_merge) can publish a simple
        # merge state string: NO_OVERLAP, PARTIAL, MERGED.
        self.declare_parameter('merge_state_topic', 'map_merge/merge_state')
        # Match m-explore-ros2 planner_frequency semantics; default tuned in YAML.
        self.declare_parameter('explore_frequency', 1.0)
        self.declare_parameter('min_frontier_size', 0.15)
        self.declare_parameter('potential_scale', 3.0)
        self.declare_parameter('gain_scale', 1.0)
        # Progress timeout is interpreted as "no meaningful distance progress"
        # for this many seconds before a goal is considered stalled.
        self.declare_parameter('progress_timeout', 45.0)
        self.declare_parameter('progress_min_delta', 0.05)
        self.declare_parameter('nearby_penalty_dist', 2.0)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('blacklist_clear_radius', 0.5)
        self.declare_parameter('visualize', True)
        self.declare_parameter('use_pose_goal_fallback', True)
        self.declare_parameter('single_robot_offloaded_nav2', False)
        # Minimum separation between robot and candidate goal used as a final
        # safety net; most shaping is done via min_goal_distance.
        self.declare_parameter('min_goal_separation', 0.5)
        self.declare_parameter('suspicious_success_distance', 0.15)
        # For very small initial maps, Nav2 can legitimately report success
        # with a relatively large distance_remaining because the frontier
        # centroid may still be outside the tiny explored region. To avoid
        # blacklisting useful early frontiers, the "suspicious success"
        # heuristic is only applied once the map has grown beyond a
        # configurable size.
        self.declare_parameter('strict_success_min_map_size', 3.0)
        # Optional return-to-origin behaviour per robot, similar to explore_node.
        self.declare_parameter('return_to_init', False)
        self.declare_parameter('robot_base_frame', 'base_footprint')
        # Status + control topics (multi-robot aware counterpart of explore/status, explore/resume).
        self.declare_parameter('status_topic', 'explore_multi/status')
        self.declare_parameter('control_topic', 'explore_multi/resume')
        # Goal selection strategy within each frontier region.
        # - 'centroid': use the geometric centroid of the frontier region
        # - 'farthest_cell': choose the frontier cell farthest from the robot
        #   (in world distance), falling back to centroid when needed.
        self.declare_parameter('goal_point_strategy', 'farthest_cell')
        # Minimum desired distance from robot to goal (metres). This is used
        # when selecting a representative cell inside a frontier region so
        # that the initial goals are not almost inside Nav2's goal tolerance.
        self.declare_parameter('min_goal_distance', 0.6)
        # Goal-clearance shaping: discourage assigning goals too close to
        # occupied cells (likely walls/obstacles in the current map).
        self.declare_parameter('goal_clearance_radius_m', 0.45)
        self.declare_parameter('goal_clearance_weight', 4.0)
        # Hard safety gate for candidate goal cells:
        # - reject goals in high-cost cells
        # - reject goals that do not have enough local obstacle clearance
        self.declare_parameter('goal_max_cell_cost', 80)
        self.declare_parameter('goal_min_clearance_gate_m', 0.30)
        # Mid-route retargeting controls.
        self.declare_parameter('retarget_enable', True)
        self.declare_parameter('retarget_cooldown_sec', 12.0)
        self.declare_parameter('retarget_min_goal_shift_m', 0.5)
        self.declare_parameter('retarget_min_utility_gain', 0.8)
        self.declare_parameter('retarget_stagnation_sec', 20.0)
        self.declare_parameter('retarget_clearance_threshold_m', 0.25)
        # Goal replacement gate controls to avoid rapid preempt/replan loops.
        self.declare_parameter('min_goal_replan_interval_s', 4.0)
        self.declare_parameter('min_goal_change_dist_m', 0.75)
        self.declare_parameter('min_progress_before_replan_m', 0.25)
        self.declare_parameter('max_stuck_time_s', 12.0)
        self.declare_parameter('allow_replan_when_no_progress', True)
        # Per-robot local map topic pattern. By default we assume the standard
        # namespaced SLAM layout '/<robot>/map'.
        self.declare_parameter('local_map_topic_pattern', '/{robot}/map')

        self.robot_names: List[str] = (
            self.get_parameter('robot_names').value)
        map_topic = self.get_parameter('map_topic').value
        self.map_topic = map_topic
        self.world_frame = self.get_parameter('world_frame').value
        self.mode: str = (
            self.get_parameter('mode').value or 'auto'
        ).lower()
        self.merge_state_topic: str = (
            self.get_parameter('merge_state_topic').value
        )
        freq = self.get_parameter('explore_frequency').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.potential_scale = self.get_parameter('potential_scale').value
        self.gain_scale = self.get_parameter('gain_scale').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.progress_min_delta = (
            self.get_parameter('progress_min_delta').value)
        self.nearby_penalty_dist = (
            self.get_parameter('nearby_penalty_dist').value)
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.blacklist_clear_radius = (
            self.get_parameter('blacklist_clear_radius').value)
        self.visualize = self.get_parameter('visualize').value
        self.use_pose_goal_fallback = (
            self.get_parameter('use_pose_goal_fallback').value)
        self.single_robot_offloaded_nav2 = (
            self.get_parameter('single_robot_offloaded_nav2').value)
        self.min_goal_separation = (
            self.get_parameter('min_goal_separation').value)
        self.suspicious_success_distance = (
            self.get_parameter('suspicious_success_distance').value)
        self.strict_success_min_map_size = (
            self.get_parameter('strict_success_min_map_size').value)
        self.return_to_init = self.get_parameter('return_to_init').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.status_topic = self.get_parameter('status_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.goal_point_strategy = (
            self.get_parameter('goal_point_strategy').value or 'farthest_cell'
        )
        self.min_goal_distance = float(
            self.get_parameter('min_goal_distance').value
        )
        self.goal_clearance_radius_m = float(
            self.get_parameter('goal_clearance_radius_m').value
        )
        self.goal_clearance_weight = float(
            self.get_parameter('goal_clearance_weight').value
        )
        self.goal_max_cell_cost = int(
            self.get_parameter('goal_max_cell_cost').value
        )
        self.goal_min_clearance_gate_m = float(
            self.get_parameter('goal_min_clearance_gate_m').value
        )
        self.retarget_enable = bool(
            self.get_parameter('retarget_enable').value
        )
        self.retarget_cooldown_sec = float(
            self.get_parameter('retarget_cooldown_sec').value
        )
        self.retarget_min_goal_shift_m = float(
            self.get_parameter('retarget_min_goal_shift_m').value
        )
        self.retarget_min_utility_gain = float(
            self.get_parameter('retarget_min_utility_gain').value
        )
        self.retarget_stagnation_sec = float(
            self.get_parameter('retarget_stagnation_sec').value
        )
        self.retarget_clearance_threshold_m = float(
            self.get_parameter('retarget_clearance_threshold_m').value
        )
        self.min_goal_replan_interval_s = float(
            self.get_parameter('min_goal_replan_interval_s').value
        )
        self.min_goal_change_dist_m = float(
            self.get_parameter('min_goal_change_dist_m').value
        )
        self.min_progress_before_replan_m = float(
            self.get_parameter('min_progress_before_replan_m').value
        )
        self.max_stuck_time_s = float(
            self.get_parameter('max_stuck_time_s').value
        )
        self.allow_replan_when_no_progress = bool(
            self.get_parameter('allow_replan_when_no_progress').value
        )
        self.local_map_topic_pattern: str = (
            self.get_parameter('local_map_topic_pattern').value
        )

        # -- state --
        self.robots: Dict[str, RobotState] = {}
        # Global merged map (from map_merge in multi-robot mode, or a single
        # robot's map in single-robot mode).
        self.latest_global_map: Optional[OccupancyGrid] = None
        # Per-robot local maps. In many setups these are the same as the maps
        # consumed by map_merge (/<robot>/map).
        self.latest_local_maps: Dict[str, OccupancyGrid] = {}
        # Map message currently associated with goal selection; set by the
        # frontier-step helpers before assigning goals so _select_goal_point
        # can convert frontier cells into world coordinates without assuming
        # a single global map source.
        self._current_goal_map: Optional[OccupancyGrid] = None
        self._map_size_x: float = 0.0
        self._map_size_y: float = 0.0
        self.exploration_complete = False
        self.goal_pubs: Dict[str, object] = {}
        self.paused: bool = False
        self.in_global_phase: bool = False
        self._merge_state_last: str = 'NO_OVERLAP'
        self.event_logger = ExplorerEventLogger()

        # -- status publishing & control --
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.resume_sub = self.create_subscription(
            Bool,
            self.control_topic,
            self._resume_callback,
            10,
        )

        # -- TF --
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -- global map subscription --
        map_qos_transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_sub_transient = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self._global_map_callback,
            map_qos_transient,
        )

        map_qos_compatible = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.map_sub_compatible = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self._global_map_callback,
            map_qos_compatible,
        )

        # -- per-robot local map subscriptions --
        self._local_map_subs = []
        for name in self.robot_names:
            topic = self.local_map_topic_pattern.format(robot=name)
            sub_t = self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, r=name: self._local_map_callback(msg, r),
                map_qos_transient,
            )
            sub_c = self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, r=name: self._local_map_callback(msg, r),
                map_qos_compatible,
            )
            self._local_map_subs.append(sub_t)
            self._local_map_subs.append(sub_c)

        # -- merge state subscription (optional) --
        self.merge_state_sub = self.create_subscription(
            StringMsg,
            self.merge_state_topic,
            self._merge_state_callback,
            10,
        )

        # -- visualisation --
        if self.visualize:
            self.marker_pub = self.create_publisher(
                MarkerArray, 'explore/frontiers', 10)

        # -- per-robot setup --
        for name in self.robot_names:
            rs = RobotState(name=name)
            rs.action_client = ActionClient(
                self, NavigateToPose, f'/{name}/navigate_to_pose')
            self.robots[name] = rs
            if self.use_pose_goal_fallback:
                self.goal_pubs[name] = self.create_publisher(
                    PoseStamped, f'/{name}/goal_pose', 10)

        self._logged_waiting_for_map = False

        # -- planning timer --
        period = 1.0 / freq if freq > 0 else 3.0
        self.plan_timer = self.create_timer(period, self._plan_tick)

        mode = 'single_robot_offloaded_nav2' if self.single_robot_offloaded_nav2 else 'multi_robot'
        self.get_logger().info(
            f'Multi-robot explorer started: robots={self.robot_names}, '
            f'map_topic={map_topic}, world_frame={self.world_frame}, '
            f'freq={freq:.2f} Hz, '
            f'use_pose_goal_fallback={self.use_pose_goal_fallback}, '
            f'mode={mode}, retarget_enable={self.retarget_enable}')
        self._publish_status('STARTED')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _global_map_callback(self, msg: OccupancyGrid):
        self.latest_global_map = msg
        # Track current map physical size in metres for heuristics that behave
        # differently on tiny initial maps vs. larger, more mature maps.
        res = msg.info.resolution
        self._map_size_x = msg.info.width * res
        self._map_size_y = msg.info.height * res

    def _local_map_callback(self, msg: OccupancyGrid, robot: str):
        # Store the most recent local map per robot. These are only used in
        # local-per-robot mode.
        self.latest_local_maps[robot] = msg

    def _publish_status(self, state: str):
        msg = String()
        # Simple text state with per-robot summary; consumer tools can parse if desired.
        per_robot = ', '.join(
            f'{rs.name}:reached={rs.goals_reached},failed={rs.goals_failed},'
            f'goal_status={rs.goal_status},goal_active={rs.goal_active}'
            for rs in self.robots.values()
        )
        msg.data = f'state={state}; robots=[{per_robot}]'
        self.status_pub.publish(msg)

    def _resume_callback(self, msg: Bool):
        if msg.data:
            if self.paused:
                self.get_logger().info('Resume requested on control topic')
            self.paused = False
            self._publish_status('IN_PROGRESS')
        else:
            if not self.paused:
                self.get_logger().info('Pause requested on control topic — cancelling active goals')
                for rs in self.robots.values():
                    if rs.goal_active:
                        self._cancel_goal(rs)
            self.paused = True
            self._publish_status('PAUSED')

    # -----------------------------------------------------------------------
    # Main planning loop
    # -----------------------------------------------------------------------

    def _plan_tick(self):
        # Decide whether we should operate in global or local mode on this tick.
        use_global = False
        if self.mode == 'global_only':
            use_global = True
        elif self.mode == 'local_only':
            use_global = False
        else:
            # auto mode: follow merge_state flag
            use_global = self.in_global_phase

        if use_global and self.latest_global_map is None:
            if not self._logged_waiting_for_map:
                mode = (
                    'single_robot_offloaded_nav2'
                    if self.single_robot_offloaded_nav2
                    else 'multi_robot'
                )
                self.get_logger().info(
                    f'Waiting for map on {self.map_topic} (mode={mode})...')
                self._logged_waiting_for_map = True
                self._publish_status('WAITING_FOR_MAP')
            return
        if not use_global and not self.latest_local_maps:
            if not self._logged_waiting_for_map:
                self.get_logger().info(
                    'Waiting for per-robot local maps in local exploration mode...')
                self._logged_waiting_for_map = True
                self._publish_status('WAITING_FOR_MAP')
            return

        if self.paused:
            # Still keep TF and map up to date, but do not assign new goals.
            self._update_robot_positions()
            return

        # update robot positions from TF
        self._update_robot_positions()

        # check for stale goals (progress timeout / lack of distance progress)
        now = self.get_clock().now().nanoseconds / 1e9
        for rs in self.robots.values():
            if not rs.goal_active or rs.last_goal_time <= 0:
                continue

            # Prefer a notion of "time since last meaningful distance progress"
            # over pure wall-clock since goal send, mirroring explore_node.
            last_prog_time = rs.last_progress_time or rs.last_goal_time
            if (now - last_prog_time) > self.progress_timeout:
                self.get_logger().warn(
                    f'[{rs.name}] Goal appears stalled for '
                    f'{self.progress_timeout:.0f}s without distance progress; '
                    f'cancelling and blacklisting')
                self._cancel_goal(rs)
                if rs.goal_position:
                    rs.blacklist.append(rs.goal_position)

        if use_global:
            self._global_frontier_step()
        else:
            self._local_frontier_step()

    def _global_frontier_step(self):
        m = self.latest_global_map
        # Use the current global map for all goal selection in this tick.
        self._current_goal_map = m
        frontiers = detect_frontiers(
            m.data,
            m.info.width,
            m.info.height,
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
                if self.return_to_init:
                    for rs in self.robots.values():
                        if rs.initial_position and not rs.returning_home:
                            self._send_return_to_init(rs)
                    self._publish_status(
                        'RETURNING_TO_ORIGIN'
                        if any(
                            r.returning_home for r in self.robots.values()
                        )
                        else 'COMPLETE'
                    )
                else:
                    self._publish_status('COMPLETE')
            return

        if self.exploration_complete:
            self.get_logger().info('New frontiers appeared — resuming exploration')
        self.exploration_complete = False

        n_idle = sum(
            1 for rs in self.robots.values()
            if not rs.goal_active and rs.position is not None)
        self.get_logger().debug(
            f'{len(frontiers)} frontiers, {n_idle} idle robot(s)')
        if n_idle > 0:
            self._publish_status('IN_PROGRESS')

        # publish visualisation
        if self.visualize:
            self._publish_frontier_markers(frontiers)

        now = self.get_clock().now().nanoseconds / 1e9
        for rs in self.robots.values():
            self._maybe_retarget_active_robot(rs, frontiers, now)

        # assign frontiers to idle robots
        assignments = assign_frontiers(
            self.robots,
            frontiers,
            self.potential_scale,
            self.gain_scale,
            self.nearby_penalty_dist,
            self.min_goal_separation,
            self.blacklist_radius,
            utility_penalty_fn=lambda rname, fr: self._frontier_penalty(self.robots[rname], fr),
        )

        for rname, fi in assignments.items():
            fr = frontiers[fi]
            self._send_goal(self.robots[rname], fr)

    def _local_frontier_step(self):
        # Per-robot frontier detection: each robot plans only on its own local
        # map. This keeps exploration local while maps have not yet been
        # confidently merged.
        any_frontiers = False
        any_idle = False

        for name, rs in self.robots.items():
            if rs.position is None:
                continue
            lm = self.latest_local_maps.get(name)
            if lm is None:
                continue
            # Use this robot's local map for goal selection while we process
            # its frontiers in this tick.
            self._current_goal_map = lm
            self._map_size_x = lm.info.width * lm.info.resolution
            self._map_size_y = lm.info.height * lm.info.resolution
            frontiers = detect_frontiers(
                lm.data,
                lm.info.width,
                lm.info.height,
                lm.info.resolution,
                lm.info.origin.position.x,
                lm.info.origin.position.y,
                self.min_frontier_size,
            )
            if not frontiers:
                continue
            any_frontiers = True

            if self.visualize:
                self._publish_frontier_markers(frontiers)

            now = self.get_clock().now().nanoseconds / 1e9
            self._maybe_retarget_active_robot(rs, frontiers, now)

            if rs.goal_active:
                continue
            any_idle = True

            # Assign a frontier for this single robot using the same utility
            # function but restricted to this robot and these frontiers.
            sub_assignments = assign_frontiers(
                {name: rs},
                frontiers,
                self.potential_scale,
                self.gain_scale,
                self.nearby_penalty_dist,
                self.min_goal_separation,
                self.blacklist_radius,
                utility_penalty_fn=lambda _rname, fr: self._frontier_penalty(rs, fr),
            )
            if name in sub_assignments:
                fr = frontiers[sub_assignments[name]]
                self._send_goal(rs, fr)

        if not any_frontiers:
            if not self.exploration_complete and any_idle:
                self.get_logger().info(
                    'No frontiers remaining on any local map — exploration complete!')
                self.exploration_complete = True
                if self.return_to_init:
                    for rs in self.robots.values():
                        if rs.initial_position and not rs.returning_home:
                            self._send_return_to_init(rs)
                    self._publish_status(
                        'RETURNING_TO_ORIGIN'
                        if any(
                            r.returning_home for r in self.robots.values()
                        )
                        else 'COMPLETE'
                    )
                else:
                    self._publish_status('COMPLETE')

    def _merge_state_callback(self, msg: StringMsg):
        state = (msg.data or '').strip().upper()
        self._merge_state_last = state or 'NO_OVERLAP'
        if state == 'MERGED':
            if not self.in_global_phase:
                self.get_logger().info(
                    'Merge state reported MERGED — switching to global exploration on merged map')
            self.in_global_phase = True

    # -----------------------------------------------------------------------
    # TF helpers
    # -----------------------------------------------------------------------

    def _update_robot_positions(self):
        for rs in self.robots.values():
            # In all modes, treat the robot base frame as "<robot>/base_footprint".
            # For multi-robot + map_merge, TF provides:
            #   map -> <robot>/map -> <robot>/odom -> <robot>/base_footprint
            # For offloaded single-robot mode, the robot publishes:
            #   map -> <robot>/odom -> <robot>/base_footprint
            base_frame = f'{rs.name}/{self.robot_base_frame}'
            try:
                t = self.tf_buffer.lookup_transform(
                    self.world_frame, base_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.5))
                rs.position = (
                    t.transform.translation.x,
                    t.transform.translation.y,
                )
                # Capture an approximate "home" pose per robot the first
                # time we see a valid transform. This is used by the optional
                # return_to_init behaviour.
                if rs.initial_position is None:
                    rs.initial_position = rs.position
                    rs.initial_orientation_w = t.transform.rotation.w or 1.0
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                # keep last known position
                pass

    # -----------------------------------------------------------------------
    # Nav2 goal management
    # -----------------------------------------------------------------------

    def _world_to_cell(self, m: OccupancyGrid, x: float, y: float) -> Optional[Tuple[int, int]]:
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        if cx < 0 or cy < 0 or cx >= m.info.width or cy >= m.info.height:
            return None
        return cx, cy

    def _goal_cell_cost(self, m: Optional[OccupancyGrid], x: float, y: float) -> int:
        if m is None:
            return 0
        cell = self._world_to_cell(m, x, y)
        if cell is None:
            return 100
        cx, cy = cell
        idx = cy * m.info.width + cx
        return int(m.data[idx])

    def _goal_clearance_m(self, m: Optional[OccupancyGrid], x: float, y: float) -> float:
        if m is None:
            return self.goal_clearance_radius_m
        cell = self._world_to_cell(m, x, y)
        if cell is None:
            return 0.0
        cx, cy = cell
        res = m.info.resolution
        radius_cells = max(1, int(self.goal_clearance_radius_m / res))
        grid = np.array(m.data, dtype=np.int16).reshape((m.info.height, m.info.width))
        min_clearance = self.goal_clearance_radius_m
        y0 = max(0, cy - radius_cells)
        y1 = min(m.info.height - 1, cy + radius_cells)
        x0 = max(0, cx - radius_cells)
        x1 = min(m.info.width - 1, cx + radius_cells)
        for yy in range(y0, y1 + 1):
            for xx in range(x0, x1 + 1):
                if grid[yy, xx] >= OCCUPIED_THRESH:
                    d = math.hypot((xx - cx) * res, (yy - cy) * res)
                    if d < min_clearance:
                        min_clearance = d
        return min_clearance

    def _clearance_penalty(self, m: Optional[OccupancyGrid], x: float, y: float) -> float:
        clearance = self._goal_clearance_m(m, x, y)
        shortfall = max(0.0, self.goal_clearance_radius_m - clearance)
        return self.goal_clearance_weight * shortfall

    def _goal_passes_safety_gate(self, m: Optional[OccupancyGrid], x: float, y: float) -> bool:
        cost = self._goal_cell_cost(m, x, y)
        if cost < 0 or cost >= self.goal_max_cell_cost:
            return False
        clearance = self._goal_clearance_m(m, x, y)
        return clearance >= self.goal_min_clearance_gate_m

    def _log_goal_safety_rejection(
        self,
        rs: RobotState,
        x: float,
        y: float,
        context: str,
    ) -> None:
        m = self._current_goal_map
        cost = self._goal_cell_cost(m, x, y)
        clearance = self._goal_clearance_m(m, x, y)
        reasons: List[str] = []
        if cost < 0:
            reasons.append('unknown_cell')
        elif cost >= self.goal_max_cell_cost:
            reasons.append('high_cost')
        if clearance < self.goal_min_clearance_gate_m:
            reasons.append('low_clearance')
        if not reasons:
            reasons.append('unknown')
        self.get_logger().debug(
            f'[{rs.name}] reject_goal_candidate ({context}): '
            f'goal=({x:.2f}, {y:.2f}); cost={cost}; '
            f'clearance_m={clearance:.2f}; '
            f'thresholds(cost<{self.goal_max_cell_cost}, '
            f'clearance>={self.goal_min_clearance_gate_m:.2f}); '
            f'reason={",".join(reasons)}'
        )

    def _frontier_penalty(self, rs: RobotState, fr: Frontier) -> float:
        gx, gy = self._select_goal_point(rs, fr)
        return self._clearance_penalty(self._current_goal_map, gx, gy)

    def _frontier_utility(self, rs: RobotState, fr: Frontier) -> float:
        gx, gy = self._select_goal_point(rs, fr)
        if rs.position is None:
            return -1e9
        if not self._goal_passes_safety_gate(self._current_goal_map, gx, gy):
            self._log_goal_safety_rejection(
                rs, gx, gy, context='frontier_utility'
            )
            return -1e9
        d = _dist(rs.position, (gx, gy))
        if d < self.min_goal_separation:
            return -1e9
        gain = self.gain_scale * fr.size_m
        cost = self.potential_scale * d
        penalty = self._clearance_penalty(self._current_goal_map, gx, gy)
        return gain - cost - penalty

    def _best_frontier_for_robot(self, rs: RobotState, frontiers: List[Frontier]) -> Tuple[Optional[Frontier], float]:
        best = None
        best_u = -1e9
        for fr in frontiers:
            if any(_dist(fr.centroid_world, bl) < self.blacklist_radius for bl in rs.blacklist):
                continue
            u = self._frontier_utility(rs, fr)
            if u > best_u:
                best_u = u
                best = fr
        return best, best_u

    def _robot_is_making_progress(self, rs: RobotState, now: float) -> bool:
        if not rs.goal_active or rs.goal_position is None or rs.position is None:
            return False
        if rs.last_pose_when_goal_sent is None:
            return False
        start_dist = _dist(rs.last_pose_when_goal_sent, rs.goal_position)
        curr_dist = _dist(rs.position, rs.goal_position)
        return (start_dist - curr_dist) >= self.min_progress_before_replan_m

    def _is_robot_stuck(self, rs: RobotState, now: float) -> bool:
        if not self.allow_replan_when_no_progress:
            return False
        if not rs.goal_active:
            return False
        if rs.last_progress_time <= 0:
            return (now - rs.active_goal_sent_time) > self.max_stuck_time_s
        return (now - rs.last_progress_time) > self.max_stuck_time_s

    def _log_replan_decision(
        self,
        rs: RobotState,
        reason: str,
        allow: bool,
        now: float,
        candidate_goal: Optional[Tuple[float, float]] = None,
    ) -> None:
        elapsed = now - (rs.active_goal_sent_time or now)
        since_progress = now - (rs.last_progress_time or rs.active_goal_sent_time or now)
        goal_delta = None
        if candidate_goal is not None and rs.goal_position is not None:
            goal_delta = _dist(candidate_goal, rs.goal_position)
        prefix = 'replan' if allow else 'skip_replan'
        details = (
            f'[{rs.name}] {prefix}: {reason}; elapsed_since_goal={elapsed:.1f}s; '
            f'since_progress={since_progress:.1f}s; '
            f'goal_delta_m={(f"{goal_delta:.2f}" if goal_delta is not None else "n/a")}'
        )
        if allow:
            self.get_logger().info(details)
        else:
            self.get_logger().debug(details)

    def _should_replace_goal(
        self,
        rs: RobotState,
        candidate_goal_xy: Tuple[float, float],
        now: float,
    ) -> Tuple[bool, str]:
        if rs.goal_position is None or not rs.goal_active:
            return True, 'no_active_goal'
        if rs.goal_status in ('aborted', 'canceled', 'failed', 'succeeded'):
            return True, f'current_goal_{rs.goal_status}'
        if (now - rs.active_goal_sent_time) < self.min_goal_replan_interval_s:
            return False, 'cooldown'
        if _dist(candidate_goal_xy, rs.goal_position) < self.min_goal_change_dist_m:
            return False, 'goal_too_similar'
        if self._robot_is_making_progress(rs, now):
            return False, 'robot_making_progress'
        if self._is_robot_stuck(rs, now):
            return True, 'stuck_timeout'
        return False, 'no_replan_condition'

    def _maybe_retarget_active_robot(self, rs: RobotState, frontiers: List[Frontier], now: float):
        if not self.retarget_enable:
            return
        if not rs.goal_active or rs.goal_pending or rs.position is None or rs.goal_position is None:
            return
        if (now - rs.last_retarget_time) < self.retarget_cooldown_sec:
            return

        current_goal = rs.goal_position
        curr_clearance = self._goal_clearance_m(self._current_goal_map, current_goal[0], current_goal[1])
        no_progress_for = now - (rs.last_progress_time or rs.last_goal_time or now)
        risky = curr_clearance < self.retarget_clearance_threshold_m
        stale = no_progress_for > self.retarget_stagnation_sec
        if not risky and not stale:
            return

        best_fr, best_u = self._best_frontier_for_robot(rs, frontiers)
        if best_fr is None:
            return
        new_goal = self._select_goal_point(rs, best_fr)
        if _dist(new_goal, current_goal) < self.retarget_min_goal_shift_m:
            return

        curr_u = -(
            self.potential_scale * _dist(rs.position, current_goal)
            + self._clearance_penalty(self._current_goal_map, current_goal[0], current_goal[1])
        )
        if best_u < (curr_u + self.retarget_min_utility_gain):
            return

        should_replace, gate_reason = self._should_replace_goal(rs, new_goal, now)
        if not should_replace:
            self._log_replan_decision(
                rs, gate_reason, allow=False, now=now, candidate_goal=new_goal)
            return

        reason = 'clearance risk' if risky else 'stagnation'
        self._log_replan_decision(
            rs, gate_reason, allow=True, now=now, candidate_goal=new_goal)
        self.get_logger().info(
            f'[{rs.name}] Retargeting active goal due to {reason}; '
            f'old=({current_goal[0]:.2f}, {current_goal[1]:.2f}) '
            f'new=({new_goal[0]:.2f}, {new_goal[1]:.2f})'
        )
        self._cancel_goal(rs)
        rs.blacklist.append(current_goal)
        rs.last_retarget_time = now
        self._send_goal(rs, best_fr)

    def _select_goal_point(self, rs: RobotState, frontier: Frontier) -> Tuple[float, float]:
        """Choose a goal point within a frontier region according to the
        configured strategy. Falls back to the centroid when needed.
        """
        # Default to centroid.
        gx, gy = frontier.centroid_world

        if self.goal_point_strategy == 'farthest_cell' and frontier.indices is not None and rs.position is not None:
            rx, ry = rs.position
            # frontier.indices is (N, 2) with (y, x) in cell coordinates.
            ys = frontier.indices[:, 0].astype(np.float32)
            xs = frontier.indices[:, 1].astype(np.float32)

            # Convert all frontier cells to world coordinates using the map
            # that was active when the frontier set was computed.
            m = self._current_goal_map
            if m is None:
                # Fallback to centroid if, for some reason, we don't have a
                # map associated with this frontier batch.
                return gx, gy
            res = m.info.resolution
            ox = m.info.origin.position.x
            oy = m.info.origin.position.y
            wx = ox + (xs + 0.5) * res
            wy = oy + (ys + 0.5) * res

            # Distances from robot to each candidate cell.
            dx = wx - rx
            dy = wy - ry
            dists = np.hypot(dx, dy)
            grid = np.array(m.data, dtype=np.int16).reshape((m.info.height, m.info.width))
            costs = grid[ys.astype(np.int32), xs.astype(np.int32)]
            valid_cost = (costs >= 0) & (costs < self.goal_max_cell_cost)
            if np.any(valid_cost):
                clearance_ok = np.zeros_like(valid_cost, dtype=bool)
                valid_indices = np.flatnonzero(valid_cost)
                for i in valid_indices:
                    clearance = self._goal_clearance_m(m, float(wx[i]), float(wy[i]))
                    if clearance >= self.goal_min_clearance_gate_m:
                        clearance_ok[i] = True
                valid_mask = valid_cost & clearance_ok
            else:
                valid_mask = valid_cost

            # Prefer cells at or beyond the configured minimum goal distance,
            # but allow closer ones if nothing meets that threshold.
            mask_far_enough = dists >= max(0.0, self.min_goal_distance)
            safe_far = valid_mask & mask_far_enough
            if np.any(safe_far):
                idx = int(np.argmax(dists * safe_far))
            elif np.any(valid_mask):
                idx = int(np.argmax(dists * valid_mask))
            elif np.any(mask_far_enough):
                idx = int(np.argmax(dists * mask_far_enough))
            else:
                idx = int(np.argmax(dists))

            gx = float(wx[idx])
            gy = float(wy[idx])
            if not np.any(valid_mask):
                self._log_goal_safety_rejection(
                    rs, gx, gy, context='select_goal_point_fallback'
                )

        return gx, gy

    def _send_goal(self, rs: RobotState, frontier: Frontier):
        if rs.action_client is None:
            self.get_logger().warn(f'[{rs.name}] No action client available')
            rs.goal_active = False
            return

        if not rs.action_client.wait_for_server(timeout_sec=0.2):
            if not rs.server_unavailable_logged:
                self.get_logger().warn(
                    f'[{rs.name}] NavigateToPose action server not available '
                    f'on /{rs.name}/navigate_to_pose')
                rs.server_unavailable_logged = True
            if self.use_pose_goal_fallback:
                self._publish_pose_fallback_goal(rs, frontier)
                return
            rs.goal_active = False
            rs.goal_pending = False
            return
        if rs.server_unavailable_logged:
            self.get_logger().info(
                f'[{rs.name}] NavigateToPose action server is now available')
            rs.server_unavailable_logged = False

        goal_x, goal_y = self._select_goal_point(rs, frontier)
        try:
            m = self._current_goal_map
            if m is not None:
                map_meta = {
                    'width': int(m.info.width),
                    'height': int(m.info.height),
                    'resolution': float(m.info.resolution),
                    'origin': {
                        'x': float(m.info.origin.position.x),
                        'y': float(m.info.origin.position.y),
                    },
                }
            else:
                map_meta = {}
            dist_to_goal = None
            if rs.position is not None:
                dist_to_goal = float(
                    math.hypot(goal_x - rs.position[0], goal_y - rs.position[1])
                )
            self.event_logger.log_goal_selected(
                robot=rs.name,
                goal_x=float(goal_x),
                goal_y=float(goal_y),
                frame=self.world_frame,
                map_topic=self.map_topic,
                map_meta=map_meta,
                merge_state=self._merge_state_last,
                score={
                    'frontier_size_cells': int(frontier.size),
                    'frontier_size_m': float(frontier.size_m),
                    'distance_to_goal_m': dist_to_goal,
                    'utility': float(self._frontier_utility(rs, frontier)),
                },
                reason='frontier_assignment',
                robot_pose_used=(
                    {'x': float(rs.position[0]), 'y': float(rs.position[1])}
                    if rs.position is not None else None
                ),
                t_ros_ns=int(self.get_clock().now().nanoseconds),
            )
        except Exception:
            pass

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.world_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        rs.goal_active = True
        rs.goal_pending = True
        rs.goal_status = 'pending'
        rs.goal_position = (goal_x, goal_y)
        now = self.get_clock().now().nanoseconds / 1e9
        rs.last_goal_time = now
        rs.active_goal_sent_time = now
        rs.last_pose_when_goal_sent = rs.position
        rs.last_distance_remaining = None
        rs.last_progress_distance = None
        rs.last_progress_time = now
        rs.returning_home = False

        # Update repeat-goal tracking. If we keep selecting essentially the
        # same goal for this robot, treat the region as exhausted after a few
        # repeats and blacklist it so other frontiers can be considered.
        REPEAT_DIST_THRESH = 0.3  # metres
        REPEAT_LIMIT = 5
        frontier_size_m = float(frontier.size_m)

        if rs.last_goal_world is not None:
            lx, ly = rs.last_goal_world
            dx = goal_x - lx
            dy = goal_y - ly
            if math.hypot(dx, dy) < REPEAT_DIST_THRESH:
                rs.repeat_goal_count += 1
            else:
                rs.repeat_goal_count = 0
        else:
            rs.repeat_goal_count = 0

        rs.last_goal_world = (goal_x, goal_y)
        rs.recent_goals.append((goal_x, goal_y, frontier_size_m))

        if rs.repeat_goal_count >= REPEAT_LIMIT:
            self.get_logger().warn(
                f'[{rs.name}] Repeatedly selecting nearly-identical goal '
                f'({goal_x:.2f}, {goal_y:.2f}); blacklisting and searching elsewhere'
            )
            rs.blacklist.append((goal_x, goal_y))
            rs.repeat_goal_count = 0

        send_future = rs.action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda f, r=rs: self._goal_feedback_callback(f, r),
        )
        try:
            self.event_logger.log_goal_sent(
                robot=rs.name,
                goal_x=float(goal_x),
                goal_y=float(goal_y),
                frame=self.world_frame,
                action_name=f'/{rs.name}/navigate_to_pose',
                t_ros_ns=int(self.get_clock().now().nanoseconds),
            )
        except Exception:
            pass
        send_future.add_done_callback(
            lambda f, r=rs: self._goal_response_callback(f, r))
        self.get_logger().info(
            f'[{rs.name}] Sending NavigateToPose ({goal_x:.2f}, {goal_y:.2f}) '
            f'— frontier size {frontier.size_m:.2f}m ({frontier.size} cells)')

    def _publish_pose_fallback_goal(self, rs: RobotState, frontier: Frontier):
        goal_pub = self.goal_pubs.get(rs.name)
        if goal_pub is None:
            rs.goal_active = False
            rs.goal_pending = False
            return

        goal_x, goal_y = self._select_goal_point(rs, frontier)

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.world_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        goal_pub.publish(goal_msg)
        rs.goal_position = (goal_x, goal_y)
        now = self.get_clock().now().nanoseconds / 1e9
        rs.last_goal_time = now
        rs.active_goal_sent_time = now
        rs.last_pose_when_goal_sent = rs.position
        rs.last_progress_time = now
        rs.last_progress_distance = None
        rs.goal_active = True
        rs.goal_pending = False
        rs.goal_status = 'executing'
        rs.goal_handle = None
        self.get_logger().warn(
            f'[{rs.name}] Falling back to PoseStamped goal_pose '
            f'({goal_x:.2f}, {goal_y:.2f})')

    def _goal_response_callback(self, future, rs: RobotState):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'[{rs.name}] Failed to send NavigateToPose goal: {exc}')
            rs.goal_active = False
            rs.goal_pending = False
            rs.goal_status = 'failed'
            rs.goal_handle = None
            if rs.goal_position is not None:
                rs.blacklist.append(rs.goal_position)
            return

        rs.goal_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{rs.name}] Goal rejected by Nav2')
            rs.goal_active = False
            rs.goal_status = 'failed'
            if rs.goal_position is not None:
                rs.blacklist.append(rs.goal_position)
            return

        rs.goal_handle = goal_handle
        rs.goal_status = 'executing'
        now = self.get_clock().now().nanoseconds / 1e9
        rs.last_goal_time = now
        rs.last_progress_time = now
        self.get_logger().debug(f'[{rs.name}] Goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, r=rs: self._goal_result_callback(f, r))

    def _goal_feedback_callback(self, feedback_msg, rs: RobotState):
        now = self.get_clock().now().nanoseconds / 1e9
        rs.last_goal_time = now
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'distance_remaining'):
            dist = feedback.distance_remaining
            rs.last_distance_remaining = dist
            # Track last time we saw a meaningful decrease in remaining
            # distance, in metres. This lets the watchdog in _plan_tick
            # reason about "no progress" rather than just elapsed time.
            if rs.last_progress_distance is None:
                rs.last_progress_distance = float(dist)
                rs.last_progress_time = now
            elif (rs.last_progress_distance - dist) > self.progress_min_delta:
                rs.last_progress_distance = float(dist)
                rs.last_progress_time = now

    def _goal_result_callback(self, future, rs: RobotState):
        status_text_map = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'[{rs.name}] Failed while waiting for NavigateToPose result: {exc}')
            result = None

        if result is None:
            try:
                self.event_logger.log_goal_result(
                    robot=rs.name,
                    status_code=-1,
                    status_text='RESULT_NONE',
                    result={
                        'goals_reached': int(rs.goals_reached),
                        'goals_failed': int(rs.goals_failed + 1),
                        'goal_position': (
                            {'x': float(rs.goal_position[0]), 'y': float(rs.goal_position[1])}
                            if rs.goal_position is not None else None
                        ),
                    },
                    t_ros_ns=int(self.get_clock().now().nanoseconds),
                )
            except Exception:
                pass
            rs.goals_failed += 1
            if rs.goal_position:
                rs.blacklist.append(rs.goal_position)
            rs.goal_active = False
            rs.goal_pending = False
            rs.goal_status = 'failed'
            rs.goal_handle = None
            rs.last_distance_remaining = None
            rs.last_progress_distance = None
            rs.last_progress_time = 0.0
            return

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            # Only apply the "suspicious success" heuristic once the map has
            # grown beyond a minimum physical size. On tiny initial maps,
            # SLAM + Nav2 can legitimately succeed while still reporting a
            # non-trivial distance_remaining because the frontier centroid
            # lies just outside the currently known free space.
            small_map = (
                self._map_size_x < self.strict_success_min_map_size
                and self._map_size_y < self.strict_success_min_map_size
            )
            suspicious = (
                rs.last_distance_remaining is not None
                and rs.last_distance_remaining > self.suspicious_success_distance
            )

            if suspicious and not small_map:
                rs.goals_failed += 1
                rs.goal_status = 'failed'
                self.get_logger().warn(
                    f'[{rs.name}] Goal reported success but distance_remaining='
                    f'{rs.last_distance_remaining:.2f}m > '
                    f'{self.suspicious_success_distance:.2f}m — treating as '
                    f'failure and blacklisting')
                if rs.goal_position:
                    rs.blacklist.append(rs.goal_position)
            else:
                rs.goals_reached += 1
                rs.goal_status = 'succeeded'
                if suspicious and small_map:
                    self.get_logger().info(
                        f'[{rs.name}] Goal reported success with '
                        f'distance_remaining={rs.last_distance_remaining:.2f}m '
                        f'on a small map (~{self._map_size_x:.1f}x'
                        f'{self._map_size_y:.1f}m); accepting as success to '
                        f'grow the initial map')
                else:
                    self.get_logger().info(
                        f'[{rs.name}] Goal reached '
                        f'(total: {rs.goals_reached})')

                # Stagnation heuristic: if recent goals for this robot have
                # stayed clustered in a small region without significant change
                # in frontier size, treat that region as exhausted and add a
                # blacklist entry at the cluster centre.
                if len(rs.recent_goals) >= 5:
                    xs = [g[0] for g in rs.recent_goals]
                    ys = [g[1] for g in rs.recent_goals]
                    sizes = [g[2] for g in rs.recent_goals]
                    cx = sum(xs) / len(xs)
                    cy = sum(ys) / len(ys)
                    max_rad = max(
                        math.hypot(x - cx, y - cy)
                        for x, y in zip(xs, ys)
                    )
                    size_span = max(sizes) - min(sizes)
                    STAGNATION_RADIUS = 0.7  # metres
                    STAGNATION_SIZE_EPS = 0.5  # metres of frontier-length change
                    if max_rad < STAGNATION_RADIUS and size_span < STAGNATION_SIZE_EPS:
                        self.get_logger().info(
                            f'[{rs.name}] Goals have stagnated in a small region; '
                            f'blacklisting cluster centre ({cx:.2f}, {cy:.2f})'
                        )
                        rs.blacklist.append((cx, cy))
        elif status == GoalStatus.STATUS_ABORTED:
            rs.goals_failed += 1
            rs.goal_status = 'aborted'
            self.get_logger().warn(
                f'[{rs.name}] Goal aborted — blacklisting')
            if rs.goal_position:
                rs.blacklist.append(rs.goal_position)
        elif status == GoalStatus.STATUS_CANCELED:
            rs.goal_status = 'canceled'
            self.get_logger().info(f'[{rs.name}] Goal cancelled')
        else:
            rs.goal_status = 'failed'
            self.get_logger().info(
                f'[{rs.name}] Goal finished with status {status}')

        try:
            self.event_logger.log_goal_result(
                robot=rs.name,
                status_code=int(status),
                status_text=status_text_map.get(int(status), f'STATUS_{int(status)}'),
                result={
                    'goals_reached': int(rs.goals_reached),
                    'goals_failed': int(rs.goals_failed),
                    'distance_remaining': (
                        float(rs.last_distance_remaining)
                        if rs.last_distance_remaining is not None else None
                    ),
                    'goal_position': (
                        {'x': float(rs.goal_position[0]), 'y': float(rs.goal_position[1])}
                        if rs.goal_position is not None else None
                    ),
                },
                t_ros_ns=int(self.get_clock().now().nanoseconds),
            )
        except Exception:
            pass

        rs.goal_active = False
        rs.goal_pending = False
        rs.goal_handle = None
        rs.last_distance_remaining = None
        rs.last_progress_distance = None
        rs.last_progress_time = 0.0
        if rs.returning_home and self.exploration_complete:
            rs.returning_home = False
            # When all robots have finished returning, mark complete.
            if not any(r.returning_home for r in self.robots.values()):
                self._publish_status('COMPLETE')

    def _cancel_goal(self, rs: RobotState):
        if rs.goal_handle is not None:
            try:
                rs.goal_handle.cancel_goal_async()
            except Exception:
                pass
        rs.goal_active = False
        rs.goal_pending = False
        rs.goal_status = 'canceled'
        rs.goal_handle = None
        rs.last_distance_remaining = None
        rs.last_progress_distance = None
        rs.last_progress_time = 0.0

    def _send_return_to_init(self, rs: RobotState):
        if rs.initial_position is None:
            return
        if rs.action_client is None:
            self.get_logger().warn(
                f'[{rs.name}] Cannot send return_to_init goal (no action client)')
            return
        if not rs.action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(
                f'[{rs.name}] NavigateToPose server unavailable for '
                f'return_to_init on /{rs.name}/navigate_to_pose')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.world_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = rs.initial_position[0]
        goal_msg.pose.pose.position.y = rs.initial_position[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = rs.initial_orientation_w or 1.0

        rs.goal_active = True
        rs.goal_pending = True
        rs.goal_status = 'pending'
        rs.goal_position = rs.initial_position
        now = self.get_clock().now().nanoseconds / 1e9
        rs.last_goal_time = now
        rs.active_goal_sent_time = now
        rs.last_pose_when_goal_sent = rs.position
        rs.last_progress_time = now
        rs.last_progress_distance = None
        rs.last_distance_remaining = None
        rs.returning_home = True

        send_future = rs.action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda f, r=rs: self._goal_feedback_callback(f, r),
        )
        send_future.add_done_callback(
            lambda f, r=rs: self._goal_response_callback(f, r))
        self.get_logger().info(
            f'[{rs.name}] Sending return_to_init NavigateToPose '
            f'({rs.initial_position[0]:.2f}, {rs.initial_position[1]:.2f})')

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
    except (KeyboardInterrupt, ExternalShutdownException):
        # Treat Ctrl+C and external shutdown events as normal exit paths.
        pass
    except Exception:
        # During shutdown the rclpy context can become invalid and raise
        # RCLError-like exceptions from the executor. Ignore these so that
        # teardown remains quiet while still running the cleanup below.
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
