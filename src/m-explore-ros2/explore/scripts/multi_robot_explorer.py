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
from nav2_msgs.action import NavigateToPose, ComputePathToPose
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
    # Monotonic per-robot sequence for NavigateToPose goals sent by explorer.
    goal_seq_counter: int = 0
    # Sequence id for the currently active/pending goal callbacks.
    active_goal_seq: int = 0
    last_pose_when_goal_sent: Optional[Tuple[float, float]] = None
    # Straight-line distance robot→goal; used to detect motion toward goal when
    # Nav2 feedback is sparse so the stall watchdog does not cancel healthy drives.
    last_dist_to_goal: Optional[float] = None
    # Track behaviour around repeated / stagnant goals. last_goal_world stores
    # the most recent goal position in world coordinates (x, y).
    last_goal_world: Optional[Tuple[float, float]] = None
    repeat_goal_count: int = 0
    # Short history of recent goals and associated frontier sizes for
    # stagnation detection. Each entry is (x, y, frontier_size_m).
    recent_goals: deque = field(default_factory=lambda: deque(maxlen=10))
    last_retarget_time: float = 0.0
    # Silence new assignments until this time (unix sec) after abort/cancel watchdog.
    next_assign_allowed_time: float = 0.0
    # When True, the next CANCELED result from Nav2 will not arm post-failure
    # cooldown (used when we cancel for retarget and immediately send a replacement
    # goal). ABORTED-as-cancel is gated on retarget_abort_blacklist_suppress_seq.
    ignore_cooldown_on_next_cancel: bool = False
    # NavigateToPose goal_seq for the replacement goal after retarget; Nav2 may
    # report ABORTED while tearing down the cancelled goal — do not blacklist.
    retarget_abort_blacklist_suppress_seq: Optional[int] = None
    # First NavigateToPose of the current leg; retargets must stay near this.
    retarget_anchor: Optional[Tuple[float, float]] = None
    # Set when we cancel Nav2 after tacit "close enough" success (see plan_tick).
    tacit_success_pending_cancel: bool = False
    # First time (unix s) the robot was within tacit_goal_success_radius_m of goal.
    near_goal_since: Optional[float] = None
    # Time when NavigateToPose action server was first seen available.
    action_server_ready_time: float = 0.0
    # Last sampled world-frame pose used to detect physical motion.
    last_motion_sample_pose: Optional[Tuple[float, float]] = None
    # Time of last_motion_sample_pose.
    last_motion_sample_time: float = 0.0
    # Last time meaningful physical movement (>= motion_progress_min_delta_m)
    # was observed from TF poses.
    last_motion_time: float = 0.0
    # True while Nav2 costmap marks the robot position as lethal/in-collision.
    nav2_lethal_active: bool = False
    # First timestamp when nav2_lethal_active became True.
    nav2_lethal_since: float = 0.0
    # Goal to preserve while robot is in lethal space; resent after recovery.
    held_goal_position: Optional[Tuple[float, float]] = None
    # Async Nav2 path preflight (compute_path_to_pose before navigate_to_pose).
    path_precheck_client: Optional[object] = None
    path_precheck_goal_handle: Optional[object] = None
    path_precheck_in_progress: bool = False
    path_precheck_frontier: Optional[Frontier] = None
    path_precheck_from_retarget: bool = False
    path_precheck_goal_xy: Optional[Tuple[float, float]] = None
    path_precheck_started_time: float = 0.0
    # First time compute_path_to_pose wait_for_server succeeded (Nav2 warm-up).
    path_precheck_server_seen_time: float = 0.0
    # Session start pose in world frame (recorded on first TF once maps are ready).
    start_pose_xy: Optional[Tuple[float, float]] = None
    # True while a return-home NavigateToPose is active or pending.
    homing_active: bool = False
    # True after successful home, skip (already near start), or max failures.
    return_home_done: bool = False
    # Count of failed homing attempts (precheck abort, Nav2 failure, stall cancel).
    return_home_failures: int = 0


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
    skip_frontier_fn: Optional[Callable[[str, Frontier], bool]] = None,
    robot_ready_fn: Optional[Callable[[str], bool]] = None,
    goal_dist_fn: Optional[Callable[[str, Frontier], float]] = None,
    assignment_robot_order_fn: Optional[Callable[[List[str]], List[str]]] = None,
) -> Tuple[Dict[str, int], bool]:
    """Assign one frontier to each idle robot.

    Returns ({robot_name: frontier_index}, relaxed_min_separation).

    If every candidate is closer than ``min_goal_separation`` (e.g. robot
    just reached a goal on that frontier), the matrix would be empty. A
    second pass uses separation 0 so exploration can continue; repeat-goal
    blacklisting still avoids infinite instant-success loops.

    When ``assignment_robot_order_fn`` is set, each robot (in that order)
    picks the best still-unassigned frontier. This avoids starvation when
    global greedy assigns the only frontier to another namespaced robot that
    is idle but not actually exploring.

    Utility for (robot, frontier):
        gain_scale * frontier.size_m  -  potential_scale * distance
        minus a penalty if another robot already targets a nearby frontier.
    """
    # robots that need a new goal
    idle_robots = {
        name: rs for name, rs in robots.items()
        if not rs.goal_active and not rs.goal_pending and rs.position is not None
        and (robot_ready_fn is None or robot_ready_fn(name))
    }
    if not idle_robots or not frontiers:
        return {}, False

    # already-assigned goal positions (from robots that ARE active)
    active_goals = [
        rs.goal_position
        for rs in robots.values()
        if rs.goal_active and rs.goal_position is not None
    ]

    robot_list = list(idle_robots.keys())
    relaxed = False
    utilities = np.full((len(robot_list), len(frontiers)), -np.inf)

    for min_sep in (min_goal_separation, 0.0):
        utilities.fill(-np.inf)
        for ri, rname in enumerate(robot_list):
            rpos = idle_robots[rname].position
            blacklist = idle_robots[rname].blacklist
            for fi, fr in enumerate(frontiers):
                if any(
                    _dist(fr.centroid_world, bl) < blacklist_radius
                    for bl in blacklist
                ):
                    continue
                if skip_frontier_fn is not None and skip_frontier_fn(rname, fr):
                    continue

                if goal_dist_fn is not None:
                    dist = float(goal_dist_fn(rname, fr))
                else:
                    dist = _dist(rpos, fr.centroid_world)
                # Prefer goals at least min_sep away; allow closer only on
                # relaxed pass after a successful leg onto the frontier.
                if dist < min_sep:
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

        if np.any(np.isfinite(utilities)):
            relaxed = min_sep < min_goal_separation
            break

    assignments: Dict[str, int] = {}
    assigned_f: set = set()
    name_to_ri = {n: i for i, n in enumerate(robot_list)}

    if assignment_robot_order_fn is not None:
        for rname in assignment_robot_order_fn(list(robot_list)):
            if rname not in name_to_ri:
                continue
            ri = name_to_ri[rname]
            best_fi: Optional[int] = None
            best_u = -1e9
            for fi in range(len(frontiers)):
                if fi in assigned_f:
                    continue
                u = utilities[ri, fi]
                if u > best_u:
                    best_u = u
                    best_fi = fi
            if best_fi is not None and best_u > -1e9:
                assignments[rname] = best_fi
                assigned_f.add(best_fi)
    else:
        assigned_r: set = set()
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

    return assignments, relaxed


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class MultiRobotExplorer(Node):

    def __init__(self):
        super().__init__('multi_robot_explorer')

        # -- parameters --
        self.declare_parameter('robot_names', ['blinky', 'pinky', 'inky', 'clyde'])
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
        # Extra filter on top of min_frontier_size: do not send Nav2 goals to
        # very small clusters (wall slivers / noise); they often fail the
        # local controller with "Failed to make progress". Set to 0 to disable.
        self.declare_parameter('min_frontier_cells_for_goal', 32)
        self.declare_parameter('potential_scale', 3.0)
        self.declare_parameter('gain_scale', 1.0)
        # Progress timeout is interpreted as "no meaningful distance progress"
        # for this many seconds before a goal is considered stalled.
        self.declare_parameter('progress_timeout', 45.0)
        self.declare_parameter('progress_min_delta', 0.05)
        self.declare_parameter('motion_progress_min_delta_m', 0.03)
        self.declare_parameter('motion_progress_window_sec', 2.0)
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
        # If NavigateToPose ends in ABORTED (e.g. controller "Failed to make
        # progress") but the robot is within this distance of the goal in the
        # map frame, count as reached and do not blacklist — Nav2 often aborts
        # near the target without a clean SUCCEEDED. Use a generous value; TF
        # vs goal can disagree with Nav2's notion of "close".
        self.declare_parameter('tacit_goal_success_radius_m', 0.15)
        # Also treat ABORT / tacit hold as success if Nav2 feedback
        # distance_remaining is at or below this (metres), but only when map-frame
        # pose is within tacit_goal_success_radius_m + this value of the goal
        # (avoids stale distance_remaining=0 while the robot is still far away).
        self.declare_parameter('tacit_abort_max_distance_remaining_m', 0.15)
        # While Nav2 still has an active NavigateToPose, cancel and count
        # success after this many seconds where pose and/or feedback indicate
        # the goal is essentially reached (see _at_nav_goal_for_tacit_hold).
        # This catches Nav2 never publishing SUCCEEDED while the robot is on
        # the target. Set 0 to disable.
        self.declare_parameter('tacit_goal_success_hold_sec', 1.0)
        # For very small initial maps, Nav2 can legitimately report success
        # with a relatively large distance_remaining because the frontier
        # centroid may still be outside the tiny explored region. To avoid
        # blacklisting useful early frontiers, the "suspicious success"
        # heuristic is only applied once the map has grown beyond a
        # configurable size.
        self.declare_parameter('strict_success_min_map_size', 3.0)
        self.declare_parameter('robot_base_frame', 'base_footprint')
        # Status + control topics (multi-robot aware counterpart of explore/status, explore/resume).
        self.declare_parameter('status_topic', 'explore_multi/status')
        self.declare_parameter('control_topic', 'explore_multi/resume')
        self.declare_parameter('return_to_start_enable', True)
        self.declare_parameter('return_to_start_skip_if_within_m', 0.25)
        self.declare_parameter('return_to_start_max_attempts', 5)
        # Goal selection strategy within each frontier region.
        # - 'centroid': centroid of the frontier; if the robot is already very
        #   close to that centroid, pick a cell farther out but within
        #   centroid_fallback_max_offset_from_centroid_m of the centroid.
        # - 'farthest_cell': choose the frontier cell farthest from the robot
        #   (in world distance), falling back to centroid when needed.
        self.declare_parameter('goal_point_strategy', 'centroid')
        # When using centroid: if the robot is already this close to the
        # centroid, pick a frontier cell farther out, but only within
        # centroid_fallback_max_offset_from_centroid_m of the centroid (avoids
        # ping-pong across the whole frontier like unbounded farthest_cell).
        self.declare_parameter('centroid_fallback_if_robot_closer_than_m', 0.55)
        self.declare_parameter('centroid_fallback_max_offset_from_centroid_m', 2.5)
        # Minimum desired distance from robot to goal (metres). This is used
        # when selecting a representative cell inside a frontier region so
        # that the initial goals are not almost inside Nav2's goal tolerance.
        self.declare_parameter('min_goal_distance', 0.6)
        # Goal-clearance shaping: discourage assigning goals too close to
        # occupied cells (likely walls/obstacles in the current map).
        self.declare_parameter('goal_clearance_radius_m', 0.45)
        self.declare_parameter('goal_unknown_neighbor_penalty_m', 0.0)
        self.declare_parameter('goal_clearance_weight', 4.0)
        # Hard safety gate for candidate goal cells:
        # - reject goals in high-cost cells
        # - reject goals that do not have enough local obstacle clearance
        self.declare_parameter('goal_max_cell_cost', 80)
        self.declare_parameter('goal_min_clearance_gate_m', 0.30)
        # Re-check selected goals against the latest map right before sending
        # to Nav2. This avoids dispatching goals that became unsafe between
        # frontier assignment and NavigateToPose dispatch.
        self.declare_parameter('goal_revalidate_before_send', True)
        # Mid-route retargeting controls.
        self.declare_parameter('retarget_enable', True)
        self.declare_parameter('retarget_cooldown_sec', 12.0)
        self.declare_parameter('retarget_min_goal_shift_m', 0.5)
        self.declare_parameter('retarget_min_utility_gain', 0.8)
        self.declare_parameter('retarget_stagnation_sec', 20.0)
        self.declare_parameter('retarget_clearance_threshold_m', 0.25)
        self.declare_parameter('retarget_opportunity_enable', True)
        self.declare_parameter('retarget_opportunity_min_utility_gain', 1.2)
        # Retargets only pick goals within this radius (m) of the leg's first
        # goal (retarget_anchor). Set to 0 to disable (allow distant retargets).
        self.declare_parameter('retarget_max_offset_from_anchor_m', 3.0)
        # Goal replacement gate controls to avoid rapid preempt/replan loops.
        self.declare_parameter('min_goal_replan_interval_s', 4.0)
        self.declare_parameter('min_goal_change_dist_m', 0.75)
        self.declare_parameter('min_progress_before_replan_m', 0.25)
        self.declare_parameter('max_stuck_time_s', 12.0)
        self.declare_parameter('allow_replan_when_no_progress', True)
        self.declare_parameter('post_failure_cooldown_sec', 2.0)
        # Grace window after action server appears: early goal rejections are
        # treated as Nav2 warm-up (lifecycle still activating) instead of
        # blacklisting goals.
        self.declare_parameter('goal_reject_warmup_sec', 12.0)
        # Hold active goals while robot is marked "in lethal space" by Nav2
        # and suppress watchdog/blacklisting for planner-start-in-lethal cases.
        self.declare_parameter('lethal_hold_enabled', True)
        self.declare_parameter(
            'nav2_lethal_topic_pattern', '/{robot}/nav2_lethal_inflation')
        self.declare_parameter('skip_blacklist_on_nav2_lethal', True)
        self.declare_parameter('lethal_hold_max_sec', 30.0)
        self.declare_parameter('lethal_retry_delay_sec', 0.5)
        # Per-robot local map topic pattern. By default we assume the standard
        # namespaced SLAM layout '/<robot>/map'.
        self.declare_parameter('local_map_topic_pattern', '/{robot}/map')
        # Call /<robot>/compute_path_to_pose before NavigateToPose so goals
        # Nav2 cannot plan through the global costmap are skipped here.
        self.declare_parameter('nav2_path_precheck_enable', True)
        self.declare_parameter('nav2_path_precheck_timeout_sec', 3.0)
        self.declare_parameter('nav2_path_min_poses', 2)
        # Must match planner_server.planner_plugins id (e.g. GridBased in burger.yaml).
        self.declare_parameter('nav2_path_precheck_planner_id', 'GridBased')

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
        self.min_frontier_cells_for_goal = int(
            self.get_parameter('min_frontier_cells_for_goal').value
        )
        self.potential_scale = self.get_parameter('potential_scale').value
        self.gain_scale = self.get_parameter('gain_scale').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.progress_min_delta = (
            self.get_parameter('progress_min_delta').value)
        self.motion_progress_min_delta_m = float(
            self.get_parameter('motion_progress_min_delta_m').value
        )
        self.motion_progress_window_sec = float(
            self.get_parameter('motion_progress_window_sec').value
        )
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
        self.tacit_goal_success_radius_m = float(
            self.get_parameter('tacit_goal_success_radius_m').value
        )
        self.tacit_abort_max_distance_remaining_m = float(
            self.get_parameter('tacit_abort_max_distance_remaining_m').value
        )
        self.tacit_goal_success_hold_sec = float(
            self.get_parameter('tacit_goal_success_hold_sec').value
        )
        self.strict_success_min_map_size = (
            self.get_parameter('strict_success_min_map_size').value)
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.status_topic = self.get_parameter('status_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.return_to_start_enable = bool(
            self.get_parameter('return_to_start_enable').value
        )
        self.return_to_start_skip_if_within_m = float(
            self.get_parameter('return_to_start_skip_if_within_m').value
        )
        self.return_to_start_max_attempts = max(
            1, int(self.get_parameter('return_to_start_max_attempts').value)
        )
        self.goal_point_strategy = (
            self.get_parameter('goal_point_strategy').value or 'centroid'
        )
        self.centroid_fallback_if_robot_closer_than_m = float(
            self.get_parameter('centroid_fallback_if_robot_closer_than_m').value
        )
        self.centroid_fallback_max_offset_from_centroid_m = float(
            self.get_parameter('centroid_fallback_max_offset_from_centroid_m').value
        )
        self.min_goal_distance = float(
            self.get_parameter('min_goal_distance').value
        )
        self.goal_clearance_radius_m = float(
            self.get_parameter('goal_clearance_radius_m').value
        )
        self.goal_unknown_neighbor_penalty_m = float(
            self.get_parameter('goal_unknown_neighbor_penalty_m').value
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
        self.goal_revalidate_before_send = bool(
            self.get_parameter('goal_revalidate_before_send').value
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
        self.retarget_opportunity_enable = bool(
            self.get_parameter('retarget_opportunity_enable').value
        )
        self.retarget_opportunity_min_utility_gain = float(
            self.get_parameter('retarget_opportunity_min_utility_gain').value
        )
        self.retarget_max_offset_from_anchor_m = float(
            self.get_parameter('retarget_max_offset_from_anchor_m').value
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
        self.post_failure_cooldown_sec = float(
            self.get_parameter('post_failure_cooldown_sec').value
        )
        self.goal_reject_warmup_sec = float(
            self.get_parameter('goal_reject_warmup_sec').value
        )
        self.lethal_hold_enabled = bool(
            self.get_parameter('lethal_hold_enabled').value
        )
        self.nav2_lethal_topic_pattern = str(
            self.get_parameter('nav2_lethal_topic_pattern').value
        )
        self.skip_blacklist_on_nav2_lethal = bool(
            self.get_parameter('skip_blacklist_on_nav2_lethal').value
        )
        self.lethal_hold_max_sec = float(
            self.get_parameter('lethal_hold_max_sec').value
        )
        self.lethal_retry_delay_sec = float(
            self.get_parameter('lethal_retry_delay_sec').value
        )
        self.local_map_topic_pattern: str = (
            self.get_parameter('local_map_topic_pattern').value
        )
        self.nav2_path_precheck_enable: bool = bool(
            self.get_parameter('nav2_path_precheck_enable').value
        )
        self.nav2_path_precheck_timeout_sec: float = float(
            self.get_parameter('nav2_path_precheck_timeout_sec').value
        )
        self.nav2_path_min_poses: int = max(
            1, int(self.get_parameter('nav2_path_min_poses').value)
        )
        _pid = self.get_parameter('nav2_path_precheck_planner_id').value
        self.nav2_path_precheck_planner_id: str = (
            str(_pid).strip() if _pid is not None else ''
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
        # Set true once past waiting-for-map so start poses can be recorded.
        self._explorer_planning_ready: bool = False
        self._return_home_status_published: str = ''
        # Rotate fair frontier assignment so this robot is tried first when idle.
        self._last_goal_finished_robot: Optional[str] = None
        self._last_no_frontier_idle_log_time: float = 0.0
        self._last_no_assignment_log_time: float = 0.0
        self._return_home_no_pose_logged: set = set()
        self.goal_pubs: Dict[str, object] = {}
        self.lethal_subs: Dict[str, object] = {}
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
            rs.path_precheck_client = ActionClient(
                self, ComputePathToPose, f'/{name}/compute_path_to_pose')
            self.robots[name] = rs
            if self.lethal_hold_enabled:
                topic = self.nav2_lethal_topic_pattern.format(robot=name)
                self.lethal_subs[name] = self.create_subscription(
                    Bool,
                    topic,
                    lambda msg, rname=name: self._nav2_lethal_callback(msg, rname),
                    10,
                )
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
            f'mode={mode}, retarget_enable={self.retarget_enable}, '
            f'retarget_opportunity={self.retarget_opportunity_enable}, '
            f'nav2_path_precheck={self.nav2_path_precheck_enable}')
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

        self._explorer_planning_ready = True

        if self.paused:
            # Still keep TF and map up to date, but do not assign new goals.
            self._update_robot_positions()
            return

        # update robot positions from TF
        self._update_robot_positions()

        now = self.get_clock().now().nanoseconds / 1e9
        # Physical motion progress: update when TF pose displacement exceeds
        # threshold, regardless of whether motion is directly goal-seeking.
        for rs in self.robots.values():
            if rs.position is None:
                continue
            if rs.last_motion_sample_pose is None:
                rs.last_motion_sample_pose = rs.position
                rs.last_motion_sample_time = now
                continue
            motion_delta = _dist(rs.position, rs.last_motion_sample_pose)
            if motion_delta >= self.motion_progress_min_delta_m:
                rs.last_motion_time = now
                rs.last_motion_sample_pose = rs.position
                rs.last_motion_sample_time = now

        # Pose-based progress: advance last_progress_time when the robot moves
        # toward the goal in the map frame, even if Nav2 feedback is quiet.
        for rs in self.robots.values():
            if not rs.goal_active or rs.goal_position is None or rs.position is None:
                continue
            d = _dist(rs.position, rs.goal_position)
            if rs.last_dist_to_goal is None:
                rs.last_dist_to_goal = d
            elif rs.last_dist_to_goal - d >= self.progress_min_delta:
                rs.last_progress_time = now
                rs.last_dist_to_goal = d
            elif d < rs.last_dist_to_goal:
                rs.last_dist_to_goal = d

        # Tacit success: Nav2 often keeps an active goal while the robot is at
        # the pose (no SUCCEEDED). Use pose + optional distance_remaining from
        # feedback; after hold_sec, cancel Nav2 and count success in result cb.
        if self.tacit_goal_success_hold_sec > 0.0:
            for rs in self.robots.values():
                if (
                    not rs.goal_active
                    or rs.goal_pending
                    or rs.goal_position is None
                    or rs.position is None
                    or rs.goal_handle is None
                ):
                    rs.near_goal_since = None
                    continue
                d = _dist(rs.position, rs.goal_position)
                if self._at_nav_goal_for_tacit_hold(rs, d):
                    rs.last_progress_time = now
                    if rs.near_goal_since is None:
                        rs.near_goal_since = now
                    elif (
                        now - rs.near_goal_since
                        >= self.tacit_goal_success_hold_sec
                    ):
                        dr = rs.last_distance_remaining
                        self.get_logger().info(
                            f'[{rs.name}] At goal for {self.tacit_goal_success_hold_sec:.1f}s '
                            f'(pose_dist={d:.2f}m remain={dr}) — canceling Nav2 and '
                            'counting success'
                        )
                        rs.tacit_success_pending_cancel = True
                        rs.ignore_cooldown_on_next_cancel = True
                        try:
                            rs.goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                        rs.near_goal_since = None
                else:
                    rs.near_goal_since = None

        # Stall watchdog: cancel only when movement is expected but TF pose has
        # remained effectively stagnant for progress_timeout seconds.
        for rs in self.robots.values():
            if not rs.goal_active or rs.last_goal_time <= 0:
                continue
            if self.lethal_hold_enabled and rs.nav2_lethal_active:
                hold_age = now - rs.nav2_lethal_since if rs.nav2_lethal_since > 0.0 else 0.0
                if self.lethal_hold_max_sec > 0.0 and hold_age > self.lethal_hold_max_sec:
                    self.get_logger().warn(
                        f'[{rs.name}] lethal hold timeout after {hold_age:.1f}s; '
                        'resuming normal watchdog behavior'
                    )
                    rs.nav2_lethal_active = False
                    rs.nav2_lethal_since = 0.0
                else:
                    if rs.goal_position is not None and rs.held_goal_position is None:
                        rs.held_goal_position = rs.goal_position
                    continue

            if rs.goal_pending or rs.goal_position is None or rs.position is None:
                continue

            dist_to_goal = _dist(rs.position, rs.goal_position)
            if dist_to_goal <= self.tacit_goal_success_radius_m:
                # Close enough to goal: let tacit success / Nav2 result handling
                # decide completion instead of watchdog cancellation.
                continue

            pose_stagnation_age = (
                (now - rs.last_motion_time)
                if rs.last_motion_time > 0.0 else float('inf')
            )
            if pose_stagnation_age > self.progress_timeout:
                self.get_logger().warn(
                    f'[{rs.name}] Goal appears stalled for '
                    f'{self.progress_timeout:.0f}s without physical movement '
                    f'(pose_stagnation_age={pose_stagnation_age:.1f}s); '
                    'cancelling and blacklisting'
                )
                self._cancel_goal(rs)
                if rs.homing_active:
                    self._return_home_note_failure(rs)
                elif rs.goal_position:
                    rs.blacklist.append(rs.goal_position)
                self._arm_post_failure_cooldown(rs)

        if self.lethal_hold_enabled:
            for rs in self.robots.values():
                if rs.nav2_lethal_active:
                    continue
                if rs.held_goal_position is None:
                    continue
                if rs.goal_active or rs.goal_pending:
                    continue
                if not self._robot_may_assign_now(rs, now):
                    continue
                self._resend_held_goal(rs)

        for rs in self.robots.values():
            if not rs.path_precheck_in_progress:
                continue
            if rs.path_precheck_started_time <= 0.0:
                continue
            if (
                now - rs.path_precheck_started_time
                < self.nav2_path_precheck_timeout_sec
            ):
                continue
            self.get_logger().warn(
                f'[{rs.name}] Nav2 path precheck timed out after '
                f'{self.nav2_path_precheck_timeout_sec:.1f}s; skipping goal'
            )
            xy = rs.path_precheck_goal_xy
            self._abort_path_precheck(
                rs,
                cancel_action=True,
                blacklist_xy=xy,
                cooldown=True,
            )

        if use_global:
            self._global_frontier_step()
        else:
            self._local_frontier_step()

        self._return_home_tick(use_global, now)
        self._maybe_publish_return_home_status()

    def _reset_return_home_state_when_exploration_resumes(self):
        self._return_home_status_published = ''
        self._return_home_no_pose_logged.clear()
        for rs in self.robots.values():
            if rs.homing_active and (rs.goal_active or rs.goal_pending):
                self._cancel_goal(rs)
            rs.return_home_done = False
            rs.homing_active = False
            rs.return_home_failures = 0

    def _return_home_tick(self, use_global: bool, now: float):
        if not self.return_to_start_enable or not self.exploration_complete:
            return

        for rs in self.robots.values():
            if rs.return_home_done:
                continue
            if rs.start_pose_xy is None:
                if rs.name not in self._return_home_no_pose_logged:
                    self._return_home_no_pose_logged.add(rs.name)
                    self.get_logger().warn(
                        f'[{rs.name}] Return home skipped: start pose was never recorded'
                    )
                rs.return_home_done = True
                continue
            if rs.goal_active or rs.goal_pending or rs.path_precheck_in_progress:
                continue
            if not self._robot_may_assign_now(rs, now):
                continue
            if rs.return_home_failures >= self.return_to_start_max_attempts:
                self.get_logger().warn(
                    f'[{rs.name}] Return home abandoned after '
                    f'{self.return_to_start_max_attempts} failed attempt(s)'
                )
                rs.return_home_done = True
                continue
            sx, sy = rs.start_pose_xy
            if rs.position is not None:
                if _dist(rs.position, rs.start_pose_xy) <= self.return_to_start_skip_if_within_m:
                    self.get_logger().info(
                        f'[{rs.name}] Within {self.return_to_start_skip_if_within_m:.2f}m of '
                        'start — skipping NavigateToPose for return home'
                    )
                    rs.return_home_done = True
                    continue
            if use_global:
                self._current_goal_map = self.latest_global_map
            else:
                self._current_goal_map = self.latest_local_maps.get(rs.name)
            if self._current_goal_map is None:
                continue

            rs.repeat_goal_count = 0
            rs.homing_active = True
            proxy = Frontier(
                centroid_world=(sx, sy),
                size=max(1, self.min_frontier_cells_for_goal),
                size_m=max(self.min_frontier_size, 0.01),
                cells=max(1, self.min_frontier_cells_for_goal),
                indices=None,
            )
            self.get_logger().info(
                f'[{rs.name}] Return home: sending goal to start '
                f'({sx:.2f}, {sy:.2f})'
            )
            self._send_goal(rs, proxy)
            if (
                not rs.goal_active
                and not rs.goal_pending
                and not rs.path_precheck_in_progress
            ):
                rs.homing_active = False

    def _maybe_publish_return_home_status(self):
        if not self.return_to_start_enable or not self.exploration_complete:
            return
        if all(rs.return_home_done for rs in self.robots.values()):
            if self._return_home_status_published != 'HOME_COMPLETE':
                self._return_home_status_published = 'HOME_COMPLETE'
                self._publish_status('HOME_COMPLETE')
            return
        finishing_explore = any(
            rs.goal_active and not rs.homing_active and not rs.return_home_done
            for rs in self.robots.values()
        )
        if finishing_explore:
            return
        if self._return_home_status_published not in ('HOMING', 'HOME_COMPLETE'):
            self._return_home_status_published = 'HOMING'
            self._publish_status('HOMING')

    def _global_frontier_step(self):
        m = self.latest_global_map
        # Use the current global map for all goal selection in this tick.
        self._current_goal_map = m
        now = self.get_clock().now().nanoseconds / 1e9
        n_idle = sum(
            1 for rs in self.robots.values()
            if not rs.goal_active and not rs.goal_pending and rs.position is not None
            and self._robot_may_assign_now(rs, now))

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
                self._publish_status('COMPLETE')
            elif n_idle > 0:
                if now - self._last_no_frontier_idle_log_time >= 15.0:
                    self._last_no_frontier_idle_log_time = now
                    self.get_logger().warning(
                        'Still no frontier regions on merged map while '
                        f'{n_idle} robot(s) are idle. If unexplored space '
                        'remains in RViz, confirm the map uses value -1 for '
                        'unknown and 0 for free, and that free cells border '
                        'unknown (frontier definition).'
                    )
            return

        if self.exploration_complete:
            self.get_logger().info('New frontiers appeared — resuming exploration')
            self._reset_return_home_state_when_exploration_resumes()
        self.exploration_complete = False

        self.get_logger().debug(
            f'{len(frontiers)} frontiers, {n_idle} idle robot(s)')
        if n_idle > 0:
            self._publish_status('IN_PROGRESS')

        # publish visualisation
        if self.visualize:
            self._publish_frontier_markers(frontiers)

        for rs in self.robots.values():
            self._maybe_retarget_active_robot(rs, frontiers, now)

        # assign frontiers to idle robots
        assignments, relaxed_sep = assign_frontiers(
            self.robots,
            frontiers,
            self.potential_scale,
            self.gain_scale,
            self.nearby_penalty_dist,
            self.min_goal_separation,
            self.blacklist_radius,
            utility_penalty_fn=lambda rname, fr: self._frontier_penalty(self.robots[rname], fr),
            skip_frontier_fn=lambda rname, fr: self._should_skip_frontier_for_assignment(
                self.robots[rname], fr),
            robot_ready_fn=lambda rname: self._robot_may_assign_now(
                self.robots[rname], now),
            goal_dist_fn=self._goal_dist_for_assignment,
            assignment_robot_order_fn=self._assignment_robot_order,
        )
        if not assignments and n_idle > 0 and frontiers:
            if now - self._last_no_assignment_log_time >= 15.0:
                self._last_no_assignment_log_time = now
                self._log_why_no_assignment(frontiers, n_idle, now)
        if relaxed_sep and assignments:
            self.get_logger().info(
                'Relaxed min_goal_separation for assignment (robot on/near '
                'frontier after a goal); repeat-goal blacklist still applies'
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

            if rs.goal_active or rs.goal_pending:
                continue
            if not self._robot_may_assign_now(rs, now):
                continue
            any_idle = True

            # Assign a frontier for this single robot using the same utility
            # function but restricted to this robot and these frontiers.
            sub_assignments, relaxed_sep = assign_frontiers(
                {name: rs},
                frontiers,
                self.potential_scale,
                self.gain_scale,
                self.nearby_penalty_dist,
                self.min_goal_separation,
                self.blacklist_radius,
                utility_penalty_fn=lambda _rname, fr: self._frontier_penalty(rs, fr),
                skip_frontier_fn=lambda _rname, fr: self._should_skip_frontier_for_assignment(rs, fr),
                robot_ready_fn=lambda _rname: self._robot_may_assign_now(rs, now),
                goal_dist_fn=lambda rname, fr: self._goal_dist_for_assignment(rname, fr),
            )
            if relaxed_sep and name in sub_assignments:
                self.get_logger().info(
                    f'[{name}] Relaxed min_goal_separation for assignment (robot '
                    'on/near frontier after a goal); repeat-goal blacklist still applies'
                )
            if name not in sub_assignments and frontiers:
                # Mirror global-mode diagnostics so local mode does not appear
                # "stuck" when frontiers exist but all candidates are gated.
                self._log_why_no_assignment(frontiers, 1, now)
            if name in sub_assignments:
                fr = frontiers[sub_assignments[name]]
                self._send_goal(rs, fr)

        if any_frontiers:
            if self.exploration_complete:
                self.get_logger().info(
                    'New frontiers appeared — resuming exploration (local mode)')
                self._reset_return_home_state_when_exploration_resumes()
            self.exploration_complete = False

        if not any_frontiers:
            if not self.exploration_complete and any_idle:
                self.get_logger().info(
                    'No frontiers remaining on any local map — exploration complete!')
                self.exploration_complete = True
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
            # TF chain on global /tf:
            #   map -> <robot>/map -> <robot>/odom -> <robot>/base_footprint
            # (map_merge in multi-robot; single_robot_world_tf_bridge + SLAM in
            # single-robot central mode.)
            base_frame = f'{rs.name}/{self.robot_base_frame}'
            try:
                t = self.tf_buffer.lookup_transform(
                    self.world_frame, base_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.5))
                rs.position = (
                    t.transform.translation.x,
                    t.transform.translation.y,
                )
                if self._explorer_planning_ready and rs.start_pose_xy is None:
                    rs.start_pose_xy = (
                        t.transform.translation.x,
                        t.transform.translation.y,
                    )
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

    def _frontier_unknown_adjacency_counts(
        self, grid: np.ndarray, ys: np.ndarray, xs: np.ndarray
    ) -> np.ndarray:
        """Per frontier cell: count of unknown 4-neighbours (prefers deeper free cells)."""
        h, w = grid.shape
        unk = grid < 0
        yi = ys.astype(np.int32)
        xi = xs.astype(np.int32)
        n = len(yi)
        out = np.zeros(n, dtype=np.float64)
        for i in range(n):
            cy, cx = int(yi[i]), int(xi[i])
            c = 0
            if cy > 0 and unk[cy - 1, cx]:
                c += 1
            if cy < h - 1 and unk[cy + 1, cx]:
                c += 1
            if cx > 0 and unk[cy, cx - 1]:
                c += 1
            if cx < w - 1 and unk[cy, cx + 1]:
                c += 1
            out[i] = float(c)
        return out

    def _clearance_penalty(self, m: Optional[OccupancyGrid], x: float, y: float) -> float:
        clearance = self._goal_clearance_m(m, x, y)
        shortfall = max(0.0, self.goal_clearance_radius_m - clearance)
        return self.goal_clearance_weight * shortfall

    def _goal_passes_safety_gate(self, m: Optional[OccupancyGrid], x: float, y: float) -> bool:
        cost = self._goal_cell_cost(m, x, y)
        # Frontier targets can lie on cells that are still marked unknown due to
        # map discretization around the free/unknown boundary. Do not reject
        # solely for unknown cost; keep hard rejection only for high-cost cells.
        if cost >= self.goal_max_cell_cost:
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

    def _return_home_note_failure(self, rs: RobotState) -> None:
        if not rs.homing_active:
            return
        rs.homing_active = False
        rs.return_home_failures += 1
        self.get_logger().warn(
            f'[{rs.name}] Return home attempt failed '
            f'({rs.return_home_failures}/{self.return_to_start_max_attempts})'
        )

    def _blacklist_or_home_fail(
        self,
        rs: RobotState,
        xy: Optional[Tuple[float, float]],
    ) -> None:
        if xy is None:
            return
        if rs.homing_active:
            self._return_home_note_failure(rs)
            return
        if not self._should_skip_blacklist_due_to_lethal(rs):
            rs.blacklist.append(xy)
        else:
            rs.held_goal_position = xy

    def _arm_post_failure_cooldown(self, rs: RobotState) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        rs.next_assign_allowed_time = now + max(0.0, self.post_failure_cooldown_sec)

    def _should_skip_blacklist_due_to_lethal(self, rs: RobotState) -> bool:
        return (
            self.lethal_hold_enabled
            and self.skip_blacklist_on_nav2_lethal
            and rs.nav2_lethal_active
        )

    def _resend_held_goal(self, rs: RobotState) -> None:
        goal = rs.held_goal_position
        if goal is None:
            return
        gx, gy = goal
        self.get_logger().info(
            f'[{rs.name}] Nav2 lethal cleared; resending held goal ({gx:.2f}, {gy:.2f})'
        )
        rs.repeat_goal_count = 0
        rs.last_goal_world = None
        proxy_frontier = Frontier(
            centroid_world=(gx, gy),
            size=max(1, self.min_frontier_cells_for_goal),
            size_m=max(self.min_frontier_size, 0.01),
            cells=max(1, self.min_frontier_cells_for_goal),
            indices=None,
        )
        self._send_goal(rs, proxy_frontier)
        if rs.goal_active or rs.goal_pending:
            rs.held_goal_position = None

    def _nav2_lethal_callback(self, msg: Bool, robot_name: str):
        rs = self.robots.get(robot_name)
        if rs is None:
            return
        lethal_now = bool(msg.data)
        now = self.get_clock().now().nanoseconds / 1e9
        if lethal_now == rs.nav2_lethal_active:
            return
        rs.nav2_lethal_active = lethal_now
        if lethal_now:
            rs.nav2_lethal_since = now
            if rs.goal_position is not None:
                rs.held_goal_position = rs.goal_position
            self.get_logger().warn(
                f'[{rs.name}] Nav2 reports lethal-space state; holding current goal'
            )
            return
        rs.nav2_lethal_since = 0.0
        if rs.held_goal_position is not None:
            rs.next_assign_allowed_time = min(
                rs.next_assign_allowed_time,
                now + max(0.0, self.lethal_retry_delay_sec),
            )
            self.get_logger().info(
                f'[{rs.name}] Nav2 lethal-space state cleared'
            )

    def _robot_may_assign_now(self, rs: RobotState, now: float) -> bool:
        return now >= rs.next_assign_allowed_time

    def _goal_point_blacklisted(
        self, rs: RobotState, gx: float, gy: float
    ) -> bool:
        return any(
            _dist((gx, gy), bl) < self.blacklist_radius for bl in rs.blacklist)

    def _frontier_too_small_for_nav_goal(self, fr: Frontier) -> bool:
        if self.min_frontier_cells_for_goal <= 0:
            return False
        return fr.size < self.min_frontier_cells_for_goal

    def _should_skip_frontier_for_assignment(
        self, rs: RobotState, fr: Frontier
    ) -> bool:
        """True if the pose Nav2 would receive for this frontier is unusable."""
        if self._frontier_too_small_for_nav_goal(fr):
            self.get_logger().debug(
                f'[{rs.name}] skip frontier: {fr.size} cells < '
                f'min_frontier_cells_for_goal ({self.min_frontier_cells_for_goal})'
            )
            return True
        gx, gy = self._select_goal_point(rs, fr)
        if self._goal_point_blacklisted(rs, gx, gy):
            return True
        if not self._goal_passes_safety_gate(self._current_goal_map, gx, gy):
            return True
        return False

    def _frontier_penalty(self, rs: RobotState, fr: Frontier) -> float:
        gx, gy = self._select_goal_point(rs, fr)
        return self._clearance_penalty(self._current_goal_map, gx, gy)

    def _frontier_utility(self, rs: RobotState, fr: Frontier) -> float:
        if self._frontier_too_small_for_nav_goal(fr):
            return -1e9
        gx, gy = self._select_goal_point(rs, fr)
        if rs.position is None:
            return -1e9
        if not self._goal_passes_safety_gate(self._current_goal_map, gx, gy):
            self._log_goal_safety_rejection(
                rs, gx, gy, context='frontier_utility'
            )
            return -1e9
        d = _dist(rs.position, (gx, gy))
        # Degenerate only when essentially same cell; otherwise penalize
        # closeness so retargeting still works after the robot reaches a goal
        # on/near the frontier (same situation as assign_frontiers relaxed pass).
        degenerate_eps = 0.02
        if d < degenerate_eps:
            return -1e9
        close_penalty = 0.0
        if d < self.min_goal_separation:
            close_penalty = (self.min_goal_separation - d) * 3.0
        gain = self.gain_scale * fr.size_m
        cost = self.potential_scale * d
        penalty = self._clearance_penalty(self._current_goal_map, gx, gy)
        return gain - cost - penalty - close_penalty

    def _best_frontier_for_robot(self, rs: RobotState, frontiers: List[Frontier]) -> Tuple[Optional[Frontier], float]:
        best = None
        best_u = -1e9
        for fr in frontiers:
            if any(_dist(fr.centroid_world, bl) < self.blacklist_radius for bl in rs.blacklist):
                continue
            if self._should_skip_frontier_for_assignment(rs, fr):
                continue
            u = self._frontier_utility(rs, fr)
            if u > best_u:
                best_u = u
                best = fr
        return best, best_u

    def _best_frontier_for_robot_within_anchor(
        self, rs: RobotState, frontiers: List[Frontier]
    ) -> Tuple[Optional[Frontier], float]:
        """Like _best_frontier_for_robot but only frontiers whose goal pose lies
        within retarget_max_offset_from_anchor_m of the leg anchor (first goal),
        or the current goal if the anchor was not set.
        """
        if self.retarget_max_offset_from_anchor_m <= 0.0:
            return self._best_frontier_for_robot(rs, frontiers)
        anchor = rs.retarget_anchor or rs.goal_position
        if anchor is None:
            return self._best_frontier_for_robot(rs, frontiers)
        ax, ay = anchor
        rmax = self.retarget_max_offset_from_anchor_m
        best = None
        best_u = -1e9
        for fr in frontiers:
            if any(
                _dist(fr.centroid_world, bl) < self.blacklist_radius
                for bl in rs.blacklist
            ):
                continue
            if self._should_skip_frontier_for_assignment(rs, fr):
                continue
            gx, gy = self._select_goal_point(rs, fr)
            if _dist((gx, gy), (ax, ay)) > rmax:
                continue
            u = self._frontier_utility(rs, fr)
            if u > best_u:
                best_u = u
                best = fr
        return best, best_u

    def _robot_is_making_progress(self, rs: RobotState, now: float) -> bool:
        if (
            rs.last_motion_time > 0.0
            and (now - rs.last_motion_time) <= self.motion_progress_window_sec
        ):
            return True
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
        if (
            rs.last_motion_time > 0.0
            and (now - rs.last_motion_time) <= self.motion_progress_window_sec
        ):
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
        since_motion = (
            (now - rs.last_motion_time)
            if rs.last_motion_time > 0.0 else float('inf')
        )
        goal_delta = None
        if candidate_goal is not None and rs.goal_position is not None:
            goal_delta = _dist(candidate_goal, rs.goal_position)
        prefix = 'replan' if allow else 'skip_replan'
        details = (
            f'[{rs.name}] {prefix}: {reason}; elapsed_since_goal={elapsed:.1f}s; '
            f'since_progress={since_progress:.1f}s; '
            f'since_motion={since_motion:.1f}s; '
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

    def _should_replace_goal_opportunity(
        self,
        rs: RobotState,
        candidate_goal_xy: Tuple[float, float],
        now: float,
    ) -> Tuple[bool, str]:
        """Allow mid-route diversion to a better frontier while the robot is
        still advancing; does not use the 'robot_making_progress' block from
        _should_replace_goal (that gate is for *stagnation* retarget only).
        """
        if rs.goal_position is None or not rs.goal_active:
            return True, 'no_active_goal'
        if (now - rs.active_goal_sent_time) < self.min_goal_replan_interval_s:
            return False, 'cooldown'
        if _dist(candidate_goal_xy, rs.goal_position) < self.min_goal_change_dist_m:
            return False, 'goal_too_similar'
        return True, 'opportunity'

    def _robot_recently_showed_progress(self, rs: RobotState, now: float) -> bool:
        """True if either goal-distance or physical motion updated recently."""
        if (
            rs.last_motion_time > 0.0
            and (now - rs.last_motion_time) <= self.motion_progress_window_sec
        ):
            return True
        if rs.last_progress_time <= 0:
            return False
        return (now - rs.last_progress_time) < 4.0

    def _maybe_retarget_active_robot(self, rs: RobotState, frontiers: List[Frontier], now: float):
        if not self.retarget_enable:
            return
        if rs.homing_active:
            return
        if not rs.goal_active or rs.goal_pending or rs.position is None or rs.goal_position is None:
            return
        if not self._robot_may_assign_now(rs, now):
            return
        if (now - rs.last_retarget_time) < self.retarget_cooldown_sec:
            return

        current_goal = rs.goal_position
        best_fr, best_u = self._best_frontier_for_robot_within_anchor(rs, frontiers)
        if best_fr is None:
            if self.retarget_max_offset_from_anchor_m > 0.0:
                ax = rs.retarget_anchor or rs.goal_position
                if ax is not None:
                    self.get_logger().debug(
                        f'[{rs.name}] Retarget skipped: no frontier with goal within '
                        f'{self.retarget_max_offset_from_anchor_m:.1f}m of anchor '
                        f'({ax[0]:.2f}, {ax[1]:.2f})'
                    )
            return
        new_goal = self._select_goal_point(rs, best_fr)
        if _dist(new_goal, current_goal) < self.retarget_min_goal_shift_m:
            return

        curr_u = -(
            self.potential_scale * _dist(rs.position, current_goal)
            + self._clearance_penalty(self._current_goal_map, current_goal[0], current_goal[1])
        )

        # While the robot is moving toward the current goal, divert if the map
        # now favors a frontier with substantially higher exploration utility.
        if self.retarget_opportunity_enable:
            advancing = (
                self._robot_is_making_progress(rs, now)
                or self._robot_recently_showed_progress(rs, now)
            )
            if advancing and best_u >= (
                curr_u + self.retarget_opportunity_min_utility_gain
            ):
                ok, opp_reason = self._should_replace_goal_opportunity(
                    rs, new_goal, now)
                if ok:
                    self._log_replan_decision(
                        rs, opp_reason, allow=True, now=now, candidate_goal=new_goal)
                    self.get_logger().info(
                        f'[{rs.name}] Retargeting for better frontier while in motion; '
                        f'old=({current_goal[0]:.2f}, {current_goal[1]:.2f}) '
                        f'new=({new_goal[0]:.2f}, {new_goal[1]:.2f})'
                    )
                    rs.ignore_cooldown_on_next_cancel = True
                    self._cancel_goal(rs)
                    rs.last_retarget_time = now
                    self._send_goal(rs, best_fr, from_retarget=True)
                    return
                self._log_replan_decision(
                    rs, opp_reason, allow=False, now=now, candidate_goal=new_goal)

        curr_clearance = self._goal_clearance_m(
            self._current_goal_map, current_goal[0], current_goal[1])
        no_progress_for = now - (rs.last_progress_time or rs.last_goal_time or now)
        risky = curr_clearance < self.retarget_clearance_threshold_m
        stale = no_progress_for > self.retarget_stagnation_sec
        if not risky and not stale:
            return

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
        rs.ignore_cooldown_on_next_cancel = True
        self._cancel_goal(rs)
        rs.blacklist.append(current_goal)
        rs.last_retarget_time = now
        self._send_goal(rs, best_fr, from_retarget=True)

    def _pick_frontier_cell_farthest_from_robot(
        self,
        rs: RobotState,
        frontier: Frontier,
        centroid_xy: Tuple[float, float],
        max_offset_from_centroid: Optional[float],
    ) -> Optional[Tuple[float, float]]:
        """Pick a frontier cell that is far from the robot (for exploration).

        If max_offset_from_centroid is set, only cells within that radius of
        the frontier centroid are considered (centroid strategy fallback).
        If None, all frontier cells are considered (farthest_cell strategy).
        Returns None if no suitable cell exists.
        """
        if frontier.indices is None or rs.position is None:
            return None
        m = self._current_goal_map
        if m is None:
            return None
        rx, ry = rs.position
        ys = frontier.indices[:, 0].astype(np.float32)
        xs = frontier.indices[:, 1].astype(np.float32)
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        wx = ox + (xs + 0.5) * res
        wy = oy + (ys + 0.5) * res

        dx = wx - rx
        dy = wy - ry
        dists = np.hypot(dx, dy)
        grid = np.array(m.data, dtype=np.int16).reshape((m.info.height, m.info.width))
        costs = grid[ys.astype(np.int32), xs.astype(np.int32)]
        unk_adj = self._frontier_unknown_adjacency_counts(grid, ys, xs)
        penalty = max(0.0, self.goal_unknown_neighbor_penalty_m)

        def _score_pick(mask: np.ndarray) -> int:
            if penalty > 0.0:
                scores = dists - penalty * unk_adj
            else:
                scores = dists
            if not np.any(mask):
                return int(np.argmax(scores))
            return int(np.argmax(np.where(mask, scores, -np.inf)))

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

        if max_offset_from_centroid is not None:
            cdx = wx - centroid_xy[0]
            cdy = wy - centroid_xy[1]
            mask_near_centroid = np.hypot(cdx, cdy) <= max_offset_from_centroid
            valid_mask = valid_mask & mask_near_centroid
            if not np.any(valid_mask):
                return None

        mask_far_enough = dists >= max(0.0, self.min_goal_distance)
        safe_far = valid_mask & mask_far_enough
        if np.any(safe_far):
            idx = _score_pick(safe_far)
        elif np.any(valid_mask):
            idx = _score_pick(valid_mask)
        elif np.any(mask_far_enough):
            idx = _score_pick(mask_far_enough)
        else:
            idx = _score_pick(np.ones_like(dists, dtype=bool))

        gx = float(wx[idx])
        gy = float(wy[idx])
        if not np.any(valid_mask):
            self._log_goal_safety_rejection(
                rs, gx, gy, context='select_goal_point_fallback'
            )
        return gx, gy

    def _select_goal_point(self, rs: RobotState, frontier: Frontier) -> Tuple[float, float]:
        """Choose a goal point within a frontier region according to the
        configured strategy. Falls back to the centroid when needed.
        """
        gx, gy = frontier.centroid_world

        if self.goal_point_strategy == 'centroid':
            if (
                rs.position is not None
                and frontier.indices is not None
                and _dist(rs.position, (gx, gy))
                < self.centroid_fallback_if_robot_closer_than_m
            ):
                picked = self._pick_frontier_cell_farthest_from_robot(
                    rs,
                    frontier,
                    (gx, gy),
                    self.centroid_fallback_max_offset_from_centroid_m,
                )
                if picked is not None:
                    gx, gy = picked

        elif self.goal_point_strategy == 'farthest_cell':
            if frontier.indices is not None and rs.position is not None:
                picked = self._pick_frontier_cell_farthest_from_robot(
                    rs, frontier, (gx, gy), None
                )
                if picked is not None:
                    gx, gy = picked

        return gx, gy

    def _goal_dist_for_assignment(self, rname: str, fr: Frontier) -> float:
        """Straight-line distance from robot to the goal pose for this frontier."""
        rs = self.robots[rname]
        if rs.position is None:
            return float('inf')
        gx, gy = self._select_goal_point(rs, fr)
        return _dist(rs.position, (gx, gy))

    def _assignment_robot_order(self, idle_names: List[str]) -> List[str]:
        """Idle robots in ``robot_names`` order, rotated so the last robot
        that completed a goal is first — avoids greedy starvation of the
        active robot when other names are idle without Nav2.
        """
        ordered = [n for n in self.robot_names if n in idle_names]
        anchor = self._last_goal_finished_robot
        if anchor and anchor in ordered:
            i = ordered.index(anchor)
            return ordered[i:] + ordered[:i]
        return ordered

    def _at_nav_goal_for_tacit_hold(self, rs: RobotState, d: float) -> bool:
        """True if map-frame distance or Nav2 feedback says we're at the goal."""
        if d <= self.tacit_goal_success_radius_m:
            return True
        dr = rs.last_distance_remaining
        # Nav2 can report distance_remaining≈0 while map-frame pose is still far
        # (stale/initial feedback). Only use the remaining-distance hint when pose
        # is within a small combined slack of the goal.
        max_d_when_using_remaining = (
            self.tacit_goal_success_radius_m
            + self.tacit_abort_max_distance_remaining_m
        )
        if (
            self.tacit_abort_max_distance_remaining_m > 0.0
            and dr is not None
            and math.isfinite(dr)
            and dr >= 0.0
            and dr <= self.tacit_abort_max_distance_remaining_m
            and d <= max_d_when_using_remaining
        ):
            return True
        return False

    def _nav2_abort_tacit_success(self, rs: RobotState) -> Tuple[bool, str]:
        """True if Nav2 ABORTED (e.g. controller gave up) but we count the leg
        as reached: same criteria as plan-loop tacit hold.
        """
        if rs.goal_position is None:
            return False, ''
        d = (
            _dist(rs.position, rs.goal_position)
            if rs.position is not None
            else float('inf')
        )
        if not self._at_nav_goal_for_tacit_hold(rs, d):
            return False, ''
        if rs.position is not None and d <= self.tacit_goal_success_radius_m:
            return True, f'pose_dist={d:.2f}m'
        dr = rs.last_distance_remaining
        return True, f'distance_remaining={dr:.2f}m'

    def _log_why_no_assignment(
        self,
        frontiers: List[Frontier],
        n_idle: int,
        now: float,
    ) -> None:
        """Throttled diagnostic when frontiers exist but assign_frontiers is empty."""
        idle_names = [
            n for n in self.robot_names
            if n in self.robots
            and not self.robots[n].goal_active
            and not self.robots[n].goal_pending
            and self.robots[n].position is not None
            and self._robot_may_assign_now(self.robots[n], now)
        ]
        if not idle_names:
            self.get_logger().warning(
                'No assignment: idle count mismatch (robots in cooldown or '
                'missing TF pose?)'
            )
            return
        order = self._assignment_robot_order(idle_names)
        rs = self.robots[order[0]]
        fr = frontiers[0]
        gx, gy = self._select_goal_point(rs, fr)
        parts: List[str] = []
        if self._frontier_too_small_for_nav_goal(fr):
            parts.append(
                f'cells<{self.min_frontier_cells_for_goal}'
            )
        if any(
            _dist(fr.centroid_world, bl) < self.blacklist_radius
            for bl in rs.blacklist
        ):
            parts.append('centroid_near_blacklist')
        if self._goal_point_blacklisted(rs, gx, gy):
            parts.append('goal_blacklisted')
        if not self._goal_passes_safety_gate(self._current_goal_map, gx, gy):
            c = self._goal_clearance_m(self._current_goal_map, gx, gy)
            cc = self._goal_cell_cost(self._current_goal_map, gx, gy)
            parts.append(f'safety(clear={c:.2f}m cost={cc})')
        d = _dist(rs.position, (gx, gy)) if rs.position else None
        self.get_logger().warning(
            f'No frontier assigned: {n_idle} idle, {len(frontiers)} frontier(s). '
            f'Sample robot={rs.name} vs frontier[0]: goal=({gx:.2f},{gy:.2f}) '
            f'dist_to_goal={d:.3f}m min_sep={self.min_goal_separation} '
            f'issues=[{",".join(parts) if parts else "none — check utility matrix"}]'
        )

    def _repeat_goal_blocks_send(
        self,
        rs: RobotState,
        goal_x: float,
        goal_y: float,
        frontier: Frontier,
    ) -> bool:
        """If True, skip sending: blacklist exhausted goal and try elsewhere."""
        REPEAT_DIST_THRESH = 0.3
        REPEAT_LIMIT = 5
        if rs.last_goal_world is not None:
            lx, ly = rs.last_goal_world
            if math.hypot(goal_x - lx, goal_y - ly) < REPEAT_DIST_THRESH:
                rs.repeat_goal_count += 1
            else:
                rs.repeat_goal_count = 0
        else:
            rs.repeat_goal_count = 0

        if rs.repeat_goal_count < REPEAT_LIMIT:
            return False

        if rs.homing_active:
            self.get_logger().warn(
                f'[{rs.name}] Return-home goal repeated too often; counting failure'
            )
            self._return_home_note_failure(rs)
            rs.repeat_goal_count = 0
            return True

        self.get_logger().warn(
            f'[{rs.name}] Repeatedly selecting nearly-identical goal '
            f'({goal_x:.2f}, {goal_y:.2f}); blacklisting and searching elsewhere'
        )
        rs.blacklist.append((goal_x, goal_y))
        rs.blacklist.append(frontier.centroid_world)
        rs.repeat_goal_count = 0
        return True

    def _abort_path_precheck(
        self,
        rs: RobotState,
        *,
        cancel_action: bool,
        blacklist_xy: Optional[Tuple[float, float]],
        cooldown: bool,
    ) -> None:
        if cancel_action and rs.path_precheck_goal_handle is not None:
            try:
                rs.path_precheck_goal_handle.cancel_goal_async()
            except Exception:
                pass
        rs.path_precheck_goal_handle = None
        rs.path_precheck_in_progress = False
        rs.path_precheck_frontier = None
        rs.path_precheck_from_retarget = False
        rs.path_precheck_goal_xy = None
        rs.path_precheck_started_time = 0.0
        rs.goal_pending = False
        if blacklist_xy is not None:
            self._blacklist_or_home_fail(rs, blacklist_xy)
        if cooldown:
            self._arm_post_failure_cooldown(rs)

    def _path_precheck_transient_fail(self, rs: RobotState, detail: str) -> None:
        """Planner compute_path_to_pose rejected the goal or send failed.

        Nav2's planner server rejects goals while server_active_ is false
        (lifecycle still activating), even if wait_for_server() is true.
        Do not blacklist — retry shortly like NavigateToPose warm-up.
        """
        self._abort_path_precheck(
            rs,
            cancel_action=False,
            blacklist_xy=None,
            cooldown=False,
        )
        now = self.get_clock().now().nanoseconds / 1e9
        retry_s = 1.0
        if (
            rs.path_precheck_server_seen_time > 0.0
            and (now - rs.path_precheck_server_seen_time)
            < self.goal_reject_warmup_sec
        ):
            retry_s = 0.5
        rs.next_assign_allowed_time = max(
            rs.next_assign_allowed_time, now + retry_s)
        self.get_logger().info(
            f'[{rs.name}] Path precheck {detail}; '
            f'retry in {retry_s:.1f}s (planner may still be activating)'
        )

    def _begin_nav2_path_precheck(
        self,
        rs: RobotState,
        frontier: Frontier,
        goal_x: float,
        goal_y: float,
        from_retarget: bool,
    ) -> None:
        rs.goal_pending = True
        rs.goal_active = False
        rs.path_precheck_in_progress = True
        rs.path_precheck_frontier = frontier
        rs.path_precheck_from_retarget = from_retarget
        rs.path_precheck_goal_xy = (goal_x, goal_y)
        rs.path_precheck_started_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

        cp_goal = ComputePathToPose.Goal()
        cp_goal.use_start = False
        cp_goal.planner_id = self.nav2_path_precheck_planner_id
        cp_goal.goal.header.frame_id = self.world_frame
        cp_goal.goal.header.stamp = self.get_clock().now().to_msg()
        cp_goal.goal.pose.position.x = goal_x
        cp_goal.goal.pose.position.y = goal_y
        cp_goal.goal.pose.position.z = 0.0
        cp_goal.goal.pose.orientation.x = 0.0
        cp_goal.goal.pose.orientation.y = 0.0
        cp_goal.goal.pose.orientation.z = 0.0
        cp_goal.goal.pose.orientation.w = 1.0

        send_future = rs.path_precheck_client.send_goal_async(cp_goal)
        send_future.add_done_callback(
            lambda f, r=rs: self._path_precheck_goal_response_callback(f, r))
        self.get_logger().info(
            f'[{rs.name}] Nav2 path precheck → ({goal_x:.2f}, {goal_y:.2f})'
        )

    def _path_precheck_goal_response_callback(self, future, rs: RobotState):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'[{rs.name}] Path precheck send failed: {exc}')
            self._path_precheck_transient_fail(rs, f'send failed ({exc})')
            return

        if not goal_handle.accepted:
            self._path_precheck_transient_fail(
                rs, 'goal rejected (planner server inactive during lifecycle)')
            return

        rs.path_precheck_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, r=rs: self._path_precheck_result_callback(f, r))

    def _path_precheck_result_callback(self, future, rs: RobotState):
        rs.path_precheck_goal_handle = None
        rs.path_precheck_in_progress = False
        goal_xy = rs.path_precheck_goal_xy
        frontier = rs.path_precheck_frontier
        from_retarget = rs.path_precheck_from_retarget
        rs.path_precheck_frontier = None
        rs.path_precheck_goal_xy = None
        rs.path_precheck_from_retarget = False
        rs.path_precheck_started_time = 0.0
        rs.goal_pending = False

        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'[{rs.name}] Path precheck result error: {exc}')
            self._blacklist_or_home_fail(rs, goal_xy)
            self._arm_post_failure_cooldown(rs)
            return

        status = wrapped.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            labels = {
                GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED',
                GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            }
            st = labels.get(int(status), f'status_{int(status)}')
            self.get_logger().info(
                f'[{rs.name}] Path precheck failed ({st}); skipping NavigateToPose'
            )
            self._blacklist_or_home_fail(rs, goal_xy)
            self._arm_post_failure_cooldown(rs)
            return

        result = wrapped.result
        path = result.path if result is not None else None
        nposes = len(path.poses) if path is not None else 0
        if nposes < self.nav2_path_min_poses:
            self.get_logger().info(
                f'[{rs.name}] Path precheck: plan too short '
                f'({nposes} < {self.nav2_path_min_poses}); skipping goal'
            )
            self._blacklist_or_home_fail(rs, goal_xy)
            self._arm_post_failure_cooldown(rs)
            return

        if frontier is None or goal_xy is None:
            return
        goal_x, goal_y = goal_xy
        self._dispatch_navigate_to_pose(
            rs, frontier, goal_x, goal_y, from_retarget)

    def _dispatch_navigate_to_pose(
        self,
        rs: RobotState,
        frontier: Frontier,
        goal_x: float,
        goal_y: float,
        from_retarget: bool,
    ) -> None:
        rs.last_goal_world = (goal_x, goal_y)
        rs.recent_goals.append((goal_x, goal_y, float(frontier.size_m)))

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.world_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        rs.goal_active = True
        rs.goal_pending = True
        rs.goal_status = 'pending'
        rs.goal_seq_counter += 1
        goal_seq = rs.goal_seq_counter
        rs.active_goal_seq = goal_seq
        if from_retarget:
            rs.retarget_abort_blacklist_suppress_seq = goal_seq
        rs.goal_position = (goal_x, goal_y)
        now = self.get_clock().now().nanoseconds / 1e9
        rs.last_goal_time = now
        rs.active_goal_sent_time = now
        rs.last_pose_when_goal_sent = rs.position
        rs.last_distance_remaining = None
        rs.last_progress_distance = None
        rs.last_progress_time = now
        rs.last_motion_sample_pose = rs.position
        rs.last_motion_sample_time = now
        rs.last_motion_time = now
        if rs.position is not None:
            rs.last_dist_to_goal = _dist(rs.position, (goal_x, goal_y))
        else:
            rs.last_dist_to_goal = None
        rs.near_goal_since = None
        rs.tacit_success_pending_cancel = False
        if not from_retarget:
            rs.retarget_anchor = (goal_x, goal_y)

        send_future = rs.action_client.send_goal_async(
            goal_msg,
            feedback_callback=(
                lambda f, r=rs, seq=goal_seq:
                self._goal_feedback_callback(f, r, seq)
            ),
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
            lambda f, r=rs, seq=goal_seq: self._goal_response_callback(f, r, seq))
        self.get_logger().info(
            f'[{rs.name}] Sending NavigateToPose ({goal_x:.2f}, {goal_y:.2f}) '
            f'— frontier size {frontier.size_m:.2f}m ({frontier.size} cells)')

    def _send_goal(
        self,
        rs: RobotState,
        frontier: Frontier,
        from_retarget: bool = False,
    ):
        if rs.action_client is None:
            self.get_logger().warn(f'[{rs.name}] No action client available')
            rs.goal_active = False
            if rs.homing_active:
                self._return_home_note_failure(rs)
            return

        if not rs.action_client.wait_for_server(timeout_sec=0.2):
            if not rs.server_unavailable_logged:
                self.get_logger().warn(
                    f'[{rs.name}] NavigateToPose action server not available '
                    f'on /{rs.name}/navigate_to_pose')
                rs.server_unavailable_logged = True
            if self.use_pose_goal_fallback:
                self._publish_pose_fallback_goal(rs, frontier, from_retarget)
                return
            rs.goal_active = False
            rs.goal_pending = False
            if rs.homing_active:
                self._return_home_note_failure(rs)
            return
        if rs.server_unavailable_logged:
            self.get_logger().info(
                f'[{rs.name}] NavigateToPose action server is now available')
            rs.server_unavailable_logged = False
            rs.action_server_ready_time = (
                self.get_clock().now().nanoseconds / 1e9
            )
        elif rs.action_server_ready_time <= 0.0:
            # Server was already up when first checked.
            rs.action_server_ready_time = (
                self.get_clock().now().nanoseconds / 1e9
            )

        goal_x, goal_y = self._select_goal_point(rs, frontier)
        if self._repeat_goal_blocks_send(rs, goal_x, goal_y, frontier):
            return
        if self.goal_revalidate_before_send:
            if not self._goal_passes_safety_gate(self._current_goal_map, goal_x, goal_y):
                self._log_goal_safety_rejection(
                    rs, goal_x, goal_y, context='pre_send_revalidate'
                )
                self._blacklist_or_home_fail(rs, (goal_x, goal_y))
                self._arm_post_failure_cooldown(rs)
                self.get_logger().info(
                    f'[{rs.name}] Goal rejected by pre-send safety revalidation; '
                    'selecting another frontier'
                )
                return

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

        if (
            self.nav2_path_precheck_enable
            and rs.path_precheck_client is not None
        ):
            if rs.path_precheck_in_progress:
                return
            if rs.path_precheck_client.wait_for_server(timeout_sec=0.2):
                now_ts = self.get_clock().now().nanoseconds / 1e9
                if rs.path_precheck_server_seen_time <= 0.0:
                    rs.path_precheck_server_seen_time = now_ts
                self._begin_nav2_path_precheck(
                    rs, frontier, goal_x, goal_y, from_retarget)
                return
            self.get_logger().debug(
                f'[{rs.name}] compute_path_to_pose unavailable; '
                'NavigateToPose without precheck')

        self._dispatch_navigate_to_pose(
            rs, frontier, goal_x, goal_y, from_retarget)

    def _publish_pose_fallback_goal(
        self,
        rs: RobotState,
        frontier: Frontier,
        from_retarget: bool = False,
    ):
        goal_pub = self.goal_pubs.get(rs.name)
        if goal_pub is None:
            rs.goal_active = False
            rs.goal_pending = False
            return

        goal_x, goal_y = self._select_goal_point(rs, frontier)
        if self._repeat_goal_blocks_send(rs, goal_x, goal_y, frontier):
            rs.goal_active = False
            rs.goal_pending = False
            return

        rs.last_goal_world = (goal_x, goal_y)
        rs.recent_goals.append((goal_x, goal_y, float(frontier.size_m)))

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.world_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        goal_pub.publish(goal_msg)
        rs.goal_seq_counter += 1
        rs.active_goal_seq = rs.goal_seq_counter
        rs.goal_position = (goal_x, goal_y)
        now = self.get_clock().now().nanoseconds / 1e9
        rs.last_goal_time = now
        rs.active_goal_sent_time = now
        rs.last_pose_when_goal_sent = rs.position
        rs.last_progress_time = now
        rs.last_progress_distance = None
        rs.last_motion_sample_pose = rs.position
        rs.last_motion_sample_time = now
        rs.last_motion_time = now
        if rs.position is not None:
            rs.last_dist_to_goal = _dist(rs.position, (goal_x, goal_y))
        else:
            rs.last_dist_to_goal = None
        if not from_retarget:
            rs.retarget_anchor = (goal_x, goal_y)
        rs.goal_active = True
        rs.goal_pending = False
        rs.goal_status = 'executing'
        rs.goal_handle = None
        self.get_logger().warn(
            f'[{rs.name}] Falling back to PoseStamped goal_pose '
            f'({goal_x:.2f}, {goal_y:.2f})')

    def _goal_response_callback(self, future, rs: RobotState, goal_seq: int):
        if goal_seq != rs.active_goal_seq:
            self.get_logger().debug(
                f'[{rs.name}] Ignoring stale goal response (seq={goal_seq}, '
                f'active_seq={rs.active_goal_seq})'
            )
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'[{rs.name}] Failed to send NavigateToPose goal: {exc}')
            rs.goal_active = False
            rs.goal_pending = False
            rs.goal_status = 'failed'
            rs.goal_handle = None
            rs.active_goal_seq = 0
            if rs.homing_active:
                self._return_home_note_failure(rs)
            elif rs.goal_position is not None and not self._should_skip_blacklist_due_to_lethal(rs):
                rs.blacklist.append(rs.goal_position)
            elif rs.goal_position is not None:
                rs.held_goal_position = rs.goal_position
            self._arm_post_failure_cooldown(rs)
            return

        rs.goal_pending = False
        if not goal_handle.accepted:
            now = self.get_clock().now().nanoseconds / 1e9
            in_warmup = (
                rs.action_server_ready_time > 0.0
                and (now - rs.action_server_ready_time) < self.goal_reject_warmup_sec
            )
            if in_warmup:
                elapsed = now - rs.action_server_ready_time
                self.get_logger().warn(
                    f'[{rs.name}] Goal rejected by Nav2 during warm-up '
                    f'({elapsed:.1f}s since action server became available); '
                    'will retry without blacklisting.'
                )
            else:
                self.get_logger().warn(f'[{rs.name}] Goal rejected by Nav2')
            rs.goal_active = False
            rs.goal_status = 'failed'
            rs.active_goal_seq = 0
            if rs.goal_position is not None and not in_warmup:
                if rs.homing_active:
                    self._return_home_note_failure(rs)
                elif not self._should_skip_blacklist_due_to_lethal(rs):
                    rs.blacklist.append(rs.goal_position)
                else:
                    rs.held_goal_position = rs.goal_position
            if in_warmup:
                # Retry quickly while Nav2 finishes lifecycle activation.
                rs.next_assign_allowed_time = now + 1.0
            else:
                self._arm_post_failure_cooldown(rs)
            return

        rs.goal_handle = goal_handle
        rs.goal_status = 'executing'
        now = self.get_clock().now().nanoseconds / 1e9
        rs.last_goal_time = now
        rs.last_progress_time = now
        self.get_logger().debug(f'[{rs.name}] Goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, r=rs, seq=goal_seq: self._goal_result_callback(f, r, seq))

    def _goal_feedback_callback(self, feedback_msg, rs: RobotState, goal_seq: int):
        if goal_seq != rs.active_goal_seq:
            return
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

    def _goal_result_callback(self, future, rs: RobotState, goal_seq: int):
        if goal_seq != rs.active_goal_seq:
            self.get_logger().debug(
                f'[{rs.name}] Ignoring stale goal result (seq={goal_seq}, '
                f'active_seq={rs.active_goal_seq})'
            )
            return
        # Fresh pose for tacit near-goal checks on ABORTED/CANCELED.
        self._update_robot_positions()
        was_homing = rs.homing_active
        now = self.get_clock().now().nanoseconds / 1e9
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
            if was_homing:
                self._return_home_note_failure(rs)
            elif rs.goal_position and not self._should_skip_blacklist_due_to_lethal(rs):
                rs.blacklist.append(rs.goal_position)
            elif rs.goal_position:
                rs.held_goal_position = rs.goal_position
            rs.goal_active = False
            rs.goal_pending = False
            rs.goal_status = 'failed'
            rs.goal_handle = None
            rs.last_distance_remaining = None
            rs.last_progress_distance = None
            rs.last_progress_time = 0.0
            rs.last_dist_to_goal = None
            rs.last_motion_time = 0.0
            rs.last_motion_sample_pose = rs.position
            rs.last_motion_sample_time = now
            self._arm_post_failure_cooldown(rs)
            return

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            if was_homing:
                rs.homing_active = False
                rs.return_home_done = True
                rs.goals_reached += 1
                rs.goal_status = 'succeeded'
                self.get_logger().info(
                    f'[{rs.name}] Return home succeeded (NavigateToPose)'
                )
                self._last_goal_finished_robot = rs.name
            else:
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
                    self._arm_post_failure_cooldown(rs)
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
                    self._last_goal_finished_robot = rs.name
        elif status == GoalStatus.STATUS_ABORTED:
            tacit_ok, tacit_why = self._nav2_abort_tacit_success(rs)
            if tacit_ok:
                if was_homing:
                    rs.homing_active = False
                    rs.return_home_done = True
                rs.goals_reached += 1
                rs.goal_status = 'succeeded'
                if was_homing:
                    self.get_logger().info(
                        f'[{rs.name}] Return home succeeded (Nav2 tacit: {tacit_why}; '
                        f'total: {rs.goals_reached})'
                    )
                else:
                    self.get_logger().info(
                        f'[{rs.name}] Goal reached (Nav2 ABORTED; tacit success '
                        f'{tacit_why}; total: {rs.goals_reached})'
                    )
                self._last_goal_finished_robot = rs.name
            elif rs.tacit_success_pending_cancel:
                # Client cancel for tacit "at goal"; bt_navigator may end as ABORTED.
                rs.tacit_success_pending_cancel = False
                rs.ignore_cooldown_on_next_cancel = False
                if was_homing:
                    rs.homing_active = False
                    rs.return_home_done = True
                rs.goals_reached += 1
                rs.goal_status = 'succeeded'
                if was_homing:
                    self.get_logger().info(
                        f'[{rs.name}] Return home succeeded (tacit cancel as ABORTED; '
                        f'total: {rs.goals_reached})'
                    )
                else:
                    self.get_logger().info(
                        f'[{rs.name}] Goal reached (tacit cancel as ABORTED near pose; '
                        f'total: {rs.goals_reached})'
                    )
                self._last_goal_finished_robot = rs.name
            elif (
                not was_homing
                and rs.retarget_abort_blacklist_suppress_seq is not None
                and goal_seq == rs.retarget_abort_blacklist_suppress_seq
            ):
                # Replacement goal right after retarget: Nav2 often reports ABORTED
                # ("Aborting handle") while tearing down the cancelled goal. Scoped
                # to this goal_seq so a later real failure is not misclassified.
                rs.ignore_cooldown_on_next_cancel = False
                rs.goal_status = 'canceled'
                self.get_logger().info(
                    f'[{rs.name}] NavigateToPose ended as ABORTED after our cancel '
                    f'(retarget); not blacklisting'
                )
            else:
                rs.goals_failed += 1
                rs.goal_status = 'aborted'
                pd = (
                    _dist(rs.position, rs.goal_position)
                    if rs.position is not None and rs.goal_position is not None
                    else None
                )
                if self._should_skip_blacklist_due_to_lethal(rs):
                    self.get_logger().warn(
                        f'[{rs.name}] Goal aborted while Nav2 lethal hold active; '
                        'keeping goal and skipping blacklist'
                    )
                    if rs.goal_position:
                        rs.held_goal_position = rs.goal_position
                else:
                    self.get_logger().warn(
                        f'[{rs.name}] Goal aborted — blacklisting '
                        f'(pose_dist_m={pd}, distance_remaining={rs.last_distance_remaining}; '
                        f'tacit: pose≤{self.tacit_goal_success_radius_m:.2f}m or '
                        f'remain≤{self.tacit_abort_max_distance_remaining_m:.2f}m)'
                    )
                    if rs.goal_position:
                        self._blacklist_or_home_fail(rs, rs.goal_position)
                self._arm_post_failure_cooldown(rs)
        elif status == GoalStatus.STATUS_CANCELED:
            if rs.tacit_success_pending_cancel:
                rs.tacit_success_pending_cancel = False
                rs.ignore_cooldown_on_next_cancel = False
                if was_homing:
                    rs.homing_active = False
                    rs.return_home_done = True
                rs.goals_reached += 1
                rs.goal_status = 'succeeded'
                if was_homing:
                    self.get_logger().info(
                        f'[{rs.name}] Return home succeeded (tacit cancel; '
                        f'total: {rs.goals_reached})'
                    )
                else:
                    self.get_logger().info(
                        f'[{rs.name}] Goal reached (tacit cancel near pose; '
                        f'total: {rs.goals_reached})'
                    )
                self._last_goal_finished_robot = rs.name
            else:
                rs.goal_status = 'canceled'
                self.get_logger().info(f'[{rs.name}] Goal cancelled')
                if rs.ignore_cooldown_on_next_cancel:
                    rs.ignore_cooldown_on_next_cancel = False
                elif was_homing:
                    self._return_home_note_failure(rs)
                    self._arm_post_failure_cooldown(rs)
                else:
                    self._arm_post_failure_cooldown(rs)
        else:
            rs.goal_status = 'failed'
            self.get_logger().info(
                f'[{rs.name}] Goal finished with status {status}')
            if was_homing:
                self._return_home_note_failure(rs)
            self._arm_post_failure_cooldown(rs)

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

        if rs.retarget_abort_blacklist_suppress_seq == goal_seq:
            rs.retarget_abort_blacklist_suppress_seq = None

        rs.goal_active = False
        rs.goal_pending = False
        rs.goal_handle = None
        rs.active_goal_seq = 0
        rs.last_distance_remaining = None
        rs.last_progress_distance = None
        rs.last_progress_time = 0.0
        rs.last_dist_to_goal = None
        rs.last_motion_time = 0.0
        rs.last_motion_sample_pose = rs.position
        rs.last_motion_sample_time = now
        rs.near_goal_since = None

    def _cancel_goal(self, rs: RobotState):
        now = self.get_clock().now().nanoseconds / 1e9
        if rs.path_precheck_in_progress:
            self._abort_path_precheck(
                rs,
                cancel_action=True,
                blacklist_xy=None,
                cooldown=False,
            )
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
        rs.last_dist_to_goal = None
        rs.last_motion_time = 0.0
        rs.last_motion_sample_pose = rs.position
        rs.last_motion_sample_time = now
        rs.near_goal_since = None

    # -----------------------------------------------------------------------
    # Visualisation
    # -----------------------------------------------------------------------

    def _publish_frontier_markers(self, frontiers: List[Frontier]):
        ma = MarkerArray()
        marker_frame = self.world_frame

        # delete old markers
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        del_marker.header.frame_id = marker_frame
        del_marker.ns = 'frontiers'
        # RViz can warn/drop markers if nested texture headers keep empty frame_id.
        del_marker.texture.header.frame_id = marker_frame
        ma.markers.append(del_marker)

        for i, fr in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = marker_frame
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
            m.texture.header.frame_id = marker_frame
            ma.markers.append(m)

        # mark assigned goals in a different colour
        idx = len(frontiers) + 2
        for rs in self.robots.values():
            if rs.goal_active and rs.goal_position:
                gm = Marker()
                gm.header.frame_id = marker_frame
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
                gm.texture.header.frame_id = marker_frame
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
