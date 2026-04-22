#!/usr/bin/env python3
"""
Simple helper node that monitors TF between the global 'map' frame and each
robot's '<robot>/map' frame and publishes a coarse merge state string that the
multi_robot_explorer can use to decide when to switch from per-robot local
exploration to global merged exploration.

State machine:
  - NO_OVERLAP : initial state, or transforms are missing for some robots.
  - PARTIAL    : transforms exist for all robots but have not yet stabilised.
  - MERGED     : transforms exist for all robots and have remained within a
                 motion threshold (with debouncing) for a configured dwell time.

  Motion debouncing: a single tick of map->robot/map motion above the threshold
  does not drop MERGED; several consecutive violations are required. This pairs
  with multi_robot_explorer merge_degrade_grace_sec to reduce MERGED<->PARTIAL
  flapping when map_merge refines weak feature matches.

  Pose health: map_merge publishes ``/robot_tf_estimate_health`` (``robot:0|1``).
  A ``0`` means that cycle reused a provisional / last TF instead of a fresh
  metric estimate — MERGED is capped at PARTIAL so the explorer does not rely on
  global mode while inter-map poses are invalid.

This avoids having to introspect internal map_merge diagnostics while still
providing a robust signal for "maps are now consistently merged".
"""

import math
import time
from typing import Dict, List, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String

import tf2_ros


class MapMergeStateMonitor(Node):
    def __init__(self) -> None:
        super().__init__('map_merge_state_monitor')

        self.declare_parameter('robot_names', ['blinky', 'pinky', 'inky', 'clyde'])
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('robot_map_suffix', '/map')
        # Maximum allowed motion (m) for a robot map frame between consecutive
        # samples before we consider the estimate unstable. map_merge can jitter
        # more than a few cm per second while refining overlap; keep this loose
        # enough that MERGED does not flap on every weak estimate tick.
        self.declare_parameter('stability_pos_threshold', 0.28)
        # Dwell time (s) for which all robot map frames must remain within the
        # stability threshold in order to declare MERGED.
        self.declare_parameter('merged_dwell_time', 12.0)
        # Require this many consecutive motion-violation ticks before leaving
        # MERGED for PARTIAL (ignores single-sample TF spikes).
        self.declare_parameter('motion_jitter_confirm_ticks', 3)
        # Publish rate (Hz).
        self.declare_parameter('rate', 1.0)
        # map_merge publishes ``robot:0|1`` on this topic (1 = fresh metric TF).
        self.declare_parameter('robot_tf_health_topic', '/robot_tf_estimate_health')
        # If no health message for this long (seconds), ignore health gating (older map_merge).
        self.declare_parameter('robot_tf_health_stale_sec', 15.0)
        # Prevent early false MERGED: require each robot map to have grown enough.
        self.declare_parameter('local_map_topic_pattern', '/{robot}/map')
        # Minimum known cells (>=0 occupancy value) each robot map must contain
        # before MERGED can be emitted.
        self.declare_parameter('min_known_cells_per_robot', 400)
        # Also require minimum explored area (known cells * resolution^2).
        self.declare_parameter('min_known_area_m2_per_robot', 1.0)

        self.robot_names: List[str] = (
            self.get_parameter('robot_names').value
        )
        self.world_frame: str = self.get_parameter('world_frame').value
        self.robot_map_suffix: str = (
            self.get_parameter('robot_map_suffix').value
        )
        self.stability_pos_threshold: float = float(
            self.get_parameter('stability_pos_threshold').value
        )
        self.merged_dwell_time: float = float(
            self.get_parameter('merged_dwell_time').value
        )
        self.motion_jitter_confirm_ticks: int = max(
            1, int(self.get_parameter('motion_jitter_confirm_ticks').value)
        )
        rate_hz: float = float(self.get_parameter('rate').value)
        self._health_topic: str = str(
            self.get_parameter('robot_tf_health_topic').value
        )
        self._health_stale_sec: float = float(
            self.get_parameter('robot_tf_health_stale_sec').value
        )
        self._local_map_topic_pattern: str = str(
            self.get_parameter('local_map_topic_pattern').value
        )
        self._min_known_cells_per_robot: int = max(
            0, int(self.get_parameter('min_known_cells_per_robot').value)
        )
        self._min_known_area_m2_per_robot: float = max(
            0.0, float(self.get_parameter('min_known_area_m2_per_robot').value)
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state_pub = self.create_publisher(String, 'map_merge/merge_state', 10)

        self._health_by_robot: Dict[str, bool] = {}
        self._health_rx_wall_time: float = 0.0
        self.create_subscription(
            String,
            self._health_topic,
            self._on_tf_health,
            10,
        )
        self._known_cells_by_robot: Dict[str, int] = {}
        self._known_area_by_robot: Dict[str, float] = {}
        for name in self.robot_names:
            topic = self._local_map_topic_pattern.format(robot=name)
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, robot=name: self._on_local_map(robot, msg),
                10,
            )

        # Track last seen positions and timestamps per robot map frame so we
        # can reason about motion over time.
        self._last_positions: Dict[str, Tuple[float, float]] = {}
        self._last_stable_time: float = 0.0
        self._current_state: str = 'NO_OVERLAP'
        self._motion_bad_streak: int = 0

        period = 1.0 / rate_hz if rate_hz > 0 else 1.0
        self.timer = self.create_timer(period, self._tick)

    def _on_tf_health(self, msg: String) -> None:
        self._health_rx_wall_time = time.monotonic()
        self._health_by_robot.clear()
        for part in msg.data.split(';'):
            part = part.strip()
            if not part or ':' not in part:
                continue
            name, flag = part.split(':', 1)
            name = name.strip()
            try:
                self._health_by_robot[name] = bool(int(flag.strip()))
            except ValueError:
                self._health_by_robot[name] = False

    def _pose_tf_estimates_ok(self) -> bool:
        """False when map_merge reports any robot reusing stale TF / bad estimate."""
        if self._health_stale_sec <= 0.0 or not self._health_by_robot:
            return True
        age = time.monotonic() - self._health_rx_wall_time
        if age > self._health_stale_sec or self._health_rx_wall_time <= 0.0:
            return True
        for name in self.robot_names:
            ok = self._health_by_robot.get(name)
            if ok is None:
                return False
            if not ok:
                return False
        return True

    def _on_local_map(self, robot: str, msg: OccupancyGrid) -> None:
        known_cells = 0
        for cell in msg.data:
            if cell >= 0:
                known_cells += 1
        self._known_cells_by_robot[robot] = known_cells
        self._known_area_by_robot[robot] = known_cells * float(msg.info.resolution) ** 2

    def _maps_mature_for_merge(self) -> bool:
        """Require each robot map to grow beyond a minimum known footprint."""
        if self._min_known_cells_per_robot <= 0 and self._min_known_area_m2_per_robot <= 0.0:
            return True
        for name in self.robot_names:
            known_cells = self._known_cells_by_robot.get(name, 0)
            known_area = self._known_area_by_robot.get(name, 0.0)
            if known_cells < self._min_known_cells_per_robot:
                return False
            if known_area < self._min_known_area_m2_per_robot:
                return False
        return True

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        all_present = True
        motion_within_thresh = True

        for name in self.robot_names:
            child_frame = f'{name}{self.robot_map_suffix}'
            try:
                t = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    child_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
            except Exception:
                all_present = False
                motion_within_thresh = False
                continue

            pos = (
                t.transform.translation.x,
                t.transform.translation.y,
            )
            last = self._last_positions.get(child_frame)
            if last is not None:
                dx = pos[0] - last[0]
                dy = pos[1] - last[1]
                dist = math.hypot(dx, dy)
                if dist > self.stability_pos_threshold:
                    motion_within_thresh = False
            self._last_positions[child_frame] = pos

        new_state = self._current_state
        if not all_present:
            self._motion_bad_streak = 0
            new_state = 'NO_OVERLAP'
            self._last_stable_time = now
        elif not motion_within_thresh:
            self._motion_bad_streak += 1
            motion_acceptable = (
                self._motion_bad_streak < self.motion_jitter_confirm_ticks
            )
        else:
            self._motion_bad_streak = 0
            motion_acceptable = True

        if all_present:
            if not motion_within_thresh and not motion_acceptable:
                new_state = 'PARTIAL'
                self._last_stable_time = now
            elif motion_within_thresh or motion_acceptable:
                # All frames present; motion is either fine or within debounce.
                if (now - self._last_stable_time) >= self.merged_dwell_time:
                    new_state = 'MERGED'
                else:
                    new_state = 'PARTIAL'

        pose_tf_ok = self._pose_tf_estimates_ok()
        maps_mature = self._maps_mature_for_merge()
        if new_state == 'MERGED' and not pose_tf_ok:
            new_state = 'PARTIAL'
        if new_state == 'MERGED' and not maps_mature:
            new_state = 'PARTIAL'

        if new_state != self._current_state:
            self.get_logger().info(
                f'Merge state changed: {self._current_state} -> {new_state}'
            )
            self._current_state = new_state

        msg = String()
        msg.data = self._current_state
        try:
            self.state_pub.publish(msg)
        except Exception:
            # During shutdown the context can become invalid; ignore publish
            # errors in that phase.
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapMergeStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Treat Ctrl+C as a normal exit path.
        pass
    except Exception:
        # Treat unexpected executor/context errors during shutdown as benign.
        pass
    finally:
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

