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
                 small motion threshold for a configured dwell time.

This avoids having to introspect internal map_merge diagnostics while still
providing a robust signal for "maps are now consistently merged".
"""

import math
from typing import Dict, List, Tuple

import rclpy
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
        # Maximum allowed motion (m) for a robot map frame over the dwell time
        # window before we consider the estimate to be unstable.
        self.declare_parameter('stability_pos_threshold', 0.10)
        # Dwell time (s) for which all robot map frames must remain within the
        # stability threshold in order to declare MERGED.
        self.declare_parameter('merged_dwell_time', 10.0)
        # Publish rate (Hz).
        self.declare_parameter('rate', 1.0)

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
        rate_hz: float = float(self.get_parameter('rate').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state_pub = self.create_publisher(String, 'map_merge/merge_state', 10)

        # Track last seen positions and timestamps per robot map frame so we
        # can reason about motion over time.
        self._last_positions: Dict[str, Tuple[float, float]] = {}
        self._last_stable_time: float = 0.0
        self._current_state: str = 'NO_OVERLAP'

        period = 1.0 / rate_hz if rate_hz > 0 else 1.0
        self.timer = self.create_timer(period, self._tick)

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        all_present = True
        all_within_thresh = True

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
                all_within_thresh = False
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
                    all_within_thresh = False
            self._last_positions[child_frame] = pos

        new_state = self._current_state
        if not all_present:
            new_state = 'NO_OVERLAP'
            self._last_stable_time = now
        elif not all_within_thresh:
            new_state = 'PARTIAL'
            self._last_stable_time = now
        else:
            # All frames present and positions stable; check dwell time.
            if (now - self._last_stable_time) >= self.merged_dwell_time:
                new_state = 'MERGED'
            else:
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

