#!/usr/bin/env python3
"""
Pose Jump Monitor
=================

Simple ROS 2 node that watches the robot pose and reports how far it
moves between successive pose updates. This is useful for diagnosing
SLAM / localization "jumps" where the robot suddenly appears in a
different place on the map.

By default it:
- Looks up the TF from "map" -> "base_footprint"
- Polls at 10 Hz
- Logs only when the linear distance between successive poses is
  greater than or equal to 0.10 m

You can override defaults with environment variables:
- POSE_JUMP_MAP_FRAME       (default: "map")
- POSE_JUMP_BASE_FRAME      (default: "base_footprint")
- POSE_JUMP_MIN_DISTANCE_M  (default: "0.10")
- POSE_JUMP_RATE_HZ         (default: "10.0")

Usage (on the Remote PC, with your usual environment):

  cd ~/turtlebot3_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash        # if you built the workspace
  source scripts/set_robot_env.sh blinky   # or your robot

  python3 scripts/pose_jump_monitor.py

Leave it running while SLAM + Nav2 + explorer are active. Whenever the
robot pose in the map frame jumps by more than the threshold, you'll
see a log like:

  [INFO] [pose_jump_monitor]: JUMP #12: dt=0.10s dist=0.42 m
      from (x=1.234, y=0.567) to (x=1.640, y=0.580)
"""

import math
import os
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class PoseJumpMonitor(Node):
    def __init__(self) -> None:
        super().__init__("pose_jump_monitor")

        # Configuration from environment with sensible defaults
        self._map_frame = os.environ.get("POSE_JUMP_MAP_FRAME", "map")
        self._base_frame = os.environ.get("POSE_JUMP_BASE_FRAME", "base_footprint")

        try:
            self._min_distance = float(os.environ.get("POSE_JUMP_MIN_DISTANCE_M", "0.10"))
        except ValueError:
            self._min_distance = 0.10

        try:
            self._rate_hz = float(os.environ.get("POSE_JUMP_RATE_HZ", "10.0"))
        except ValueError:
            self._rate_hz = 10.0

        if self._rate_hz <= 0.0:
            self._rate_hz = 10.0

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

        self._prev_pose: Optional[Tuple[float, float, float]] = None  # (x, y, t_sec)
        self._jump_count: int = 0
        self._total_jump_distance: float = 0.0
        self._max_jump_distance: float = 0.0

        self.get_logger().info(
            f"Starting pose jump monitor: map_frame='{self._map_frame}', "
            f"base_frame='{self._base_frame}', min_distance={self._min_distance:.3f} m, "
            f"rate={self._rate_hz:.1f} Hz"
        )

        timer_period = 1.0 / self._rate_hz
        self._timer = self.create_timer(timer_period, self._on_timer)

    def _on_timer(self) -> None:
        # Query the latest available transform from map -> base
        try:
            tf = self._buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            # Don't spam logs every cycle; only warn occasionally.
            self.get_logger().debug(f"TF lookup failed: {ex}")
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        t_sec = float(tf.header.stamp.sec) + float(tf.header.stamp.nanosec) * 1e-9

        if self._prev_pose is None:
            self._prev_pose = (x, y, t_sec)
            return

        px, py, pt = self._prev_pose
        dx = x - px
        dy = y - py
        dist = math.hypot(dx, dy)
        dt = t_sec - pt

        if dist >= self._min_distance:
            self._jump_count += 1
            self._total_jump_distance += dist
            if dist > self._max_jump_distance:
                self._max_jump_distance = dist

            self.get_logger().info(
                f"JUMP #{self._jump_count}: dt={dt:.2f}s dist={dist:.3f} m\n"
                f"    from (x={px:.3f}, y={py:.3f}) to (x={x:.3f}, y={y:.3f})"
            )

        # Update previous pose regardless of whether it was a "jump"
        self._prev_pose = (x, y, t_sec)

    def print_summary(self) -> None:
        if self._jump_count == 0:
            self.get_logger().info("No jumps detected (>= min_distance threshold).")
            return

        avg = self._total_jump_distance / float(self._jump_count)
        self.get_logger().info(
            "===== Pose Jump Summary =====\n"
            f"  Total jumps:     {self._jump_count}\n"
            f"  Min distance:    {self._min_distance:.3f} m\n"
            f"  Max jump:        {self._max_jump_distance:.3f} m\n"
            f"  Avg jump:        {avg:.3f} m\n"
        )


def main() -> int:
    rclpy.init()
    node = PoseJumpMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User requested shutdown (Ctrl+C) — fall through to summary + cleanup.
        pass
    finally:
        # Print a concise summary of what we observed before shutting down.
        try:
            node.print_summary()
        except Exception:
            # Don't let summary errors interfere with clean shutdown.
            node.get_logger().warn("Failed to print pose jump summary during shutdown.")

        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

