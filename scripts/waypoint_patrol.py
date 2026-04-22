#!/usr/bin/env python3
"""
Nav2 shape waypoint patrol.

Sends sequential NavigateToPose goals to the robot's Nav2 stack, cycling
through waypoints that form a rectangle, circle, or 5-pointed star. Shapes
are centered at the robot's current pose at start (from TF map -> base_footprint).

Requires:
  - Terminal 1: ros2 launch turtlebot3_bringup robot.launch.py
  - Terminal 2: ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py use_sim_time:=False use_rviz:=False
  - Run this script (Terminal 3) on the robot or a PC with the same ROS_DOMAIN_ID.
"""

import argparse
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros


FT_TO_M = 0.3048


def rectangle_waypoints(
    cx: float, cy: float,
    length_ft: float = 3.0, width_ft: float = 1.5,
) -> List[Tuple[float, float]]:
    """Four corners of a rectangle (closed loop). Length and width in feet."""
    L = length_ft * FT_TO_M
    W = width_ft * FT_TO_M
    half_l = L / 2.0
    half_w = W / 2.0
    return [
        (cx - half_l, cy - half_w),
        (cx + half_l, cy - half_w),
        (cx + half_l, cy + half_w),
        (cx - half_l, cy + half_w),
        (cx - half_l, cy - half_w),  # close loop
    ]


def circle_waypoints(
    cx: float, cy: float,
    diameter_ft: float = 2.0,
    num_points: int = 16,
) -> List[Tuple[float, float]]:
    """Points on a circle (closed loop). Diameter in feet."""
    r = (diameter_ft / 2.0) * FT_TO_M
    waypoints = []
    for i in range(num_points + 1):  # +1 to close (first point repeated)
        angle = 2.0 * math.pi * i / num_points
        waypoints.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
    return waypoints


def star_waypoints(
    cx: float, cy: float,
    size_ft: float = 2.0,
) -> List[Tuple[float, float]]:
    """5-pointed star: outer radius = size_ft/2 (in feet), inner at half radius. 10 vertices + close."""
    outer_r = (size_ft / 2.0) * FT_TO_M
    inner_r = outer_r * 0.5
    waypoints = []
    for i in range(10):
        angle_deg = 90 + i * 36  # start at top (90°), step 36° (5-point star)
        angle_rad = math.radians(angle_deg)
        r = outer_r if i % 2 == 0 else inner_r
        waypoints.append((cx + r * math.cos(angle_rad), cy + r * math.sin(angle_rad)))
    waypoints.append(waypoints[0])  # close loop
    return waypoints


class WaypointPatrolNode(Node):
    def __init__(self, args):
        super().__init__("waypoint_patrol")
        self.args = args
        self.frame_id = args.frame_id
        self.goal_timeout_sec = args.goal_timeout
        self.cycles_remaining = args.cycles  # 0 = infinite
        self.current_goal_handle = None

        self._action_client = ActionClient(self, NavigateToPose, args.action_name)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def wait_for_action_server(self) -> bool:
        self.get_logger().info(f"Waiting for action server '{self.args.action_name}'...")
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Action server not available.")
            return False
        self.get_logger().info("Action server available.")
        return True

    def get_robot_pose_in_map(self) -> Tuple[float, float]:
        try:
            t = self._tf_buffer.lookup_transform(
                self.frame_id, "base_footprint", rclpy.time.Time(),
                timeout=Duration(seconds=5.0),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}. Using (0, 0) as center.")
            return (0.0, 0.0)

    def build_waypoints(self) -> List[Tuple[float, float]]:
        cx, cy = self.get_robot_pose_in_map()
        self.get_logger().info(f"Shape center (map): x={cx:.3f}, y={cy:.3f}")

        if self.args.shape == "rectangle":
            return rectangle_waypoints(
                cx, cy,
                length_ft=self.args.rectangle_length_ft,
                width_ft=self.args.rectangle_width_ft,
            )
        elif self.args.shape == "circle":
            return circle_waypoints(
                cx, cy,
                diameter_ft=self.args.circle_diameter_ft,
                num_points=self.args.circle_points,
            )
        elif self.args.shape == "star":
            return star_waypoints(cx, cy, size_ft=self.args.star_size_ft)
        else:
            raise ValueError(f"Unknown shape: {self.args.shape}")

    def send_goal_and_wait(self, waypoint: Tuple[float, float]) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint[0]
        goal_msg.pose.pose.position.y = waypoint[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0

        self.get_logger().info(f"Sending goal ({waypoint[0]:.3f}, {waypoint[1]:.3f})")
        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=None,
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.result():
            self.get_logger().error("Send goal failed (server rejected or timeout).")
            return False

        self.current_goal_handle = send_future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            self.current_goal_handle = None
            return False

        result_future = self.current_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future,
            timeout_sec=max(self.goal_timeout_sec, 1.0),
        )
        self.current_goal_handle = None

        if not result_future.done() or not result_future.result():
            self.get_logger().warn("Goal result timeout or invalid.")
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached.")
            return True
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted.")
            return False
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal canceled.")
            return False
        else:
            self.get_logger().warn(f"Goal ended with status {status}.")
            return False

    def cancel_current_goal(self):
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

    def run(self) -> None:
        if not self.wait_for_action_server():
            return

        waypoints = self.build_waypoints()
        self.get_logger().info(f"Patrol shape '{self.args.shape}' with {len(waypoints)} waypoints, cycles={self.args.cycles} (0=infinite).")

        cycle = 0
        while True:
            cycle += 1
            if self.cycles_remaining != 0 and cycle > self.cycles_remaining:
                self.get_logger().info(f"Completed {self.cycles_remaining} cycle(s).")
                break

            if self.cycles_remaining == 0:
                self.get_logger().info(f"Starting cycle {cycle} (infinite mode).")
            else:
                self.get_logger().info(f"Starting cycle {cycle}/{self.cycles_remaining}.")

            for i, wp in enumerate(waypoints):
                if not rclpy.ok():
                    return
                success = self.send_goal_and_wait(wp)
                if not success and self.args.stop_on_failure:
                    self.get_logger().warn("Stopping patrol due to goal failure.")
                    return


def parse_args():
    p = argparse.ArgumentParser(description="Nav2 shape waypoint patrol")
    p.add_argument("--shape", choices=["rectangle", "circle", "star"], default="rectangle",
                   help="Shape to patrol (default: rectangle)")
    p.add_argument("--cycles", type=int, default=0,
                   help="Number of cycles (0 = infinite, default: 0)")
    p.add_argument("--goal-timeout", type=float, default=120.0,
                   help="Timeout per goal in seconds (default: 120)")
    p.add_argument("--frame-id", default="map",
                   help="Frame for goals (default: map)")
    p.add_argument("--action-name", default="navigate_to_pose",
                   help="Nav2 NavigateToPose action name (default: navigate_to_pose)")
    p.add_argument("--stop-on-failure", action="store_true",
                   help="Stop patrol on first goal failure")
    # Shape overrides (in feet)
    p.add_argument("--rectangle-length-ft", type=float, default=3.0)
    p.add_argument("--rectangle-width-ft", type=float, default=1.5)
    p.add_argument("--circle-diameter-ft", type=float, default=2.0)
    p.add_argument("--circle-points", type=int, default=16)
    p.add_argument("--star-size-ft", type=float, default=2.0)
    return p.parse_args()


def main(args=None):
    parsed = parse_args()
    rclpy.init(args=args)
    node = WaypointPatrolNode(parsed)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; canceling current goal.")
        node.cancel_current_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
