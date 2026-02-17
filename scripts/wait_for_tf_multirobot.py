#!/usr/bin/env python3
"""
Wait for the multi-robot TF tree before starting Nav2.

Blocks until map -> blinky/base_footprint and map -> pinky/base_footprint
are available (or times out). Ensures SLAM and tf_relay have built the full tree.
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main() -> int:
    base_frames = os.environ.get("TF_WAIT_BASE_FRAMES", "blinky/base_footprint,pinky/base_footprint").split(",")
    map_frame = os.environ.get("TF_WAIT_MAP_FRAME", "map")
    timeout = float(os.environ.get("TF_WAIT_TIMEOUT_SEC", "60.0"))

    rclpy.init()
    node = Node("wait_for_tf_multirobot")
    buffer = Buffer()
    TransformListener(buffer, node)

    node.get_logger().info(
        f"Waiting for TF: {map_frame} -> each of {base_frames}. Timeout: {timeout:.1f}s"
    )

    # Warm up: let buffer receive TF messages (DDS needs time to connect)
    node.get_logger().info("Warming up TF buffer (3s)...")
    warmup_end = time.time() + 3.0
    while rclpy.ok() and time.time() < warmup_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    start = time.time()
    last_log = start
    while rclpy.ok() and (time.time() - start) < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        ok = True
        missing = []
        for b in base_frames:
            b = b.strip()
            if not b:
                continue
            try:
                if not buffer.can_transform(map_frame, b, rclpy.time.Time()):
                    ok = False
                    missing.append(b)
            except Exception:
                ok = False
                missing.append(b)
        if ok:
            node.get_logger().info("TF tree ready for all robots.")
            node.destroy_node()
            rclpy.shutdown()
            return 0
        if time.time() - last_log >= 10.0:
            node.get_logger().info(
                f"Still waiting... missing: {map_frame} -> {missing} "
                f"({time.time() - start:.0f}s elapsed)"
            )
            last_log = time.time()

    node.get_logger().error(
        f"Timed out waiting for {map_frame} -> {base_frames}. "
        "Ensure multirobot_slam, domain bridges, and tf_relay are running."
    )
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    sys.exit(main())
