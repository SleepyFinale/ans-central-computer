#!/usr/bin/env python3
"""
Live XY trajectory plotter with age-based color fading.

Subscribes to a ROS 2 odometry topic and plots position updates in 2D.
Older points are rendered with a different color than newer points to
make motion direction and recent movement easy to see.
"""

import argparse
import sys
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class LiveXYFadePlotter(Node):
    """Subscribe to odometry and keep a rolling XY history."""

    def __init__(self, topic: str, history: int) -> None:
        super().__init__("live_xy_fade_plotter")
        self._history = max(2, int(history))
        self._xs = deque(maxlen=self._history)
        self._ys = deque(maxlen=self._history)

        self._subscription = self.create_subscription(
            Odometry,
            topic,
            self._odom_callback,
            10,
        )
        self.get_logger().info(
            f"Subscribed to {topic} with history={self._history} points"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        self._xs.append(float(pos.x))
        self._ys.append(float(pos.y))

    def get_points(self):
        return np.asarray(self._xs, dtype=float), np.asarray(self._ys, dtype=float)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot live XY points from a ROS2 odometry topic with age-based color fading."
        )
    )
    parser.add_argument(
        "--topic",
        default="/blinky/fix",
        help="Odometry topic to subscribe to (default: /blinky/odometry/gps)",
    )
    parser.add_argument(
        "--history",
        type=int,
        default=800,
        help="Maximum number of recent points to keep (default: 800)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=12.0,
        help="Plot refresh rate in Hz (default: 12.0)",
    )
    parser.add_argument(
        "--cmap",
        default="viridis",
        help="Matplotlib colormap for point age (default: viridis)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    refresh_hz = max(1.0, float(args.rate))
    pause_s = 1.0 / refresh_hz

    rclpy.init()
    node = LiveXYFadePlotter(topic=args.topic, history=args.history)

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            xs, ys = node.get_points()

            ax.clear()
            ax.set_title(f"Live XY trail: {args.topic}")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.grid(True, alpha=0.3)
            ax.axis("equal")

            if xs.size > 0:
                ages = np.linspace(0.0, 1.0, xs.size)
                ax.scatter(
                    xs,
                    ys,
                    c=ages,
                    cmap=args.cmap,
                    s=22,
                    alpha=0.9,
                    edgecolors="none",
                )
                ax.plot(xs, ys, linewidth=0.9, alpha=0.5, color="gray")
                ax.scatter(xs[-1], ys[-1], color="red", s=60, label="latest")
                ax.legend(loc="best")
                ax.text(
                    0.02,
                    0.02,
                    f"samples: {xs.size}",
                    transform=ax.transAxes,
                    fontsize=10,
                    bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.6},
                )

            fig.canvas.draw_idle()
            plt.pause(pause_s)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.close(fig)

    return 0


if __name__ == "__main__":
    sys.exit(main())
