#!/usr/bin/env python3

import sys
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


class MultiTopicPlotter(Node):
    def __init__(self, history: int = 800):
        super().__init__("multi_topic_plotter")

        self._history = max(2, int(history))

        # GPS topics (lat/lon)
        self.fix_lat = deque(maxlen=self._history)
        self.fix_lon = deque(maxlen=self._history)

        self.filtered_lat = deque(maxlen=self._history)
        self.filtered_lon = deque(maxlen=self._history)

        # Odometry topics (x/y)
        self.odom_filt_x = deque(maxlen=self._history)
        self.odom_filt_y = deque(maxlen=self._history)

        self.odom_gps_x = deque(maxlen=self._history)
        self.odom_gps_y = deque(maxlen=self._history)

        self.odom_wheel_x = deque(maxlen=self._history)
        self.odom_wheel_y = deque(maxlen=self._history)

        # Subscriptions
        self.create_subscription(
            NavSatFix, "/blinky/fix", self._fix_callback, 10
        )

        self.create_subscription(
            NavSatFix, "/blinky/gps/filtered", self._gps_filtered_callback, 10
        )

        self.create_subscription(
            Odometry, "/blinky/odometry/filtered", self._odom_filtered_callback, 10
        )

        self.create_subscription(
            Odometry, "/blinky/odometry/gps", self._odom_gps_callback, 10
        )

        self.create_subscription(
            Odometry, "/blinky/odom", self._odom_wheel_callback, 10
        )

        self.get_logger().info("Subscribed to all topics")

    # =====================
    # Callbacks
    # =====================
    def _fix_callback(self, msg: NavSatFix):
        self.fix_lat.append(msg.latitude)
        self.fix_lon.append(msg.longitude)

    def _gps_filtered_callback(self, msg: NavSatFix):
        self.filtered_lat.append(msg.latitude)
        self.filtered_lon.append(msg.longitude)

    def _odom_filtered_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_filt_x.append(p.x)
        self.odom_filt_y.append(p.y)

    def _odom_gps_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_gps_x.append(p.x)
        self.odom_gps_y.append(p.y)

    def _odom_wheel_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_wheel_x.append(p.x)
        self.odom_wheel_y.append(p.y)


def plot_with_fade(ax, xs, ys, label, cmap):
    xs = np.asarray(xs)
    ys = np.asarray(ys)

    if xs.size == 0:
        return None  # important

    ages = np.linspace(0.0, 1.0, xs.size)

    ax.scatter(xs, ys, c=ages, cmap=cmap, s=20, alpha=0.9)
    ax.plot(xs, ys, linewidth=0.8, alpha=0.5)

    # THIS is what legend will use
    point = ax.scatter(xs[-1], ys[-1], s=60, label=label)

    return point


def main():
    rclpy.init()
    node = MultiTopicPlotter(history=800)

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            # =====================
            # GPS PLOT
            # =====================
            ax1.clear()
            ax1.set_title("GPS (lat/lon)")
            ax1.set_xlabel("Longitude")
            ax1.set_ylabel("Latitude")
            ax1.grid(True)
            ax1.axis("equal")

            artists = []

            a = plot_with_fade(ax1, node.fix_lon, node.fix_lat, "fix", "viridis")
            b = plot_with_fade(ax1, node.filtered_lon, node.filtered_lat, "filtered", "plasma")

            if a: artists.append(a)
            if b: artists.append(b)

            if artists:
                ax1.legend()

            # =====================
            # ODOM PLOT
            # =====================
            ax2.clear()
            ax2.set_title("Odometry (x/y)")
            ax2.set_xlabel("X (m)")
            ax2.set_ylabel("Y (m)")
            ax2.grid(True)
            ax2.axis("equal")

            artists = []

            a = plot_with_fade(ax2, node.odom_gps_x, node.odom_gps_y, "odom_gps", "Greens")
            b = plot_with_fade(ax2, node.odom_filt_x, node.odom_filt_y, "odom_filtered", "Blues")
            c = plot_with_fade(ax2, node.odom_wheel_x, node.odom_wheel_y, "odom_wheel", "Reds")

            if a: artists.append(a)
            if b: artists.append(b)
            if c: artists.append(c)

            if artists:
                ax2.legend()

            plt.pause(0.08)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())