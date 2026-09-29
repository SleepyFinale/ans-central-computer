#!/usr/bin/env python3
"""Republish /<robot>/tf_zenoh onto /<robot>/tf for domain_bridge and Nav2.

The robot sends TF on the *_zenoh topics so the Zenoh bridge never subscribes
to slam_toolbox's /tf publisher. This node restores the names the rest of the
central stack already uses.
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

_BEST_EFFORT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
_BEST_EFFORT_LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
_RELIABLE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_RELIABLE_LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TfZenohReceive(Node):
    def __init__(self, robot: str) -> None:
        super().__init__('tf_zenoh_receive')
        self._pub_tf = self.create_publisher(TFMessage, f'/{robot}/tf', _RELIABLE)
        self._pub_static = self.create_publisher(
            TFMessage, f'/{robot}/tf_static', _RELIABLE_LATCHED)
        self.create_subscription(
            TFMessage, f'/{robot}/tf_zenoh', self._on_tf, _BEST_EFFORT)
        self.create_subscription(
            TFMessage, f'/{robot}/tf_static_zenoh', self._on_static, _BEST_EFFORT_LATCHED)
        self.get_logger().info(
            f'TF zenoh receive: /{robot}/tf_zenoh -> /{robot}/tf')

    def _on_tf(self, msg: TFMessage) -> None:
        self._pub_tf.publish(msg)

    def _on_static(self, msg: TFMessage) -> None:
        self._pub_static.publish(msg)


def main() -> int:
    if len(sys.argv) != 2 or not sys.argv[1].strip():
        print('usage: tf_zenoh_receive.py <robot>', file=sys.stderr)
        return 2
    robot = sys.argv[1].strip().strip('/')
    rclpy.init()
    node = TfZenohReceive(robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
