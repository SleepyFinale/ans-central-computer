#!/usr/bin/env python3
"""Turn what the robots send over Zenoh into the topics the central stack reads.

For every robot name given on the command line:
  /<robot>/tf_zenoh         -> /tf          (robot frames already carry <robot>/)
  /<robot>/tf_static_zenoh  -> /tf_static   (one latched message for the fleet)
  /<robot>/map_wire_z       -> /<robot>/map (zlib-compressed OccupancyGrid)

The robots send TF on *_zenoh topics so the Zenoh bridge never holds a
reliable subscription on slam_toolbox's /tf, and send the map only in
compressed form.
"""

from __future__ import annotations

import sys
import zlib

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from std_msgs.msg import UInt8MultiArray
from tf2_msgs.msg import TFMessage

_BEST_EFFORT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
_BEST_EFFORT_LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
_TF = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _prefixed(frame_id: str, robot: str) -> str:
    """Give a bare robot frame the robot prefix. The world frame `map` stays shared."""
    if frame_id == 'map' or frame_id.startswith(robot + '/'):
        return frame_id
    return f'{robot}/{frame_id}'


class FleetZenohReceive(Node):
    def __init__(self, robots: list[str]) -> None:
        super().__init__('fleet_zenoh_receive')
        self._pub_tf = self.create_publisher(TFMessage, '/tf', _TF)
        self._pub_static = self.create_publisher(TFMessage, '/tf_static', _LATCHED)
        # (parent, child) -> TransformStamped for every robot. Late /tf_static
        # subscribers get the last message only, so it must hold the whole fleet.
        self._static: dict = {}
        self._map_pubs = {}
        for robot in robots:
            self._map_pubs[robot] = self.create_publisher(
                OccupancyGrid, f'/{robot}/map', _LATCHED)
            self.create_subscription(
                TFMessage, f'/{robot}/tf_zenoh',
                lambda msg, r=robot: self._on_tf(msg, r), _BEST_EFFORT)
            self.create_subscription(
                TFMessage, f'/{robot}/tf_static_zenoh',
                lambda msg, r=robot: self._on_static(msg, r), _BEST_EFFORT_LATCHED)
            self.create_subscription(
                UInt8MultiArray, f'/{robot}/map_wire_z',
                lambda msg, r=robot: self._on_map_wire(msg, r), _LATCHED)
        self.get_logger().info(
            f'Fleet receive for {robots}: tf_zenoh -> /tf, '
            'tf_static_zenoh -> /tf_static, map_wire_z -> /<robot>/map')

    @staticmethod
    def _prefix_all(msg: TFMessage, robot: str) -> list:
        out = []
        for tf in msg.transforms:
            tf.header.frame_id = _prefixed(tf.header.frame_id, robot)
            tf.child_frame_id = _prefixed(tf.child_frame_id, robot)
            if tf.header.frame_id != tf.child_frame_id:
                out.append(tf)
        return out

    def _on_tf(self, msg: TFMessage, robot: str) -> None:
        transforms = self._prefix_all(msg, robot)
        if transforms:
            self._pub_tf.publish(TFMessage(transforms=transforms))

    def _on_static(self, msg: TFMessage, robot: str) -> None:
        changed = False
        for tf in self._prefix_all(msg, robot):
            key = (tf.header.frame_id, tf.child_frame_id)
            old = self._static.get(key)
            if old is None or old.transform != tf.transform:
                self._static[key] = tf
                changed = True
        # The robot resends its static set every second; republish only news.
        if changed:
            self._pub_static.publish(TFMessage(transforms=list(self._static.values())))

    def _on_map_wire(self, msg: UInt8MultiArray, robot: str) -> None:
        try:
            grid = deserialize_message(zlib.decompress(bytes(msg.data)), OccupancyGrid)
        except Exception as exc:  # noqa: BLE001 - a bad frame must not stop the relay
            self.get_logger().warning(
                f'/{robot}/map_wire_z: cannot decode ({exc})', throttle_duration_sec=5.0)
            return
        self._map_pubs[robot].publish(grid)


def main() -> int:
    robots = [r.strip().strip('/') for r in sys.argv[1:] if r.strip().strip('/')]
    if not robots:
        print('usage: fleet_zenoh_receive.py <robot> [<robot> ...]', file=sys.stderr)
        return 2
    rclpy.init()
    node = FleetZenohReceive(robots)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl+C can invalidate the context mid-wait before KeyboardInterrupt.
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
