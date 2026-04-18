#!/usr/bin/env python3
"""
Relay a namespaced SLAM map to global /map.

Used in single-robot central mode: map_merge is skipped, but fleet Nav2 on the
robot remaps map -> /map and global RViz (central_global_map.rviz) expects /map.

Optional: subscribe to /<robot>/map_wire_z (UInt8MultiArray, zlib-compressed
serialized OccupancyGrid) instead of raw /map to reduce DDS bandwidth.
"""

import time
import zlib

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from std_msgs.msg import UInt8MultiArray

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

_WARN_THROTTLE_SEC = 5.0


class Relay(Node):
    def __init__(self):
        super().__init__('single_robot_map_relay')
        self.declare_parameter('source_map', '')
        self.declare_parameter('source_map_wire_z', '')
        src = self.get_parameter('source_map').get_parameter_value().string_value.strip()
        src_wire = self.get_parameter(
            'source_map_wire_z').get_parameter_value().string_value.strip()

        if not src and not src_wire:
            raise ValueError(
                'Set source_map (e.g. /pinky/map) and/or source_map_wire_z '
                '(e.g. /pinky/map_wire_z)')

        self._pub = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self._last_warn_mono = 0.0

        if src_wire:
            self.create_subscription(
                UInt8MultiArray, src_wire, self._cb_wire, MAP_QOS)
            self.get_logger().info(f'Map relay (wire): {src_wire} -> /map')
            return

        self.create_subscription(OccupancyGrid, src, self._cb_raw, MAP_QOS)
        self.get_logger().info(f'Map relay: {src} -> /map')

    def _cb_raw(self, msg: OccupancyGrid):
        self._pub.publish(msg)

    def _cb_wire(self, msg: UInt8MultiArray):
        if not msg.data:
            self._warn_throttle('map_wire_z: empty data, skipping')
            return
        raw = bytes(msg.data)
        try:
            blob = zlib.decompress(raw)
            grid = deserialize_message(blob, OccupancyGrid)
        except zlib.error as e:
            self._warn_throttle(f'map_wire_z: zlib error: {e}')
            return
        except Exception as e:
            self._warn_throttle(f'map_wire_z: deserialize error: {e}')
            return
        self._pub.publish(grid)

    def _warn_throttle(self, text: str):
        now = time.monotonic()
        if now - self._last_warn_mono >= _WARN_THROTTLE_SEC:
            self._last_warn_mono = now
            self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Relay()
    except ValueError as e:
        print(e, file=__import__('sys').stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main() or 0)
