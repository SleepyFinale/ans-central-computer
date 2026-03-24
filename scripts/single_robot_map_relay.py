#!/usr/bin/env python3
"""
Relay a namespaced SLAM map to global /map.

Used in single-robot central mode: map_merge is skipped, but fleet Nav2 on the
robot remaps map -> /map and global RViz (central_global_map.rviz) expects /map.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class Relay(Node):
    def __init__(self):
        super().__init__('single_robot_map_relay')
        self.declare_parameter('source_map', '')
        src = self.get_parameter('source_map').get_parameter_value().string_value.strip()
        if not src:
            raise ValueError('Parameter source_map is required (e.g. /pinky/map)')

        self._pub = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self.create_subscription(OccupancyGrid, src, self._cb, MAP_QOS)
        self.get_logger().info(f'Map relay: {src} -> /map')

    def _cb(self, msg: OccupancyGrid):
        self._pub.publish(msg)


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
