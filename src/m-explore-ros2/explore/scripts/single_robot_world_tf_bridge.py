#!/usr/bin/env python3
"""
Publish static TF: world frame ``map`` -> ``<robot>/map`` (identity).

Used when the central PC runs in single-robot mode: SLAM on the robot uses
frame {robot}/map, while the explorer and fleet Nav2 expect goals in ``map``.
Map merge is skipped for one robot, so this bridge supplies the missing link.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class Bridge(Node):
    def __init__(self):
        super().__init__('single_robot_world_tf_bridge')
        self.declare_parameter('robot_name', '')
        robot = self.get_parameter('robot_name').get_parameter_value().string_value
        if not robot.strip():
            raise ValueError('Parameter robot_name must be non-empty')
        robot = robot.strip()
        child = f'{robot}/map'
        br = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        br.sendTransform(t)
        self.get_logger().info(f'Published static TF map -> {child} (identity)')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bridge()
    except ValueError as e:
        print(e, file=sys.stderr)
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
