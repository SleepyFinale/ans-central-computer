#!/usr/bin/env python3
"""
TF fallback: publishes map -> blinky/odom and map -> pinky/odom (identity) as static.
Connects the TF tree so Nav2 can start. When SLAM publishes map->odom, it takes over
for corrections; our static identity remains as a valid path.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def make_identity_transform(parent: str, child: str) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.header.stamp = rclpy.time.Time().to_msg()
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    return t


class TFMapOdomFallback(Node):
    def __init__(self):
        super().__init__('tf_map_odom_fallback')
        self.declare_parameter('robot_odom_frames', ['blinky/odom', 'pinky/odom'])
        self.robot_odom = self.get_parameter('robot_odom_frames').value
        self.map_frame = 'map'

        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = [
            make_identity_transform(self.map_frame, odom_f)
            for odom_f in self.robot_odom
        ]
        self.broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f'TF map->odom fallback: published static identity map -> {self.robot_odom}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TFMapOdomFallback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
