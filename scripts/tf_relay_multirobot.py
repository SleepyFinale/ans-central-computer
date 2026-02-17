#!/usr/bin/env python3
"""
TF relay for multi-robot: subscribes to blinky/tf, pinky/tf (and _static)
and republishes to /tf with frame ID prefixes so SLAM can see both robots.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def prefix_frame(frame_id: str, prefix: str) -> str:
    """Add prefix if frame is not already prefixed."""
    if not prefix or frame_id.startswith(prefix + '/'):
        return frame_id
    return f"{prefix}/{frame_id}"


class TFRelayNode(Node):
    def __init__(self):
        super().__init__('tf_relay_multirobot')

        self.declare_parameter('robot_prefixes', ['blinky', 'pinky'])
        self.robot_prefixes = self.get_parameter('robot_prefixes').value

        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', 10)

        self.subscriptions = []
        for prefix in self.robot_prefixes:
            # Dynamic TF
            sub = self.create_subscription(
                TFMessage,
                f'{prefix}/tf',
                lambda msg, p=prefix: self.tf_callback(msg, p, is_static=False),
                100
            )
            self.subscriptions.append(sub)
            # Static TF
            sub_static = self.create_subscription(
                TFMessage,
                f'{prefix}/tf_static',
                lambda msg, p=prefix: self.tf_callback(msg, p, is_static=True),
                10
            )
            self.subscriptions.append(sub_static)

        self.get_logger().info(
            f'TF relay started: merging {self.robot_prefixes} -> /tf'
        )

    def tf_callback(self, msg: TFMessage, prefix: str, is_static: bool):
        out = TFMessage()
        for t in msg.transforms:
            from copy import deepcopy
            t_out = deepcopy(t)
            t_out.header.frame_id = prefix_frame(t.header.frame_id, prefix)
            t_out.child_frame_id = prefix_frame(t.child_frame_id, prefix)
            out.transforms.append(t_out)
        if out.transforms:
            if is_static:
                self.tf_static_pub.publish(out)
            else:
                self.tf_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TFRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
