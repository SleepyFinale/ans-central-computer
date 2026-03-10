#!/usr/bin/env python3
"""
TF relay for multi-robot: subscribes to blinky/tf, pinky/tf, inky/tf (and _static)
and republishes to /tf with frame ID prefixes so SLAM can see all robots.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

TF_STATIC_QOS = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def prefix_frame(frame_id: str, prefix: str) -> str:
    """Add prefix if frame is not already prefixed."""
    if not prefix or frame_id.startswith(prefix + '/'):
        return frame_id
    return f"{prefix}/{frame_id}"


class TFRelayNode(Node):
    def __init__(self):
        super().__init__('tf_relay_multirobot')

        self.declare_parameter('robot_prefixes', ['blinky', 'pinky', 'inky'])
        self.declare_parameter('prefix_frames', True)
        self.robot_prefixes = self.get_parameter('robot_prefixes').value
        self.prefix_frames = self.get_parameter('prefix_frames').value

        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', TF_STATIC_QOS)

        for prefix in self.robot_prefixes:
            # Dynamic TF
            self.create_subscription(
                TFMessage,
                f'{prefix}/tf',
                lambda msg, p=prefix: self.tf_callback(msg, p, is_static=False),
                100
            )
            # Static TF (transient_local to match domain bridge / tf2 listeners)
            self.create_subscription(
                TFMessage,
                f'{prefix}/tf_static',
                lambda msg, p=prefix: self.tf_callback(msg, p, is_static=True),
                TF_STATIC_QOS
            )

        self.get_logger().info(
            f'TF relay started: merging {self.robot_prefixes} -> /tf '
            f'(prefix_frames={self.prefix_frames})'
        )

    def tf_callback(self, msg: TFMessage, prefix: str, is_static: bool):
        try:
            if not msg.transforms:
                return
            out = TFMessage()
            for t in msg.transforms:
                from copy import deepcopy
                t_out = deepcopy(t)
                if self.prefix_frames:
                    t_out.header.frame_id = prefix_frame(t.header.frame_id, prefix)
                    t_out.child_frame_id = prefix_frame(t.child_frame_id, prefix)
                else:
                    # Pass frames through unchanged; still relay from
                    # <robot>/tf to /tf so consumers that listen only on /tf
                    # can see them.
                    t_out.header.frame_id = t.header.frame_id
                    t_out.child_frame_id = t.child_frame_id
                out.transforms.append(t_out)
            if is_static:
                self.tf_static_pub.publish(out)
            else:
                self.tf_pub.publish(out)
        except Exception as e:
            self.get_logger().warning(f"TF callback error (skipping): {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TFRelayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
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
