#!/usr/bin/env python3
"""
Diagnose multi-robot TF tree: report what's working and where it breaks.
Run with ROS_DOMAIN_ID=50 (same as multirobot_slam and domain bridges).

Usage: python3 scripts/diagnose_multirobot_tf.py
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener


def qos_transient():
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TFDiagnostics(Node):
    def __init__(self):
        super().__init__('tf_diagnostics')
        self.buffer = Buffer()
        TransformListener(self.buffer, self)
        self.robots = ['blinky', 'pinky']
        self.results = []

    def log(self, msg: str, ok: bool | None = None):
        if ok is True:
            prefix = '[OK]'
        elif ok is False:
            prefix = '[MISSING]'
        else:
            prefix = '[INFO]'
        self.results.append(f'{prefix} {msg}')
        print(f'{prefix} {msg}')

    def run(self):
        print('\n=== Multi-Robot TF Diagnostics (ROS_DOMAIN_ID should be 50) ===\n')

        domain = os.environ.get('ROS_DOMAIN_ID', '?')
        self.log(f'ROS_DOMAIN_ID = {domain}', ok=(domain == '50') if domain != '?' else None)

        # 1. Check topics
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)

        topic_checks = [
            '/tf', '/tf_static',
            '/blinky/tf', '/pinky/tf',
            '/blinky/scan', '/pinky/scan',
            '/map',
        ]
        for topic in topic_checks:
            try:
                n = self.count_publishers(topic)
                self.log(f'Topic {topic}: {n} publisher(s)', ok=(n > 0))
            except Exception as e:
                self.log(f'Topic {topic}: error - {e}', ok=False)

        # 2. Let TF buffer fill
        self.log('Waiting 3s for TF buffer...', ok=None)
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)

        # 3. Check specific transforms (critical = required for Nav2)
        now = rclpy.time.Time()
        critical_checks = [
            ('blinky/odom', 'blinky/base_footprint', 'Robot TF (tf_relay): blinky'),
            ('pinky/odom', 'pinky/base_footprint', 'Robot TF (tf_relay): pinky'),
            ('map', 'blinky/odom', 'Fallback/SLAM map->blinky/odom'),
            ('map', 'pinky/odom', 'Fallback/SLAM map->pinky/odom'),
            ('map', 'blinky/base_footprint', 'Full chain: map->blinky/base_footprint'),
            ('map', 'pinky/base_footprint', 'Full chain: map->pinky/base_footprint'),
        ]
        optional_checks = [
            ('map', 'blinky/map', 'Optional: map->blinky/map (SLAM local)'),
            ('map', 'pinky/map', 'Optional: map->pinky/map (SLAM local)'),
            ('blinky/map', 'blinky/odom', 'Optional: SLAM map->odom blinky'),
            ('pinky/map', 'pinky/odom', 'Optional: SLAM map->odom pinky'),
        ]
        for parent, child, desc in critical_checks:
            try:
                self.buffer.lookup_transform(child, parent, now)
                self.log(f'{desc}', ok=True)
            except Exception as e:
                self.log(f'{desc}: {e}', ok=False)
        for parent, child, desc in optional_checks:
            try:
                self.buffer.lookup_transform(child, parent, now)
                self.log(f'{desc}', ok=True)
            except Exception:
                self.log(f'{desc}: not yet (slam_toolbox publishes after map build)', ok=None)

        # 4. Sample /tf and /tf_static content
        from tf2_msgs.msg import TFMessage
        self.tf_samples = []
        self.tf_static_samples = []

        def on_tf(msg):
            self.tf_samples.append(msg)
        def on_tf_static(msg):
            self.tf_static_samples.append(msg)

        self.create_subscription(TFMessage, '/tf', on_tf, 10)
        self.create_subscription(TFMessage, '/tf_static', on_tf_static, qos_transient())

        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.tf_static_samples:
            seen = set()
            for msg in self.tf_static_samples:
                for t in msg.transforms:
                    key = (t.header.frame_id, t.child_frame_id)
                    if key not in seen:
                        seen.add(key)
                        self.log(f'  tf_static: {t.header.frame_id} -> {t.child_frame_id}', ok=None)
        if self.tf_samples:
            # show a few recent transforms
            seen = set()
            for msg in reversed(self.tf_samples[-5:]):
                for t in msg.transforms:
                    key = (t.header.frame_id, t.child_frame_id)
                    if key not in seen:
                        seen.add(key)
                        self.log(f'  tf: {t.header.frame_id} -> {t.child_frame_id}', ok=None)

        print('\n=== Summary ===')
        ok_count = sum(1 for r in self.results if r.startswith('[OK]'))
        miss_count = sum(1 for r in self.results if r.startswith('[MISSING]'))
        if miss_count > 0:
            print(f'Issues found ({miss_count} critical). Check [MISSING] items above.')
            print('\nTypical causes:')
            if not any('blinky/tf' in r and '[OK]' in r for r in self.results):
                print('  - /blinky/tf: domain bridges or Blinky robot not running')
            if not any('pinky/tf' in r and '[OK]' in r for r in self.results):
                print('  - /pinky/tf: domain bridges or Pinky robot not running')
            if any('Fallback/SLAM map->' in r and '[MISSING]' in r for r in self.results):
                print('  - map->*_/odom: tf_map_odom_fallback not publishing. Restart multirobot_slam.')
            if any('Full chain' in r and '[MISSING]' in r for r in self.results):
                print('  - Full chain: ensure tf_relay, tf_map_odom_fallback, and robots are running')
        else:
            print('All critical checks passed. Nav2 and explore should work.')
            print('Optional [INFO] items (blinky/map, pinky/map) appear once slam_toolbox builds maps.')
        print()


def main():
    rclpy.init()
    node = TFDiagnostics()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
