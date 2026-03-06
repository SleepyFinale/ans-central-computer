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
        self.robots = ['blinky', 'pinky', 'inky']
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

        # 1. Check topics (what should exist given the domain bridges)
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)

        critical_topics = [
            '/tf',
            '/tf_static',
            '/blinky/tf',
            '/pinky/tf',
            '/inky/tf',
            '/map',
        ]
        # Laser scans stay on the robots; they are optional from the
        # central PC's point of view (Nav2 + SLAM use them locally).
        optional_scan_topics = [
            '/blinky/scan',
            '/pinky/scan',
            '/inky/scan',
        ]

        for topic in critical_topics:
            try:
                n = self.count_publishers(topic)
                self.log(f'Topic {topic}: {n} publisher(s)', ok=(n > 0))
            except Exception as e:
                self.log(f'Topic {topic}: error - {e}', ok=False)

        for topic in optional_scan_topics:
            try:
                n = self.count_publishers(topic)
                # Report scans as informational only, since in this
                # architecture they are not bridged to the central PC.
                self.log(f'Topic {topic}: {n} publisher(s)', ok=None if n == 0 else True)
            except Exception as e:
                self.log(f'Topic {topic}: error - {e}', ok=None)

        # 2. Let TF buffer fill
        self.log('Waiting 3s for TF buffer...', ok=None)
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)

        # 3. Check specific transforms
        #
        # Critical checks are things that must exist for Nav2 and the
        # central multi_robot_explorer to work:
        #   - odom -> base_footprint for each robot (via tf_relay)
        #   - map -> <robot>/map from map_merge
        #   - full chain map -> <robot>/base_footprint
        now = rclpy.time.Time()
        critical_checks = [
            ('blinky/odom', 'blinky/base_footprint',
             'Robot TF (odom->base_footprint): blinky'),
            ('pinky/odom', 'pinky/base_footprint',
             'Robot TF (odom->base_footprint): pinky'),
            ('inky/odom', 'inky/base_footprint',
             'Robot TF (odom->base_footprint): inky'),
            ('map', 'blinky/map',
             'World TF (map_merge): map->blinky/map'),
            ('map', 'pinky/map',
             'World TF (map_merge): map->pinky/map'),
            ('map', 'inky/map',
             'World TF (map_merge): map->inky/map'),
            ('map', 'blinky/base_footprint',
             'Explorer chain: map->blinky/base_footprint'),
            ('map', 'pinky/base_footprint',
             'Explorer chain: map->pinky/base_footprint'),
            ('map', 'inky/base_footprint',
             'Explorer chain: map->inky/base_footprint'),
        ]
        optional_checks = [
            ('blinky/map', 'blinky/odom',
             'SLAM TF: blinky/map->blinky/odom'),
            ('pinky/map', 'pinky/odom',
             'SLAM TF: pinky/map->pinky/odom'),
            ('inky/map', 'inky/odom',
             'SLAM TF: inky/map->inky/odom'),
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
            if not any('Topic /blinky/tf:' in r and '[OK]' in r for r in self.results):
                print('  - /blinky/tf: domain bridges or Blinky robot not running')
            if not any('Topic /pinky/tf:' in r and '[OK]' in r for r in self.results):
                print('  - /pinky/tf: domain bridges or Pinky robot not running')
            if not any('Topic /inky/tf:' in r and '[OK]' in r for r in self.results):
                print('  - /inky/tf: domain bridges or Inky robot not running')
            if any('Robot TF (odom->base_footprint):' in r and '[MISSING]' in r for r in self.results):
                print('  - odom->base_footprint: base TF on the robot or tf_relay may not be running.')
            if any('World TF (map_merge):' in r and '[MISSING]' in r for r in self.results):
                print('  - map-><robot>/map: map_merge may not be running, or publish_tf may be false.')
            if any('SLAM TF:' in r and '[MISSING]' in r for r in self.results):
                print('  - <robot>/map-><robot>/odom: slam_toolbox on that robot may not be publishing yet.')
            if any('Explorer chain:' in r and '[MISSING]' in r for r in self.results):
                print('  - Explorer chain: ensure map_merge, slam_toolbox, tf_relay, and all robots are running.')
        else:
            print('All critical checks passed. Multi-robot Nav2 and explorer should work.')
            print('Optional [INFO] items (SLAM TF) appear once slam_toolbox builds maps.')
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
