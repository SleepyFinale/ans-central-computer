#!/usr/bin/env python3
"""
Diagnose multi-robot TF tree: report what's working and where it breaks.
Run with ROS_DOMAIN_ID=50 (same as the fleet / central setup).

Expected global chain per robot (after tf_relay + map_merge or single-robot bridge):
  map -> <robot>/map -> <robot>/odom -> <robot>/base_footprint

Equivalent manual checks (central PC or robot, same ROS_DOMAIN_ID):

  ros2 run tf2_ros tf2_echo map <robot>/map
  ros2 run tf2_ros tf2_echo map <robot>/base_footprint

If Nav2 logs ``Invalid frame ID "map" ... frame does not exist``, the TF buffer
has no ``map`` frame yet: merged ``/map`` OccupancyGrid metadata alone does not
register ``map`` in tf2. You need ``map_merge`` (or the single-robot bridge) to
publish ``map -> <robot>/map`` on ``/tf``.

Usage: python3 scripts/diagnose_multirobot_tf.py
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

# Fleet order for TF diagnostics (must match namespaces / multi_robot_explorer).
ROBOTS = ['blinky', 'pinky', 'inky', 'clyde']


def qos_transient():
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TFDiagnostics(Node):
    def __init__(self):
        super().__init__('tf_diagnostics')
        self.buffer = Buffer()
        TransformListener(self.buffer, self)
        self.robots = list(ROBOTS)
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

        critical_topics = ['/tf', '/tf_static'] + [f'/{r}/tf' for r in self.robots] + ['/map']
        # Laser scans stay on the robots; they are optional from the
        # central PC's point of view (Nav2 + SLAM use them locally).
        optional_scan_topics = [f'/{r}/scan' for r in self.robots]

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
        #
        # tf2_ros: lookup_transform(target_frame, source_frame, time, ...)
        # e.g. pose of base in odom: target=odom, source=base_footprint
        now = self.get_clock().now()
        lookup_timeout = Duration(seconds=2.0)
        critical_checks = []
        optional_checks = []
        for r in self.robots:
            critical_checks.extend([
                (f'{r}/odom', f'{r}/base_footprint',
                 f'Robot TF (odom->base_footprint): {r}'),
                ('map', f'{r}/map',
                 f'World TF (map_merge): map->{r}/map'),
                ('map', f'{r}/base_footprint',
                 f'Explorer chain: map->{r}/base_footprint'),
            ])
            optional_checks.append(
                (f'{r}/map', f'{r}/odom',
                 f'SLAM TF: {r}/map->{r}/odom'),
            )
        for target, source, desc in critical_checks:
            try:
                self.buffer.lookup_transform(
                    target, source, now, timeout=lookup_timeout)
                self.log(f'{desc}', ok=True)
            except Exception as e:
                self.log(f'{desc}: {e}', ok=False)
        for target, source, desc in optional_checks:
            try:
                self.buffer.lookup_transform(
                    target, source, now, timeout=lookup_timeout)
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
            for r in self.robots:
                label = r.capitalize()
                topic_key = f'Topic /{r}/tf:'
                if not any(topic_key in res and '[OK]' in res for res in self.results):
                    print(
                        f'  - /{r}/tf: domain bridges or {label} robot not running'
                    )
            if any('Robot TF (odom->base_footprint):' in r and '[MISSING]' in r for r in self.results):
                print('  - odom->base_footprint: base TF on the robot or tf_relay may not be running.')
            if any('World TF (map_merge):' in r and '[MISSING]' in r for r in self.results):
                print('  - map-><robot>/map: map_merge may not be running, publish_tf false, or pose '
                      'estimates not ready yet (see publish_provisional_tf in map_merge params).')
            if any('Explorer chain:' in r and '[MISSING]' in r for r in self.results):
                print('  - Explorer chain: ensure map_merge, slam_toolbox, tf_relay, and all robots are running.')
                print('  - Nav2 "Invalid frame ID map": run tf2_echo map <robot>/base_footprint; '
                      'need map_merge map-><robot>/map on /tf (not just /map topic).')
            if any('SLAM TF:' in r and '[MISSING]' in r for r in self.results):
                print('  - <robot>/map-><robot>/odom: slam_toolbox on that robot may not be publishing yet.')
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
