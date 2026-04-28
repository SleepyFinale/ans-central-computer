#!/usr/bin/env python3
"""
Diagnose multi-robot TF tree: report what's working and where it breaks.
Run with the same ROS_DOMAIN_ID as the central PC and all robots (e.g. 50 or 64).

Expected global chain per robot (after tf_relay + map_merge or single-robot bridge):
  map -> <robot>/map -> <robot>/odom -> <robot>/base_footprint

Equivalent manual checks (central PC or robot, same ROS_DOMAIN_ID):

  ros2 run tf2_ros tf2_echo map <robot>/map
  ros2 run tf2_ros tf2_echo map <robot>/base_footprint

If Nav2 logs ``Invalid frame ID "map" ... frame does not exist``, the TF buffer
has no ``map`` frame yet: merged ``/map`` OccupancyGrid metadata alone does not
register ``map`` in tf2. You need ``map_merge`` (or the single-robot bridge) to
publish ``map -> <robot>/map`` on ``/tf``.

Usage: python3 scripts/diagnostics/diagnose_multirobot_tf.py
"""

import math
import os
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

# Default fleet names (used only if no /<robot>/tf topics are found yet).
DEFAULT_ROBOTS = ['blinky', 'pinky', 'inky', 'clyde']

# Past query times (seconds) for multi-hop TF; map_merge / SLAM / odom stamps often skew.
_TF_LOOKBACK_SEC = (
    0.0,
    0.05,
    0.1,
    0.25,
    0.5,
    0.75,
    1.0,
    1.5,
    2.0,
    3.0,
    4.0,
    5.0,
    6.0,
    8.0,
)


def qos_transient():
    """QoS profile matching latched-like map topics on ROS 2."""
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def discover_robots_from_graph(node: Node) -> list[str]:
    """Robot prefixes seen on the graph.

    - ``/<robot>/map`` with an OccupancyGrid type: include **if the topic exists**
      (do not require ``count_publishers``; transient-local maps often report 0 on
      a subscriber-only central PC even though data is visible on ``/map``).
    - ``/<robot>/tf``: include when ``count_publishers`` > 0 (robot publishing its
      namespaced TF topic).
    """
    names: set[str] = set()
    try:
        for topic, types in node.get_topic_names_and_types():
            if not topic.startswith('/'):
                continue
            parts = topic.strip('/').split('/')
            if len(parts) != 2:
                continue
            name, leaf = parts[0], parts[1]
            if not name:
                continue
            if leaf == 'map' and any('OccupancyGrid' in t for t in types):
                names.add(name)
                continue
            if leaf == 'tf':
                try:
                    if node.count_publishers(topic) > 0:
                        names.add(name)
                except Exception:
                    continue
    except Exception:
        pass
    return sorted(names)


class TFDiagnostics(Node):
    def __init__(self):
        super().__init__('tf_diagnostics')
        # Longer cache than default so past-time lookups for long TF chains stay valid.
        self.buffer = Buffer(cache_time=Duration(seconds=30.0))
        TransformListener(self.buffer, self)
        self.robots: list[str] = []
        self.results = []
        self._merged_map: Optional[OccupancyGrid] = None

    def _lookup_try_many(
        self,
        target_frame: str,
        source_frame: str,
    ) -> tuple[bool, Exception | None]:
        """Try tf2 latest (time 0) then several past wall times; return (ok, last_error)."""
        clock_type = self.get_clock().clock_type
        last_err: Exception | None = None
        for i, lb in enumerate(_TF_LOOKBACK_SEC):
            if lb == 0.0:
                query_time = Time(nanoseconds=0, clock_type=clock_type)
            else:
                query_time = self.get_clock().now() - Duration(
                    nanoseconds=int(lb * 1_000_000_000)
                )
            timeout = Duration(seconds=2.0) if i == 0 else Duration(seconds=0.4)
            try:
                self.buffer.lookup_transform(
                    target_frame, source_frame, query_time, timeout=timeout)
                return True, None
            except Exception as e:
                last_err = e
        return False, last_err

    def lookup_transform_robust(
        self,
        target_frame: str,
        source_frame: str,
        desc: str,
        *,
        optional: bool = False,
    ) -> None:
        """Resolve TF using tf2 'latest' (time 0) then past wall-clock times.

        Multi-hop lookups (e.g. map -> base_footprint) can fail with
        'extrapolation into the future' when stamps on map_merge / SLAM / odom
        diverge by seconds; try several lookback intervals (fresh ``now`` each
        time) so the query lands inside all segments' overlap.
        """
        ok, last_err = self._lookup_try_many(target_frame, source_frame)
        if ok:
            self.log(desc, ok=True)
            return
        if optional:
            self.log(
                f'{desc}: not yet (slam_toolbox publishes after map build): {last_err}',
                ok=None,
            )
        else:
            self.log(f'{desc}: {last_err}', ok=False)

    @staticmethod
    def _world_to_map_cell(
        grid: OccupancyGrid, x: float, y: float,
    ) -> Optional[Tuple[int, int]]:
        """Map-frame position (x,y) to occupancy cell indices; None if outside grid."""
        info = grid.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        q = info.origin.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        res = info.resolution
        dx = x - ox
        dy = y - oy
        ix = (math.cos(yaw) * dx + math.sin(yaw) * dy) / res
        iy = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / res
        mx = int(math.floor(ix))
        my = int(math.floor(iy))
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def check_merged_map_footprint(self, robot: str) -> None:
        """Sample merged /map at robot base pose in frame map (Nav2 lethal alignment check)."""
        if self._merged_map is None:
            return
        ok_tf, _ = self._lookup_try_many('map', f'{robot}/base_footprint')
        if not ok_tf:
            self.log(
                f'Merged /map vs TF: {robot} (skip cell sample; map->{robot}/base_footprint unresolved)',
                ok=None,
            )
            return
        clock_type = self.get_clock().clock_type
        query_time = Time(nanoseconds=0, clock_type=clock_type)
        try:
            t = self.buffer.lookup_transform(
                'map',
                f'{robot}/base_footprint',
                query_time,
                timeout=Duration(seconds=2.0),
            )
        except Exception as e:
            self.log(f'Merged /map vs TF: {robot} lookup failed: {e}', ok=None)
            return
        x = t.transform.translation.x
        y = t.transform.translation.y
        cell = self._world_to_map_cell(self._merged_map, x, y)
        if cell is None:
            self.log(
                f'Merged /map vs TF: {robot} base ({x:.2f},{y:.2f}) outside grid '
                f'({self._merged_map.info.width}x{self._merged_map.info.height}) — '
                'possible resize lag or TF/map mismatch',
                ok=False,
            )
            return
        mx, my = cell
        idx = my * self._merged_map.info.width + mx
        val = self._merged_map.data[idx]
        desc = f'Merged /map vs TF: {robot} cell ({mx},{my}) occupancy={val}'
        if val < 0:
            self.log(f'{desc} (unknown)', ok=None)
        elif val >= 90:
            self.log(
                f'{desc} (occupied) — if robot is physically in free space, check '
                'map_merge map->robot/map and Nav2 static layer (/map_relay)',
                ok=False,
            )
        else:
            self.log(f'{desc} (free)', ok=True)

    def lookup_explorer_chain(self, robot: str) -> None:
        """map -> base_footprint; accept composed hops if one multi-hop lookup fails on stamp skew."""
        desc = f'Explorer chain: map->{robot}/base_footprint'
        ok, last_err = self._lookup_try_many('map', f'{robot}/base_footprint')
        if ok:
            self.log(desc, ok=True)
            return
        legs = (
            ('map', f'{robot}/map'),
            (f'{robot}/map', f'{robot}/odom'),
            (f'{robot}/odom', f'{robot}/base_footprint'),
        )
        if all(self._lookup_try_many(a, b)[0] for a, b in legs):
            self.log(
                f'{desc} (composed OK: each hop resolves; multi-hop alone hit stamp skew)',
                ok=True,
            )
            return
        self.log(f'{desc}: {last_err}', ok=False)

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
        print('\n=== Multi-Robot TF Diagnostics ===\n')

        domain = os.environ.get('ROS_DOMAIN_ID', '?')
        self.log(
            f'ROS_DOMAIN_ID = {domain} (must match central PC and every robot)',
            ok=None,
        )

        def on_map(msg: OccupancyGrid) -> None:
            self._merged_map = msg

        self.create_subscription(OccupancyGrid, '/map', on_map, qos_transient())

        # 1) Check topic visibility expected from central-side startup.
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)

        discovered = discover_robots_from_graph(self)
        self.robots = discovered if discovered else list(DEFAULT_ROBOTS)
        if discovered:
            self.log(f'Robots under test (from graph): {", ".join(self.robots)}', ok=None)
        else:
            self.log(
                f'No /<robot>/tf publishers yet; checking default list: {", ".join(self.robots)}',
                ok=None,
            )

        critical_topics = ['/tf', '/tf_static'] + [f'/{r}/tf' for r in self.robots] + ['/map']
        # Laser scans stay on the robots; they are optional from the
        # central PC's point of view (Nav2 + SLAM use them locally).
        optional_scan_topics = [f'/{r}/scan' for r in self.robots]

        for topic in critical_topics:
            try:
                n = self.count_publishers(topic)
            except Exception as e:
                self.log(f'Topic {topic}: error - {e}', ok=False)
                continue

            is_global = topic in ('/tf', '/tf_static', '/map')
            is_robot_tf = (
                topic.endswith('/tf')
                and topic not in ('/tf', '/tf_static')
                and topic.count('/') == 2
            )
            if is_global:
                self.log(f'Topic {topic}: {n} publisher(s)', ok=(n > 0))
            elif is_robot_tf:
                if discovered:
                    # Central PC often sees 0 publishers on /<robot>/tf while tf_relay
                    # still merges that robot onto global /tf.
                    if n > 0:
                        self.log(f'Topic {topic}: {n} publisher(s)', ok=True)
                    else:
                        self.log(
                            f'Topic {topic}: 0 publisher(s) '
                            f'(namespaced TF may only be relayed to /tf here; not an error)',
                            ok=None,
                        )
                else:
                    # Default fleet: absent robots are [INFO], not failed checks.
                    self.log(f'Topic {topic}: {n} publisher(s)', ok=True if n > 0 else None)
            else:
                self.log(f'Topic {topic}: {n} publisher(s)', ok=(n > 0))

        for topic in optional_scan_topics:
            try:
                n = self.count_publishers(topic)
                # Report scans as informational only, since in this
                # architecture they are not bridged to the central PC.
                self.log(f'Topic {topic}: {n} publisher(s)', ok=None if n == 0 else True)
            except Exception as e:
                self.log(f'Topic {topic}: error - {e}', ok=None)

        # 2) Let TF buffer fill before transform checks.
        self.log('Waiting 3s for TF buffer...', ok=None)
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)

        rediscovered = discover_robots_from_graph(self)
        if rediscovered:
            merged = sorted(set(self.robots) | set(rediscovered))
            if merged != self.robots:
                self.robots = merged
                self.log(
                    f'Robots after TF wait (refreshed from graph): {", ".join(self.robots)}',
                    ok=None,
                )

        # 3) Check specific transforms used by explorer and Nav2.
        #
        # Critical checks are things that must exist for Nav2 and the
        # central multi_robot_explorer to work:
        #   - odom -> base_footprint for each robot (via tf_relay)
        #   - map -> <robot>/map from map_merge
        #   - full chain map -> <robot>/base_footprint
        #
        # tf2_ros: lookup_transform(target_frame, source_frame, time, ...)
        # e.g. pose of base in odom: target=odom, source=base_footprint
        for r in self.robots:
            self.lookup_transform_robust(
                f'{r}/odom', f'{r}/base_footprint',
                f'Robot TF (odom->base_footprint): {r}')
            self.lookup_transform_robust(
                'map', f'{r}/map',
                f'World TF (map_merge): map->{r}/map')
            self.lookup_transform_robust(
                f'{r}/map', f'{r}/odom',
                f'SLAM TF: {r}/map->{r}/odom',
                optional=True,
            )
            self.lookup_explorer_chain(r)

        if self._merged_map is not None:
            self.log(
                f'Merged /map snapshot: {self._merged_map.info.width}x'
                f'{self._merged_map.info.height} @ {self._merged_map.info.resolution:.3f} m/cell',
                ok=None,
            )
            for r in self.robots:
                self.check_merged_map_footprint(r)
        else:
            self.log(
                'Merged /map: no OccupancyGrid received yet (optional pose-vs-map check skipped)',
                ok=None,
            )

        # 4) Sample /tf and /tf_static contents for quick operator inspection.
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
                if any(topic_key in res and '[MISSING]' in res for res in self.results):
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
                print('  - If the error mentions extrapolation into the future, TF stamps on '
                      'map / SLAM / odom may be skewed; the script retries several past times—'
                      'run again after a few seconds or check clock sync (NTP) on Pis + central PC.')
                print('  - Nav2 "Invalid frame ID map": run tf2_echo map <robot>/base_footprint; '
                      'need map_merge map-><robot>/map on /tf (not just /map topic).')
            if any('SLAM TF:' in r and '[MISSING]' in r for r in self.results):
                print('  - <robot>/map-><robot>/odom: slam_toolbox on that robot may not be publishing yet.')
        else:
            print('All critical checks passed. Multi-robot Nav2 and explorer should work.')
            print('Optional [INFO] items (SLAM TF) appear once slam_toolbox builds maps.')
        if len(self.robots) >= 2:
            print(
                '\nTip: If robots still look swapped on the merged map but TF is OK, '
                'check each Pi hostname (or use explicit robot_name:=... on bringup) '
                'so namespaces match the physical robot.'
            )
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
