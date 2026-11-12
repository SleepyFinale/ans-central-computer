#!/usr/bin/env python3
"""Score a multi-robot exploration run from a central debug bag.

Record with scripts/diagnostics/start_central_debug_bag.sh, then:

    python3 scripts/diagnostics/analyze_fleet_run.py ~/.ros/nav2_debug_central/bag-<id> \\
        [--events ~/.ros/nav2_debug_central/session-<...>.jsonl]

Reports, per robot:
- distance driven and the share of time standing still (moved less than
  still_m over still_window_sec), split into "with a goal" and "no goal"
  when the action status topic is in the bag;
- gaps between one goal ending and the next one starting (from the
  explorer's event log, if given);
and for the fleet:
- merged-map known area over time (m^2) and when it stopped growing;
- overlap between the robots' maps at the end: area known to two or more
  robots over the union, placed with the last map -> <robot>/map TF;
- when map_merge first published a feature-matched alignment (TF stops
  being the side-by-side placement) and how much the alignment jumped
  afterwards.
"""

import argparse
import json
import math
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=''),
        rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = [t for t in types if t in topics or any(
        t.startswith(p[:-1]) for p in topics if p.endswith('*'))]
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield topic, deserialize_message(data, cls[topic]), t * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, ath + bth)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--robots', nargs='+', default=['inky', 'clyde'])
    ap.add_argument('--events', default='')
    ap.add_argument('--still_m', type=float, default=0.05)
    ap.add_argument('--still_window_sec', type=float, default=3.0)
    args = ap.parse_args()
    robots = args.robots

    topics = ['/tf', '/map'] + [f'/{r}/map' for r in robots] + [
        f'/{r}/navigate_to_pose/_action/status' for r in robots]
    odom = defaultdict(list)        # robot -> [(t, x, y)] in <robot>/odom
    map_odom = {}                   # robot -> latest (x, y, th) <robot>/map -> odom
    local_pose = defaultdict(list)  # robot -> [(t, x, y)] in <robot>/map
    world_tf = defaultdict(list)    # robot -> [(t, x, y, th)] map -> <robot>/map
    last_local_map = {}
    merged_area = []                # (t, m^2 known)
    goal_active = defaultdict(list)  # robot -> [(t, active)]
    t0 = None
    for topic, msg, t in read_bag(args.bag, topics):
        if t0 is None:
            t0 = t
        if topic == '/tf':
            for tr in msg.transforms:
                p, c = tr.header.frame_id, tr.child_frame_id
                v = (tr.transform.translation.x, tr.transform.translation.y,
                     yaw_of(tr.transform.rotation))
                for r in robots:
                    if p == f'{r}/odom' and c == f'{r}/base_footprint':
                        mo = map_odom.get(r, (0.0, 0.0, 0.0))
                        g = compose(mo, v)
                        local_pose[r].append((t, g[0], g[1]))
                    elif p == f'{r}/map' and c == f'{r}/odom':
                        map_odom[r] = v
                    elif p == 'map' and c == f'{r}/map':
                        world_tf[r].append((t, *v))
        elif topic == '/map':
            known = np.count_nonzero(np.frombuffer(bytes(msg.data), dtype=np.int8) >= 0)
            merged_area.append((t, known * msg.info.resolution ** 2))
        elif topic.endswith('/map'):
            last_local_map[topic.split('/')[1]] = msg
        elif topic.endswith('_action/status'):
            r = topic.split('/')[1]
            act = any(s.status in (1, 2, 3) for s in msg.status_list)
            goal_active[r].append((t, act))
    if t0 is None:
        print('empty bag')
        return
    t_end = max(p[-1][0] for p in local_pose.values()) if local_pose else t0
    print(f'bag length {t_end - t0:.0f} s')

    def active_at(r, t):
        ga = goal_active.get(r)
        if not ga:
            return None
        lo, hi = 0, len(ga) - 1
        if t < ga[0][0]:
            return False
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if ga[mid][0] <= t:
                lo = mid
            else:
                hi = mid - 1
        return ga[lo][1]

    print('\n== motion (pose in <robot>/map)')
    for r in robots:
        ps = local_pose.get(r)
        if not ps or len(ps) < 2:
            print(f'{r}: no TF')
            continue
        arr = np.array(ps)
        dist = float(np.sum(np.hypot(np.diff(arr[:, 1]), np.diff(arr[:, 2]))))
        # resample at 0.5 s
        ts = np.arange(arr[0, 0], arr[-1, 0], 0.5)
        xs = np.interp(ts, arr[:, 0], arr[:, 1])
        ys = np.interp(ts, arr[:, 0], arr[:, 2])
        k = max(1, int(args.still_window_sec / 0.5))
        still = np.zeros(len(ts), dtype=bool)
        for i in range(len(ts)):
            j0, j1 = max(0, i - k // 2), min(len(ts) - 1, i + k // 2)
            seg_x, seg_y = xs[j0:j1 + 1], ys[j0:j1 + 1]
            still[i] = (np.hypot(seg_x - seg_x[0], seg_y - seg_y[0]).max() < args.still_m)
        frac = still.mean()
        line = f'{r}: drove {dist:.1f} m in {ts[-1] - ts[0]:.0f} s, still {100 * frac:.0f}%'
        if goal_active.get(r):
            act = np.array([bool(active_at(r, t)) for t in ts])
            line += (f' (still with goal {100 * np.mean(still & act):.0f}%, '
                     f'still without goal {100 * np.mean(still & ~act):.0f}%)')
        print(line)
        # longest still stretches
        runs = []
        i = 0
        while i < len(ts):
            if still[i]:
                j = i
                while j + 1 < len(ts) and still[j + 1]:
                    j += 1
                runs.append((ts[j] - ts[i] + 0.5, ts[i] - t0, xs[i], ys[i]))
                i = j + 1
            else:
                i += 1
        runs.sort(reverse=True)
        for dur, start, x, y in runs[:5]:
            if dur >= 5.0:
                print(f'    still {dur:5.1f} s from t={start:6.1f} s at ({x:.2f}, {y:.2f})')

    print('\n== merged map known area')
    if merged_area:
        ma = np.array(merged_area)
        for frac_t in (0.25, 0.5, 0.75, 1.0):
            i = min(len(ma) - 1, int(frac_t * (len(ma) - 1)))
            print(f'  t={ma[i, 0] - t0:6.0f} s  {ma[i, 1]:6.1f} m^2')
        final = ma[-1, 1]
        for q in (0.5, 0.9, 0.95):
            i = int(np.argmax(ma[:, 1] >= q * final))
            print(f'  {int(q * 100)}% of final area at t={ma[i, 0] - t0:.0f} s')
        print('  (merged area counts both maps side by side before the merge, '
              'and overlap twice only until it merges)')

    print('\n== merge alignment (map -> <robot>/map)')
    first_match = None
    for r in robots:
        w = world_tf.get(r)
        if not w:
            print(f'{r}: no TF')
            continue
        arr = np.array(w)
        # jumps between consecutive distinct samples
        d = np.hypot(np.diff(arr[:, 1]), np.diff(arr[:, 2]))
        dth = np.abs(np.angle(np.exp(1j * np.diff(arr[:, 3]))))
        big = np.where((d > 0.3) | (dth > math.radians(10)))[0]
        print(f'{r}: {len(arr)} samples, {len(big)} jumps > 0.3 m / 10 deg')
        for i in big[:8]:
            print(f'    t={arr[i + 1, 0] - t0:6.1f} s  jump {d[i]:.2f} m '
                  f'{math.degrees(dth[i]):.1f} deg -> ({arr[i + 1, 1]:.2f}, '
                  f'{arr[i + 1, 2]:.2f}, {math.degrees(arr[i + 1, 3]):.1f} deg)')
    # relative transform between the first two robots: when it settles
    if len(robots) >= 2 and all(world_tf.get(r) for r in robots[:2]):
        a, b = np.array(world_tf[robots[0]]), np.array(world_tf[robots[1]])
        ts = np.arange(max(a[0, 0], b[0, 0]), min(a[-1, 0], b[-1, 0]), 0.5)
        rel = []
        for t in ts:
            ia = min(len(a) - 1, np.searchsorted(a[:, 0], t))
            ib = min(len(b) - 1, np.searchsorted(b[:, 0], t))
            ax, ay, ath = a[ia, 1:]
            bx, by, bth = b[ib, 1:]
            c, s = math.cos(-ath), math.sin(-ath)
            rel.append((t, c * (bx - ax) - s * (by - ay), s * (bx - ax) + c * (by - ay),
                        math.atan2(math.sin(bth - ath), math.cos(bth - ath))))
        rel = np.array(rel)
        final = rel[-1, 1:]
        ok = (np.hypot(rel[:, 1] - final[0], rel[:, 2] - final[1]) < 0.3) & (
            np.abs(np.angle(np.exp(1j * (rel[:, 3] - final[2])))) < math.radians(5))
        # first time it reached the final alignment and stayed there
        idx = len(ok)
        while idx > 0 and ok[idx - 1]:
            idx -= 1
        if idx < len(ok):
            first_match = rel[idx, 0] - t0
            print(f'{robots[1]} relative to {robots[0]} settles at t={first_match:.0f} s '
                  f'(final rel {final[0]:.2f}, {final[1]:.2f}, {math.degrees(final[2]):.1f} deg); '
                  f'agrees with final {100 * ok[idx:].mean():.0f}% after that, '
                  f'{100 * ok.mean():.0f}% of the whole run')

    print('\n== overlap between robot maps at the end')
    grids = {}
    for r in robots:
        m = last_local_map.get(r)
        w = world_tf.get(r)
        if m is None or not w:
            continue
        g = np.frombuffer(bytes(m.data), dtype=np.int8).reshape(m.info.height, m.info.width)
        rr, cc = np.nonzero(g >= 0)
        res = m.info.resolution
        lx = m.info.origin.position.x + (cc + 0.5) * res
        ly = m.info.origin.position.y + (rr + 0.5) * res
        _, tx, ty, th = w[-1]
        c, s = math.cos(th), math.sin(th)
        wx, wy = tx + c * lx - s * ly, ty + s * lx + c * ly
        grids[r] = set(zip(np.floor(wx / res).astype(int).tolist(),
                           np.floor(wy / res).astype(int).tolist()))
        print(f'{r}: {len(grids[r]) * res * res:.1f} m^2 known')
    if len(grids) >= 2:
        res = 0.05
        cnt = defaultdict(int)
        for cells in grids.values():
            for cell in cells:
                cnt[cell] += 1
        union = len(cnt)
        shared = sum(1 for v in cnt.values() if v >= 2)
        print(f'union {union * res * res:.1f} m^2, known to 2+ robots '
              f'{shared * res * res:.1f} m^2 ({100 * shared / max(1, union):.0f}%)')
        if first_match is None:
            print('  (maps were never feature-matched: this overlap is of the '
                  'side-by-side placement and means nothing)')

    if args.events:
        print('\n== gaps between goals (explorer event log)')
        ev = [json.loads(line) for line in open(args.events)]
        by = defaultdict(list)
        for e in ev:
            if e.get('event') in ('goal_sent', 'goal_result') and e.get('robot'):
                by[e['robot']].append((e['t_ros_ns'] * 1e-9, e['event']))
        for r, lst in by.items():
            lst.sort()
            gaps = []
            last_end = None
            for t, kind in lst:
                if kind == 'goal_result':
                    last_end = t
                elif kind == 'goal_sent' and last_end is not None:
                    gaps.append(t - last_end)
                    last_end = None
            n = sum(1 for _, k in lst if k == 'goal_sent')
            if gaps:
                g = np.array(gaps)
                print(f'{r}: {n} goals, gap median {np.median(g):.1f} s, '
                      f'p90 {np.percentile(g, 90):.1f} s, max {g.max():.1f} s, '
                      f'total {g.sum():.0f} s')
            else:
                print(f'{r}: {n} goals')


if __name__ == '__main__':
    main()
