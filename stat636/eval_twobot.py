#!/usr/bin/env python3
"""Held-out test on a real two-robot run. Settings below were frozen before looking at this run.

At regular times, take the map each robot held at that moment and ask each method for the
Clyde -> Inky transform. Also reads what the live map_merge node had published by then.

Run with system Python after sourcing ROS 2.
"""
import json
import sys
from multiprocessing import Pool

import numpy as np
import rosbag2_py
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message
from scipy.optimize import linear_sum_assignment
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from map_graph import build_graph
from matching import akaze_baseline, clearance_similarity, faq, geodesic_kernel, map_agreement, ransac_rigid
from registration import Rigid, register_refined
from run_real import build_jobs, evaluate, free_overlap

SPACING = 0.25
RESTARTS = 8
STEP_S = 15.0
SUCCESS_M = 0.30
WRONG_ACCEPT_LIMIT = 0.05


def read_topics(path, wanted):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in wanted:
            yield topic, deserialize_message(data, wanted[topic]), stamp * 1e-9


def read_maps(path, topic):
    out = []
    for _, msg, stamp in read_topics(path, {topic: OccupancyGrid}):
        info = msg.info
        out.append((stamp, np.array(msg.data, dtype=np.int8).reshape(info.height, info.width),
                    np.array([info.origin.position.x, info.origin.position.y]), info.resolution))
    return out


def graph_method(ga, oa, gb, ob, res, rng):
    """Both unseeded methods at the frozen settings. Returns {name: (T, conflict)}."""
    A, B = build_graph(ga, res, oa, spacing=SPACING), build_graph(gb, res, ob, spacing=SPACING)
    if min(A.n, B.n) < 5:
        return {}, A, B
    Ka, Kb = geodesic_kernel(A), geodesic_kernel(B)
    C = clearance_similarity(A, B)
    weight = np.abs(Ka).sum() / C.size * C.shape[0]
    options = {
        'geodesic_clearance': [m for m, _ in faq(Ka, Kb, C=C, weight=weight, restarts=RESTARTS, rng=rng)],
        'clearance_only': [np.column_stack(linear_sum_assignment(-(C + (0 if k == 0 else rng.normal(0, .05, C.shape)))))
                           for k in range(RESTARTS)],
    }
    found = {}
    for name, candidates in options.items():
        best = (-np.inf, None, 1.0)
        for matches in candidates:
            T, _ = ransac_rigid(A.xy, B.xy, matches, rng=rng)
            if T is not None:
                score, conflict, _ = map_agreement(ga, oa, gb, ob, res, T)
                if score > best[0]:
                    best = (score, T, conflict)
        if best[1] is not None:
            found[name] = (best[1], best[2])
    return found, A, B


def training_case(job):
    kind, a, b, label, ga, oa, gb, ob, res, T_ref, seed = job
    found, _, B = graph_method(ga, oa, gb, ob, res, np.random.default_rng(seed))
    return {name: (conflict, evaluate(T, T_ref, B.xy) < SUCCESS_M) for name, (T, conflict) in found.items()}


def freeze_gates():
    """Largest conflict threshold whose wrong-accept share on the older runs stays within the limit."""
    with Pool(16) as pool:
        rows = pool.map(training_case, build_jobs(), chunksize=4)
    gates = {}
    for name in ('geodesic_clearance', 'clearance_only'):
        points = [r[name] for r in rows if name in r]
        best, kept_most = 0.0, -1
        for gate in np.arange(0.02, 0.30, 0.005):
            kept = [ok for conflict, ok in points if conflict <= gate]
            if kept and 1 - np.mean(kept) <= WRONG_ACCEPT_LIMIT and len(kept) > kept_most:
                best, kept_most = float(gate), len(kept)
        gates[name] = best
    return gates


def test_case(job):
    t, ga, oa, gb, ob, res, T_ref, gates, seed = job
    rng = np.random.default_rng(seed)
    found, A, B = graph_method(ga, oa, gb, ob, res, rng)
    row = dict(t=t, n_a=A.n, n_b=B.n, overlap=float(free_overlap(ga, oa, gb, ob, res, T_ref)[0]),
               free_a=float(np.sum(ga == 0) * res ** 2), free_b=float(np.sum(gb == 0) * res ** 2))
    if B.n == 0:
        return row
    T, info = akaze_baseline(ga, oa, gb, ob, res)
    row['akaze'] = dict(answered=T is not None, error=None if T is None else evaluate(T, T_ref, B.xy), **info)
    for name, (T, conflict) in found.items():
        row[name] = dict(error=evaluate(T, T_ref, B.xy), conflict=conflict, accepted=conflict <= gates[name])
    return row


def live_map_merge(central_bag, T_ref, sample_xy):
    """Error over time of the transform the running map_merge node published."""
    latest = {}
    series = []
    for _, msg, stamp in read_topics(central_bag, {'/tf': TFMessage}):
        for tf in msg.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id in ('inky/map', 'clyde/map'):
                q, p = tf.transform.rotation, tf.transform.translation
                latest[tf.child_frame_id] = Rigid(2 * np.arctan2(q.z, q.w), np.array([p.x, p.y]))
        if len(latest) == 2:
            inky, clyde = latest['inky/map'], latest['clyde/map']
            composed = (clyde.apply(sample_xy) - inky.t) @ inky.R
            error = np.linalg.norm(composed - T_ref.apply(sample_xy), axis=1).mean()
            series.append((stamp, float(error)))
    return np.array(series)


def main():
    central_bag = sys.argv[1]
    print('freezing gates on older runs only ...')
    gates = freeze_gates()
    print('gates', gates)

    inky = read_maps('data/bags/twobot1/inky', '/inky/map')
    clyde = read_maps('data/bags/twobot1/clyde', '/clyde/map')
    _, ga, oa, res = inky[-1]
    _, gb, ob, _ = clyde[-1]
    T_ref, score, rival = register_refined(ga, oa, gb, ob, res)
    print(f'reference: theta={np.rad2deg(T_ref.theta):.2f} deg t={np.round(T_ref.t, 2)} score/rival={score / rival:.2f}')
    conflict = map_agreement(ga, oa, gb, ob, res, T_ref)[1]
    print(f'reference conflict rate {conflict:.1%}')
    np.savez_compressed('data/twobot1_final.npz', ga=ga, oa=oa, gb=gb, ob=ob, res=res,
                        theta=T_ref.theta, t=T_ref.t)

    start = max(inky[0][0], clyde[0][0])
    end = min(inky[-1][0], clyde[-1][0])
    jobs = []
    for k, t in enumerate(np.arange(start + STEP_S, end + 1e-3, STEP_S)):
        a = max((m for m in inky if m[0] <= t), key=lambda m: m[0])
        b = max((m for m in clyde if m[0] <= t), key=lambda m: m[0])
        jobs.append((float(t - start), a[1], a[2], b[1], b[2], a[3], T_ref, gates, 9000 + k))
    with Pool(16) as pool:
        rows = pool.map(test_case, jobs)

    sample_xy = build_graph(gb, res, ob, spacing=SPACING).xy
    live = live_map_merge(central_bag, T_ref, sample_xy)
    for row in rows:
        t = start + row['t']
        before = live[live[:, 0] <= t] if len(live) else live
        row['live_error'] = float(before[-1, 1]) if len(before) else None

    states = [(stamp, msg.data) for _, msg, stamp in read_topics(central_bag, {'/map_merge/merge_state': String})]
    for row in rows:
        t = start + row['t']
        before = [s for stamp, s in states if stamp <= t]
        row['merge_state'] = before[-1] if before else None

    json.dump(dict(gates=gates, rows=rows, theta=float(T_ref.theta), t=T_ref.t.tolist(), start=float(start)),
              open('results/twobot1.json', 'w'), indent=0)

    def ok(e):
        return e is not None and e < SUCCESS_M

    print(f"{'t':>5}{'free I':>8}{'free C':>8}{'overlap':>9}{'nI':>5}{'nC':>5}{'AKAZE':>9}{'live':>9} {'state':>11}{'clear':>13}{'geo+clear':>13}")
    for r in rows:
        ak = r.get('akaze', {})
        def show(m):
            d = r.get(m)
            return '-' if d is None else f"{d['error']:.2f}{'*' if d['accepted'] else ' '}"
        print(f"{r['t']:5.0f}{r['free_a']:8.1f}{r['free_b']:8.1f}{r['overlap']:9.0%}{r['n_a']:5d}{r['n_b']:5d}"
              f"{('%.2f' % ak['error']) if ak.get('answered') else 'none':>9}"
              f"{('%.2f' % r['live_error']) if r['live_error'] is not None else '-':>9} {str(r['merge_state']):>11}"
              f"{show('clearance_only'):>13}{show('geodesic_clearance'):>13}")
    n = len(rows)
    print('(* = passes the frozen gate; errors in metres)')
    print(f"time points {n}")
    print(f"AKAZE offline: correct {sum(ok(r.get('akaze', {}).get('error')) for r in rows)}, wrong {sum(r.get('akaze', {}).get('answered', False) and not ok(r['akaze']['error']) for r in rows)}, no answer {sum(not r.get('akaze', {}).get('answered', False) for r in rows)}")
    print(f"live map_merge published transform: correct {sum(ok(r['live_error']) for r in rows)}, wrong {sum(r['live_error'] is not None and not ok(r['live_error']) for r in rows)}")
    for m in ('clearance_only', 'geodesic_clearance'):
        have = [r[m] for r in rows if m in r]
        print(f"{m}: raw correct {sum(ok(d['error']) for d in have)} of {n}; accepted correct {sum(d['accepted'] and ok(d['error']) for d in have)}, accepted wrong {sum(d['accepted'] and not ok(d['error']) for d in have)}, rejected {sum(not d['accepted'] for d in have)}")


if __name__ == '__main__':
    main()
