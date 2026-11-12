#!/usr/bin/env python3
"""Held-out test on a real two-robot run. Settings below were frozen before looking at this run.

At regular times, take the map each robot held at that moment and ask each method for the
Clyde -> Inky transform. Also reads what the live map_merge node had published by then.

Run with system Python after sourcing ROS 2:  python3 eval_twobot.py twobot1 twobot4
"""
import json
import os
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
from matching import akaze_baseline, akaze_estimate, clearance_similarity, consistency_match, faq, \
    geodesic_kernel, map_agreement, ransac_rigid
from registration import Rigid, register_refined
from run_real import build_jobs, evaluate, free_overlap

SPACING = 0.25
RESTARTS = 8
STEP_S = 15.0
SUCCESS_M = 0.30
WRONG_ACCEPT_LIMIT = 0.05
# Added 2026-11-10 after both two-robot runs showed accepted answers at 0% overlap: an answer
# needs this much area known in both maps before the gate will consider it.
MIN_SHARED_M2 = 3.0
METHODS = ('clearance_only', 'geodesic_clearance', 'partial_geodesic', 'partial_euclidean')


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
    # Partial matching, settings fixed in advance and not tuned (top_k=8, sigma=0.15 m).
    options['partial_geodesic'] = [consistency_match(A, B, 'geodesic')]
    options['partial_euclidean'] = [consistency_match(A, B, 'euclidean')]
    found = {}
    for name, candidates in options.items():
        best = (-np.inf, None, 1.0, 0)
        for matches in candidates:
            T, _ = ransac_rigid(A.xy, B.xy, matches, rng=rng)
            if T is not None:
                score, conflict, known = map_agreement(ga, oa, gb, ob, res, T)
                if score > best[0]:
                    best = (score, T, conflict, known)
        if best[1] is not None:
            found[name] = (best[1], best[2], best[3] * res ** 2)
    return found, A, B


def training_case(job):
    kind, a, b, label, ga, oa, gb, ob, res, T_ref, seed = job
    found, _, B = graph_method(ga, oa, gb, ob, res, np.random.default_rng(seed))
    out = {name: (conflict, bool(evaluate(T, T_ref, B.xy) < SUCCESS_M), shared)
           for name, (T, conflict, shared) in found.items()}
    out['overlap'] = float(free_overlap(ga, oa, gb, ob, res, T_ref)[0])
    return out


def freeze_gates():
    """Largest conflict threshold whose wrong-accept share on the older runs stays within the limit."""
    with Pool(16) as pool:
        rows = pool.map(training_case, build_jobs(), chunksize=4)
    gates = {}
    print(f"development pairs ({len(rows)}): share correct before any gate")
    for name in METHODS:
        low = [r.get(name, (1, False, 0))[1] for r in rows if r['overlap'] < 0.5]
        high = [r.get(name, (1, False, 0))[1] for r in rows if r['overlap'] >= 0.5]
        print(f"  {name:20s} overlap<0.5: {np.mean(low):4.0%} (n={len(low)})   overlap>=0.5: {np.mean(high):4.0%} (n={len(high)})")
        points = [r[name] for r in rows if name in r]
        best, kept_most = 0.0, -1
        for gate in np.arange(0.02, 0.30, 0.005):
            kept = [ok for conflict, ok, shared in points if conflict <= gate and shared >= MIN_SHARED_M2]
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
    # Centre of Clyde's grid, where map_merge measures how far two alignments disagree.
    row['centre_b'] = (ob + 0.5 * res * np.array([gb.shape[1], gb.shape[0]])).tolist()
    T, info = akaze_baseline(ga, oa, gb, ob, res)
    row['akaze'] = dict(answered=T is not None, error=None if T is None else evaluate(T, T_ref, B.xy), **info)
    T, info = akaze_estimate(ga, oa, gb, ob, res)
    row['akaze_raw'] = dict(info, error=None if T is None else evaluate(T, T_ref, B.xy),
                            theta=None if T is None else float(T.theta),
                            t=None if T is None else T.t.tolist())
    for name, (T, conflict, shared) in found.items():
        row[name] = dict(error=evaluate(T, T_ref, B.xy), conflict=conflict, shared=shared,
                         accepted=bool(conflict <= gates[name] and shared >= MIN_SHARED_M2),
                         theta=float(T.theta), t=T.t.tolist())
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


def ok(e):
    return e is not None and e < SUCCESS_M


def main():
    print('freezing gates on older runs only ...')
    gates = freeze_gates()
    print('gates', {k: round(v, 3) for k, v in gates.items()})
    for run in sys.argv[1:]:
        print(f'\n===== {run} =====')
        evaluate_run(run, gates)


def evaluate_run(RUN, gates):
    central_bag = f'data/bags/{RUN}/central'
    # A robot's own bag has every map message; when it is missing (robot off
    # the network after the run), the central bag's copy is used instead.
    def robot_maps(robot):
        own = f'data/bags/{RUN}/{robot}'
        return read_maps(own if os.path.isdir(own) else central_bag, f'/{robot}/map')

    inky = robot_maps('inky')
    clyde = robot_maps('clyde')
    _, ga, oa, res = inky[-1]
    _, gb, ob, _ = clyde[-1]
    T_ref, score, rival = register_refined(ga, oa, gb, ob, res)
    print(f'reference: theta={np.rad2deg(T_ref.theta):.2f} deg t={np.round(T_ref.t, 2)} score/rival={score / rival:.2f}')
    conflict = map_agreement(ga, oa, gb, ob, res, T_ref)[1]
    print(f'reference conflict rate {conflict:.1%}')
    np.savez_compressed(f'data/{RUN}_final.npz', ga=ga, oa=oa, gb=gb, ob=ob, res=res,
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
              open(f'results/{RUN}.json', 'w'), indent=0)

    print(f"{'t':>5}{'free I':>8}{'free C':>8}{'overlap':>9}{'nI':>5}{'nC':>5}{'AKAZE':>8}{'live':>8}"
          + ''.join(f'{m[:12]:>14}' for m in METHODS))
    for r in rows:
        ak = r.get('akaze', {})
        def show(m):
            d = r.get(m)
            return '-' if d is None else f"{d['error']:.2f}{'*' if d['accepted'] else ' '}"
        print(f"{r['t']:5.0f}{r['free_a']:8.1f}{r['free_b']:8.1f}{r['overlap']:9.0%}{r['n_a']:5d}{r['n_b']:5d}"
              f"{('%.2f' % ak['error']) if ak.get('answered') else 'none':>8}"
              f"{('%.2f' % r['live_error']) if r['live_error'] is not None else '-':>8}"
              + ''.join(f'{show(m):>14}' for m in METHODS))
    n = len(rows)
    print('(* = passes the frozen gate; errors in metres)')

    def first(times):
        return min(times) if times else None

    print(f"time points {n}")
    print(f"AKAZE offline: correct {sum(ok(r.get('akaze', {}).get('error')) for r in rows)}, wrong {sum(r.get('akaze', {}).get('answered', False) and not ok(r['akaze']['error']) for r in rows)}, no answer {sum(not r.get('akaze', {}).get('answered', False) for r in rows)}; first correct at {first([r['t'] for r in rows if ok(r.get('akaze', {}).get('error'))])} s")
    print(f"live map_merge TF: correct {sum(ok(r['live_error']) for r in rows)}, wrong {sum(r['live_error'] is not None and not ok(r['live_error']) for r in rows)}")
    for m in METHODS:
        have = [r[m] for r in rows if m in r]
        print(f"{m}: raw correct {sum(ok(d['error']) for d in have)}; accepted correct {sum(d['accepted'] and ok(d['error']) for d in have)}, accepted wrong {sum(d['accepted'] and not ok(d['error']) for d in have)}, rejected {sum(not d['accepted'] for d in have)}; "
              f"first accepted correct at {first([r['t'] for r in rows if m in r and r[m]['accepted'] and ok(r[m]['error'])])} s")


if __name__ == '__main__':
    main()
