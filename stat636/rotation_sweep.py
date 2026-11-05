#!/usr/bin/env python3
"""Known-answer test: does each method depend on the relative orientation of the two maps?

Rotate Clyde's final map by a chosen angle and align it to Inky's final map. The true transform
is the reference composed with that rotation, so the answer is known at every angle.
"""
import json
from multiprocessing import Pool

import cv2
import numpy as np

from eval_twobot import graph_method
from matching import akaze_baseline
from registration import Rigid

d = np.load('data/twobot1_final.npz')
ga, oa, gb, ob, res = d['ga'], d['oa'], d['gb'], d['ob'], float(d['res'])
T_ref = Rigid(float(d['theta']), d['t'])
ANGLES = list(range(0, 181, 5))
SEEDS = 5


def rotated_grid(grid, origin, degrees):
    theta = np.deg2rad(degrees)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    h, w = grid.shape
    size = int(np.hypot(h, w)) + 4
    centre, out = np.array([w / 2, h / 2]), np.array([size / 2, size / 2])
    M = np.column_stack([R, out - R @ centre])
    warped = cv2.warpAffine((grid + 1).astype(np.uint8), M, (size, size), flags=cv2.INTER_NEAREST, borderValue=0)
    new_origin = np.zeros(2)
    # p_new = R p_old + (out - R centre); world uses each grid's own origin.
    # x_old = origin + res p_old, x_new = new_origin + res p_new, so x_old = R^T (x_new - res shift) + ...
    shift = res * (out - R @ centre)
    to_old = Rigid(-theta, origin - R.T @ shift)
    return warped.astype(np.int16).astype(np.int8) - 1, new_origin, to_old


def compose(first_then, second):
    """Transform applying `second` first, then `first_then`."""
    return Rigid(first_then.theta + second.theta, first_then.R @ second.t + first_then.t)


def run(job):
    angle, seed = job
    rng = np.random.default_rng(seed)
    gr, orot, to_old = rotated_grid(gb, ob, angle)
    truth = compose(T_ref, to_old)
    found, _, B = graph_method(ga, oa, gr, orot, res, rng)
    err = lambda T: float(np.linalg.norm(T.apply(B.xy) - truth.apply(B.xy), axis=1).mean())
    T, info = akaze_baseline(ga, oa, gr, orot, res)
    row = dict(angle=angle, relative=(angle + np.rad2deg(T_ref.theta)) % 360, n_b=B.n,
               akaze=None if T is None else err(T), akaze_inliers=info['inliers'], akaze_conf=info['confidence'])
    for name, (Tm, conflict) in found.items():
        row[name] = err(Tm)
    return row


if __name__ == '__main__':
    jobs = [(a, 100 * a + s) for a in ANGLES for s in range(SEEDS)]
    with Pool(16) as pool:
        rows = pool.map(run, jobs)
    json.dump(rows, open('results/rotation_sweep.json', 'w'))
    print(f"{'added':>6}{'relative':>9}{'n_b':>5}{'AKAZE ok':>10}{'inliers':>9}{'conf':>7}{'clear ok':>10}{'geo+clear ok':>14}")
    ok = lambda v: v is not None and v < 0.30
    for a in ANGLES:
        sub = [r for r in rows if r['angle'] == a]
        print(f"{a:6d}{sub[0]['relative']:9.1f}{sub[0]['n_b']:5d}"
              f"{'yes' if ok(sub[0]['akaze']) else ('wrong' if sub[0]['akaze'] is not None else 'none'):>10}"
              f"{sub[0]['akaze_inliers']:9d}{sub[0]['akaze_conf']:7.2f}"
              f"{np.mean([ok(r.get('clearance_only')) for r in sub]):>10.0%}{np.mean([ok(r.get('geodesic_clearance')) for r in sub]):>14.0%}")
    near = [r for r in rows if min(r['relative'] % 90, 90 - r['relative'] % 90) <= 10]
    far = [r for r in rows if min(r['relative'] % 90, 90 - r['relative'] % 90) > 10]
    for name, group in (('within 10 deg of grid axes', near), ('more than 10 deg from grid axes', far)):
        print(f"{name}: n={len(group)}, AKAZE {np.mean([ok(r['akaze']) for r in group]):.0%}, "
              f"clearance {np.mean([ok(r.get('clearance_only')) for r in group]):.0%}, geo+clear {np.mean([ok(r.get('geodesic_clearance')) for r in group]):.0%}")
