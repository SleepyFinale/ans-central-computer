#!/usr/bin/env python3
"""Which map_merge gate stops AKAZE? Scores its estimate at each time point with the gates relaxed.

Usage (after sourcing ROS 2): python3 akaze_gates.py hall1
"""
import sys

import cv2
import numpy as np

from eval_twobot import STEP_S, SUCCESS_M, read_maps
from matching import _as_image
from registration import Rigid

RUN = sys.argv[1]
d = np.load(f'data/{RUN}_final.npz')
T_ref = Rigid(float(d['theta']), d['t'])
res = float(d['res'])
sample = d['ob'] + (np.argwhere(d['gb'] == 0)[::50, ::-1] + 0.5) * res
inky = read_maps(f'data/bags/{RUN}/inky', '/inky/map')
clyde = read_maps(f'data/bags/{RUN}/clyde', '/clyde/map')
start, end = max(inky[0][0], clyde[0][0]), min(inky[-1][0], clyde[-1][0])
rows = []
for t in np.arange(start + STEP_S, end + 1e-3, STEP_S):
    a = max((m for m in inky if m[0] <= t), key=lambda m: m[0])
    b = max((m for m in clyde if m[0] <= t), key=lambda m: m[0])
    feats = [cv2.detail.computeImageFeatures2(cv2.AKAZE_create(), _as_image(g)) for g in (a[1], b[1])]
    row = dict(t=t - start, error=None, inliers=0, ratio=0.0, conf=0.0)
    try:
        pairwise = cv2.detail_AffineBestOf2NearestMatcher(False, False, 0.3, 6).apply2(feats)
        m = next((x for x in pairwise if x.src_img_idx == 1 and x.dst_img_idx == 0), None)
    except cv2.error:
        m = None
    if m is not None and m.H is not None and m.H.size:
        H = np.asarray(m.H, float)
        theta = np.arctan2(H[1, 0], H[0, 0])
        T = Rigid(theta, a[2] + res * H[:2, 2] - Rigid(theta, np.zeros(2)).R @ b[2])
        n = len(m.getMatches())
        row.update(error=float(np.linalg.norm(T.apply(sample) - T_ref.apply(sample), axis=1).mean()),
                   inliers=int(m.num_inliers), ratio=m.num_inliers / n if n else 0.0, conf=float(m.confidence))
    rows.append(row)

ok = lambda r: r['error'] is not None and r['error'] < SUCCESS_M
gates = {
    'map_merge gates (inliers>=8, ratio>=0.5, conf>=0.65)': lambda r: r['inliers'] >= 8 and r['ratio'] >= 0.5 and r['conf'] >= 0.65,
    'without the inlier-ratio gate': lambda r: r['inliers'] >= 8 and r['conf'] >= 0.65,
    'inliers>=8 only': lambda r: r['inliers'] >= 8,
    'no gates': lambda r: r['error'] is not None,
}
print(f"{RUN}: {len(rows)} time points")
for r in rows:
    if r["inliers"] >= 8 and r["conf"] >= 0.65:
        print(f"    t={r['t']:4.0f} ratio={r['ratio']:.2f} inliers={r['inliers']:3d} conf={r['conf']:.2f} {'right' if ok(r) else 'WRONG'}")
for name, gate in gates.items():
    passed = [r for r in rows if gate(r)]
    right = [r for r in passed if ok(r)]
    print(f'  {name:52s} correct {len(right):3d}  wrong {len(passed) - len(right):3d}  first correct at {min((r["t"] for r in right), default=None)}')
