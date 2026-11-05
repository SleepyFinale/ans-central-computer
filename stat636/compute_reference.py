#!/usr/bin/env python3
"""Register every pair of final maps; save the reference transforms and an overlay figure."""
import glob
import itertools
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from registration import register_refined

runs = {os.path.basename(p).split('_')[0]: np.load(p) for p in sorted(glob.glob('data/maps/*.npz'))}


def final(d):
    i = len(d['fractions']) - 1
    return d[f'grid_{i}'], d[f'origin_{i}'], float(d[f'resolution_{i}'])


def cells_world(grid, origin, res, mask):
    rc = np.argwhere(mask)
    return origin + (rc[:, ::-1] + .5) * res


pairs = list(itertools.combinations(runs, 2))
reference = {}
fig, axes = plt.subplots(2, 5, figsize=(16, 6.5))
for ax, (a, b) in zip(axes.flat, pairs):
    ga, oa, res = final(runs[a])
    gb, ob, _ = final(runs[b])
    T, score, rival = register_refined(ga, oa, gb, ob, res)
    walls_a = cells_world(ga, oa, res, ga > 50)
    walls_b = T.apply(cells_world(gb, ob, res, gb > 50))
    # Fraction of B's walls within 10 cm of a wall in A, in the overlap bounding box.
    from scipy.spatial import cKDTree
    near = cKDTree(walls_a).query(walls_b)[0] < 0.10
    reference[f'{a}/{b}'] = {'theta': float(T.theta), 't': T.t.tolist(), 'score': float(score),
                             'rival': float(rival), 'wall_agreement': float(near.mean())}
    print(f'{a}/{b}: theta={np.rad2deg(T.theta):7.2f} deg t=({T.t[0]:.2f},{T.t[1]:.2f}) '
          f'score/rival={score / rival:.2f} walls within 10cm={near.mean():.0%}')
    ax.scatter(*walls_a.T, s=1.5, c='k', label=a)
    ax.scatter(*walls_b.T, s=1.5, c='tab:red', label=b)
    ax.set_aspect('equal'); ax.set_title(f'{a} (black) / {b} (red)', fontsize=9); ax.tick_params(labelsize=6)
plt.tight_layout(); plt.savefig('figures/reference_overlays.png', dpi=90)
json.dump(reference, open('data/reference_transforms.json', 'w'), indent=1)
