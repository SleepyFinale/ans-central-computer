#!/usr/bin/env python3
"""Overlay the final Inky and Clyde maps under the reference transform, for a visual check."""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from registration import Rigid

RUN = sys.argv[1] if len(sys.argv) > 1 else 'twobot1'
d = np.load(f'data/{RUN}_final.npz')
ga, oa, gb, ob, res = d['ga'], d['oa'], d['gb'], d['ob'], float(d['res'])
T = Rigid(float(d['theta']), d['t'])


def cells(grid, origin, mask):
    return origin + (np.argwhere(mask)[:, ::-1] + 0.5) * res


wa, wb = cells(ga, oa, ga > 50), T.apply(cells(gb, ob, gb > 50))
near_b = cKDTree(wa).query(wb)[0] < 0.10
near_a = cKDTree(wb).query(wa)[0] < 0.10
print(f'walls within 10 cm: Clyde onto Inky {near_b.mean():.0%}, Inky onto Clyde {near_a.mean():.0%}')
print(f'free area: Inky {np.sum(ga == 0) * res ** 2:.1f} m2, Clyde {np.sum(gb == 0) * res ** 2:.1f} m2')
fig, axes = plt.subplots(1, 3, figsize=(13, 4.6))
for ax, grid, origin, name in ((axes[0], ga, oa, 'Inky final map'), (axes[1], gb, ob, 'Clyde final map')):
    img = np.full(grid.shape, .65); img[grid == 0] = 1; img[grid > 50] = 0
    ax.imshow(img, cmap='gray', origin='lower', vmin=0, vmax=1,
              extent=[origin[0], origin[0] + grid.shape[1] * res, origin[1], origin[1] + grid.shape[0] * res])
    ax.set_title(name, fontsize=10)
axes[2].scatter(*wa.T, s=2, c='k', label='Inky walls')
axes[2].scatter(*wb.T, s=2, c='tab:red', label='Clyde walls, transformed')
axes[2].set_aspect('equal'); axes[2].legend(fontsize=8, loc='best'); axes[2].set_title('Reference overlay', fontsize=10)
plt.tight_layout(); plt.savefig(f'figures/{RUN}_reference.png', dpi=90)
