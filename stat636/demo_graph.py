#!/usr/bin/env python3
"""Draw each run's final map with its free-space graph."""
import glob
import os
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from map_graph import build_graph, clean_free_space

files = sorted(glob.glob('data/maps/*.npz'))
fig, axes = plt.subplots(2, len(files), figsize=(3.6 * len(files), 7))
for k, path in enumerate(files):
    d = np.load(path)
    i = len(d['fractions']) - 1
    grid, res, origin = d[f'grid_{i}'], float(d[f'resolution_{i}']), d[f'origin_{i}']
    start = time.time()
    g = build_graph(grid, res, origin)
    extent = [origin[0], origin[0] + grid.shape[1] * res, origin[1], origin[1] + grid.shape[0] * res]
    img = np.full(grid.shape, 0.65); img[grid == 0] = 1; img[grid > 50] = 0
    axes[0, k].imshow(img, cmap='gray', origin='lower', extent=extent, vmin=0, vmax=1)
    axes[1, k].imshow(clean_free_space(grid, res), cmap='Greys', alpha=.25, origin='lower', extent=extent)
    for i_, j_ in zip(*np.nonzero(np.triu(g.lengths))):
        axes[1, k].plot(g.xy[[i_, j_], 0], g.xy[[i_, j_], 1], 'k-', lw=.8)
    degree = (g.lengths > 0).sum(1)
    axes[1, k].scatter(*g.xy.T, s=14, c=np.where(degree >= 3, 'tab:red', np.where(degree == 1, 'tab:orange', 'tab:blue')), zorder=3)
    m = int(degree.sum() / 2)
    print(f'{os.path.basename(path)}: n={g.n} m={m} mean_deg={degree.mean():.2f} '
          f'junctions={(degree>=3).sum()} ends={(degree==1).sum()} cycles={m-g.n+1} ({time.time()-start:.1f}s)')
    axes[0, k].set_title(os.path.basename(path)[:-4], fontsize=9)
    axes[1, k].set_title(f'n={g.n}, m={m}', fontsize=9)
    for a in axes[:, k]:
        a.set_aspect('equal'); a.tick_params(labelsize=6)
plt.tight_layout(); plt.savefig('figures/free_space_graphs.png', dpi=100)
