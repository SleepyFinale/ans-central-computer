#!/usr/bin/env python3
"""Figures for the STAT 636 presentation. Run with system Python after sourcing ROS 2 (hall2 snapshots come from bags)."""
import json
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

sys.path.insert(0, '.')
from map_graph import build_graph
from registration import Rigid

NAVY, TEAL, CORAL, GREY = '#1F2A36', '#0F7C80', '#E4572E', '#8A94A0'
for name in ('Carlito', 'Liberation Sans', 'DejaVu Sans'):
    if any(name in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = name
        break
plt.rcParams.update({'font.size': 13, 'axes.edgecolor': '#C9CED4', 'axes.labelcolor': NAVY,
                     'xtick.color': GREY, 'ytick.color': GREY, 'axes.titlecolor': NAVY, 'axes.titlesize': 15})
OUT = 'presentation/figures'


def map_image(grid):
    img = np.ones(grid.shape + (3,))
    img[grid < 0] = (0.90, 0.91, 0.93)
    img[grid > 50] = (0.12, 0.16, 0.21)
    return img


def extent(grid, origin, res):
    return [origin[0], origin[0] + grid.shape[1] * res, origin[1], origin[1] + grid.shape[0] * res]


def wall_xy(grid, origin, res):
    rc = np.argwhere(grid > 50)
    return origin + (rc[:, ::-1] + 0.5) * res


def clean(ax):
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_aspect('equal')


def fig_problem():
    d = np.load('data/hall3_final.npz')
    ga, oa, gb, ob, res = d['ga'], d['oa'], d['gb'], d['ob'], float(d['res'])
    T = Rigid(float(d['theta']), d['t'])
    plt.rcParams.update({'font.size': 17, 'axes.titlesize': 19})
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.3))
    axes[0].imshow(map_image(ga), origin='lower', extent=extent(ga, oa, res))
    axes[0].set_title("Inky's map (its own frame)")
    axes[1].imshow(map_image(gb), origin='lower', extent=extent(gb, ob, res))
    axes[1].set_title("Clyde's map (its own frame)")
    wa, wb = wall_xy(ga, oa, res), T.apply(wall_xy(gb, ob, res))
    axes[2].scatter(*wa.T, s=0.6, c=NAVY, label='Inky')
    axes[2].scatter(*wb.T, s=0.6, c=TEAL, label='Clyde, aligned')
    axes[2].set_title('Aligned: rotate 270°, shift (−6.1, 2.5) m')
    leg = axes[2].legend(loc='lower left', markerscale=12, frameon=False, fontsize=15)
    for ax in axes:
        clean(ax)
    plt.tight_layout(rect=(0, 0, 1, 0.97), w_pad=2.0)
    plt.savefig(f'{OUT}/fig_problem.png', dpi=200, transparent=False, facecolor='white', bbox_inches='tight', pad_inches=0.15)
    plt.close()
    plt.rcParams.update({'font.size': 13, 'axes.titlesize': 15})


def fig_graph():
    d = np.load('data/maps/large6_inky.npz')
    i = len(d['fractions']) - 1
    grid, res, origin = d[f'grid_{i}'], float(d[f'resolution_{i}']), d[f'origin_{i}']
    g = build_graph(grid, res, origin, spacing=0.5)
    D = g.geodesic
    order = np.argsort(D[0])               # nodes ordered by travel distance from one node
    A = g.adjacency[np.ix_(order, order)]
    K = np.exp(-D[np.ix_(order, order)] ** 2 / 2.0)
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2), gridspec_kw={'width_ratios': [1.15, 1, 1]})
    axes[0].imshow(map_image(grid), origin='lower', extent=extent(grid, origin, res))
    for a, b in zip(*np.nonzero(np.triu(g.lengths))):
        axes[0].plot(g.xy[[a, b], 0], g.xy[[a, b], 1], color=TEAL, lw=1.6)
    axes[0].scatter(*g.xy.T, s=22, c=TEAL, edgecolors='white', linewidths=0.6, zorder=3)
    axes[0].set_title(f'Free-space graph: {g.n} nodes, mean degree {g.adjacency.sum() / g.n:.1f}')
    clean(axes[0])
    axes[1].imshow(A, cmap='Greys', interpolation='nearest')
    axes[1].set_title('Adjacency A: almost empty')
    axes[2].imshow(K, cmap='GnBu', interpolation='nearest')
    axes[2].set_title('Geodesic kernel K: dense')
    for ax in axes[1:]:
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_xlabel('nodes (ordered by travel distance)', fontsize=11, color=GREY)
    plt.tight_layout()
    plt.savefig(f'{OUT}/fig_graph.png', dpi=200, facecolor='white')
    plt.close()


def fig_hall2():
    from eval_twobot import read_maps
    res_json = json.load(open('results/hall2.json'))
    T_ref = Rigid(res_json['theta'], np.array(res_json['t']))
    row = next(r for r in res_json['rows'] if r['t'] == 120.0)
    wrong = Rigid(row['geodesic_clearance']['theta'], np.array(row['geodesic_clearance']['t']))
    inky = read_maps('data/bags/hall2/inky', '/inky/map')
    clyde = read_maps('data/bags/hall2/clyde', '/clyde/map')
    start = max(inky[0][0], clyde[0][0])
    t = start + 120.0
    a = max((m for m in inky if m[0] <= t), key=lambda m: m[0])
    b = max((m for m in clyde if m[0] <= t), key=lambda m: m[0])
    ga, oa, res = a[1], a[2], a[3]
    gb, ob = b[1], b[2]
    wb = wall_xy(gb, ob, res)
    pts = np.vstack([wall_xy(ga, oa, res), wrong.apply(wb), T_ref.apply(wb)])
    lo, hi = pts.min(0) - 0.8, pts.max(0) + 0.8
    err = row['geodesic_clearance']['error']
    with plt.rc_context({'font.size': 18, 'axes.titlesize': 20}):
        fig, axes = plt.subplots(1, 2, figsize=(10, 5.4))
        for ax, T, col, ttl in ((axes[0], wrong, CORAL, f'Graph matching: ~90\u00b0 off ({err:.0f} m)'),
                                (axes[1], T_ref, TEAL, 'Correct alignment')):
            ax.imshow(map_image(ga), origin='lower', extent=extent(ga, oa, res))
            ax.scatter(*T.apply(wb).T, s=1.6, c=col)
            ax.set_title(ttl, color=col if col == CORAL else NAVY)
            clean(ax)
            ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        plt.tight_layout(w_pad=2.0)
        plt.savefig(f'{OUT}/fig_hall2.png', dpi=220, facecolor='white', bbox_inches='tight', pad_inches=0.1)
        plt.close()
    print('hall2 overlap at 120 s:', row['overlap'])


if __name__ == '__main__':
    fig_problem()
    fig_graph()
    fig_hall2()
    print('done')
