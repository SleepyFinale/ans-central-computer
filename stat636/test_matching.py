#!/usr/bin/env python3
"""Known-answer checks: every method must recover a transform we applied ourselves."""
import cv2
import numpy as np

from map_graph import MapGraph, build_graph
from matching import akaze_baseline, centred_adjacency, consistency_match, faq, geodesic_kernel, map_agreement, \
    ransac_rigid, spectral_match
from registration import Rigid, register

d = np.load('data/maps/large6_inky.npz')
i = len(d['fractions']) - 1
grid, res, origin = d[f'grid_{i}'], float(d[f'resolution_{i}']), d[f'origin_{i}']
rng = np.random.default_rng(0)


def error(T_est, T_true, xy):
    return np.linalg.norm(T_est.apply(xy) - T_true.apply(xy), axis=1).mean()


# Rotate the raster by a known angle about its centre, keeping labels discrete.
theta = np.deg2rad(37.0)
c, s = np.cos(theta), np.sin(theta)
R = np.array([[c, -s], [s, c]])
h, w = grid.shape
size = int(np.hypot(h, w)) + 4
centre, out = np.array([w / 2, h / 2]), np.array([size / 2, size / 2])
M = np.column_stack([R, out - R @ centre])
image = (grid + 1).astype(np.uint8)
warped = cv2.warpAffine(image, M, (size, size), flags=cv2.INTER_NEAREST, borderValue=0)
grid_b = warped.astype(np.int16).astype(np.int8) - 1
origin_b = np.array([3.0, -2.0])

# M maps pixels of A into B; invert in world coordinates to get the B -> A transform.
truth = Rigid(-theta, origin + res * (centre - R.T @ out) - R.T @ origin_b)
# Direct check of truth: B cells must land on the matching A label.
landed = map_agreement(grid, origin, grid_b, origin_b, res, truth)
print(f'conflict rate under the true transform: {landed[1]:.1%}')

ga, gb = build_graph(grid, res, origin), build_graph(grid_b, res, origin_b)
print(f'graph sizes {ga.n}, {gb.n}')
xy_b = gb.xy

# 1. Exhaustive registration.
T, _, _ = register(grid, origin, grid_b, origin_b, res, np.deg2rad(np.arange(0, 360, 1.0)))
print(f'registration error {error(T, truth, xy_b):.3f} m')

# 2. AKAZE baseline.
T, info = akaze_baseline(grid, origin, grid_b, origin_b, res)
print('akaze', info, 'error', None if T is None else round(error(T, truth, xy_b), 3))

# 3. Exact graph copy with shuffled nodes: FAQ must recover the permutation.
perm = rng.permutation(ga.n)
shuffled = MapGraph(xy=ga.xy[perm], clearance=ga.clearance[perm],
                    lengths=ga.lengths[np.ix_(perm, perm)])
truth_pairs = {int(p): k for k, p in enumerate(perm)}  # node in A -> node in shuffled
for name, Ka, Kb in [('adjacency', centred_adjacency(ga), centred_adjacency(shuffled)),
                     ('geodesic', geodesic_kernel(ga), geodesic_kernel(shuffled))]:
    best = max(faq(Ka, Kb, restarts=8, rng=rng), key=lambda c: c[1])[0]
    print(f'faq {name} on an exact copy: {np.mean([truth_pairs[a] == b for a, b in best]):.0%} correct')
seeds = [(int(p), k) for k, p in enumerate(perm)][:5]
sp = spectral_match(geodesic_kernel(ga), geodesic_kernel(shuffled), seeds)
print(f'spectral with 5 seeds on an exact copy: {np.mean([truth_pairs[a] == b for a, b in sp]):.0%} correct')

# 4. RANSAC gets the transform back from clean matches with 40% gross outliers.
true_T = Rigid(0.9, np.array([1.0, -2.0]))
pts = rng.uniform(-3, 3, (60, 2))
target = true_T.apply(pts)
matches = np.column_stack([np.arange(60), np.arange(60)])
matches[:24, 1] = rng.permutation(60)[:24]
T, inliers = ransac_rigid(target, pts, matches, rng=rng)
print(f'ransac error {error(T, true_T, pts):.3f} m, inliers {inliers}')

# 5. Partial matching: B is a shuffled copy of A with a third of its nodes removed.
kept = np.sort(rng.permutation(ga.n)[:int(ga.n * 0.67)])
partial = MapGraph(xy=true_T.apply(ga.xy[kept]), clearance=ga.clearance[kept], lengths=ga.lengths[np.ix_(kept, kept)])
found = consistency_match(ga, partial, 'euclidean')
right = np.mean([kept[b] == a for a, b in found])
print(f'partial matching on a 67% sub-copy: {len(found)} matched of {len(kept)} possible, {right:.0%} of them correct')
