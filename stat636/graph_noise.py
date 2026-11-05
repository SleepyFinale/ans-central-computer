#!/usr/bin/env python3
"""How similar are the free-space graphs of the same place across runs? Calibrates the simulation."""
import itertools
import json

import numpy as np

from map_graph import build_graph
from registration import Rigid
from run_real import load_runs, snapshot, true_pairs

runs = load_runs()
reference = json.load(open('data/reference_transforms.json'))
stats = []
for a, b in itertools.combinations(runs, 2):
    ref = reference[f'{a}/{b}']
    T = Rigid(ref['theta'], np.array(ref['t']))
    ga, oa, res = snapshot(runs[a], 1.0)
    gb, ob, _ = snapshot(runs[b], 1.0)
    A, B = build_graph(ga, res, oa), build_graph(gb, res, ob)
    pairs = true_pairs(A, B, T)
    i, j = pairs[:, 0], pairs[:, 1]
    edges_a, edges_b = A.adjacency[np.ix_(i, i)], B.adjacency[np.ix_(j, j)]
    upper = np.triu_indices(len(pairs), 1)
    shared = np.sum(edges_a[upper] * edges_b[upper])
    union = np.sum((edges_a[upper] + edges_b[upper]) > 0)
    geo_r = np.corrcoef(A.geodesic[np.ix_(i, i)][upper], B.geodesic[np.ix_(j, j)][upper])[0, 1]
    euc_a = np.linalg.norm(A.xy[i][:, None] - A.xy[i][None, :], axis=-1)[upper]
    clear_r = np.corrcoef(A.clearance[i], B.clearance[j])[0, 1]
    stats.append(dict(pair=f'{a}/{b}', n_a=A.n, n_b=B.n, n_true=len(pairs), node_share=len(pairs) / min(A.n, B.n),
                      edge_jaccard=shared / max(1, union), geodesic_r=geo_r, clearance_r=clear_r,
                      geo_vs_euclid=float(np.median(A.geodesic[np.ix_(i, i)][upper] / np.maximum(euc_a, 1e-6)))))
    print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in stats[-1].items()})
summary = {k: float(np.median([s[k] for s in stats])) for k in stats[0] if k != 'pair'}
print('median', {k: round(v, 2) for k, v in summary.items()})
json.dump(summary, open('results/graph_noise.json', 'w'), indent=1)
