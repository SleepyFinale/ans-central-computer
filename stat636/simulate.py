#!/usr/bin/env python3
"""Simulation: when can graph matching align two partial views of one latent spatial graph?

The latent graph is a random geometric network shaped like the real ones: nodes spread
over a 2D area, a minimum spanning tree plus a few short extra edges, and a mean degree near
2.2. Each view keeps a window of the area, flips a share of its edges, jitters node positions
and observes a noisy node feature (clearance). Noise defaults come from results/graph_noise.json.
"""
import itertools
import json
from multiprocessing import Pool

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components, minimum_spanning_tree
from scipy.spatial import cKDTree

from map_graph import MapGraph
from matching import centred_adjacency, clearance_similarity, faq, geodesic_kernel, ransac_rigid, seeds_only
from registration import Rigid

OVERLAPS = (0.2, 0.4, 0.6, 0.8, 1.0)
EDGE_NOISE = (0.0, 0.2, 0.4)     # share of a view's edges replaced by other short edges
NODE_DROPOUT = (0.0, 0.38)       # share of a view's nodes without a counterpart; 0.38 matches real graphs
CLEARANCE_SD = 0.08              # gives a clearance correlation near the measured 0.97
REPS = 40
SUCCESS_M = 0.30


def latent_graph(rng, n=90, width=12.0, height=8.0, extra=0.12):
    xy = rng.uniform((0, 0), (width, height), (n, 2))
    # Thin to a minimum spacing of about 0.5 m, like nodes along a skeleton.
    keep = []
    tree_points = []
    for i in range(n):
        if all(np.linalg.norm(xy[i] - p) > 0.5 for p in tree_points):
            keep.append(i)
            tree_points.append(xy[i])
    xy = xy[keep]
    dist = np.linalg.norm(xy[:, None] - xy[None, :], axis=-1)
    tree = minimum_spanning_tree(csr_matrix(dist)).toarray() > 0
    adjacency = tree | tree.T
    near = np.argwhere(np.triu(dist < 1.5, 1) & ~adjacency)
    for i, j in near[rng.permutation(len(near))[:int(extra * len(xy))]]:
        adjacency[i, j] = adjacency[j, i] = True
    clearance = rng.uniform(0.15, 1.2, len(xy))
    return xy, adjacency, clearance


def make_view(xy, adjacency, clearance, mask, edge_noise, dropout, rng):
    """Returns the observed graph and, for each of its nodes, its latent node id."""
    # Dropped nodes are bypassed so the graph stays a skeleton, as with shifted sampling.
    adj = adjacency.copy()
    for i in np.flatnonzero(mask & (rng.random(len(xy)) < dropout)):
        neighbours = np.flatnonzero(adj[i])
        adj[np.ix_(neighbours, neighbours)] = True
        adj[i, :] = adj[:, i] = False
        mask = mask.copy()
        mask[i] = False
    np.fill_diagonal(adj, False)
    ids = np.flatnonzero(mask)
    adj = adj[np.ix_(ids, ids)]
    pts = xy[ids] + rng.normal(0, 0.05, (len(ids), 2))

    edges = np.argwhere(np.triu(adj, 1))
    dist = np.linalg.norm(pts[:, None] - pts[None, :], axis=-1)
    if len(edges) and edge_noise > 0:
        flip = edges[rng.random(len(edges)) < edge_noise]
        for i, j in flip:
            adj[i, j] = adj[j, i] = False
        candidates = np.argwhere(np.triu(dist < 1.5, 1) & ~adj)
        for i, j in candidates[rng.permutation(len(candidates))[:len(flip)]]:
            adj[i, j] = adj[j, i] = True
    _, labels = connected_components(csr_matrix(adj), directed=False)
    big = np.flatnonzero(labels == np.bincount(labels).argmax())
    lengths = (adj * dist)[np.ix_(big, big)]
    noisy = clearance[ids[big]] + rng.normal(0, CLEARANCE_SD, len(big))
    return MapGraph(xy=pts[big], clearance=noisy, lengths=lengths), ids[big]


def one_trial(job):
    overlap, edge_noise, dropout, seed = job
    rng = np.random.default_rng(seed)
    xy, adjacency, clearance = latent_graph(rng)
    width = xy[:, 0].max()
    # Two windows along x whose shared share of either window equals `overlap`.
    win = width / (2 - overlap)
    view_a, ids_a = make_view(xy, adjacency, clearance, xy[:, 0] <= win, edge_noise, dropout, rng)
    view_b, ids_b = make_view(xy, adjacency, clearance, xy[:, 0] >= width - win, edge_noise, dropout, rng)
    T_true = Rigid(rng.uniform(0, 2 * np.pi), rng.uniform(-5, 5, 2))
    view_b.xy = T_true.apply(view_b.xy)  # B lives in its own frame
    T_ref = Rigid(-T_true.theta, -T_true.R.T @ T_true.t)  # B -> A

    index_b = {node: k for k, node in enumerate(ids_b)}
    truth = np.array([(a, index_b[node]) for a, node in enumerate(ids_a) if node in index_b], int).reshape(-1, 2)
    row = dict(overlap=overlap, edge_noise=edge_noise, dropout=dropout, n_a=view_a.n, n_b=view_b.n, n_true=len(truth))
    if len(truth) < 5:
        return row

    truth_set = {tuple(p) for p in truth}

    def score(candidates):
        results = []
        for matches in candidates:
            T, inliers = ransac_rigid(view_a.xy, view_b.xy, matches, rng=rng)
            error = None if T is None else float(np.linalg.norm(T.apply(view_b.xy) - T_ref.apply(view_b.xy), axis=1).mean())
            results.append((inliers, error, sum((a, b) in truth_set for a, b in matches) / len(truth)))
        best = max(results, key=lambda r: r[0])
        return dict(success=best[1] is not None and best[1] < SUCCESS_M, accuracy=best[2])

    Ka, Kb = geodesic_kernel(view_a), geodesic_kernel(view_b)
    Aa, Ab = centred_adjacency(view_a), centred_adjacency(view_b)
    C = clearance_similarity(view_a, view_b)
    weight = np.abs(Ka).sum() / C.size * C.shape[0]
    row['adjacency'] = score([m for m, _ in faq(Aa, Ab, restarts=8, rng=rng)])
    row['geodesic'] = score([m for m, _ in faq(Ka, Kb, restarts=8, rng=rng)])
    row['geodesic_clearance'] = score([m for m, _ in faq(Ka, Kb, C=C, weight=weight, restarts=8, rng=rng)])
    jitter = [linear_sum_assignment(-(C + (0 if k == 0 else rng.normal(0, 0.05, C.shape)))) for k in range(8)]
    row['clearance_only'] = score([np.column_stack(j) for j in jitter])
    chosen = truth[rng.choice(len(truth), 2, replace=False)]
    seeds = [tuple(int(x) for x in p) for p in chosen]
    T_seed = seeds_only(view_a.xy, view_b.xy, seeds)
    row['seeds_only_s2'] = dict(success=bool(np.linalg.norm(T_seed.apply(view_b.xy) - T_ref.apply(view_b.xy), axis=1).mean() < SUCCESS_M))
    row['geodesic_s2'] = score([m for m, _ in faq(Ka, Kb, seeds=seeds, restarts=3, rng=rng)])
    return row


if __name__ == '__main__':
    jobs = [(o, e, d, hash((o, e, d, r)) % (2 ** 31)) for o, e, d, r in
            itertools.product(OVERLAPS, EDGE_NOISE, NODE_DROPOUT, range(REPS))]
    print(len(jobs), 'trials')
    with Pool(16) as pool:
        rows = pool.map(one_trial, jobs, chunksize=8)
    json.dump(rows, open('results/simulation.json', 'w'))
    methods = ['adjacency', 'geodesic', 'clearance_only', 'geodesic_clearance', 'seeds_only_s2', 'geodesic_s2']
    print(f"{'overlap':>8}{'edge':>6}{'drop':>6}{'n':>5}{'true':>6}" + ''.join(f'{m[:14]:>16}' for m in methods))
    for o, e, d in itertools.product(OVERLAPS, EDGE_NOISE, NODE_DROPOUT):
        sub = [r for r in rows if (r['overlap'], r['edge_noise'], r['dropout']) == (o, e, d) and 'adjacency' in r]
        if not sub:
            continue
        cells = ''.join(f"{np.mean([r[m]['success'] for r in sub]):>16.0%}" for m in methods)
        print(f"{o:>8}{e:>6}{d:>6}{np.median([r['n_a'] for r in sub]):>5.0f}{np.median([r['n_true'] for r in sub]):>6.0f}" + cells)
