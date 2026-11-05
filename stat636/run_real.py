#!/usr/bin/env python3
"""Evaluate the matchers on real map pairs from independent runs of the same area.

Two sources of partial overlap:
  snapshot: early map messages from each run, which are truly partial views.
  crop: final maps with each run limited to one side of a line, for lower overlap.
"""
import glob
import itertools
import json
import os
from multiprocessing import Pool

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

from map_graph import build_graph
from matching import akaze_baseline, akaze_seeds, centred_adjacency, clearance_similarity, faq, \
    geodesic_kernel, map_agreement, ransac_rigid, seeds_only, spectral_match
from registration import Rigid

SNAPSHOT_FRACTIONS = (0.1, 0.3, 0.5, 1.0)
CROP_OVERLAPS = (0.1, 0.2, 0.3, 0.5, 0.7)
SUCCESS_M = 0.30
MATCH_M = 0.25


def load_runs():
    return {os.path.basename(p).split('_')[0]: dict(np.load(p)) for p in sorted(glob.glob('data/maps/*.npz'))}


def snapshot(run, fraction):
    k = int(np.flatnonzero(np.isclose(run['fractions'], fraction))[0])
    return run[f'grid_{k}'], run[f'origin_{k}'], float(run[f'resolution_{k}'])


def cell_xy(grid, origin, res):
    rows, cols = np.indices(grid.shape)
    return origin + (np.stack([cols, rows], -1) + 0.5) * res


def crop_pair(ga, oa, gb, ob, res, T, overlap, axis_index):
    """Limit A and B to opposite ends of a principal axis of the shared frame."""
    xa, xb = cell_xy(ga, oa, res), T.apply(cell_xy(gb, ob, res))
    free = np.vstack([xa[ga == 0], xb[gb == 0]])
    centre = free.mean(0)
    _, vectors = np.linalg.eigh(np.cov((free - centre).T))
    axis = vectors[:, -1 - axis_index]
    keep = 0.5 + overlap / 2
    ua, ub = (xa - centre) @ axis, (xb - centre) @ axis
    limit_a = np.quantile((free - centre) @ axis, keep)
    limit_b = np.quantile((free - centre) @ axis, 1 - keep)
    return np.where(ua <= limit_a, ga, -1), np.where(ub >= limit_b, gb, -1)


def free_overlap(ga, oa, gb, ob, res, T):
    rc = np.argwhere(gb == 0)
    world = T.apply(ob + (rc[:, ::-1] + 0.5) * res)
    cells = np.floor((world - oa) / res).astype(int)
    inside = (cells[:, 0] >= 0) & (cells[:, 0] < ga.shape[1]) & (cells[:, 1] >= 0) & (cells[:, 1] < ga.shape[0])
    both = np.sum(ga[cells[inside, 1], cells[inside, 0]] == 0)
    free_a, free_b = np.sum(ga == 0), len(rc)
    return both / min(free_a, free_b), both / (free_a + free_b - both)


def true_pairs(graph_a, graph_b, T):
    """Mutual nearest node pairs within MATCH_M under the reference transform."""
    if graph_a.n == 0 or graph_b.n == 0:
        return np.empty((0, 2), int)
    b_in_a = T.apply(graph_b.xy)
    d_ab, a_for_b = cKDTree(graph_a.xy).query(b_in_a)
    _, b_for_a = cKDTree(b_in_a).query(graph_a.xy)
    pairs = [(a_for_b[b], b) for b in range(graph_b.n) if d_ab[b] < MATCH_M and b_for_a[a_for_b[b]] == b]
    return np.array(pairs, int).reshape(-1, 2)


def evaluate(T_est, T_ref, xy_b):
    return float(np.linalg.norm(T_est.apply(xy_b) - T_ref.apply(xy_b), axis=1).mean())


def score_candidates(candidates, case, rng):
    """One hypothesis per candidate match set; select blind by inliers and by map agreement."""
    ga, oa, gb, ob, res, graph_a, graph_b, T_ref, pairs = case
    truth = {tuple(p) for p in pairs}
    results = []
    for matches in candidates:
        T, inliers = ransac_rigid(graph_a.xy, graph_b.xy, matches, rng=rng)
        accuracy = sum((a, b) in truth for a, b in matches) / max(1, len(truth))
        if T is None:
            results.append(dict(inliers=0, accuracy=accuracy, error=None, agree=-np.inf, conflict=1.0, known=0))
            continue
        score, conflict, known = map_agreement(ga, oa, gb, ob, res, T)
        results.append(dict(inliers=inliers, accuracy=accuracy, error=evaluate(T, T_ref, graph_b.xy),
                            agree=score, conflict=conflict, known=known))
    return {'by_inliers': max(results, key=lambda r: r['inliers']),
            'by_agreement': max(results, key=lambda r: r['agree'])}


def run_case(job):
    kind, a, b, label, ga, oa, gb, ob, res, T_ref, seed = job
    rng = np.random.default_rng(seed)
    graph_a = build_graph(ga, res, oa)
    graph_b = build_graph(gb, res, ob)
    overlap, iou = free_overlap(ga, oa, gb, ob, res, T_ref)
    row = dict(kind=kind, a=a, b=b, label=label, n_a=graph_a.n, n_b=graph_b.n,
               overlap=float(overlap), iou=float(iou))
    if min(graph_a.n, graph_b.n) < 5:
        return row
    pairs = true_pairs(graph_a, graph_b, T_ref)
    row['n_true'] = len(pairs)
    case = (ga, oa, gb, ob, res, graph_a, graph_b, T_ref, pairs)

    T, info = akaze_baseline(ga, oa, gb, ob, res)
    row['akaze'] = dict(info, error=None if T is None else evaluate(T, T_ref, graph_b.xy))

    Ka, Kb = geodesic_kernel(graph_a), geodesic_kernel(graph_b)
    Aa, Ab = centred_adjacency(graph_a), centred_adjacency(graph_b)
    C = clearance_similarity(graph_a, graph_b)
    def run(key, A, B, seeds=(), Csim=None, weight=0.0, restarts=8):
        found = faq(A, B, seeds=seeds, C=Csim, weight=weight, restarts=restarts, rng=rng)
        row[key] = score_candidates([m for m, _ in found], case, rng)

    clearance_weight = np.abs(Ka).sum() / C.size * C.shape[0]
    run('adjacency', Aa, Ab)
    run('geodesic', Ka, Kb)
    run('geodesic_clearance', Ka, Kb, Csim=C, weight=clearance_weight)

    # Control: node features only, with the same number of hypotheses but no edges.
    jitter = [linear_sum_assignment(-(C + (0 if k == 0 else rng.normal(0, 0.05, C.shape)))) for k in range(8)]
    row['clearance_only'] = score_candidates([np.column_stack(j) for j in jitter], case, rng)

    # Control: random matchings through the same RANSAC and selection.
    n = min(graph_a.n, graph_b.n)
    randoms = [np.column_stack([rng.permutation(graph_a.n)[:n], rng.permutation(graph_b.n)[:n]]) for _ in range(8)]
    row['random'] = score_candidates(randoms, case, rng)

    for s in (2, 5):
        if len(pairs) >= s:
            chosen = rng.choice(len(pairs), s, replace=False)
            seeds = [tuple(int(x) for x in pairs[i]) for i in chosen]
            # Control: what the oracle seeds give with no matching.
            row[f'seeds_only_s{s}'] = {'error': evaluate(seeds_only(graph_a.xy, graph_b.xy, seeds), T_ref, graph_b.xy)}
            run(f'geodesic_s{s}', Ka, Kb, seeds=seeds, restarts=3)
            run(f'geodesic_clearance_s{s}', Ka, Kb, seeds=seeds, Csim=C, weight=clearance_weight, restarts=3)
            run(f'adjacency_s{s}', Aa, Ab, seeds=seeds, restarts=3)
            if s == 5:
                row['spectral_s5'] = score_candidates([spectral_match(Ka, Kb, seeds)], case, rng)

    # Realistic seeds: from AKAZE, may be wrong, and available even when AKAZE declines to answer.
    real_seeds = akaze_seeds(ga, oa, graph_a.xy, gb, ob, graph_b.xy, res)
    b_in_a = T_ref.apply(graph_b.xy)
    row['akaze_seed_count'] = len(real_seeds)
    row['akaze_seed_good'] = int(sum(np.linalg.norm(graph_a.xy[a] - b_in_a[b]) < 2 * MATCH_M for a, b in real_seeds))
    if len(real_seeds) >= 2:
        row['seeds_only_akaze'] = {'error': evaluate(seeds_only(graph_a.xy, graph_b.xy, real_seeds), T_ref, graph_b.xy)}
        run('geodesic_clearance_akaze', Ka, Kb, seeds=real_seeds, Csim=C, weight=clearance_weight, restarts=3)
    return row


def build_jobs():
    runs = load_runs()
    reference = json.load(open('data/reference_transforms.json'))
    jobs = []
    seed = 0
    for a, b in itertools.combinations(runs, 2):
        ref = reference[f'{a}/{b}']
        T = Rigid(ref['theta'], np.array(ref['t']))
        for fa, fb in itertools.product(SNAPSHOT_FRACTIONS, repeat=2):
            ga, oa, res = snapshot(runs[a], fa)
            gb, ob, _ = snapshot(runs[b], fb)
            seed += 1
            jobs.append(('snapshot', a, b, f'{fa}/{fb}', ga, oa, gb, ob, res, T, seed))
        ga, oa, res = snapshot(runs[a], 1.0)
        gb, ob, _ = snapshot(runs[b], 1.0)
        for overlap in CROP_OVERLAPS:
            for axis in (0, 1):
                ca, cb = crop_pair(ga, oa, gb, ob, res, T, overlap, axis)
                seed += 1
                jobs.append(('crop', a, b, f'{overlap}/{axis}', ca, oa, cb, ob, res, T, seed))
    return jobs


def clean(value):
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (np.floating, np.integer)):
        value = value.item()
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


if __name__ == '__main__':
    jobs = build_jobs()
    print(f'{len(jobs)} cases')
    with Pool() as pool:
        rows = pool.map(run_case, jobs, chunksize=4)
    json.dump([clean(r) for r in rows], open('results/real_pairs.json', 'w'), indent=0)
    print('done')
