#!/usr/bin/env python3
"""Does node spacing change the result? Reruns the unseeded methods at several graph spacings."""
import json
from multiprocessing import Pool

import numpy as np
from scipy.optimize import linear_sum_assignment

from map_graph import build_graph
from matching import clearance_similarity, faq, geodesic_kernel, ransac_rigid, map_agreement
from run_real import build_jobs, evaluate

SPACINGS = (0.25, 0.5, 0.75, 1.0)


def run(job):
    kind, a, b, label, ga, oa, gb, ob, res, T_ref, seed = job
    rng = np.random.default_rng(seed)
    out = dict(kind=kind, a=a, b=b)
    for spacing in SPACINGS:
        A, B = build_graph(ga, res, oa, spacing=spacing), build_graph(gb, res, ob, spacing=spacing)
        if min(A.n, B.n) < 5:
            continue
        Ka, Kb = geodesic_kernel(A), geodesic_kernel(B)
        C = clearance_similarity(A, B)
        weight = np.abs(Ka).sum() / C.size * C.shape[0]
        options = {
            'geodesic': [m for m, _ in faq(Ka, Kb, restarts=8, rng=rng)],
            'geodesic_clearance': [m for m, _ in faq(Ka, Kb, C=C, weight=weight, restarts=8, rng=rng)],
            'clearance_only': [np.column_stack(linear_sum_assignment(-(C + (0 if k == 0 else rng.normal(0, .05, C.shape))))) for k in range(8)],
        }
        for name, candidates in options.items():
            best = (-np.inf, None)
            for matches in candidates:
                T, _ = ransac_rigid(A.xy, B.xy, matches, rng=rng)
                if T is not None:
                    score = map_agreement(ga, oa, gb, ob, res, T)[0]
                    if score > best[0]:
                        best = (score, evaluate(T, T_ref, B.xy))
            out[f'{name}@{spacing}'] = best[1] is not None and best[1] < 0.30
        out[f'n@{spacing}'] = A.n
    return out


if __name__ == '__main__':
    jobs = build_jobs()
    with Pool(16) as pool:
        rows = pool.map(run, jobs, chunksize=4)
    json.dump(rows, open('results/spacing_sweep.json', 'w'))
    for spacing in SPACINGS:
        for name in ('clearance_only', 'geodesic', 'geodesic_clearance'):
            key = f'{name}@{spacing}'
            vals = [r.get(key, False) for r in rows]
            print(f"spacing {spacing:4}: {name:20s} {np.mean(vals):4.0%}  median n={np.median([r.get(f'n@{spacing}', 0) for r in rows]):.0f}")
