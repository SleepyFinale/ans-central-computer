#!/usr/bin/env python3
"""Do FAQ's restarts agree with each other, and does disagreement flag wrong alignments?

For every scored time point, rerun FAQ geodesic + clearance exactly as eval_twobot.py does
(same seed and random-number order), fit a transform to each of the 8 restarts, and count how
many land on the winning alignment (within 0.5 m and 8 deg at the centre of Clyde's grid, the
merger's tolerance). Then compare that count for accepted right and accepted wrong answers.

Usage: python3 restart_agreement.py twobot1 twobot4 small1 linktest1 hall1 hall2 hall3 hall4 hall5
(system python with ROS sourced)
"""

import json
import os
import sys
from multiprocessing import Pool

import numpy as np

from confirm_rule import disagree
from eval_twobot import MIN_SHARED_M2, RESTARTS, SPACING, STEP_S, SUCCESS_M, read_maps
from map_graph import build_graph
from matching import clearance_similarity, faq, geodesic_kernel, map_agreement, ransac_rigid
from registration import Rigid
from run_real import evaluate


def case(job):
    t, ga, oa, gb, ob, res, T_ref, gate, seed = job
    rng = np.random.default_rng(seed)
    A, B = build_graph(ga, res, oa, spacing=SPACING), build_graph(gb, res, ob, spacing=SPACING)
    if min(A.n, B.n) < 5:
        return None
    Ka, Kb = geodesic_kernel(A), geodesic_kernel(B)
    C = clearance_similarity(A, B)
    weight = np.abs(Ka).sum() / C.size * C.shape[0]
    candidates = [m for m, _ in faq(Ka, Kb, C=C, weight=weight, restarts=RESTARTS, rng=rng)]
    for _ in range(1, RESTARTS):          # the clearance-only method's draws, to keep the rng in step
        rng.normal(0, .05, C.shape)
    fits = []
    for matches in candidates:
        T, _ = ransac_rigid(A.xy, B.xy, matches, rng=rng)
        if T is not None:
            score, conflict, known = map_agreement(ga, oa, gb, ob, res, T)
            fits.append((score, T, conflict, known))
    if not fits:
        return None
    best = max(fits, key=lambda f: f[0])
    centre = ob + 0.5 * res * np.array([gb.shape[1], gb.shape[0]])
    agree = sum(not disagree(best[1], f[1], centre) for f in fits)
    # Distinct alignments among the restarts (greedy clustering at the same tolerance).
    reps = []
    for f in fits:
        if all(disagree(r, f[1], centre) for r in reps):
            reps.append(f[1])
    return dict(t=t, error=float(evaluate(best[1], T_ref, B.xy)), agree=agree, fitted=len(fits),
                distinct=len(reps), accepted=bool(best[2] <= gate and best[3] * res ** 2 >= MIN_SHARED_M2))


def jobs_for(run):
    d = json.load(open(f'results/{run}.json'))
    T_ref, gate = Rigid(d['theta'], np.array(d['t'])), d['gates']['geodesic_clearance']
    central = f'data/bags/{run}/central'
    def maps(robot):
        own = f'data/bags/{run}/{robot}'
        return read_maps(own if os.path.isdir(own) else central, f'/{robot}/map')
    inky, clyde = maps('inky'), maps('clyde')
    start, end = max(inky[0][0], clyde[0][0]), min(inky[-1][0], clyde[-1][0])
    jobs = []
    for k, t in enumerate(np.arange(start + STEP_S, end + 1e-3, STEP_S)):
        a = max((m for m in inky if m[0] <= t), key=lambda m: m[0])
        b = max((m for m in clyde if m[0] <= t), key=lambda m: m[0])
        jobs.append((float(t - start), a[1], a[2], b[1], b[2], a[3], T_ref, gate, 9000 + k))
    return jobs, {r['t']: r.get('geodesic_clearance', {}).get('error') for r in d['rows']}


def main():
    out = {}
    for run in sys.argv[1:]:
        jobs, recorded = jobs_for(run)
        with Pool(16) as pool:
            rows = [r for r in pool.map(case, jobs) if r is not None]
        match = sum(recorded.get(r['t']) is not None and abs(recorded[r['t']] - r['error']) < 1e-6 for r in rows)
        print(f'{run}: {len(rows)} points, {match} reproduce eval_twobot exactly', flush=True)
        out[run] = rows
    json.dump(out, open('results/restart_agreement.json', 'w'), indent=0)

    groups = {'accepted, right': [], 'accepted, wrong': [], 'rejected': []}
    for run, rows in out.items():
        for r in rows:
            key = 'rejected' if not r['accepted'] else 'accepted, right' if r['error'] < SUCCESS_M else 'accepted, wrong'
            groups[key].append((run, r))
    print('\nrestarts agreeing with the winner (of 8):')
    for key, items in groups.items():
        agree = np.array([r['agree'] for _, r in items])
        hist = np.bincount(agree, minlength=9)[1:]
        print(f'  {key:16s} n={len(items):3d} median {np.median(agree):.0f}  counts for 1..8: {hist.tolist()}')
    print('\naccepted, wrong, by run:')
    for run, r in groups['accepted, wrong']:
        print(f"  {run:8s} t={r['t']:5.0f} err {r['error']:5.1f} m  agree {r['agree']}/{r['fitted']}  distinct {r['distinct']}")
    print('\nif accepted answers needed at least k of 8 restarts to agree:')
    right, wrong = groups['accepted, right'], groups['accepted, wrong']
    for k in range(1, 9):
        print(f"  k={k}: keep right {sum(r['agree'] >= k for _, r in right):3d}/{len(right)}, "
              f"keep wrong {sum(r['agree'] >= k for _, r in wrong):2d}/{len(wrong)} "
              f"(gross {sum(r['agree'] >= k and r['error'] >= 2 for _, r in wrong)})")


if __name__ == '__main__':
    main()
