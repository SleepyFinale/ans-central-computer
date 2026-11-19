#!/usr/bin/env python3
"""Do wrong FAQ answers have a near-equal rival? Best vs best disagreeing candidate, by map agreement.

Usage: python3 rival_ratio.py   (system python with ROS sourced; about 10 min)
"""
import json, sys
import numpy as np

from eval_twobot import read_maps, SPACING, RESTARTS
from map_graph import build_graph
from matching import clearance_similarity, faq, geodesic_kernel, map_agreement, ransac_rigid
from registration import Rigid
from confirm_rule import disagree

def candidates(run, t_rel):
    d = json.load(open(f'results/{run}.json'))
    T_ref = Rigid(d['theta'], np.array(d['t']))
    inky = read_maps(f'data/bags/{run}/inky', '/inky/map'); clyde = read_maps(f'data/bags/{run}/clyde', '/clyde/map')
    start = max(inky[0][0], clyde[0][0]); t = start + t_rel
    a = max((m for m in inky if m[0] <= t), key=lambda m: m[0]); b = max((m for m in clyde if m[0] <= t), key=lambda m: m[0])
    ga, oa, res = a[1], a[2], a[3]; gb, ob = b[1], b[2]
    A, B = build_graph(ga, res, oa, spacing=SPACING), build_graph(gb, res, ob, spacing=SPACING)
    Ka, Kb = geodesic_kernel(A), geodesic_kernel(B); C = clearance_similarity(A, B)
    w = np.abs(Ka).sum() / C.size * C.shape[0]
    rng = np.random.default_rng(1)
    out = []
    centre = ob + 0.5 * res * np.array([gb.shape[1], gb.shape[0]])
    for m, _ in faq(Ka, Kb, C=C, weight=w, restarts=16, rng=rng):
        T, inl = ransac_rigid(A.xy, B.xy, m, rng=rng)
        if T is None: continue
        sc, cf, kn = map_agreement(ga, oa, gb, ob, res, T)
        err = float(np.linalg.norm(T.apply(B.xy) - T_ref.apply(B.xy), axis=1).mean())
        out.append((sc, cf, kn, inl, err, T))
    out.sort(key=lambda x: -x[0])
    best = out[0]
    rival = next((o for o in out[1:] if disagree(best[5], o[5], centre)), None)
    return best, rival

for run, ts in (('hall2', (45, 105, 165, 255, 330, 600, 750)), ('hall3', (210, 300)), ('small1', (135, 300)), ('hall5', (180, 300))):
    for t in ts:
        best, rival = candidates(run, t)
        r = '-' if rival is None else f'rival score {rival[0]:7.0f} conflict {rival[1]:.3f} err {rival[4]:5.1f} m'
        ratio = None if rival is None or best[0] <= 0 else rival[0] / best[0]
        print(f'{run:7s} t={t:4d}  best score {best[0]:7.0f} conflict {best[1]:.3f} err {best[4]:5.1f} m | {r} | ratio {ratio if ratio is None else round(ratio,2)}')
