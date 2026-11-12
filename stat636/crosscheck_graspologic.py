#!/usr/bin/env python3
"""Check our FAQ implementation against graspologic's graph_match on the same inputs.

Run with the project virtualenv:  .venv/bin/python crosscheck_graspologic.py
"""
import itertools
import json

import numpy as np
from graspologic.match import graph_match

from map_graph import MapGraph, build_graph
from matching import centred_adjacency, faq, geodesic_kernel
from registration import Rigid
from run_real import load_runs, snapshot, true_pairs

RESTARTS = 8
rng = np.random.default_rng(0)
runs = load_runs()
reference = json.load(open('data/reference_transforms.json'))


def objective(A, B, matches):
    i, j = matches[:, 0], matches[:, 1]
    return float(np.sum(A[np.ix_(i, i)] * B[np.ix_(j, j)]))


def ours(A, B):
    return max(faq(A, B, restarts=RESTARTS, rng=rng), key=lambda c: c[1])[0]


def theirs(A, B):
    result = graph_match(A, B, n_init=RESTARTS, rng=int(rng.integers(1 << 31)), padding='naive')
    return np.column_stack([result.indices_A, result.indices_B])


def accuracy(matches, truth):
    truth = {tuple(p) for p in truth}
    return sum((a, b) in truth for a, b in matches) / max(1, len(truth))


# 1. Exact shuffled copy: both should recover the permutation with the geodesic kernel.
g, origin, res = snapshot(runs['large6'], 1.0)
A = build_graph(g, res, origin)
perm = rng.permutation(A.n)
B = MapGraph(xy=A.xy[perm], clearance=A.clearance[perm], lengths=A.lengths[np.ix_(perm, perm)])
truth = np.array([(int(p), k) for k, p in enumerate(perm)])
for name, kernel in (('adjacency', centred_adjacency), ('geodesic', geodesic_kernel)):
    Ka, Kb = kernel(A), kernel(B)
    print(f'exact copy, {name:9s}: ours {accuracy(ours(Ka, Kb), truth):4.0%}   graspologic {accuracy(theirs(Ka, Kb), truth):4.0%}')

# 2. Real pairs of final maps: compare the objective reached and node accuracy.
print(f"{'pair':16s}{'kernel':>10s}{'obj ours':>10s}{'obj grasp':>11s}{'acc ours':>10s}{'acc grasp':>11s}")
summary = {('adjacency', 'ours'): [], ('adjacency', 'grasp'): [], ('geodesic', 'ours'): [], ('geodesic', 'grasp'): []}
higher = {'adjacency': 0, 'geodesic': 0}
for a, b in itertools.combinations(runs, 2):
    ref = reference[f'{a}/{b}']
    T = Rigid(ref['theta'], np.array(ref['t']))
    ga, oa, res = snapshot(runs[a], 1.0)
    gb, ob, _ = snapshot(runs[b], 1.0)
    A, B = build_graph(ga, res, oa), build_graph(gb, res, ob)
    truth = true_pairs(A, B, T)
    for name, kernel in (('adjacency', centred_adjacency), ('geodesic', geodesic_kernel)):
        Ka, Kb = kernel(A), kernel(B)
        mo, mg = ours(Ka, Kb), theirs(Ka, Kb)
        oo, og = objective(Ka, Kb, mo), objective(Ka, Kb, mg)
        ao, ag = accuracy(mo, truth), accuracy(mg, truth)
        summary[(name, 'ours')].append(ao)
        summary[(name, 'grasp')].append(ag)
        higher[name] += oo >= og
        print(f'{a + "/" + b:16s}{name:>10s}{oo:10.1f}{og:11.1f}{ao:10.0%}{ag:11.0%}')
for name in ('adjacency', 'geodesic'):
    print(f"{name}: mean node accuracy ours {np.mean(summary[(name, 'ours')]):.0%}, graspologic {np.mean(summary[(name, 'grasp')]):.0%}; "
          f"our objective at least as high in {higher[name]} of 10 pairs")
