#!/usr/bin/env python3
"""Tabulate real-pair results by method and overlap, with confidence intervals clustered by run pair."""
import json
import numpy as np

SUCCESS_M = 0.30
rows = [r for r in json.load(open('results/real_pairs.json')) if 'akaze' in r]
rng = np.random.default_rng(1)
pair_names = sorted({(r['a'], r['b']) for r in rows})
good = lambda e: e is not None and e < SUCCESS_M


def picker(method, select):
    def pick(r):
        if method not in r:
            return None
        return r[method][select]['error'] if select else r[method]['error']
    return pick


def rate_ci(subset, pick, draws=2000):
    """Success rate over all cases (a method that could not run counts as a miss), bootstrapping run pairs."""
    per_pair = {p: [] for p in pair_names}
    for r in subset:
        per_pair[(r['a'], r['b'])].append(good(pick(r)))
    values = [v for vs in per_pair.values() for v in vs]
    if not values:
        return None
    boots = []
    keys = list(per_pair)
    for _ in range(draws):
        sample = [v for i in rng.integers(len(keys), size=len(keys)) for v in per_pair[keys[i]]]
        if sample:
            boots.append(np.mean(sample))
    return np.mean(values), np.quantile(boots, .025), np.quantile(boots, .975)


methods = [
    ('AKAZE (map_merge gates)', lambda r: r['akaze']['error']),
    ('control: random matching', picker('random', 'by_agreement')),
    ('control: clearance, no edges', picker('clearance_only', 'by_agreement')),
    ('FAQ adjacency', picker('adjacency', 'by_agreement')),
    ('FAQ geodesic', picker('geodesic', 'by_agreement')),
    ('FAQ geodesic+clearance', picker('geodesic_clearance', 'by_agreement')),
    ('  same, selected by inliers', picker('geodesic_clearance', 'by_inliers')),
    ('control: 2 oracle seeds, no match', picker('seeds_only_s2', None)),
    ('adjacency, 2 oracle', picker('adjacency_s2', 'by_agreement')),
    ('geodesic, 2 oracle', picker('geodesic_s2', 'by_agreement')),
    ('geodesic+clear, 2 oracle', picker('geodesic_clearance_s2', 'by_agreement')),
    ('control: 5 oracle seeds, no match', picker('seeds_only_s5', None)),
    ('adjacency, 5 oracle', picker('adjacency_s5', 'by_agreement')),
    ('geodesic, 5 oracle', picker('geodesic_s5', 'by_agreement')),
    ('spectral, 5 oracle', picker('spectral_s5', 'by_agreement')),
    ('control: AKAZE seeds, no match', picker('seeds_only_akaze', None)),
    ('geodesic+clear, AKAZE seeds', picker('geodesic_clearance_akaze', 'by_agreement')),
]
bins = [(0, .5), (.5, .9), (.9, 1.01)]
print(f'{len(rows)} cases, {len(pair_names)} run pairs. Success = mean node error < {SUCCESS_M} m. 95% CI bootstraps run pairs.')
print(f"{'':36s}" + ''.join(f'{f"overlap {lo:.1f}-{min(hi,1):.1f}":>22s}' for lo, hi in bins) + f"{'all':>22s}")
groups = [[r for r in rows if lo <= r['overlap'] < hi] for lo, hi in bins] + [rows]
print(f"{'cases':36s}" + ''.join(f'{len(g):>22d}' for g in groups))
for name, pick in methods:
    cells = []
    for g in groups:
        c = rate_ci(g, pick)
        cells.append('-' if c is None else f'{c[0]:4.0%} ({c[1]:3.0%}-{c[2]:3.0%})')
    print(f'{name:36s}' + ''.join(f'{c:>22s}' for c in cells))

n_seed = [r['akaze_seed_count'] for r in rows]
print(f"\nAKAZE seeds: median count {np.median(n_seed):.0f}; cases with >=2 seeds {np.mean([n >= 2 for n in n_seed]):.0%}; "
      f"share of seeds near correct {sum(r['akaze_seed_good'] for r in rows) / max(1, sum(n_seed)):.0%}")
ak = [r['akaze'] for r in rows]
print(f"AKAZE outcomes: correct {np.mean([good(a['error']) for a in ak]):.0%}, wrong {np.mean([a['error'] is not None and not good(a['error']) for a in ak]):.0%}, declined {np.mean([a['error'] is None for a in ak]):.0%}")

# Accept/reject gate on the conflict rate, threshold chosen on the other run pairs.
for m in ('geodesic_clearance', 'geodesic_clearance_akaze'):
    subset = [r for r in rows if m in r]
    accepted_right = accepted_wrong = 0
    for held in pair_names:
        train = [r for r in subset if (r['a'], r['b']) != held]
        test = [r for r in subset if (r['a'], r['b']) == held]
        best_gate, best_kept = 0.0, -1
        for gate in np.arange(0.02, 0.30, 0.005):
            kept = [r for r in train if r[m]['by_agreement']['conflict'] <= gate]
            if kept and np.mean([not good(r[m]['by_agreement']['error']) for r in kept]) <= 0.05 and len(kept) > best_kept:
                best_gate, best_kept = gate, len(kept)
        for r in test:
            if r[m]['by_agreement']['conflict'] <= best_gate:
                if good(r[m]['by_agreement']['error']):
                    accepted_right += 1
                else:
                    accepted_wrong += 1
    print(f'{m} with held-out gate (train wrong-accept <= 5%), {len(subset)} cases: '
          f'correct and accepted {accepted_right / len(subset):.0%}, wrong and accepted {accepted_wrong / len(subset):.0%}, '
          f'rejected {1 - (accepted_right + accepted_wrong) / len(subset):.0%}')
