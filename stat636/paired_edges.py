#!/usr/bin/env python3
"""Do graph edges add anything beyond the clearance feature? Paired, bootstrapping run pairs."""
import json
import numpy as np

rows = json.load(open('results/spacing_sweep.json'))
pairs = sorted({(r['a'], r['b']) for r in rows})
rng = np.random.default_rng(2)
for spacing in (0.25, 0.5):
    for a_name, b_name in (('geodesic_clearance', 'clearance_only'),):
        by_pair = {p: [] for p in pairs}
        for r in rows:
            by_pair[(r['a'], r['b'])].append(int(r.get(f'{a_name}@{spacing}', False)) - int(r.get(f'{b_name}@{spacing}', False)))
        d = [v for vs in by_pair.values() for v in vs]
        boots = [np.mean([v for i in rng.integers(len(pairs), size=len(pairs)) for v in by_pair[pairs[i]]]) for _ in range(4000)]
        print(f"spacing {spacing}: {a_name} minus {b_name}: {np.mean(d):+.1%} "
              f"(95% CI {np.quantile(boots,.025):+.1%} to {np.quantile(boots,.975):+.1%}); "
              f"edges only helped {sum(x>0 for x in d)}, only hurt {sum(x<0 for x in d)}, of {len(d)}")
