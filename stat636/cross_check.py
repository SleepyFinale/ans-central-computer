#!/usr/bin/env python3
"""Combine FAQ graph matching and AKAZE: let one method confirm the other.

The graph method answers earlier but its corridor errors repeat (hall2), so two of
its own answers agreeing proves little. AKAZE's errors are of another kind. Here an
alignment is committed when, at one time point, an accepted FAQ estimate and an
AKAZE estimate agree within the merger's tolerance (0.5 m and 8 deg at the centre
of Clyde's grid), or by each method's own two-in-a-row rule where the rule allows.
Switching to a different alignment needs the same evidence on the new one.

Which AKAZE estimates may confirm is the variable under test: with the merger's
gates, without the inlier-ratio gate, or with only inlier count and scale.

Usage: python3 cross_check.py twobot1 twobot4 small1 linktest1 hall1 hall2 hall3 hall4 hall5
"""

import json
import sys

import numpy as np

from confirm_rule import SUCCESS_M, GROSS_M, accepted_estimate, disagree, replay
from registration import Rigid

AKAZE_GATES = {
    # inliers, ratio, confidence, max scale error: the merger's gates as of c44811b
    'merger gates': dict(inliers=8, ratio=0.25, confidence=0.65, scale=0.05),
    'no ratio gate': dict(inliers=8, ratio=0.0, confidence=0.65, scale=0.05),
    'inliers + scale only': dict(inliers=8, ratio=0.0, confidence=0.0, scale=0.05),
}


def akaze(row, gate):
    d = row.get('akaze_raw')
    if (d is None or d['theta'] is None or d['inliers'] < gate['inliers'] or d['ratio'] < gate['ratio']
            or d['confidence'] < gate['confidence'] or abs(d['scale'] - 1.0) > gate['scale']):
        return None
    return Rigid(d['theta'], np.array(d['t'])), d['error']


def cross_replay(rows, gate):
    """Held alignment over time; a commit needs FAQ and AKAZE to agree, or AKAZE twice in a row."""
    held = held_error = None
    last_akaze = None
    timeline = []
    for row in rows:
        centre = row.get('centre_b')
        f = accepted_estimate(row, 'geodesic_clearance')
        a = akaze(row, gate)
        candidate = None
        if centre is not None:
            if f is not None and a is not None and not disagree(f[0], a[0], centre):
                candidate = a            # report the AKAZE estimate; they agree anyway
            elif a is not None and last_akaze is not None and not disagree(last_akaze[0], a[0], centre):
                candidate = a            # AKAZE's own two-in-a-row rule
        if candidate is not None:
            held, held_error = candidate
        elif held is not None and centre is not None:
            # Keep the alignment; refresh it from any single answer that agrees with it.
            for est in (a, f):
                if est is not None and not disagree(held, est[0], centre):
                    held, held_error = est
                    break
        if a is not None:
            last_akaze = a
        timeline.append((row['t'], held_error))
    return timeline


def score(timeline):
    right = [t for t, e in timeline if e is not None and e < SUCCESS_M]
    wrong = [e for t, e in timeline if e is not None and e >= SUCCESS_M]
    return dict(right=len(right), wrong=len(wrong), gross=sum(e >= GROSS_M for e in wrong),
                first=right[0] if right else None)


def main():
    runs = sys.argv[1:]
    results = {}
    for run in runs:
        rows = sorted(json.load(open(f'results/{run}.json'))['rows'], key=lambda r: r['t'])
        res = {'AKAZE alone': score(replay(rows, 'akaze')[0]),
               'FAQ alone': score(replay(rows, 'geodesic_clearance')[0])}
        for name, gate in AKAZE_GATES.items():
            res[f'cross-check, AKAZE {name}'] = score(cross_replay(rows, gate))
        results[run] = res
    json.dump(results, open('results/cross_check.json', 'w'), indent=1)
    names = list(results[runs[0]])
    print(f"{'run':10s}" + ''.join(f'{n[:28]:>30s}' for n in names))
    for run in runs:
        print(f'{run:10s}' + ''.join(
            f"{results[run][n]['right']:>8d}/{results[run][n]['wrong']:<3d} g{results[run][n]['gross']:<3d} {str(results[run][n]['first']):>7s}  "
            for n in names))
    print('\nTotals and timing against AKAZE alone')
    for n in names:
        right = sum(results[r][n]['right'] for r in runs)
        wrong = sum(results[r][n]['wrong'] for r in runs)
        gross = sum(results[r][n]['gross'] for r in runs)
        runs_wrong = sum(results[r][n]['wrong'] > 0 for r in runs)
        diffs = [results[r][n]['first'] - results[r]['AKAZE alone']['first'] for r in runs
                 if results[r][n]['first'] is not None and results[r]['AKAZE alone']['first'] is not None]
        earlier = sum(d < 0 for d in diffs)
        print(f'{n:40s} right {right:4d} wrong {wrong:3d} (gross {gross:3d}, runs with any {runs_wrong}); '
              f'first right vs AKAZE: earlier in {earlier}/{len(diffs)}, median {np.median(diffs):+.0f} s')


if __name__ == '__main__':
    main()
