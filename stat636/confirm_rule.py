#!/usr/bin/env python3
"""Replay map_merge's confirmation rule over each method's answers in time.

map_merge (since commit c44811b) does not use an alignment as soon as one
estimate passes its gates:

- the first alignment is used only after two estimates in a row agree with
  each other;
- once one is in use, an estimate that disagrees with it replaces it only
  after three estimates in a row agree with the new one;
- a time point with no accepted estimate counts neither for nor against.

Two estimates agree when, at the centre of Clyde's grid, they differ by at
most 0.5 m and 8 degrees. All of these numbers are map_merge's, fixed before
this replay; none were tuned here.

Each method's accepted estimates come from results/<run>.json
(eval_twobot.py), one every 15 s. map_merge itself estimates about once a
second, so the replay here waits longer for agreement than the live merger
does: compare methods with each other, not with live timings.

Usage: python3 confirm_rule.py twobot1 twobot4 small1 linktest1 hall1 hall2
"""

import json
import math
import sys

import numpy as np

from registration import Rigid

MAX_SHIFT_M = 0.5
MAX_TURN_RAD = math.radians(8.0)
CONFIRM_FIRST = 2
CONFIRM_SWITCH = 3
SUCCESS_M = 0.30
GROSS_M = 2.0
METHODS = ('akaze', 'clearance_only', 'geodesic_clearance', 'partial_geodesic', 'partial_euclidean')


def accepted_estimate(row, method):
    """(Rigid, error) for this time point's accepted estimate, or None."""
    if method == 'akaze':
        d = row.get('akaze_raw')
        # map_merge's gates as of c44811b: 8 inliers, ratio 0.25, confidence 0.65, scale within 5%.
        if (d is None or d['theta'] is None or d['inliers'] < 8 or d['ratio'] < 0.25
                or d['confidence'] < 0.65 or abs(d['scale'] - 1.0) > 0.05):
            return None
    else:
        d = row.get(method)
        if d is None or not d['accepted']:
            return None
    return Rigid(d['theta'], np.array(d['t'])), d['error']


def disagree(T1, T2, centre):
    shift = np.linalg.norm(T1.apply(np.array([centre]))[0] - T2.apply(np.array([centre]))[0])
    turn = abs(math.atan2(math.sin(T1.theta - T2.theta), math.cos(T1.theta - T2.theta)))
    return shift > MAX_SHIFT_M or turn > MAX_TURN_RAD


def replay(rows, method):
    """Committed alignment at each time point: list of (t, error or None)."""
    held = held_error = None
    pending = None
    count = 0
    out = []
    switches = 0
    for row in rows:
        est = accepted_estimate(row, method)
        centre = row.get('centre_b')
        if est is not None and centre is not None:
            T, error = est
            if held is None:
                same = pending is not None and not disagree(pending[0], T, centre)
                count = count + 1 if same else 1
                pending = (T, error)
                if count >= CONFIRM_FIRST:
                    held, held_error = T, error
                    pending, count = None, 0
            elif disagree(held, T, centre):
                same = pending is not None and not disagree(pending[0], T, centre)
                count = count + 1 if same else 1
                pending = (T, error)
                if count >= CONFIRM_SWITCH:
                    held, held_error = T, error
                    pending, count = None, 0
                    switches += 1
            else:
                # Agrees with the alignment in use: refresh it, as rememberAlignment does.
                held, held_error = T, error
                pending, count = None, 0
        out.append((row['t'], held_error))
    return out, switches


def summarise(run):
    data = json.load(open(f'results/{run}.json'))
    rows = sorted(data['rows'], key=lambda r: r['t'])
    result = {}
    for method in METHODS:
        timeline, switches = replay(rows, method)
        right = [t for t, e in timeline if e is not None and e < SUCCESS_M]
        wrong = [(t, e) for t, e in timeline if e is not None and e >= SUCCESS_M]
        gross = [t for t, e in wrong if e >= GROSS_M]
        raw = [accepted_estimate(r, method) for r in rows]
        raw_right = sum(1 for x in raw if x is not None and x[1] < SUCCESS_M)
        raw_wrong = sum(1 for x in raw if x is not None and x[1] >= SUCCESS_M)
        result[method] = dict(points=len(rows), right=len(right), wrong=len(wrong), gross=len(gross),
                              first_right=right[0] if right else None,
                              first_wrong=wrong[0][0] if wrong else None, switches=switches,
                              raw_right=raw_right, raw_wrong=raw_wrong)
    live = [r for r in rows if r.get('live_error') is not None]
    first_live = next((r['t'] for r in live if r['live_error'] < SUCCESS_M), None)
    result['live'] = dict(first_right=first_live)
    return result


def main():
    runs = sys.argv[1:]
    table = {run: summarise(run) for run in runs}
    json.dump(table, open('results/confirm_rule.json', 'w'), indent=1)
    names = {'akaze': 'AKAZE (current gates)', 'clearance_only': 'Clearance only',
             'geodesic_clearance': 'FAQ geodesic+clearance', 'partial_geodesic': 'Partial, geodesic',
             'partial_euclidean': 'Partial, Euclidean'}
    for run in runs:
        print(f"\n{run}: {table[run]['akaze']['points']} time points; live map_merge first right at "
              f"{table[run]['live']['first_right']} s")
        print(f"{'method':24s}{'raw r/w':>10s}{'held right':>12s}{'held wrong':>12s}{'gross':>7s}"
              f"{'first right':>13s}{'first wrong':>13s}{'switches':>10s}")
        for m in METHODS:
            d = table[run][m]
            print(f"{names[m]:24s}{d['raw_right']:>5d}/{d['raw_wrong']:<4d}{d['right']:>12d}{d['wrong']:>12d}"
                  f"{d['gross']:>7d}{str(d['first_right']):>13s}{str(d['first_wrong']):>13s}{d['switches']:>10d}")
    print('\nTotals over runs (time points with a held alignment)')
    for m in METHODS:
        right = sum(table[r][m]['right'] for r in runs)
        wrong = sum(table[r][m]['wrong'] for r in runs)
        gross = sum(table[r][m]['gross'] for r in runs)
        runs_with_wrong = sum(table[r][m]['wrong'] > 0 for r in runs)
        print(f"{names[m]:24s} right {right:4d}  wrong {wrong:4d} (gross {gross:3d}); runs with any wrong: {runs_with_wrong} of {len(runs)}")


if __name__ == '__main__':
    main()
