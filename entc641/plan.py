#!/usr/bin/env python3
"""Make and track the run plan for the ENTC 641 experiment.

    python3 entc641/plan.py create [--seed 641] [--centre]
    python3 entc641/plan.py status
    python3 entc641/plan.py add-confirm --best A B C D [--pairs 3]

create writes data/plan.csv: replicate 1 is session 1 and replicate 2 is
session 2 (each session is a block), each the 8 conditions of the 2^(4-1)
design in its own random order. --centre adds one centre-point run per start
configuration to each session, at random positions.

add-confirm appends confirmation pairs (best setting against the current
default, run back to back, order random within each pair) once analyze.py has
picked a best setting. Coded levels are -1/0/+1; B must be -1 or +1.

status shows which runs are done (data/runs/<id>/meta.json with status
"complete") and which run is next.
"""

import argparse
import csv
import json
import random
import sys

import design

FIELDS = ['run_id', 'session', 'order', 'type', 'condition', 'pair', 'arm',
          'A', 'B', 'C', 'D', 'coordination_weight', 'start',
          'potential_scale', 'gain_scale', 'max_speed']


def load_plan():
    if not design.PLAN_CSV.exists():
        sys.exit(f'no plan yet: run  python3 entc641/plan.py create')
    with open(design.PLAN_CSV, newline='') as f:
        return list(csv.DictReader(f))


def write_plan(rows):
    design.DATA.mkdir(parents=True, exist_ok=True)
    with open(design.PLAN_CSV, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)


def make_row(run_id, session, order, kind, coded, condition='', pair='', arm=''):
    a, b, c, d = coded
    row = {'run_id': run_id, 'session': session, 'order': order, 'type': kind,
           'condition': condition, 'pair': pair, 'arm': arm,
           'A': a, 'B': b, 'C': c, 'D': d}
    row.update(design.actual_settings(a, b, c, d))
    return row


def run_status(run_id):
    meta = design.RUNS / run_id / 'meta.json'
    if not meta.exists():
        return 'pending'
    try:
        return json.loads(meta.read_text()).get('status', 'unknown')
    except json.JSONDecodeError:
        return 'unknown'


def cmd_create(args):
    if design.PLAN_CSV.exists() and not args.force:
        sys.exit(f'{design.PLAN_CSV} exists; pass --force to replace it '
                 '(only before any run has been done)')
    rng = random.Random(args.seed)
    rows = []
    n = 0
    for session in (1, 2):
        items = [('factorial', cond, (a, b, c, d)) for cond, a, b, c, d in design.design_matrix()]
        rng.shuffle(items)
        if args.centre:
            for b in (-1, 1):
                items.insert(rng.randrange(len(items) + 1), ('centre', '', (0, b, 0, 0)))
        for order, (kind, cond, coded) in enumerate(items, start=1):
            n += 1
            rows.append(make_row(f'R{n:02d}', session, order, kind, coded, condition=cond))
    write_plan(rows)
    print(f'wrote {design.PLAN_CSV} ({len(rows)} runs, seed {args.seed})')
    cmd_status(args)


def cmd_add_confirm(args):
    rows = load_plan()
    best = tuple(args.best)
    if best[1] not in (-1, 1):
        sys.exit('B (start configuration) must be -1 or +1')
    rng = random.Random(args.seed)
    n = len(rows)
    first_pair = 1 + max([int(r['pair']) for r in rows if r['pair']] or [0])
    for p in range(first_pair, first_pair + args.pairs):
        arms = [('best', best), ('default', design.DEFAULT_CODED)]
        rng.shuffle(arms)
        for order, (arm, coded) in enumerate(arms, start=1):
            n += 1
            rows.append(make_row(f'R{n:02d}', 'confirm', order, 'confirm', coded,
                                 pair=p, arm=arm))
    write_plan(rows)
    print(f'added {args.pairs} confirmation pairs (best {best} vs default {design.DEFAULT_CODED})')
    cmd_status(args)


def cmd_status(_args):
    rows = load_plan()
    nxt = None
    print(f'{"run":5} {"sess":7} {"type":9} {"A":>2} {"B":>2} {"C":>2} {"D":>2}  '
          f'{"coord":>5} {"start":15} {"pot:gain":9} {"speed":>5}  status')
    for r in rows:
        st = run_status(r['run_id'])
        if st != 'complete' and nxt is None:
            nxt = r['run_id']
        extra = f" pair {r['pair']} {r['arm']}" if r['type'] == 'confirm' else ''
        print(f"{r['run_id']:5} {r['session']:7} {r['type']:9} {r['A']:>2} {r['B']:>2} "
              f"{r['C']:>2} {r['D']:>2}  {r['coordination_weight']:>5} {r['start']:15} "
              f"{r['potential_scale'] + ':' + r['gain_scale']:9} {r['max_speed']:>5}  {st}{extra}")
    done = sum(run_status(r['run_id']) == 'complete' for r in rows)
    print(f'\n{done}/{len(rows)} complete. Next: {nxt or "none"}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('create')
    c.add_argument('--seed', type=int, default=641)
    c.add_argument('--centre', action='store_true', help='add centre-point runs')
    c.add_argument('--force', action='store_true')
    c.set_defaults(func=cmd_create)
    s = sub.add_parser('status')
    s.set_defaults(func=cmd_status)
    a = sub.add_parser('add-confirm')
    a.add_argument('--best', type=float, nargs=4, required=True, metavar=('A', 'B', 'C', 'D'))
    a.add_argument('--pairs', type=int, default=3)
    a.add_argument('--seed', type=int, default=1205)
    a.set_defaults(func=cmd_add_confirm)
    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
