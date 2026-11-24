#!/usr/bin/env python3
"""Analyze the ENTC 641 experiment with the standardized method.

    python3 entc641/analyze.py                  # all runs
    python3 entc641/analyze.py --exclude R02    # sensitivity: leave runs out

Reads data/plan.csv and data/responses.csv (from extract.py) and writes
results/summary.md and its figures (results/excluding-<runs>/ with --exclude).

Two responses, each with its own model:
  T90_s          time to 90% coverage of the reference area
  run_length_s   time until the explorer declares exploration complete

Steps (course method, lectures 13-15):
  1. input data: every run's factor levels and responses
  2. plots first: main-effect and interaction plots with every replicate
  3. calculation matrix, effects, Pareto chart
  4. variance of each condition, pooled variance, S_effect, t-test of each effect
  5. model of the significant effects, residual plots
  6. centre points against the model's prediction at the centre
  7. secondary measures by coordination level (descriptive only)
"""

import argparse
import csv

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import stats  # noqa: E402

import design  # noqa: E402
import doe  # noqa: E402

RESPONSES = [
    ('T90_s', 'Time to 90% coverage', 's'),
    ('run_length_s', 'Time to exploration complete', 's'),
]
SECONDARY = [
    ('idle_pct', 'Idle with no goal', '%'),
    ('separation_m', 'Median robot separation', 'm'),
    ('other_closer_pct', 'Goals the other robot was closer to', '%'),
    ('distance_m', 'Distance driven, both robots', 'm'),
    ('goal_success_pct', 'Goal success (succeeded / (succeeded + aborted))', '%'),
]
COLUMNS = list(design.FACTORS) + list(design.INTERACTIONS)
LABEL = {c: (f'{c}={design.INTERACTIONS[c]}' if c in design.INTERACTIONS else c) for c in COLUMNS}
LEVEL_NAMES = {
    'A': {-1: 'off (0)', 1: 'on (1.0)'},
    'B': {-1: 'same room', 1: 'different rooms'},
    'C': {-1: '2 : 1', 1: '1 : 2'},
    'D': {-1: '0.15 m/s', 1: '0.22 m/s'},
}

md = []


def say(line=''):
    md.append(line)


def table(header, rows):
    say('| ' + ' | '.join(header) + ' |')
    say('|' + '|'.join(' --- ' for _ in header) + '|')
    for r in rows:
        say('| ' + ' | '.join(str(x) for x in r) + ' |')
    say()


def num(row, col):
    v = row.get(col, '')
    return None if v in ('', None) else float(v)


def coded(row):
    return tuple(int(float(row[f])) for f in design.FACTORS)


def savefig(out, fig, name):
    fig.tight_layout()
    fig.savefig(out / name, dpi=130)
    plt.close(fig)
    return name


def input_data(fact, centre):
    say('## 1. Input data')
    say()
    rows = []
    for r in sorted(fact + centre, key=lambda r: (r['type'] != 'factorial',
                                                  int(r['condition'] or 99), r['run_id'])):
        rows.append([r['run_id'], r['session'], r['condition'] or 'centre',
                     *(f'{float(r[f]):+g}' for f in design.FACTORS),
                     *(f'{num(r, c):.1f}' for c, _, _ in RESPONSES)])
    table(['Run', 'Session', 'Condition', 'A', 'B', 'C', 'D'] + [l for _, l, _ in RESPONSES], rows)


def effect_plots(out, fact, centre, col, label, unit):
    fig, axes = plt.subplots(1, 4, figsize=(13, 3.4), sharey=True)
    rng = np.random.default_rng(0)
    for ax, fac in zip(axes, design.FACTORS):
        i = design.FACTORS.index(fac)
        means = []
        for lvl in (-1, 1):
            ys = [num(r, col) for r in fact if coded(r)[i] == lvl]
            ax.scatter(lvl + rng.uniform(-0.06, 0.06, len(ys)), ys, s=16, color='0.55', zorder=2)
            means.append(np.mean(ys))
        ax.plot((-1, 1), means, '-o', color='tab:blue', lw=2, zorder=3)
        cy = [num(r, col) for r in centre]
        if cy and fac != 'B':
            ax.scatter([0] * len(cy), cy, marker='D', s=22, color='tab:orange', zorder=3,
                       label='centre runs')
        ax.set_xticks((-1, 1))
        ax.set_xticklabels([LEVEL_NAMES[fac][-1], LEVEL_NAMES[fac][1]], fontsize=8)
        ax.set_title(f'{fac}: {design.FACTOR_NAMES[fac]}', fontsize=9)
    axes[0].set_ylabel(f'{label} ({unit})')
    if centre:
        axes[0].legend(fontsize=7, loc='upper right')
    fig.suptitle(f'Main effects: {label} (grey = each run, blue = mean)', fontsize=10)
    main_name = savefig(out, fig, f'main_effects_{col}.png')

    fig, axes = plt.subplots(1, 3, figsize=(11, 3.4), sharey=True)
    for ax, pair in zip(axes, design.INTERACTIONS):
        f1, f2 = pair
        i1, i2 = design.FACTORS.index(f1), design.FACTORS.index(f2)
        for lvl2, colour in ((-1, 'tab:orange'), (1, 'tab:green')):
            means = [np.mean([num(r, col) for r in fact
                              if coded(r)[i1] == lvl1 and coded(r)[i2] == lvl2]) for lvl1 in (-1, 1)]
            ax.plot((-1, 1), means, '-o', color=colour, label=f'{f2} = {LEVEL_NAMES[f2][lvl2]}')
        ax.set_xticks((-1, 1))
        ax.set_xticklabels([LEVEL_NAMES[f1][-1], LEVEL_NAMES[f1][1]], fontsize=8)
        ax.set_xlabel(f'{f1}: {design.FACTOR_NAMES[f1]}', fontsize=9)
        ax.set_title(f'{pair} (confounded with {design.INTERACTIONS[pair]})', fontsize=9)
        ax.legend(fontsize=7)
    axes[0].set_ylabel(f'{label} ({unit})')
    fig.suptitle(f'Interactions: {label} (parallel lines = no interaction)', fontsize=10)
    inter_name = savefig(out, fig, f'interactions_{col}.png')
    return main_name, inter_name


def analyze_response(out, fact, centre, col, label, unit):
    say(f'## {label} ({unit})')
    say()
    main_name, inter_name = effect_plots(out, fact, centre, col, label, unit)
    say('### Plots')
    say()
    say(f'![Main effects]({main_name})')
    say()
    say(f'![Interactions]({inter_name})')
    say()

    conds = doe.group_conditions(fact, design.FACTORS, design.columns, col)
    if len(conds) < 8:
        say('Not all 8 conditions have data.')
        say()
        return
    res = doe.analyze(conds, COLUMNS)

    say('### Calculation matrix and effects')
    say()
    rows = []
    for n, c in enumerate(conds, start=1):
        rows.append([n] + [f'{c.signs[k]:+d}' for k in COLUMNS]
                    + [', '.join(f'{y:.1f}' for y in c.ys), f'{np.mean(c.ys):.1f}',
                       f'{np.var(c.ys, ddof=1):.1f}' if len(c.ys) > 1 else ''])
    table(['Test'] + [LABEL[k] for k in COLUMNS] + ['y (replicates)', 'ȳ', 'S²'], rows)
    table(['Effect', 'E (' + unit + ')', 't', 'Significant'],
          [[LABEL[k], f'{res.effects[k]:+.1f}', f'{res.t[k]:+.2f}' if res.has_variance else '',
            'yes' if k in res.significant else ''] for k in COLUMNS])
    if res.has_variance:
        say(f'Average {res.average:.1f} {unit}. Pooled variance Sp² = {res.pooled_var:.1f} '
            f'({res.dof} dof), S²_effect = 4·Sp²/N = {res.s_effect ** 2:.1f} (N = {res.n_y}), '
            f'S_effect = {res.s_effect:.1f}. Two-tailed t critical at α = 0.05: {res.t_crit:.3f}, '
            f'so an effect must exceed {res.t_crit * res.s_effect:.1f} {unit} in size.')
        say()

    order = sorted(COLUMNS, key=lambda k: abs(res.effects[k]), reverse=True)
    fig, ax = plt.subplots(figsize=(6.5, 3.4))
    ax.bar([LABEL[k] for k in order], [abs(res.effects[k]) for k in order],
           color=['tab:red' if k in res.significant else 'tab:gray' for k in order])
    if res.has_variance:
        ax.axhline(res.t_crit * res.s_effect, color='k', ls='--', lw=1,
                   label=f'significance limit {res.t_crit * res.s_effect:.1f} {unit}')
        ax.legend(fontsize=8)
    ax.set_ylabel(f'|E| ({unit})')
    ax.set_title(f'Pareto chart of effects: {label}', fontsize=10)
    say(f'![Pareto chart]({savefig(out, fig, f"pareto_{col}.png")})')
    say()
    if not res.has_variance:
        return

    say('### Model and residuals')
    say()
    coef = res.coefficients()
    say(f'ŷ = {res.average:.1f} ' + ' '.join(f'{b:+.1f}·X{k}' for k, b in coef.items())
        + f'   ({unit}, coded X = ±1)' + ('' if coef else ': no significant effect, so the '
                                          'model is the average'))
    say()
    pts = [r for r in fact if num(r, col) is not None]
    y = np.array([num(r, col) for r in pts])
    fit = np.array([res.predict(design.columns(*coded(r))) for r in pts])
    resid = y - fit
    s = resid.std(ddof=1)
    z = resid / s
    order_no = np.array([int(r['run_id'][1:]) for r in pts])
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.4))
    for ax, x, name in ((axes[0], fit, f'fitted ŷ ({unit})'), (axes[1], order_no, 'run number')):
        ax.scatter(x, z, s=18)
        for xi, zi, r in zip(x, z, pts):
            if abs(zi) > 2:
                ax.annotate(r['run_id'], (xi, zi), fontsize=8, xytext=(4, 0),
                            textcoords='offset points')
        ax.axhline(0, color='k', lw=0.8)
        for lim in (-2, 2):
            ax.axhline(lim, color='0.6', lw=0.8, ls=':')
        ax.set_xlabel(name, fontsize=9)
        ax.set_ylabel('normalized residual', fontsize=9)
    stats.probplot(z, dist='norm', plot=axes[2])
    axes[2].set_title('Normal probability plot', fontsize=9)
    fig.suptitle(f'Residuals: {label}', fontsize=10)
    say(f'![Residuals]({savefig(out, fig, f"residuals_{col}.png")})')
    say()
    big = [f'{r["run_id"]} ({zi:+.1f})' for r, zi in zip(pts, z) if abs(zi) > 2]
    say(f'Residual standard deviation {s:.1f} {unit}. '
        + (f'Normalized residuals beyond ±2: {", ".join(big)}.' if big
           else 'No normalized residual beyond ±2.'))
    say()

    cpts = [r for r in centre if num(r, col) is not None]
    if cpts:
        say('### Centre points')
        say()
        say('Centre runs (A, C, D = 0) against the model at the same setting. A gap well beyond '
            f'the residual SD ({s:.1f} {unit}) would mean the response curves between the levels '
            'and a straight-line model is not enough.')
        say()
        rows = []
        for r in cpts:
            pred = res.predict(design.columns(*coded(r)))
            rows.append([r['run_id'], LEVEL_NAMES['B'][coded(r)[1]], f'{num(r, col):.1f}',
                         f'{pred:.1f}', f'{num(r, col) - pred:+.1f}'])
        table(['Run', 'B', f'Observed ({unit})', 'Model', 'Difference'], rows)


def secondary(fact):
    say('## Secondary measures by coordination level (descriptive)')
    say()
    rows = []
    for col, label, unit in SECONDARY:
        off = [num(r, col) for r in fact if coded(r)[0] == -1 and num(r, col) is not None]
        on = [num(r, col) for r in fact if coded(r)[0] == 1 and num(r, col) is not None]
        rows.append([f'{label} ({unit})', f'{np.mean(off):.1f}', f'{np.mean(on):.1f}',
                     f'{min(off + on):.1f}–{max(off + on):.1f}'])
    table(['Measure', 'A off: mean', 'A on: mean', 'Range, all runs'], rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--exclude', nargs='+', default=[], metavar='RUN')
    args = ap.parse_args()

    out = design.RESULTS
    if args.exclude:
        out = out / ('excluding-' + '-'.join(sorted(args.exclude)))
    out.mkdir(parents=True, exist_ok=True)
    with open(design.RESPONSES_CSV, newline='') as fh:
        resp = [r for r in csv.DictReader(fh) if r['run_id'] not in args.exclude]
    fact = [r for r in resp if r['type'] == 'factorial']
    centre = [r for r in resp if r['type'] == 'centre']

    say('# ENTC 641 experiment: analysis')
    say()
    say(f'{len(fact)} factorial and {len(centre)} centre-point runs'
        + (f' (excluded: {", ".join(args.exclude)})' if args.exclude else '')
        + '. Design 2^(4-1) with D = ABC, two replicates; AB, AC and AD are confounded with '
        'CD, BD and BC.')
    say()
    input_data(fact, centre)
    for col, label, unit in RESPONSES:
        analyze_response(out, fact, centre, col, label, unit)
    secondary(fact)
    (out / 'summary.md').write_text('\n'.join(md) + '\n')
    print(f'wrote {out / "summary.md"} and figures in {out}')


if __name__ == '__main__':
    main()
