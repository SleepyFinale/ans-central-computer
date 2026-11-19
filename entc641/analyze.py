#!/usr/bin/env python3
"""Analyze the ENTC 641 experiment: effects, significance, models, residuals,
coverage curves, optimization and confirmation.

    python3 entc641/analyze.py

Reads data/plan.csv and data/responses.csv (from extract.py) and writes
results/: summary.md (all tables), effects_<response>.csv, and figures. Works
with partial data: after one replicate it gives effects and Pareto charts only
(no variance yet); t-tests, models and optimization need both replicates.

Steps, each a method from the course:
  1. descriptive statistics, box plots by factor level, Pearson correlations
  2. first look: two-sample t-test of T90 with coordination off vs on
  3. calculation matrix, effects and Pareto chart per response
  4. pooled variance, S_effect, t-test of every effect (alpha 0.05)
  5. model of the significant effects; residuals vs fitted, run order, start
     voltage and each factor; normal probability plot
  6. residuals regressed on start voltage; if significant, a multivariable
     regression with voltage as an extra variable
  7. session (block) check: paired t-test of replicate 1 vs 2 by condition
  8. coverage curves and the fitted time constant tau
  9. curvature check with centre points (if any were run)
 10. optimization: minimize predicted T90 subject to goal success and battery
     limits, for each start configuration; sensitivity to the limits
 11. confirmation pairs (if run): paired t-test and prediction intervals
"""

import csv
import itertools
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import optimize, stats  # noqa: E402

import design  # noqa: E402
import doe  # noqa: E402

RESPONSES = [
    # column, label, unit, goal
    ('T90_s', 'Time to 90% coverage', 's', 'min'),
    ('coverage_cap_pct', 'Coverage at 8 min', '%', 'max'),
    ('tau_s', 'Coverage time constant tau', 's', 'min'),
    ('idle_pct', 'Idle with no goal', '%', 'min'),
    ('goal_success_pct', 'Goal success rate', '%', 'max'),
    ('distance_m', 'Distance driven (both robots)', 'm', None),
    ('battery_drop_v', 'Battery voltage drop', 'V', 'min'),
]
EXTRA = [('start_voltage_v', 'Start voltage', 'V'), ('separation_m', 'Robot separation', 'm'),
         ('other_closer_pct', 'Goals other robot closer', '%'), ('cpu_pct', 'Pi CPU', '%'),
         ('merge_time_s', 'Merge time', 's')]
COLUMNS = list(design.FACTORS) + list(design.INTERACTIONS)
LABEL = {c: (f'{c}={design.INTERACTIONS[c]}' if c in design.INTERACTIONS else c) for c in COLUMNS}

SUCCESS_LIMIT = 90.0     # goal success >= this (%)
BATTERY_LIMIT = 0.15     # battery drop <= this (V per run)

OUT = design.RESULTS
md = []


def say(line=''):
    md.append(line)


def table(header, rows):
    say('| ' + ' | '.join(header) + ' |')
    say('|' + '|'.join(' --- ' for _ in header) + '|')
    for r in rows:
        say('| ' + ' | '.join(str(x) for x in r) + ' |')
    say()


def f(x, nd=2):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return ''
    return f'{x:.{nd}f}'


def nd(unit):
    return 3 if unit == 'V' else 2


def num(row, col):
    v = row.get(col, '')
    return None if v in ('', None) else float(v)


def load():
    with open(design.PLAN_CSV, newline='') as fh:
        plan = {r['run_id']: r for r in csv.DictReader(fh)}
    with open(design.RESPONSES_CSV, newline='') as fh:
        resp = list(csv.DictReader(fh))
    for r in resp:
        p = plan.get(r['run_id'], {})
        r['order_global'] = list(plan).index(r['run_id']) + 1 if r['run_id'] in plan else 0
        r['session'] = r.get('session') or p.get('session', '')
    return plan, resp


def savefig(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / name, dpi=120)
    plt.close(fig)
    return name


# ---------------------------------------------------------------- 1. descriptive
def descriptive(fact):
    say('## 1. Descriptive statistics (factorial runs)')
    say()
    rows = []
    for col, label, unit, _ in RESPONSES + [(c, l, u, None) for c, l, u in EXTRA]:
        v = np.array([num(r, col) for r in fact if num(r, col) is not None])
        if len(v) == 0:
            continue
        k = nd(unit)
        rows.append([label, unit, len(v), f(v.mean(), k),
                     f(v.std(ddof=1) if len(v) > 1 else None, k), f(v.min(), k), f(v.max(), k)])
    table(['Response', 'Unit', 'n', 'Mean', 'SD', 'Min', 'Max'], rows)
    cens = sum(int(r.get('T90_censored') or 0) for r in fact)
    if cens:
        say(f'{cens} run(s) never reached 90% coverage; their T90 is recorded as the 480 s cap '
            '(censored), which understates their true time.')
        say()

    fig, axes = plt.subplots(len(RESPONSES), 4, figsize=(12, 2.3 * len(RESPONSES)))
    for i, (col, label, unit, _) in enumerate(RESPONSES):
        for j, fac in enumerate(design.FACTORS):
            ax = axes[i, j]
            data = [[num(r, col) for r in fact if num(r, col) is not None
                     and int(float(r[fac])) == lvl] for lvl in (-1, 1)]
            if all(data):
                ax.boxplot(data, labels=['-1', '+1'])
            if i == 0:
                ax.set_title(f'{fac}: {design.FACTOR_NAMES[fac]}', fontsize=9)
            if j == 0:
                ax.set_ylabel(f'{label} ({unit})', fontsize=8)
            ax.tick_params(labelsize=8)
    say(f'![Box plots by factor level]({savefig(fig, "boxplots.png")})')
    say()

    cols = [c for c, *_ in RESPONSES] + [c for c, *_ in EXTRA]
    names = [l for _, l, *_ in RESPONSES] + [l for _, l, _ in EXTRA]
    data = np.array([[num(r, c) if num(r, c) is not None else np.nan for c in cols] for r in fact])
    keep = [k for k in range(len(cols)) if np.sum(~np.isnan(data[:, k])) >= 3
            and np.nanstd(data[:, k]) > 0]
    if len(keep) >= 2:
        n = len(keep)
        R = np.full((n, n), np.nan)
        for a, b in itertools.product(range(n), repeat=2):
            x, y = data[:, keep[a]], data[:, keep[b]]
            ok = ~np.isnan(x) & ~np.isnan(y)
            if ok.sum() >= 3:
                R[a, b] = stats.pearsonr(x[ok], y[ok])[0]
        fig, ax = plt.subplots(figsize=(9, 7.5))
        im = ax.imshow(R, cmap='RdBu_r', vmin=-1, vmax=1)
        ax.set_xticks(range(n))
        ax.set_yticks(range(n))
        ax.set_xticklabels([names[k] for k in keep], rotation=60, ha='right', fontsize=8)
        ax.set_yticklabels([names[k] for k in keep], fontsize=8)
        for a, b in itertools.product(range(n), repeat=2):
            if not np.isnan(R[a, b]):
                ax.text(b, a, f'{R[a, b]:.2f}', ha='center', va='center', fontsize=7)
        fig.colorbar(im, ax=ax, label='Pearson r')
        say('Pearson correlation between responses and covariates '
            f'(|r| > {critical_r(len(fact)):.2f} is significant at 0.05 for n = {len(fact)}):')
        say()
        say(f'![Correlation matrix]({savefig(fig, "correlation.png")})')
        say()
        with open(OUT / 'correlation.csv', 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow([''] + [names[k] for k in keep])
            for a in range(n):
                w.writerow([names[keep[a]]] + [f(R[a, b], 3) for b in range(n)])


def critical_r(n):
    if n <= 3:
        return float('nan')
    t = stats.t.ppf(0.975, n - 2)
    return t / math.sqrt(n - 2 + t * t)


# ---------------------------------------------------------------- 2. first look
def first_look(fact):
    say('## 2. First look: coordination off vs on (two-sample t-test)')
    say()
    off = [num(r, 'T90_s') for r in fact if int(float(r['A'])) == -1 and num(r, 'T90_s') is not None]
    on = [num(r, 'T90_s') for r in fact if int(float(r['A'])) == 1 and num(r, 'T90_s') is not None]
    if len(off) < 2 or len(on) < 2:
        say('Not enough runs yet.')
        say()
        return
    t, p = stats.ttest_ind(off, on, equal_var=True)
    say(f'T90 with coordination off: mean {np.mean(off):.1f} s (n {len(off)}); on: mean '
        f'{np.mean(on):.1f} s (n {len(on)}). Pooled two-sample t = {t:.2f}, p = {p:.3f}. '
        'This ignores the other factors; the factorial analysis below separates them.')
    say()


# ---------------------------------------------------------------- 3-6. effects and model
def effects_for(fact, col):
    conds = doe.group_conditions(fact, design.FACTORS, design.columns, col)
    if len(conds) < 8:
        return None
    return doe.analyze(conds, COLUMNS)


def factorial_analysis(fact, col, label, unit):
    res = effects_for(fact, col)
    say(f'### {label} ({unit})')
    say()
    if res is None:
        say('Not all 8 conditions have data yet.')
        say()
        return None
    k = nd(unit)
    rows = []
    for c in COLUMNS:
        rows.append([LABEL[c], f(res.effects[c], k), f(res.t.get(c)) if res.has_variance else '',
                     'yes' if c in res.significant else ''])
    say(f'Average {res.average:.{k}f} {unit}.' + (
        f' Pooled variance Sp² = {res.pooled_var:.{k + 2}g} ({res.dof} dof), S_effect = '
        f'{res.s_effect:.{k + 1}g}, t critical = {res.t_crit:.3f}.' if res.has_variance else
        ' No replicates yet, so no significance test.'))
    say()
    table(['Effect', 'E', 't', 'Significant'], rows)
    with open(OUT / f'effects_{col}.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['effect', 'E', 't', 'significant'])
        for r in rows:
            w.writerow(r)

    order = sorted(COLUMNS, key=lambda c: abs(res.effects[c]), reverse=True)
    fig, ax = plt.subplots(figsize=(6.5, 3.6))
    colours = ['tab:red' if c in res.significant else 'tab:gray' for c in order]
    ax.bar([LABEL[c] for c in order], [abs(res.effects[c]) for c in order], color=colours)
    if res.has_variance:
        ax.axhline(res.t_crit * res.s_effect, color='k', ls='--', lw=1,
                   label=f't crit x S_effect = {res.t_crit * res.s_effect:.2f}')
        ax.legend(fontsize=8)
    ax.set_ylabel(f'|effect| ({unit})')
    ax.set_title(f'Pareto chart of effects: {label}', fontsize=10)
    ax.tick_params(axis='x', labelsize=8)
    say(f'![Pareto chart]({savefig(fig, f"pareto_{col}.png")})')
    say()

    if not res.has_variance:
        return res
    coef = res.coefficients()
    say('Model (coded units, significant effects only): '
        + f'ŷ = {res.average:.{k}f} ' + ' '.join(f'{b:+.{k}f}·{c}' for c, b in coef.items()))
    say()
    residual_analysis(fact, col, label, unit, res)
    return res


def residual_analysis(fact, col, label, unit, res):
    pts = [(r, num(r, col)) for r in fact if num(r, col) is not None]
    fitted = np.array([res.predict(design.columns(*(int(float(r[x])) for x in design.FACTORS)))
                       for r, _ in pts])
    y = np.array([v for _, v in pts])
    resid = y - fitted
    sd = resid.std(ddof=1) if len(resid) > 1 else 1.0
    norm = resid / sd if sd > 0 else resid
    order = np.array([r['order_global'] for r, _ in pts])
    volt = np.array([num(r, 'start_voltage_v') if num(r, 'start_voltage_v') is not None
                     else np.nan for r, _ in pts])

    fig, axes = plt.subplots(2, 4, figsize=(13, 6))
    panels = [(fitted, 'fitted ' + unit), (order, 'run order'), (volt, 'start voltage (V)')]
    for ax, (x, name) in zip(axes[0, :3], panels):
        ax.scatter(x, norm, s=18)
        ax.axhline(0, color='k', lw=0.8)
        ax.set_xlabel(name, fontsize=9)
        ax.set_ylabel('normalized residual', fontsize=9)
    stats.probplot(norm, dist='norm', plot=axes[0, 3])
    axes[0, 3].set_title('Normal probability plot', fontsize=9)
    for ax, fac in zip(axes[1], design.FACTORS):
        x = np.array([int(float(r[fac])) for r, _ in pts])
        ax.scatter(x + np.random.default_rng(0).uniform(-0.05, 0.05, len(x)), norm, s=18)
        ax.axhline(0, color='k', lw=0.8)
        ax.set_xticks([-1, 1])
        ax.set_xlabel(f'{fac}: {design.FACTOR_NAMES[fac]}', fontsize=9)
    fig.suptitle(f'Residuals: {label}', fontsize=10)
    say(f'![Residual plots]({savefig(fig, f"residuals_{col}.png")})')
    say()

    w, p_norm = stats.shapiro(resid) if len(resid) >= 3 else (None, None)
    big = np.nonzero(np.abs(norm) > 2)[0]
    say(f'Residual SD {sd:.3f} {unit}. Shapiro-Wilk p = {f(p_norm, 3)} '
        '(below 0.05 would mean the residuals are not normal). '
        + (f'Normalized residuals beyond ±2: {", ".join(pts[i][0]["run_id"] for i in big)}.'
           if len(big) else 'No normalized residual beyond ±2.'))
    say()

    ok = ~np.isnan(volt)
    if ok.sum() >= 4 and np.std(volt[ok]) > 0:
        lr = stats.linregress(volt[ok], resid[ok])
        say(f'Residuals vs start voltage: slope {lr.slope:.2f} {unit}/V, r = {lr.rvalue:.2f}, '
            f'p = {lr.pvalue:.3f}.')
        if lr.pvalue < 0.05:
            cols = res.significant
            X = [[1.0] + [design.columns(*(int(float(r[x])) for x in design.FACTORS))[c]
                          for c in cols] + [num(r, 'start_voltage_v') - np.nanmean(volt)]
                 for (r, _), good in zip(pts, ok) if good]
            beta, r2, adj, tvals, dof = doe.least_squares(X, y[ok])
            names = ['intercept'] + cols + ['start voltage (centred)']
            say(' Voltage explains part of the residual, so it is added as a variable '
                f'(multivariable least squares, R² = {r2:.3f}, adjusted {adj:.3f}, {dof} dof):')
            say()
            table(['Term', 'Coefficient', 't'],
                  [[n, f(b, 3), f(t) if tvals is not None else ''] for n, b, t in
                   zip(names, beta, tvals if tvals is not None else [None] * len(beta))])
        else:
            say(' No sign that battery level biased this response.')
            say()


# ---------------------------------------------------------------- 7. blocks
def block_check(fact, col, label):
    by = {}
    for r in fact:
        v = num(r, col)
        if v is None:
            continue
        by.setdefault(r['condition'], {})[str(r['session'])] = v
    pairs = [(d['1'], d['2']) for d in by.values() if '1' in d and '2' in d]
    if len(pairs) < 3:
        return None
    a, b = zip(*pairs)
    t, p = stats.ttest_rel(a, b)
    return [label, len(pairs), f(np.mean(np.subtract(a, b))), f(t), f(p, 3)]


# ---------------------------------------------------------------- 8. coverage curves
def coverage_curves(fact):
    say('## 8. Coverage curves')
    say()
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
    any_curve = False
    for r in fact:
        path = design.RUNS / r['run_id'] / 'coverage.csv'
        if not path.exists():
            continue
        with open(path, newline='') as fh:
            rows = list(csv.DictReader(fh))
        t = [float(x['t_s']) for x in rows]
        c = [float(x['coverage_pct']) for x in rows]
        a, b = int(float(r['A'])), int(float(r['B']))
        ax = axes[0 if b < 0 else 1]
        ax.plot(t, c, color='tab:blue' if a > 0 else 'tab:orange', alpha=0.8,
                ls='-' if int(float(r['D'])) > 0 else '--', lw=1.2)
        any_curve = True
    if not any_curve:
        plt.close(fig)
        say('No coverage curves yet.')
        say()
        return
    for ax, title in zip(axes, ('B = -1: same room', 'B = +1: different rooms')):
        ax.axhline(90, color='k', lw=0.8, ls=':')
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('time since first goal (s)')
    axes[0].set_ylabel('coverage of reference area (%)')
    from matplotlib.lines import Line2D
    axes[1].legend([Line2D([], [], color='tab:blue'), Line2D([], [], color='tab:orange'),
                    Line2D([], [], color='k'), Line2D([], [], color='k', ls='--')],
                   ['A +1 coordination on', 'A -1 off', 'D +1 0.22 m/s', 'D -1 0.15 m/s'],
                   fontsize=8, loc='lower right')
    say(f'![Coverage curves]({savefig(fig, "coverage_curves.png")})')
    say()
    r2 = [num(r, 'tau_r2') for r in fact if num(r, 'tau_r2') is not None]
    if r2:
        say(f'Linearized fit ln(1 − A) = t_d/τ − t/τ: R² median {np.median(r2):.3f} '
            f'(range {min(r2):.3f}–{max(r2):.3f}). τ is analyzed as a response in section 3.')
        say()


# ---------------------------------------------------------------- 9. curvature
def curvature(fact, centre):
    say('## 9. Curvature check with centre points')
    say()
    if not centre:
        say('No centre-point runs in the plan.')
        say()
        return
    rows = []
    for col, label, unit, _ in RESPONSES:
        c = [num(r, col) for r in centre if num(r, col) is not None]
        fv = [num(r, col) for r in fact if num(r, col) is not None]
        if len(c) >= 2 and len(fv) >= 2:
            t, p = stats.ttest_ind(c, fv, equal_var=True)
            rows.append([label, f(np.mean(c)), f(np.mean(fv)), f(t), f(p, 3),
                         'yes' if p < 0.05 else ''])
    if rows:
        say('Two-sample t-test of centre-point runs against the factorial runs. A significant '
            'difference means the response curves between the levels, so the best setting may '
            'lie between them (fit a quadratic in that factor).')
        say()
        table(['Response', 'Centre mean', 'Factorial mean', 't', 'p', 'Curvature'], rows)
    else:
        say('Not enough centre-point runs yet.')
        say()


# ---------------------------------------------------------------- 10. optimization
def optimize_settings(models):
    say('## 10. Optimization')
    say()
    t90, succ, batt = models.get('T90_s'), models.get('goal_success_pct'), models.get('battery_drop_v')
    if t90 is None or not t90.has_variance:
        say('Needs the T90 model with both replicates.')
        say()
        return None
    say(f'Minimize predicted T90 over coded A, C, D in [-1, +1], for each start configuration B, '
        f'subject to predicted goal success ≥ {SUCCESS_LIMIT:.0f}% and predicted battery drop ≤ '
        f'{BATTERY_LIMIT:.2f} V. Each model keeps only its significant effects; a response with '
        'no significant effect is its average everywhere.')
    say()

    def solve(b, s_lim, b_lim):
        def pred(m, x):
            a, c, d = x
            return m.predict(design.columns(a, b, c, d))

        cons = []
        if succ is not None:
            cons.append({'type': 'ineq', 'fun': lambda x: pred(succ, x) - s_lim})
        if batt is not None:
            cons.append({'type': 'ineq', 'fun': lambda x: b_lim - pred(batt, x)})
        best = None
        for start in itertools.product((-0.9, 0.0, 0.9), repeat=3):
            r = optimize.minimize(lambda x: pred(t90, x), start, method='SLSQP',
                                  bounds=[(-1, 1)] * 3, constraints=cons)
            feasible = all(c['fun'](r.x) >= -1e-6 for c in cons)
            if feasible and (best is None or r.fun < best.fun - 1e-9):
                best = r
        return best

    results = {}
    rows = []
    for b in (-1, 1):
        r = solve(b, SUCCESS_LIMIT, BATTERY_LIMIT)
        if r is None:
            rows.append([b, 'no feasible setting', '', '', '', '', '', ''])
            continue
        a, c, d = (float(np.clip(v, -1, 1)) for v in r.x)
        a, c, d = (0.0 if abs(v) < 1e-6 else v for v in (a, c, d))
        signs = design.columns(a, b, c, d)
        pi = t90.prediction_interval(signs)
        act = design.actual_settings(a, b, c, d)
        results[b] = (r.fun, (a, b, c, d), pi)
        rows.append([b, f'{a:+.2f}', f'{c:+.2f}', f'{d:+.2f}',
                     f'{act["coordination_weight"]}, {act["potential_scale"]}:{act["gain_scale"]}, '
                     f'{act["max_speed"]} m/s',
                     f'{r.fun:.1f} ± {pi:.1f}',
                     f(succ.predict(signs), 1) if succ else '',
                     f(batt.predict(signs), 3) if batt else ''])
    table(['B', 'A', 'C', 'D', 'Settings (coord, pot:gain, speed)', 'Predicted T90 (s, 95% PI)',
           'Pred. success %', 'Pred. drop V'], rows)

    if not results:
        return None
    b_best = min(results, key=lambda k: results[k][0])
    t_best, coded, _ = results[b_best]
    say(f'Best overall: B = {b_best:+d} ({design.actual_settings(*coded)["start"]}), '
        f'predicted T90 {t_best:.1f} s. Two-level models put the optimum at a corner of the '
        'region unless a constraint binds; a centre-point curvature result in section 9 would '
        'call for a quadratic term before trusting an interior optimum.')
    say()

    sens = []
    for s_lim, b_lim in [(85, BATTERY_LIMIT), (95, BATTERY_LIMIT), (SUCCESS_LIMIT, 0.12),
                         (SUCCESS_LIMIT, 0.18)]:
        r = solve(b_best, s_lim, b_lim)
        sens.append([f'{s_lim:.0f}%', f'{b_lim:.2f} V',
                     'infeasible' if r is None else f'{r.fun:.1f}',
                     '' if r is None else ', '.join(f'{v:+.2f}' for v in r.x)])
    say('Sensitivity of the optimum to the limits (same B):')
    say()
    table(['Success ≥', 'Drop ≤', 'Predicted T90 (s)', 'A, C, D'], sens)
    say('Add confirmation runs for this setting with:')
    say()
    say('```bash')
    say('python3 entc641/plan.py add-confirm --best '
        + ' '.join(f'{v:g}' for v in (round(coded[0], 2), coded[1], round(coded[2], 2),
                                     round(coded[3], 2))))
    say('```')
    say()
    return coded


# ---------------------------------------------------------------- 11. confirmation
def confirmation(confirm, models):
    say('## 11. Confirmation runs')
    say()
    t90 = models.get('T90_s')
    pairs = {}
    for r in confirm:
        if num(r, 'T90_s') is not None:
            pairs.setdefault(r['pair'], {})[r['arm']] = r
    full = {p: d for p, d in pairs.items() if 'best' in d and 'default' in d}
    if not full:
        say('No complete confirmation pairs yet.')
        say()
        return
    rows = []
    for p, d in sorted(full.items()):
        best, dflt = d['best'], d['default']
        coded = tuple(float(best[x]) for x in design.FACTORS)
        signs = design.columns(*coded)
        pred = t90.predict(signs) if t90 else None
        pi = t90.prediction_interval(signs) if t90 else None
        y = num(best, 'T90_s')
        inside = '' if pi is None else ('yes' if abs(y - pred) <= pi else 'NO')
        rows.append([p, f(y, 1), f(num(dflt, 'T90_s'), 1), f(pred, 1),
                     '' if pi is None else f'{pred - pi:.1f} to {pred + pi:.1f}', inside])
    table(['Pair', 'Best T90 (s)', 'Default T90 (s)', 'Predicted', '95% PI', 'Inside PI'], rows)
    if len(full) >= 2:
        a = [num(d['best'], 'T90_s') for d in full.values()]
        b = [num(d['default'], 'T90_s') for d in full.values()]
        t, p = stats.ttest_rel(a, b)
        say(f'Paired t-test, best vs default: mean difference {np.mean(np.subtract(a, b)):.1f} s, '
            f't = {t:.2f}, p = {p:.3f} ({len(a)} pairs).')
        say()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    plan, resp = load()
    fact = [r for r in resp if r['type'] == 'factorial']
    centre = [r for r in resp if r['type'] == 'centre']
    confirm = [r for r in resp if r['type'] == 'confirm']
    say('# ENTC 641 experiment: analysis')
    say()
    say(f'{len(fact)} factorial, {len(centre)} centre-point and {len(confirm)} confirmation runs '
        f'extracted, of {len(plan)} in the plan. Design 2^(4-1), D = ABC; '
        'AB, AC and AD are confounded with CD, BD and BC.')
    say()
    descriptive(fact)
    first_look(fact)

    say('## 3-6. Effects, significance, models and residuals')
    say()
    models = {}
    for col, label, unit, _ in RESPONSES:
        models[col] = factorial_analysis(fact, col, label, unit)

    say('## 7. Session (block) check')
    say()
    rows = [b for b in (block_check(fact, c, l) for c, l, *_ in RESPONSES) if b]
    if rows:
        say('Paired t-test of replicate 1 (session 1) against replicate 2 (session 2), paired by '
            'condition. A significant difference means something changed between sessions.')
        say()
        table(['Response', 'Pairs', 'Mean diff (1 - 2)', 't', 'p'], rows)
    else:
        say('Needs both sessions.')
        say()

    coverage_curves(fact)
    curvature(fact, centre)
    optimize_settings(models)
    confirmation(confirm, models)

    (OUT / 'summary.md').write_text('\n'.join(md) + '\n')
    print(f'wrote {OUT / "summary.md"} and figures in {OUT}')


if __name__ == '__main__':
    main()
