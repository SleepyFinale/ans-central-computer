#!/usr/bin/env python3
"""Turn each finished run into one row of responses (data/responses.csv).

    source scripts/env/central_env.bash
    python3 entc641/extract.py            # every complete run that has no row yet
    python3 entc641/extract.py --all      # redo every complete run
    python3 entc641/extract.py R03 R04    # these runs only (replaces their rows)

Coverage is measured against the reference map, not the live merged map (which
counts overlap twice until the maps merge): each robot's final map is aligned to
the reference by exhaustive grid registration (stat636/registration.py), and at
every 5 s the share of the reference's known cells that either robot has seen is
the coverage. Check data/runs/<id>/registration.png for each run.

Responses (window: t0 = first goal, to t0 + 480 s, or earlier when the explorer
declared exploration complete; coverage after that stays at its last value):
  T90_s              time to 90% coverage (480 and censored=1 if never reached)
  coverage_cap_pct   coverage at 480 s
  tau_s, tau_r2      time constant of A(t) = 1 - exp(-(t - td)/tau), fitted on
                     ln(1 - A) = td/tau - t/tau by least squares, A < 0.99
  idle_pct           time standing still (< 5 cm in 3 s) with no active goal, both robots
  goal_success_pct   SUCCEEDED / (SUCCEEDED + ABORTED); cancels are the explorer retargeting
  distance_m         both robots together
  battery_drop_v     median voltage in the first 30 s minus the last 30 s, robot mean
  start_voltage_v    median voltage in the first 30 s, robot mean (covariate)
  separation_m       median distance between the robots
  other_closer_pct   goals where the other robot was closer to the goal (straight line)
  cpu_pct            mean Pi CPU use, robot mean
  merge_time_s       when the live map merge settled (blank if it never did)
"""

import argparse
import csv
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import bagdata  # noqa: E402
import design  # noqa: E402
from run_trial import exploration_status  # noqa: E402
from stat636.registration import Rigid, register_refined  # noqa: E402

STEP = 5.0
TARGET = 0.90
MIN_RIVAL_RATIO = 1.15

FIELDS = ['run_id', 'session', 'type', 'condition', 'pair', 'arm', 'A', 'B', 'C', 'D',
          'T90_s', 'T90_censored', 'coverage_cap_pct', 'tau_s', 'tau_r2', 'idle_pct',
          'goal_success_pct', 'goals_succeeded', 'goals_aborted', 'distance_m',
          'battery_drop_v', 'start_voltage_v', 'separation_m', 'other_closer_pct',
          'cpu_pct', 'merge_time_s', 'run_length_s', 'end_reason', 'flags']


def rigid_inverse_apply(T, xy):
    return (np.asarray(xy) - T.t) @ T.R


def rigid_from_pose(x, y, th):
    return Rigid(th, np.array([x, y]))


def rigid_compose(a, b):
    """a after b."""
    return Rigid(a.theta + b.theta, a.apply(b.t[None, :])[0])


def rigid_inverse(a):
    return Rigid(-a.theta, -(a.t @ a.R))


def align_robots(run, ref, flags):
    """Rigid transform from each robot's map frame into the reference frame."""
    T, ok = {}, {}
    for r in run.robots:
        snap = run.maps[r][-1]
        Tr, score, rival = register_refined(ref.grid, np.array(ref.origin), snap.grid,
                                            np.array(snap.origin), ref.res)
        ratio = score / rival if rival > 0 else float('inf')
        T[r] = Tr
        ok[r] = score > 0 and ratio >= MIN_RIVAL_RATIO
        print(f'    {r}: registered to reference, rotation {math.degrees(Tr.theta):.1f} deg, '
              f'score/rival {ratio:.2f}{"" if ok[r] else "  (weak)"}')
    weak = [r for r in run.robots if not ok[r]]
    good = [r for r in run.robots if ok[r]]
    for r in weak:
        # Place a weakly registered robot through a well registered one and the
        # live map -> <robot>/map TF at the end of the run.
        if good and run.world_tf.get(r) and run.world_tf.get(good[0]):
            g = good[0]
            Wg = rigid_from_pose(*run.world_tf[g][-1][1:])
            Wr = rigid_from_pose(*run.world_tf[r][-1][1:])
            T[r] = rigid_compose(T[g], rigid_compose(rigid_inverse(Wg), Wr))
            flags.append(f'{r}_placed_via_{g}')
        else:
            flags.append(f'{r}_weak_registration')
    return T


def coverage_series(run, ref, T, times):
    rows, cols = np.nonzero(ref.grid >= 0)
    pts = np.column_stack([ref.origin[0] + (cols + 0.5) * ref.res,
                           ref.origin[1] + (rows + 0.5) * ref.res])
    local = {r: rigid_inverse_apply(T[r], pts) for r in run.robots}
    snap_t = {r: [(s.t,) for s in run.maps[r]] for r in run.robots}
    cache = {}

    def seen(r, i):
        if (r, i) not in cache:
            s = run.maps[r][i]
            c = np.floor((local[r][:, 0] - s.origin[0]) / s.res).astype(int)
            rr = np.floor((local[r][:, 1] - s.origin[1]) / s.res).astype(int)
            h, w = s.grid.shape
            inside = (c >= 0) & (c < w) & (rr >= 0) & (rr < h)
            m = np.zeros(len(pts), dtype=bool)
            m[inside] = s.grid[rr[inside], c[inside]] >= 0
            cache[(r, i)] = m
        return cache[(r, i)]

    out = []
    for t in times:
        m = np.zeros(len(pts), dtype=bool)
        for r in run.robots:
            i = bagdata.latest_before(snap_t[r], t)
            if i >= 0:
                m |= seen(r, i)
        out.append(m.mean())
    return np.array(out)


def time_to(cov, rel, target):
    above = np.nonzero(cov >= target)[0]
    if len(above) == 0:
        return None
    i = above[0]
    if i == 0:
        return rel[0]
    # linear interpolation between samples
    f = (target - cov[i - 1]) / (cov[i] - cov[i - 1])
    return rel[i - 1] + f * (rel[i] - rel[i - 1])


def fit_tau(rel, cov):
    """Least squares on ln(1 - A) = b + m t  ->  tau = -1/m, td = -b/m."""
    keep = (cov > 0.02) & (cov < 0.99)
    if keep.sum() < 4:
        return None, None
    x, y = rel[keep], np.log(1.0 - cov[keep])
    m, b = np.polyfit(x, y, 1)
    pred = b + m * x
    r2 = 1 - np.sum((y - pred) ** 2) / np.sum((y - y.mean()) ** 2)
    return (-1.0 / m if m < 0 else None), r2


def still_and_idle(run, r, t0, t1, still_m=0.05, window=3.0):
    ps = np.array(run.pose.get(r, []))
    if len(ps) < 2:
        return None, None
    ts = np.arange(t0, t1, 0.5)
    xs = np.interp(ts, ps[:, 0], ps[:, 1])
    ys = np.interp(ts, ps[:, 0], ps[:, 2])
    k = max(1, int(window / 0.5)) // 2
    still = np.zeros(len(ts), dtype=bool)
    for i in range(len(ts)):
        j0, j1 = max(0, i - k), min(len(ts) - 1, i + k)
        still[i] = np.hypot(xs[j0:j1 + 1] - xs[j0], ys[j0:j1 + 1] - ys[j0]).max() < still_m
    ga = run.goal_active.get(r, [])
    active = np.zeros(len(ts), dtype=bool)
    for n, t in enumerate(ts):
        i = bagdata.latest_before(ga, t)
        active[n] = i >= 0 and ga[i][1]
    dist = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
    return 100.0 * np.mean(still & ~active), dist


def pose_at(run, r, t):
    ps = run.pose.get(r, [])
    i = bagdata.latest_before(ps, t)
    return None if i < 0 else np.array(ps[i][1:])


def median_in(series, t0, t1):
    v = [x for t, x in series if t0 <= t <= t1]
    return float(np.median(v)) if v else None


def merge_time(run, t0):
    if len(run.robots) < 2 or not all(run.world_tf.get(r) for r in run.robots[:2]):
        return None
    a, b = (np.array(run.world_tf[r]) for r in run.robots[:2])
    ts = np.arange(max(a[0, 0], b[0, 0]), min(a[-1, 0], b[-1, 0]), 0.5)
    if len(ts) == 0:
        return None
    rel = []
    for t in ts:
        ia = min(len(a) - 1, np.searchsorted(a[:, 0], t))
        ib = min(len(b) - 1, np.searchsorted(b[:, 0], t))
        ax, ay, ath = a[ia, 1:]
        bx, by, bth = b[ib, 1:]
        c, s = math.cos(-ath), math.sin(-ath)
        rel.append((c * (bx - ax) - s * (by - ay), s * (bx - ax) + c * (by - ay),
                    math.atan2(math.sin(bth - ath), math.cos(bth - ath))))
    rel = np.array(rel)
    fin = rel[-1]
    ok = (np.hypot(rel[:, 0] - fin[0], rel[:, 1] - fin[1]) < 0.3) & (
        np.abs(np.angle(np.exp(1j * (rel[:, 2] - fin[2])))) < math.radians(5))
    idx = len(ok)
    while idx > 0 and ok[idx - 1]:
        idx -= 1
    # Settled only at the very end, or from the start (side-by-side placement
    # never moved): no feature-matched merge to report.
    if idx >= len(ok) - 10 or idx == 0:
        return None
    return float(ts[idx] - t0)


def plot_registration(run, ref, T, path):
    fig, ax = plt.subplots(figsize=(7, 7))
    h, w = ref.grid.shape
    img = np.full(ref.grid.shape, 0.85)
    img[ref.grid == 0] = 1.0
    img[ref.grid > 50] = 0.45
    ox, oy = ref.origin
    ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower',
              extent=(ox, ox + w * ref.res, oy, oy + h * ref.res))
    for r, colour in zip(run.robots, ('tab:blue', 'tab:orange')):
        s = run.maps[r][-1]
        rr, cc = np.nonzero(s.grid > 50)
        xy = np.column_stack([s.origin[0] + (cc + 0.5) * s.res, s.origin[1] + (rr + 0.5) * s.res])
        p = T[r].apply(xy)
        ax.scatter(p[:, 0], p[:, 1], s=1, c=colour, label=f'{r} walls')
    ax.legend(loc='upper right', markerscale=8)
    ax.set_title(f'{run.run_id}: robot maps placed on the reference (walls should line up)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def extract(run_dir, ref):
    run = bagdata.load_run(run_dir)
    meta, plan = run.meta, run.meta.get('plan') or {}
    flags = []
    if run.t0 is None:
        raise ValueError('no t0 in meta.json (the run never sent a goal)')
    for r in run.robots:
        if not run.maps.get(r):
            raise ValueError(f'no /{r}/map in the bag')
    cap = meta.get('cap_sec') or design.CAP_SEC
    t0, t1 = run.t0, run.t0 + cap
    # Scoring stops when the explorer declares exploration complete; driving
    # home (or failing to) afterwards is not exploring.
    end_reason = 'cap'
    complete_t, _ = exploration_status(run_dir, run.robots)
    if complete_t is not None and complete_t < t1:
        t1, end_reason = complete_t, 'explored'

    T = align_robots(run, ref, flags)
    rel = np.arange(0.0, cap + 1e-6, STEP)
    cov = coverage_series(run, ref, T, t0 + rel)
    with open(run_dir / 'coverage.csv', 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['t_s', 'coverage_pct'])
        wr.writerows((f'{t:.1f}', f'{100 * c:.2f}') for t, c in zip(rel, cov))
    plot_registration(run, ref, T, run_dir / 'registration.png')

    t90 = time_to(cov, rel, TARGET)
    censored = t90 is None
    tau, r2 = fit_tau(rel, cov)

    idle, dist = [], []
    for r in run.robots:
        i, d = still_and_idle(run, r, t0, t1)
        if i is None:
            flags.append(f'{r}_no_pose')
            continue
        idle.append(i)
        dist.append(d)

    ok = bad = 0
    for e in run.events:
        t = (e.get('t_ros_ns') or 0) * 1e-9
        if e.get('event') == 'goal_result' and t0 <= t <= t1:
            ok += e.get('status_text') == 'SUCCEEDED'
            bad += e.get('status_text') == 'ABORTED'

    starts, drops = [], []
    for r in run.robots:
        v0 = median_in(run.voltage.get(r, []), t0, t0 + 30)
        v1 = median_in(run.voltage.get(r, []), t1 - 30, t1)
        if v0 is None or v1 is None:
            flags.append(f'{r}_no_battery')
            continue
        starts.append(v0)
        drops.append(v0 - v1)

    sep = []
    if len(run.robots) >= 2:
        a, b = run.robots[:2]
        for t in np.arange(t0, t1, 1.0):
            pa, pb = pose_at(run, a, t), pose_at(run, b, t)
            if pa is not None and pb is not None:
                sep.append(np.hypot(*(T[a].apply(pa[None, :])[0] - T[b].apply(pb[None, :])[0])))

    closer = total = 0
    if len(run.robots) >= 2:
        for e in run.events:
            t = (e.get('t_ros_ns') or 0) * 1e-9
            if e.get('event') != 'goal_sent' or not (t0 <= t <= t1):
                continue
            r = e.get('robot')
            others = [o for o in run.robots if o != r]
            g = e.get('goal') or {}
            if r not in T or g.get('frame') != f'{r}/map' or not others:
                continue
            goal = T[r].apply(np.array([[g['x'], g['y']]]))[0]
            pr, po = pose_at(run, r, t), pose_at(run, others[0], t)
            if pr is None or po is None:
                continue
            d_own = np.hypot(*(T[r].apply(pr[None, :])[0] - goal))
            d_other = np.hypot(*(T[others[0]].apply(po[None, :])[0] - goal))
            total += 1
            closer += d_other < d_own

    cpu = []
    for r in run.robots:
        v = [c for t, c in run.pi.get(r, []) if t0 <= t <= t1]
        if v:
            cpu.append(float(np.mean(v)))
        else:
            flags.append(f'{r}_no_cpu')

    def fmt(x, nd=2):
        return '' if x is None else f'{x:.{nd}f}'

    mt = merge_time(run, t0)
    return {
        'run_id': run.run_id, 'session': plan.get('session', ''), 'type': plan.get('type', ''),
        'condition': plan.get('condition', ''), 'pair': plan.get('pair', ''),
        'arm': plan.get('arm', ''),
        'A': plan.get('A', ''), 'B': plan.get('B', ''), 'C': plan.get('C', ''), 'D': plan.get('D', ''),
        'T90_s': fmt(cap if censored else t90, 1), 'T90_censored': int(censored),
        'coverage_cap_pct': fmt(100 * cov[-1]),
        'tau_s': fmt(tau, 1), 'tau_r2': fmt(r2, 3),
        'idle_pct': fmt(np.mean(idle) if idle else None),
        'goal_success_pct': fmt(100 * ok / (ok + bad) if ok + bad else None),
        'goals_succeeded': ok, 'goals_aborted': bad,
        'distance_m': fmt(sum(dist) if dist else None),
        'battery_drop_v': fmt(np.mean(drops) if drops else None, 3),
        'start_voltage_v': fmt(np.mean(starts) if starts else None, 3),
        'separation_m': fmt(float(np.median(sep)) if sep else None),
        'other_closer_pct': fmt(100 * closer / total if total else None),
        'cpu_pct': fmt(np.mean(cpu) if cpu else None),
        'merge_time_s': fmt(mt, 1),
        'run_length_s': fmt(t1 - t0, 1), 'end_reason': end_reason,
        'flags': ' '.join(flags),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='*')
    ap.add_argument('--all', action='store_true')
    args = ap.parse_args()

    ref = bagdata.load_reference()
    existing = {}
    if design.RESPONSES_CSV.exists():
        with open(design.RESPONSES_CSV, newline='') as f:
            existing = {row['run_id']: row for row in csv.DictReader(f)}

    complete = []
    for meta in sorted(design.RUNS.glob('*/meta.json')):
        d = meta.parent
        if '.' in d.name:      # R03.discarded-...
            continue
        m = bagdata.json.loads(meta.read_text())
        if m.get('status') == 'complete' and not m.get('reference'):
            complete.append(d)
    if args.runs:
        todo = [design.RUNS / r for r in args.runs]
    elif args.all:
        todo = complete
    else:
        todo = [d for d in complete if d.name not in existing]
    if not todo:
        print('nothing to extract (pass --all to redo every run)')

    failed = 0
    for d in todo:
        print(f'{d.name}:')
        try:
            row = extract(d, ref)
        except (ValueError, FileNotFoundError, OSError) as e:
            print(f'    skipped: {e}')
            failed += 1
            continue
        existing[d.name] = row
        print(f'    T90 {row["T90_s"]} s{" (censored)" if row["T90_censored"] else ""}, '
              f'coverage {row["coverage_cap_pct"]}% at cap, idle {row["idle_pct"]}%, '
              f'success {row["goal_success_pct"]}%, drop {row["battery_drop_v"]} V'
              + (f'  [{row["flags"]}]' if row['flags'] else ''))

    design.DATA.mkdir(parents=True, exist_ok=True)
    with open(design.RESPONSES_CSV, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for k in sorted(existing):
            w.writerow({f: existing[k].get(f, '') for f in FIELDS})
    print(f'wrote {design.RESPONSES_CSV} ({len(existing)} runs)')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
