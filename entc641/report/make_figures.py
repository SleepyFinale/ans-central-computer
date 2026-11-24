"""Figures for the ENTC 641 report that analyze.py does not make.

  fig_system.png     the fleet: two robots, the central PC, what crosses the link
  fig_io.png         the experiment as a box: factors in, responses out, nuisance
  fig_coverage.png   coverage against time for all 20 runs, by start configuration
  fig_merge.png      when the maps merged against T90, by coordination level

Run from anywhere: python3 entc641/report/make_figures.py
"""

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import design  # noqa: E402

OUT = HERE / 'figures'
BLUE, ORANGE, GREY, DARK = '#1f77b4', '#ff7f0e', '#7f7f7f', '#222222'


def box(ax, x, y, w, h, text, fc='#eef3fa', ec=BLUE, size=9, weight='normal'):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.08',
                                fc=fc, ec=ec, lw=1.2))
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=size,
            weight=weight, color=DARK, wrap=True)


def arrow(ax, p, q, text=None, both=False, color=DARK, size=8, offset=(0, 0.12)):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle='<|-|>' if both else '-|>',
                                 mutation_scale=12, lw=1.1, color=color))
    if text:
        ax.text((p[0] + q[0]) / 2 + offset[0], (p[1] + q[1]) / 2 + offset[1], text,
                ha='center', va='bottom', fontsize=size, color=color)


def system():
    fig, ax = plt.subplots(figsize=(10, 4.6))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 4.6)
    ax.axis('off')
    for i, name in enumerate(('Inky', 'Clyde')):
        y0 = 2.45 - i * 2.3
        ax.add_patch(FancyBboxPatch((0.1, y0), 3.4, 2.0, boxstyle='round,pad=0.02',
                                    fc='#fafafa', ec=GREY, lw=1, ls='--'))
        ax.text(0.25, y0 + 1.8, f'{name}: TurtleBot3 Burger, Raspberry Pi', fontsize=9,
                weight='bold')
        box(ax, 0.3, y0 + 0.95, 1.4, 0.6, 'LiDAR, wheels\nbattery')
        box(ax, 1.9, y0 + 0.95, 1.4, 0.6, 'SLAM Toolbox\n(own map)')
        box(ax, 1.1, y0 + 0.15, 1.6, 0.6, 'Nav2\n(drives to a goal)')
        arrow(ax, (1.7, y0 + 1.25), (1.9, y0 + 1.25))
        arrow(ax, (2.6, y0 + 0.95), (2.2, y0 + 0.75))
    ax.add_patch(FancyBboxPatch((5.6, 0.25), 4.3, 4.1, boxstyle='round,pad=0.02',
                                fc='#fafafa', ec=GREY, lw=1, ls='--'))
    ax.text(5.75, 4.1, 'Central PC', fontsize=9, weight='bold')
    box(ax, 5.9, 2.9, 3.7, 0.8, 'Map merge\n(aligns the two robot maps into one)')
    box(ax, 5.9, 1.45, 3.7, 1.0, 'Multi-robot frontier explorer\nranks frontiers, gives each robot a goal\n'
        '(factors A, C set here)', fc='#fff4e8', ec=ORANGE)
    box(ax, 5.9, 0.45, 3.7, 0.6, 'Logging: ROS bag, explorer events\n(the data in this report)',
        fc='#f2f2f2', ec=GREY)
    arrow(ax, (7.75, 2.9), (7.75, 2.45))
    arrow(ax, (7.75, 1.45), (7.75, 1.05))
    arrow(ax, (3.5, 3.6), (5.9, 3.3), 'maps, poses', size=8, offset=(0, 0.1))
    arrow(ax, (3.5, 1.3), (5.9, 3.0), 'maps, poses', size=8, offset=(0.75, 0.05))
    arrow(ax, (5.9, 2.0), (2.7, 2.75), 'goals', size=8, offset=(-0.6, 0.05))
    arrow(ax, (5.9, 1.7), (2.7, 0.45), 'goals', size=8, offset=(-0.5, -0.45))
    ax.text(4.55, 0.05, 'Wi-Fi link (Zenoh over Tailscale)', ha='center', fontsize=8, color=GREY)
    ax.text(1.8, -0.05, 'Factor D (max speed) is set in Nav2 on each robot', ha='center',
            fontsize=8, color=ORANGE)
    fig.tight_layout()
    return fig


def io_diagram():
    fig, ax = plt.subplots(figsize=(10, 4.4))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 4.4)
    ax.axis('off')
    box(ax, 3.7, 1.2, 2.6, 2.0, 'Two-robot\nexploration\nof the three-room area\n(480 s cap)',
        fc='#eef3fa', ec=BLUE, size=10, weight='bold')
    factors = [('A', 'Coordination weight', '0 / 1.0'),
               ('B', 'Start configuration', 'same room / different rooms'),
               ('C', 'Frontier ranking (potential : gain)', '2 : 1 / 1 : 2'),
               ('D', 'Max linear speed', '0.15 / 0.22 m/s')]
    ax.text(1.6, 4.05, 'Factors (inputs), coded −1 / +1', ha='center', fontsize=9, weight='bold')
    for i, (k, name, lv) in enumerate(factors):
        y = 3.35 - i * 0.78
        box(ax, 0.05, y, 3.1, 0.6, f'{k}: {name}\n{lv}', fc='#fff4e8', ec=ORANGE, size=8)
        arrow(ax, (3.15, y + 0.3), (3.7, 2.2 + (1.5 - i) * 0.25))
    ax.text(8.4, 4.05, 'Responses (outputs)', ha='center', fontsize=9, weight='bold')
    for i, (name, d) in enumerate([('Y1: T90 (s)', 'time until the robots have\nseen 90% of the reference map'),
                                   ('Y2: Time to exploration\ncomplete (s)', 'until no reachable\nfrontier is left')]):
        y = 2.65 - i * 1.25
        box(ax, 6.85, y, 3.1, 0.95, f'{name}\n{d}', fc='#eef7ee', ec='#2ca02c', size=8)
        arrow(ax, (6.3, 2.2 + (0.5 - i) * 0.5), (6.85, y + 0.47))
    ax.text(4.3, 0.75, 'Nuisance: battery (start ≥ 11.6 V), door (held open),\nsession (block), Wi-Fi load, people (randomized order)',
            ha='center', va='top', fontsize=8, color=GREY)
    arrow(ax, (5.0, 0.8), (5.0, 1.2), color=GREY)
    ax.text(8.4, 0.75, 'Described only: idle time, separation,\ndistance, goal success',
            ha='center', va='top', fontsize=8, color=GREY)
    fig.tight_layout()
    return fig


def rows():
    with open(design.RESPONSES_CSV, newline='') as fh:
        return list(csv.DictReader(fh))


def coverage(resp):
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for r in resp:
        path = design.RUNS / r['run_id'] / 'coverage.csv'
        with open(path, newline='') as fh:
            pts = [(float(a['t_s']), float(a['coverage_pct'])) for a in csv.DictReader(fh)]
        apart = r['B'] == '1'
        ax.plot([p[0] for p in pts], [p[1] for p in pts], lw=1.1, alpha=0.8,
                color=ORANGE if apart else BLUE,
                ls='--' if r['type'] == 'centre' else '-')
    ax.axhline(90, color='k', lw=0.8, ls=':')
    ax.text(395, 91, '90%', ha='right', fontsize=8)
    ax.plot([], [], color=BLUE, label='B: same room')
    ax.plot([], [], color=ORANGE, label='B: different rooms')
    ax.plot([], [], color=GREY, ls='--', label='centre runs (dashed)')
    ax.legend(fontsize=8, loc='lower right')
    ax.set_xlim(0, 400)
    ax.set_ylim(0, 100)
    ax.set_xlabel('time since first goal (s)')
    ax.set_ylabel('coverage of reference map (%)')
    ax.set_title('Coverage against time, all 20 runs', fontsize=10)
    fig.tight_layout()
    return fig


def merge(resp):
    fig, ax = plt.subplots(figsize=(5.6, 4.2))
    for a, color, label in (('-1', BLUE, 'A off'), ('1', ORANGE, 'A on'), ('0', GREY, 'centre (A = 0.5)')):
        sel = [r for r in resp if r['A'] == a]
        ax.scatter([float(r['T90_s']) for r in sel], [float(r['merge_time_s']) for r in sel],
                   color=color, label=label, s=28)
    lim = 210
    ax.plot([0, lim], [0, lim], color='k', lw=0.8, ls=':')
    ax.text(190, 170, 'merged at T90', rotation=45, fontsize=8, ha='center')
    ax.text(40, 185, 'maps merged after T90:\ncoordination inactive\nfor most of the window',
            fontsize=8, color=DARK)
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_xlabel('T90 (s)')
    ax.set_ylabel('map merge time (s)')
    ax.set_title('When the maps merged, against T90', fontsize=10)
    ax.legend(fontsize=8, loc='lower right')
    fig.tight_layout()
    return fig


def main():
    OUT.mkdir(exist_ok=True)
    resp = rows()
    for name, fig in (('fig_system.png', system()), ('fig_io.png', io_diagram()),
                      ('fig_coverage.png', coverage(resp)), ('fig_merge.png', merge(resp))):
        fig.savefig(OUT / name, dpi=200, bbox_inches='tight')
        plt.close(fig)
        print('wrote', OUT / name)


if __name__ == '__main__':
    main()
