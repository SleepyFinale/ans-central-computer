"""Factors, levels and the 2^(4-1) design of the ENTC 641 exploration experiment.

Shared by plan.py, run_trial.py, extract.py and analyze.py. Changing a level here
changes what the runner sets on the robots, so do it before the first run only.

Coded levels are -1 / +1 (centre points 0). Factor D is generated as D = ABC,
the course's 2^(4-1) fractional factorial (main effects confounded only with
three-way interactions; AB=CD, AC=BD, AD=BC).
"""

from itertools import product
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'data'
RUNS = DATA / 'runs'
PLAN_CSV = DATA / 'plan.csv'
RESPONSES_CSV = DATA / 'responses.csv'
REFERENCE_NPZ = DATA / 'reference.npz'
RESULTS = ROOT / 'results'

ROBOTS = ('inky', 'clyde')
CAP_SEC = 480.0          # each run is scored over its first 8 minutes
MIN_START_VOLTAGE = 11.6

FACTORS = ('A', 'B', 'C', 'D')
FACTOR_NAMES = {
    'A': 'Coordination weight',
    'B': 'Start configuration',
    'C': 'Frontier ranking',
    'D': 'Max speed',
}

# Interaction columns estimable in the 2^(4-1) design; each is confounded with
# the pair named after it.
INTERACTIONS = {'AB': 'CD', 'AC': 'BD', 'AD': 'BC'}


def actual_settings(a, b, c, d):
    """Robot and explorer settings for coded levels (-1, 0 or +1)."""
    # A: coordination_weight 0 .. 1.0
    coordination_weight = round(0.5 + 0.5 * a, 3)
    # B: qualitative, no centre level
    if b > 0:
        start = 'different_rooms'
    elif b < 0:
        start = 'same_room'
    else:
        raise ValueError('factor B (start configuration) has no centre level')
    # C: potential_scale : gain_scale from 2 : 1 (near frontiers) to 1 : 2 (large)
    potential_scale = round(1.5 - 0.5 * c, 3)
    gain_scale = round(1.5 + 0.5 * c, 3)
    # D: max linear speed 0.15 .. 0.22 m/s
    max_speed = round(0.185 + 0.035 * d, 3)
    return {
        'coordination_weight': coordination_weight,
        'start': start,
        'potential_scale': potential_scale,
        'gain_scale': gain_scale,
        'max_speed': max_speed,
    }


START_INSTRUCTIONS = {
    'same_room': 'Both robots in the SAME room, about 1 m apart, facing into the room.',
    'different_rooms': 'Robots in DIFFERENT rooms (the fixed spots you marked), facing into each room.',
}


def design_matrix():
    """The 8 conditions in standard order: (condition, A, B, C, D) with D = ABC."""
    rows = []
    for i, (c, b, a) in enumerate(product((-1, 1), repeat=3), start=1):
        rows.append((i, a, b, c, a * b * c))
    return rows


def columns(a, b, c, d):
    """Coded model columns for one run: main effects and the interaction pairs."""
    return {'A': a, 'B': b, 'C': c, 'D': d, 'AB': a * b, 'AC': a * c, 'AD': a * d}


DEFAULT_CODED = (1, 1, -1, 1)
"""Current explorer defaults: coordination on, robots apart, 2 : 1 ranking, 0.22 m/s."""
