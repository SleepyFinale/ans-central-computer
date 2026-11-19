"""Read one ENTC 641 run folder: robot maps, poses, goal status, battery, events.

Needs the ROS environment (source scripts/env/central_env.bash) for rosbag2_py.
The bag reader and frame helpers come from scripts/diagnostics/analyze_fleet_run.py.
"""

import csv
import glob
import json
import sys
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np

import design

sys.path.insert(0, str(design.ROOT.parent / 'scripts' / 'diagnostics'))
sys.path.insert(0, str(design.ROOT.parent))
try:
    from analyze_fleet_run import compose, read_bag, yaw_of  # noqa: E402
except ImportError as e:  # rosbag2_py missing
    sys.exit(f'{e}\nLoad the ROS environment first:  source scripts/env/central_env.bash')


@dataclass
class MapSnap:
    t: float
    grid: np.ndarray        # int8, row 0 at origin y
    origin: tuple           # (x, y) of cell (0, 0) corner
    res: float


@dataclass
class Run:
    run_id: str
    meta: dict
    robots: list
    t0: float                                   # first goal, seconds (ROS time)
    maps: dict = field(default_factory=lambda: defaultdict(list))      # robot -> [MapSnap]
    pose: dict = field(default_factory=lambda: defaultdict(list))      # robot -> [(t, x, y)] in <robot>/map
    world_tf: dict = field(default_factory=lambda: defaultdict(list))  # robot -> [(t, x, y, th)] map -> <robot>/map
    goal_active: dict = field(default_factory=lambda: defaultdict(list))  # robot -> [(t, bool)]
    voltage: dict = field(default_factory=lambda: defaultdict(list))   # robot -> [(t, V)]
    events: list = field(default_factory=list)
    pi: dict = field(default_factory=dict)      # robot -> [(t, cpu %)]


def snap_from_msg(msg, t):
    info = msg.info
    grid = np.frombuffer(bytes(msg.data), dtype=np.int8).reshape(info.height, info.width).copy()
    return MapSnap(t, grid, (info.origin.position.x, info.origin.position.y), info.resolution)


def load_run(run_dir):
    meta = json.loads((run_dir / 'meta.json').read_text())
    robots = meta['robots']
    t0 = meta.get('t0_ros_ns')
    run = Run(run_dir.name, meta, robots, t0 * 1e-9 if t0 else None)

    topics = ['/tf'] + [f'/{r}/{s}' for r in robots for s in
                        ('map', 'navigate_to_pose/_action/status', 'battery_state')]
    map_odom = {}
    for topic, msg, t in read_bag(str(run_dir / 'bag'), topics):
        if topic == '/tf':
            for tr in msg.transforms:
                p, c = tr.header.frame_id, tr.child_frame_id
                v = (tr.transform.translation.x, tr.transform.translation.y,
                     yaw_of(tr.transform.rotation))
                for r in robots:
                    if p == f'{r}/odom' and c == f'{r}/base_footprint':
                        g = compose(map_odom.get(r, (0.0, 0.0, 0.0)), v)
                        run.pose[r].append((t, g[0], g[1]))
                    elif p == f'{r}/map' and c == f'{r}/odom':
                        map_odom[r] = v
                    elif p == 'map' and c == f'{r}/map':
                        run.world_tf[r].append((t, *v))
            continue
        r = topic.split('/')[1]
        if topic.endswith('/map'):
            run.maps[r].append(snap_from_msg(msg, t))
        elif topic.endswith('_action/status'):
            run.goal_active[r].append((t, any(s.status in (1, 2, 3) for s in msg.status_list)))
        elif topic.endswith('battery_state'):
            run.voltage[r].append((t, float(msg.voltage)))

    for path in glob.glob(str(run_dir / 'session-*.jsonl')):
        with open(path) as f:
            for line in f:
                try:
                    run.events.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    run.events.sort(key=lambda e: e.get('t_ros_ns') or 0)

    for r in robots:
        path = run_dir / f'pi_{r}.csv'
        if path.exists():
            rows = []
            with open(path, newline='') as f:
                for row in csv.DictReader(f):
                    try:
                        rows.append((int(row['epoch_ms']) * 1e-3, float(row['cpu_usage_pct'])))
                    except (KeyError, ValueError):
                        pass
            run.pi[r] = rows
    return run


def latest_before(series, t):
    """Index of the last (t_i, ...) entry with t_i <= t, or -1."""
    lo, hi = 0, len(series)
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    return lo - 1


def load_reference():
    if not design.REFERENCE_NPZ.exists():
        sys.exit('no reference map yet: record one (README) and run  python3 entc641/reference.py REF')
    z = np.load(design.REFERENCE_NPZ)
    return MapSnap(0.0, z['grid'], tuple(z['origin']), float(z['res']))
