#!/usr/bin/env python3
"""Two checks on the held-out run: does the input topic drive AKAZE, and is the live merged map itself aligned?"""
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

from eval_twobot import read_maps, read_topics
from map_graph import build_graph
from matching import akaze_baseline, map_agreement
from registration import Rigid, register_refined
from run_real import evaluate

d = np.load('data/twobot1_final.npz')
T_ref = Rigid(float(d['theta']), d['t'])
res = float(d['res'])

# Check 1: AKAZE on /map against /map_live at the same moments of the same run.
maps = {name: read_maps(f'data/bags/twobot1/{robot}', f'/{robot}/{name.split("_", 1)[1]}')
        for robot in ('inky', 'clyde') for name in (f'{robot}_map', f'{robot}_map_live')}
start = max(maps['inky_map'][0][0], maps['clyde_map'][0][0])
end = min(maps['inky_map'][-1][0], maps['clyde_map'][-1][0])
sample = build_graph(d['gb'], res, d['ob'], spacing=0.25).xy


def latest(series, t):
    return max((m for m in series if m[0] <= t), key=lambda m: m[0])


print('topic      correct  wrong  none   (same 43 time points, offline AKAZE at map_merge gates)')
per_time = {}
for topic, ka, kb in (('map', 'inky_map', 'clyde_map'), ('map_live', 'inky_map_live', 'clyde_map_live')):
    right = wrong = none = 0
    onset = None
    for t in np.arange(start + 15, end + 1e-3, 15):
        a, b = latest(maps[ka], t), latest(maps[kb], t)
        T, _ = akaze_baseline(a[1], a[2], b[1], b[2], a[3])
        if T is None:
            none += 1
        elif evaluate(T, T_ref, sample) < 0.30:
            right += 1
            onset = onset if onset is not None else t - start
        else:
            wrong += 1
    print(f'{topic:10s} {right:7d} {wrong:6d} {none:5d}   first correct at t={onset}')

# Check 2: is the live merged grid aligned, whatever the TF says?
merged = read_maps('data/bags/twobot1/central', '/map')
_, gm, om, rm = merged[-1]
ga, oa, gb, ob = d['ga'], d['oa'], d['gb'], d['ob']
T_inky_to_merged, s1, r1 = register_refined(gm, om, ga, oa, res)
T_clyde_to_merged, s2, r2 = register_refined(gm, om, gb, ob, res)
print(f'merged grid {gm.shape}, origin {np.round(om, 2)}; registration score/rival inky {s1 / r1:.2f}, clyde {s2 / r2:.2f}')
print('conflict of each final map against merged grid under registration:',
      round(map_agreement(gm, om, ga, oa, res, T_inky_to_merged)[1], 3),
      round(map_agreement(gm, om, gb, ob, res, T_clyde_to_merged)[1], 3))
through = (T_clyde_to_merged.apply(sample) - T_inky_to_merged.t) @ T_inky_to_merged.R
print(f'Clyde -> Inky implied by the merged grid: error vs reference {np.linalg.norm(through - T_ref.apply(sample), axis=1).mean():.3f} m')

# What exactly did TF carry at the end?
last = {}
parents = {}
for _, msg, stamp in read_topics('data/bags/twobot1/central', {'/tf': TFMessage}):
    for tf in msg.transforms:
        if tf.child_frame_id in ('inky/map', 'clyde/map'):
            q, p = tf.transform.rotation, tf.transform.translation
            last[tf.child_frame_id] = (tf.header.frame_id, round(2 * np.degrees(np.arctan2(q.z, q.w)), 2), round(p.x, 3), round(p.y, 3))
            parents.setdefault(tf.child_frame_id, set()).add(tf.header.frame_id)
print('final TF:', last, 'parents seen:', parents)
for name, T in (('inky', T_inky_to_merged), ('clyde', T_clyde_to_merged)):
    print(f'{name}/map -> merged frame by registration: theta={np.rad2deg(T.theta) % 360:.2f} deg, t={np.round(T.t, 3)}')

fig, ax = plt.subplots(1, 1, figsize=(5, 5))
img = np.full(gm.shape, .65); img[gm == 0] = 1; img[gm > 50] = 0
ax.imshow(img, cmap='gray', origin='lower', vmin=0, vmax=1,
          extent=[om[0], om[0] + gm.shape[1] * res, om[1], om[1] + gm.shape[0] * res])
ax.set_title('Live merged /map, end of run', fontsize=10)
plt.tight_layout(); plt.savefig('figures/twobot1_live_merged.png', dpi=80)
