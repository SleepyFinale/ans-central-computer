# STAT 636 project: aligning robot maps by graph matching

Question: if each robot's occupancy grid is turned into a graph of its free space, can graph
matching recover how two robots' maps line up, and when?

Working question (reframed 2026-11-10, with the professor's agreement that a well documented
negative result is acceptable): **when does graph structure help align two robots' maps, and what
limits it?**

Status as of 2026-11-10: three true two-robot runs scored. In the two small-room runs graph-based
matching roughly ties the AKAZE merger. In the hallway run (`hall1`) the merger never answered and
graph-based matching did, from 42% overlap, but it also accepted wrong answers at 9-20% overlap.
Nothing was right below 42% overlap.

## Data

| Source | What it is | Used for |
| --- | --- | --- |
| `large2`, `3`, `6`, `7`, `8` (Inky, 2026-11-05) | Five single-robot runs of one area, about 8 x 8 m, 26-37 m2 free | Development, 260 map pairs from 10 run pairs |
| `twobot1` (Inky and Clyde, 2026-11-05) | Two-robot run, robots started together, 11 minutes, 43 time points | Held-out test for the FAQ methods |
| `twobot4` (Inky and Clyde, 2026-11-10) | Two-robot run, robots started in different rooms, stopped by hand after 6 minutes, 23 time points | Held-out test for the FAQ methods |
| `hall1` (Inky and Clyde, 2026-11-10) | Two-robot run in a larger area with hallways, robots started apart, stopped by hand after 13 minutes, 52 time points; final free area 140 m2 (Inky) and 68 m2 (Clyde) | Held-out test for every method, partial matching included |

The truth is a **reference, not a measurement**: exhaustive registration of final maps, checked by
eye (`figures/reference_overlays.png`, `figures/twobot1_reference.png`). Walls agree within 10 cm
for 89-98%. Success means mean node error under 0.30 m.

## Results

Development pairs, success rate with 95% CI bootstrapped over run pairs (`summarize_real.py`,
spacing 0.5 m):

| Method | Overlap < 0.5 | 0.5-0.9 | > 0.9 | All |
| --- | --- | --- | --- | --- |
| AKAZE with `map_merge` gates | 0% | 14% | 20% | 14% (4-25) |
| Random matching (control) | 0% | 0% | 3% | 2% (0-3) |
| Clearance, no edges (control) | 0% | 52% | 57% | 44% (36-53) |
| FAQ on adjacency | 0% | 5% | 6% | 5% (3-6) |
| FAQ on geodesic kernel | 0% | 17% | 18% | 14% (10-19) |
| FAQ on geodesic kernel + clearance | 6% | 52% | 65% | 49% (41-58) |
| 5 oracle seeds, no matching (control) | 50% | 98% | 100% | 89% (87-92) |
| FAQ geodesic, 5 oracle seeds | 40% | 100% | 99% | 88% (85-90) |

Two-robot runs (`eval_twobot.py twobot1 twobot4`). Counts of time points where the method gave an
accepted answer that was correct / accepted and wrong, and when it was first right:

| Method | `twobot1` (43 points) | First right | `twobot4` (23 points) | First right |
| --- | --- | --- | --- | --- |
| AKAZE, offline | 31 / 0 | 195 s | 13 / 0 | 165 s |
| Clearance only, gated | 16 / 0 | 210 s | 15 / 0 | 90 s |
| FAQ geodesic + clearance, gated | 21 / 1 | 210 s | 16 / 0 | 105 s |
| Partial matching, geodesic, gated | 1 / 1 | 210 s | 11 / 1 | 120 s |
| Partial matching, Euclidean, gated | 30 / 2 | 195 s | 16 / 2 | 120 s |

Hallway run `hall1` (`eval_twobot.py hall1`, `akaze_gates.py hall1`), 52 time points. Overlap stayed
at 9-20% until 270 s, then rose to 42-84%. All settings were fixed before this run.

| Method | Accepted correct / wrong | First right | Overlap then |
| --- | --- | --- | --- |
| AKAZE with `map_merge` gates | 0 / 0 | never | - |
| AKAZE without the inlier-ratio gate | 20 / 1 | 330 s | 60% |
| Clearance only, gated | 8 / 2 | 330 s | 60% |
| FAQ geodesic + clearance, gated | 24 / 3 | 300 s | 43% |
| Partial matching, geodesic, gated | 11 / 5 | 330 s | 60% |
| Partial matching, Euclidean, gated | 31 / 9 | 285 s | 42% |

- The live merger stayed on independent placement for the whole run. AKAZE's estimate was right at
  32 time points, but the inlier-ratio gate (>= 0.5) rejected all of them: at the end it had 30
  inliers of 113 matches (0.27). That gate was not binding in the two small-room runs.
- 2026-11-12: the gate is now the parameter `estimation_min_inlier_ratio`, set to 0.25 for the fleet.
  Replaying this run's maps through the rebuilt merger at 3x speed, it first published a correct
  transform at about 285 s and stayed within 0.12 m of the reference from then on. Not yet run live.
- The reference for this run was cross-checked: the ungated AKAZE estimate agrees with it within
  0.24 m, and sliding Clyde's map along the corridor in either direction roughly doubles the
  conflict rate.
- Eight of the nine wrong accepts for Euclidean partial matching came between 45 and 150 s, at
  9-18% overlap. The conflict gate is not safe at low overlap in this larger space.

Three-room run `small1` (2026-11-12; misnamed, it is the three-room area with no hallways; stopped by
hand after 10 minutes, 38 time points). All settings were fixed before this run.

| Method | Accepted correct / wrong | First right | Overlap then |
| --- | --- | --- | --- |
| AKAZE, offline (same with the 0.5 and 0.25 ratio gate) | 28 / 0 | 165 s | 91% |
| Clearance only, gated | 22 / 0 | 135 s | 92% |
| FAQ geodesic + clearance, gated | 31 / 0 | 105 s | 71% |
| Partial matching, geodesic, gated | 24 / 0 | 165 s | 91% |
| Partial matching, Euclidean, gated | 31 / 0 | 120 s | 77% |

The live merger never merged in this run, for a reason unrelated to matching: `start_rviz_central.sh`
probes `/<robot>/map` for all four fleet robots, `map_merge` ran its first topic discovery half a
second later and added Blinky and Pinky as robots, and with two empty maps every estimate returned
early ("2 map(s) have fewer than 2 features", 548 times). So this run does not test the relaxed gate
live. Fixed the same day: discovery now skips a map topic that has no publisher. In a replay of this
run's maps with a probe subscribed to all four robots, only Inky and Clyde were added and the merger
published a correct transform from about 114 s on (worst error 0.09 m). Not yet run live.

Hallway run `hall2` (2026-11-12, after commits `7d1da15` and `c44811b`: relaxed ratio gate 0.25,
rigid-only alignments, first alignment needs two agreeing estimates). Inky went out of Wi-Fi range at
about 1050 s and the fleet stopped itself; Inky's own bag kept recording and was copied once it
reconnected. 74 time points, timed from the start of the robot bags. All settings fixed before this
run. (`results/hall2_central_only.json` is the earlier scoring from the central bag alone.)

| Method | Accepted correct / wrong | First right |
| --- | --- | --- |
| Live `map_merge` (new code) | 36 / 0 after its first answer | 585 s |
| AKAZE offline, no ratio gate | 33 / 1 | 615 s |
| Clearance only, gated | 29 / 16 | 540 s |
| FAQ geodesic + clearance, gated | 39 / 14 | 510 s |
| Partial matching, geodesic, gated | 29 / 26 | 630 s |
| Partial matching, Euclidean, gated | 33 / 35 | 630 s |

- The live merger's first alignment had an inlier ratio of 0.27; the old 0.5 gate would have rejected
  it. After merging it never switched alignment and stayed within 0.08 m of the reference.
- Almost all wrong accepts by the graph methods were gross (2 m or more: 12-35 per method), mostly
  while overlap was 5-13% and the maps were long, self-similar corridors. The conflict gate is not a
  safe accept rule in hallways. The live merger's two-agreeing-estimates rule is what made AKAZE safe
  here; the graph methods were scored without such a rule.
- The reference is less distinctive than in other runs (best score 1.46x the best rival orientation,
  against 1.8-2.5 elsewhere), but the live merger agrees with it to within 0.08 m.

## Does map_merge's confirmation rule make the graph methods safe? (2026-11-12)

`confirm_rule.py` replays the rule `map_merge` uses since commit `c44811b` over each method's accepted
answers in time: the first alignment needs two estimates in a row that agree (within 0.5 m and 8 deg
at the centre of Clyde's grid), and a switch needs three. These numbers are `map_merge`'s, fixed
before the replay. AKAZE is replayed under the same rule with `map_merge`'s current gates (8
inliers, ratio 0.25, confidence 0.65, scale within 5%). Answers come every 15 s here, against about
1 s live, so compare methods with each other, not with live timings.

Time points at which the held alignment was right / wrong, and when it was first right:

| Run | AKAZE | Clearance only | FAQ geodesic + clearance | Partial, geodesic | Partial, Euclidean |
| --- | --- | --- | --- | --- | --- |
| `twobot1` | 30 / 0, 210 s | 29 / 0, 225 s | 29 / 0, 225 s | 0 / 0, never | 30 / 0, 210 s |
| `twobot4` | 12 / 0, 180 s | 17 / 0, 105 s | 16 / 0, 120 s | 15 / 0, 135 s | 16 / 0, 120 s |
| `small1` | 27 / 0, 180 s | 29 / 0, 150 s | 31 / 0, 120 s | 27 / 0, 180 s | 30 / 0, 135 s |
| `linktest1` | 9 / 0, 165 s | 7 / 0, 195 s | 11 / 0, 135 s | 10 / 0, 150 s | 8 / 0, 180 s |
| `hall1` | 30 / 0, 345 s | 25 / 0, 420 s | 32 / 0, 315 s | 29 / 0, 360 s | 32 / 1, 300 s |
| `hall2` | 32 / 0, 645 s | 30 / 42, 630 s | 37 / 35, 570 s | 30 / 35, 660 s | 31 / 36, 660 s |
| `hall3` | 82 / 0, 225 s | 77 / 0, 300 s | 77 / 3, 255 s | 72 / 13, 345 s | 79 / 7, 270 s |
| `hall4` | 20 / 0, 675 s | 22 / 36, 645 s | 24 / 0, 615 s | 0 / 55, never | 0 / 53, never |
| All eight | 242 / 0 | 236 / 78 | 257 / 38 | 183 / 103 | 226 / 97 |

- The rule removes every wrong commitment in the three-room runs and `hall1` (one 0.3-0.5 m near miss
  remains for Euclidean partial matching in `hall1`). FAQ geodesic + clearance then answers first in
  five of six runs: 15-60 s before AKAZE in the three-room runs, 30 s in `hall1`.
- It does not help in `hall2`. Every graph method commits at 45 s, at 7-18% overlap, to an alignment
  rotated by about 90 deg, and holds a wrong alignment for about 9 minutes. The early estimates are
  wrong in the same way each time (the corridor loop looks alike turned by a right angle), so they
  agree with each other and pass a rule built to filter out noise.
- AKAZE never held a wrong alignment in any run, but it answers later.
- Conclusion so far: confirming over time fixes random wrong answers, not systematic ones. Graph
  structure built from these free-space graphs does not tell apart a corridor loop and its rotation.
  A rule that would have prevented `hall2` (for example a minimum shared area before the first
  commitment) would be fitted to that one run; it needs a new hallway run to test.
- `linktest1` (2026-11-12, three-room area, the Wi-Fi failsafe test) is new held-out data here.
- `hall3` (2026-11-12, hallways, scored after the rule was fixed) is the held-out repeat of `hall2`.
  FAQ geodesic + clearance held no gross wrong alignment this time (3 time points 0.3-2 m off), but
  answered 30 s after AKAZE (255 s against 225 s; the live merger was right from 195 s). Both partial
  matching methods again committed to gross wrong alignments early (11 and 7 time points 2 m or more
  off). So in hallways the graph methods are not faster than AKAZE and some are still unsafe; in the
  three-room runs FAQ geodesic + clearance stays faster and safe.
- `hall4` (2026-11-12, hallways, stopped by hand after about 17 min; Clyde mapped only one corridor,
  43 m2). The reference is the least distinctive so far (best score 1.34x the best rival orientation)
  but puts Clyde's corridor on Inky's top corridor as in `hall2` and `hall3`, with 95% of Clyde's walls
  within 10 cm. FAQ geodesic + clearance was right first (615 s) and never wrong; both partial
  matching methods held gross wrong alignments the whole run. The live merger never merged: it
  rejected estimates whose scale was 6.5-7% off. That gate is right: across all eight runs every AKAZE
  estimate with scale 5-10% off was wrong (7 of 7) and every one within 5% was right. Offline, under
  the same gates, AKAZE was first right at 675 s; why the live merger still had not merged by the end
  is not explained (the offline baseline skips bundle adjustment and map padding).

How far each row is a fair test:

- Spacing, restarts and the conflict gate were frozen on the single-robot runs before either
  two-robot run was scored.
- The gate's minimum shared area (3 m2) was added after both runs showed accepted answers at 0%
  overlap, so its effect on these two runs is not a held-out result.
- Partial matching was written after `twobot1` and `twobot4` had been looked at. Its settings
  (`top_k=8`, `sigma=0.15` m) were set once and not tuned, but the method choice itself was informed
  by those runs.
- Time points within a run are strongly correlated: this is n = 2 runs, not 66 samples.
- In `twobot4` the first right answers came at 72-78% overlap for the graph methods and 94% for
  AKAZE; in `twobot1` all methods first succeeded at 86%.

## What we learned

1. **Adjacency alone fails.** FAQ on adjacency got 13% of nodes right on an exact shuffled copy of a
   graph; the geodesic kernel got 100%. These graphs are sparse and nearly tree-like.
2. **Oracle seeds make the matching look good without helping it.** Five seeds alone fix the
   transform, so seeded results say little about matching.
3. **Edges add little beyond the clearance feature:** +5 points at 0.25 m spacing (95% CI 0 to +10)
   and +9 at 0.5 m (+4 to +15), paired across development pairs.
4. **Node placement looks like the main noise source.** Between runs, geodesic distance (r = 0.98)
   and clearance (r = 0.97) repeat well, edge Jaccard is 0.67, but only 62% of nodes have a
   counterpart within 25 cm. In simulation, 38% node dropout takes the geodesic method from 100% to
   8% at full overlap (`simulate.py`).
5. **Nothing tested works below about 55% overlap**, on either two-robot run, with or without
   partial matching. That is where a better merger is needed.
6. **Geodesic distance is the wrong invariant for partial maps.** Partial matching on geodesic
   distances got 4 of 43 raw on `twobot1`; on Euclidean distances, 31. A path through unexplored
   space is missing from one map, so graph distances differ while straight-line distances do not.
   The part of the graph that helps is the node set and its geometry, not the edges.
7. **Starting in different rooms did not give low overlap for long.** The area is about 37 m2, so
   the smaller map is soon contained in the larger. A real low-overlap test needs a larger space.

## Open questions and limits

- **Why AKAZE scores 14% on the older pairs and 72% on `twobot1` is unexplained.** Ruled out: input
  topic (`/map` and `map_live` both give 31 of 43) and map rotation (`rotation_sweep.py`: correct at
  all 37 angles). So the development comparison may flatter graph matching.
- Selection by map agreement uses the same kind of score that defines the reference. Inlier-count
  selection, which does not, is lower (38% against 49%).
- 0.25 m spacing was picked after seeing the development pairs. `twobot1` is the only clean test.
- The gate still accepts wrong answers at 5-26% overlap (2 of 43 and 2 of 23 for Euclidean partial
  matching, one of them a 0.36 m near miss). AKAZE accepted none.
- Partial matching prunes candidates by clearance (top 8), so it is not independent of that feature.
- `twobot4` was stopped by hand before exploration finished.
- In `simulate.py`, edge flips can disconnect a tree, so edge noise also shrinks the graph. Those two
  effects are confounded, and some low-overlap cells have few usable trials.
- The matchers are our own numpy and scipy code. graspologic 3.4.4 is installed in `.venv` (see
  `requirements.txt`) and used as a cross-check (`crosscheck_graspologic.py`): on an exact shuffled
  copy both recover 100% of nodes with the geodesic kernel and under 20% with adjacency; on the 10
  real pairs of final maps, unseeded and without the clearance term, node accuracy is 8% (ours) and
  4% (graspologic), and our objective is at least as high in 10 of 10 pairs. So the weak unseeded
  result is a property of the problem, not of our implementation.
- `akaze_baseline` skips `map_merge`'s bundle-adjustment polish.

## Reproduce

```bash
cd ~/central-computer/stat636
source /opt/ros/humble/setup.bash
python3 extract_maps.py data/bags/large2 -o data/maps/large2_inky.npz   # repeat per run
python3 compute_reference.py
python3 test_matching.py        # known-answer checks
python3 run_real.py && python3 summarize_real.py
python3 graph_noise.py
python3 simulate.py
python3 spacing_sweep.py && python3 paired_edges.py
python3 eval_twobot.py twobot1 twobot4 small1 linktest1 hall1 hall2 hall3 hall4
python3 confirm_rule.py twobot1 twobot4 small1 linktest1 hall1 hall2 hall3 hall4
python3 akaze_gates.py hall1
python3 check_discrepancies.py
python3 rotation_sweep.py
.venv/bin/python crosscheck_graspologic.py   # needs the virtualenv, not ROS
```

Bags are not in git (`data/bags/`). Originals are on the robots under `~/turtlebot3/logs/`; the
central bag from `twobot1` is under `~/.ros/nav2_debug_central/`.
