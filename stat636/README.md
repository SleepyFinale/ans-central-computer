# STAT 636 project: aligning robot maps by graph matching

Question: if each robot's occupancy grid is turned into a graph of its free space, can graph
matching recover how two robots' maps line up, and when?

Status as of 2026-11-05: first pass done. **So far graph matching does not beat the current
AKAZE merger on the only true two-robot run.** The results below include the checks that changed
the conclusion along the way.

## Data

| Source | What it is | Used for |
| --- | --- | --- |
| `large2`, `3`, `6`, `7`, `8` (Inky, 2026-11-05) | Five single-robot runs of one area, about 8 x 8 m, 26-37 m2 free | Development, 260 map pairs from 10 run pairs |
| `twobot1` (Inky and Clyde, 2026-11-05) | One two-robot run, 11 minutes, 43 time points | Held-out test |

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

Held-out `twobot1` (`eval_twobot.py`; spacing, restarts and gate frozen on older runs only). Counts
of 43 time points:

| Method | Correct | Wrong | No answer or rejected |
| --- | --- | --- | --- |
| AKAZE, offline | 31 | 0 | 12 |
| FAQ geodesic + clearance, no gate | 29 | 14 | 0 |
| FAQ geodesic + clearance, gated | 21 | 2 | 20 |
| Clearance only, gated | 16 | 1 | 26 |

AKAZE first succeeds at t = 195 s and graph matching at t = 210 s, both at about 86% overlap. These
43 time points are one run and strongly correlated, so treat this as n = 1.

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
5. **Nothing tested works reliably below about 50% overlap**, which is where a better merger is
   needed.

## Open questions and limits

- **Why AKAZE scores 14% on the older pairs and 72% on `twobot1` is unexplained.** Ruled out: input
  topic (`/map` and `map_live` both give 31 of 43) and map rotation (`rotation_sweep.py`: correct at
  all 37 angles). So the development comparison may flatter graph matching.
- Selection by map agreement uses the same kind of score that defines the reference. Inlier-count
  selection, which does not, is lower (38% against 49%).
- 0.25 m spacing was picked after seeing the development pairs. `twobot1` is the only clean test.
- The gate accepted two wrong answers at t = 15 and 30 s, when overlap was 0-5%. It needs a minimum
  overlap condition.
- Partial matching is not implemented yet: every node in the smaller graph is forced to match, which
  is a likely cause of failure at low overlap.
- In `simulate.py`, edge flips can disconnect a tree, so edge noise also shrinks the graph. Those two
  effects are confounded, and some low-overlap cells have few usable trials.
- Implemented with numpy, scipy and OpenCV (central has no pip) rather than graspologic.
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
python3 eval_twobot.py data/bags/twobot1/central
python3 check_discrepancies.py
python3 rotation_sweep.py
```

Bags are not in git (`data/bags/`). Originals are on the robots under `~/turtlebot3/logs/`; the
central bag from `twobot1` is under `~/.ros/nav2_debug_central/`.
