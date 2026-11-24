# ENTC 641 experiment: two-robot exploration DOE

Code for the ENTC 641 final project: a 2^(4-1) fractional factorial experiment
(D = ABC, two replicates, plus centre points) on Inky and Clyde exploring the
three-room area. Proposal: https://claude.ai/artifact/SFeCgkozFEzgUpQeFkTvro

| Factor | -1 | +1 | Set by |
| --- | --- | --- | --- |
| A: `coordination_weight` | 0 | 1.0 | explorer parameter (`EXPLORER_EXTRA_PARAMS`) |
| B: start | same room, ~1 m apart | different rooms | you, when placing the robots |
| C: `potential_scale` : `gain_scale` | 2 : 1 | 1 : 2 | explorer parameters |
| D: max speed | 0.15 m/s | 0.22 m/s | `set_robot_speed.sh` via `FLEET_AFTER_SLAM_HOOK` |

Centre points use A = 0.5, C = 1.5 : 1.5, D = 0.185 m/s. Levels live in
`design.py`; change them only before the first run.

## Files

| File | Does |
| --- | --- |
| `design.py` | factors, levels, design matrix, paths, 480 s cap, 11.6 V start limit |
| `plan.py` | makes `data/plan.csv` (random order per session), shows status, adds confirmation pairs |
| `run_trial.py` | runs one run end to end and saves everything in `data/runs/<id>/` |
| `set_robot_speed.sh` | sets Nav2's top speed on a robot (called by `start_fleet.sh`) |
| `reference.py` | saves the reference map that coverage is measured against |
| `extract.py` | one row of responses per run into `data/responses.csv` |
| `doe.py` | effects, pooled variance, t-tests, model and prediction interval (course method) |
| `check_example20.py` | checks `doe.py` against class Example 20 |
| `analyze.py` | standardized-method analysis of T90 and time to exploration complete into `results/summary.md` |

Two small additions outside this folder: `scripts/core/start_central.sh` takes
`EXPLORER_EXTRA_PARAMS="name:=value ..."` and `scripts/core/start_fleet.sh`
runs `FLEET_AFTER_SLAM_HOOK <robot> <ssh target>` after each robot's Nav2 is up.

## Before the first run

1. Door open. Mark two start spots in the same room (about 1 m apart) and one
   spot in each of two different rooms, so every run starts the same way.
2. Shakedown run with the new coordination code and the speed hook (not analyzed):

   ```bash
   python3 entc641/run_trial.py SHAKE1 --coded 1 1 -1 -1
   ```

   Check `data/runs/SHAKE1/fleet/<stamp>/inky-hook.log` shows the speed set to
   0.15, and that the robots explored normally. The hook has not been tried
   with Nav2 running yet; if it fails, the fleet stops with its error.
3. Reference map: let Inky alone map the whole area (no cap; press Enter when
   done), then save it and look at `data/reference.png`:

   ```bash
   python3 entc641/run_trial.py REF --reference
   source scripts/env/central_env.bash
   python3 entc641/reference.py REF
   ```

## Each run

```bash
python3 entc641/plan.py status          # what is next
python3 entc641/run_trial.py R01        # place robots as printed, press Enter
```

The runner prints the settings and start placement, starts the fleet, records
the bag and Pi monitors, stops 480 s after the first goal, and asks for a note.
It warns if the last run ended below 11.6 V. Run the plan in order: R01-R10 are
session 1, R11-R20 session 2.

A run that went wrong (robot off Wi-Fi, someone moved a robot, battery too low,
Ctrl+C): repeat it with `--redo`. The old attempt is kept as
`R05.discarded-<time>` and never analyzed.

## After runs (any time, also mid-way)

```bash
source scripts/env/central_env.bash
python3 entc641/extract.py      # new runs only; --all redoes every run
python3 entc641/analyze.py
```

Open each `data/runs/<id>/registration.png`: the robots' walls must line up
with the reference. A weak alignment is flagged in the `flags` column.
`results/summary.md` has every table and links the figures.

The analysis follows the course's standardized method for two responses,
T90 and time to exploration complete: input data, main-effect and interaction
plots, calculation matrix and effects, Pareto chart, t-test of each effect from
the pooled variance, model of the significant effects, residual plots, and the
centre runs against the model. Other measures (idle time, separation, goal
success, distance) are only summarized by coordination level.

To see how much one run drives the result, leave it out:

```bash
python3 entc641/analyze.py --exclude R02     # writes results/excluding-R02/
```

## How responses are measured

- **t0** is the explorer's first goal; every response uses t0 to t0 + 480 s.
- **Coverage** is the share of the reference map's known cells that either
  robot has seen, from each robot's own map aligned to the reference (exhaustive
  grid registration, `stat636/registration.py`). The live merged map is not
  used: it counts overlap twice until the maps merge.
- **T90** is when coverage reaches 90%. A run that never does gets 480 s and
  `T90_censored = 1`; `coverage_cap_pct` covers those runs.
- **Goal success** counts SUCCEEDED and ABORTED goals; cancels are the explorer
  retargeting and are left out.
- **Battery drop** compares the median voltage in the first and last 30 s of
  the window, both while driving.
- **merge_time_s** is when the live map -> robot TF settled (a heuristic, as in
  `analyze_fleet_run.py`); treat it as secondary.

Bags are not committed (see `.gitignore`); everything else in `data/` is.
