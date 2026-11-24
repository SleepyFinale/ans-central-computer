# ENTC 641 final project: report and presentation plan

Due Wed Dec 2 on Canvas, with the YouTube link on the cover page. Grade: report
35 pts, video 15 pts. The video uses PowerPoint in presenter mode, runs 5-7 min,
and must not be read from a script.

Title (from the proposal): **Does Coordination Pay Off? A Fractional Factorial
Experiment on Two-Robot Exploration with a TurtleBot3 Fleet**

Source material: the proposal doc (https://claude.ai/artifact/SFeCgkozFEzgUpQeFkTvro)
already has the problem statement, the system, the factor, nuisance and response
tables, the pilot data, the design and the run checks. Most of report sections 1-4
can be adapted from it. `results/summary.md` has every number for section 5.

## Scope agreed with the instructor (from the proposal's feedback table)

- Two responses, each with its own model: T90 and time to exploration complete.
- The method is the standardized method with t-tests of the effects. ANOVA,
  separate t-tests, correlation, curve fitting and optimization were **removed**
  and stay out of the report.
- Plot the data before choosing tools.
- The other measures (idle time, separation, distance, goal success) are only
  described, grouped by coordination level.

"Completeness of data analysis" (15 pts) therefore comes from doing every step of
the standardized method fully for both responses, checking the model, and handling
the unusual runs openly. It does not come from adding more tools.

## Headline results

| Question | Answer | Evidence |
| --- | --- | --- |
| What makes 90% coverage faster? | Starting the robots in different rooms (B) | E_B = -42.2 s, t = -3.11 (critical value 2.306) |
| What makes the whole exploration finish sooner? | Higher top speed (D) | E_D = -61.5 s, t = -2.32 |
| Does coordination (A) help? | No detectable effect, and the sign points the wrong way | E_A = +11.5 s (T90), +33.0 s (complete), neither significant |
| Frontier ranking (C)? | No effect | E_C ≈ -2 s for both responses |
| Models | ŷ_T90 = 98.0 - 21.1·X_B; ŷ_complete = 217.8 - 30.7·X_D | residual SD 27.1 s and 52.5 s |
| Is a straight-line model enough? | Mostly | centre runs fall within about one residual SD, except R01 (T90) and R20 (complete) |
| Unusual runs | R02 (T90, +2.9), R16 (complete, +2.7) | both kept as real outcomes; leaving out R02 is in `results/excluding-R02/` |

## Report outline (about 20 pages)

| § | Section | Rubric (pts) | Pages | Content | Figures / tables |
| --- | --- | --- | --- | --- | --- |
| – | Cover | Format (5) | 1 | Title, name, course, date, **YouTube link** | photo of the robots |
| – | Abstract | Format | 0.5 | The question, the design, the two significant effects, coordination's null result | |
| 1 | Introduction | Intro & lit (5) | 1.5 | Why multi-robot exploration matters; the fleet (Inky, Clyde, central PC, SLAM Toolbox, Nav2, frontier explorer, Zenoh); goal: predict exploration time from settings we control | system diagram |
| 2 | Literature review | Intro & lit | 1.5-2 | Frontier exploration (Yamauchi 1997, 1998); coordinated exploration with utility discounting (Burgard et al. 2005); map merging (Birk & Carpin 2006; Hörner 2016, `m-explore`); DOE for robot parameter tuning; the gap: few controlled factorial studies on real robots | |
| 3 | Problem statement and formulation | Problem (5) | 2.5 | Pilot evidence (idle time, crowding, long runs); the system as a box with inputs and outputs; factors and levels with reasons; nuisance factors and how each was handled; the two responses and why a reference map is used | input/output diagram; factor, nuisance and response tables; pilot table; reference map |
| 4 | Experimental design and data collection | Data analysis (15) | 2.5 | 2⁴⁻¹ with D = ABC; alias pairs; 2 replicates as sessions (blocks) + 4 centre runs; randomized order; 480 s cap; run protocol and redo rule; coverage by registration to the reference | design matrix; plan table; one `registration.png`; one run's coverage-vs-time curve |
| 5 | Data analysis | Data analysis | 7-8 | standardized method, both responses (below) | |
| 6 | Discussion | Disc. & concl. (5) | 2 | Why B and D matter and A does not; the confounding caveat (AC = BD at +20.9 s for T90, AD = BC at -48.0 s for completion, both below the critical size); R02 and R16 from their logs; the far end of the area as a recurring Nav2 trouble spot; descriptive secondary measures | secondary-measures table |
| 7 | Conclusions and recommendations | Disc. & concl. | 1 | Start the robots apart and drive faster; redesign coordination before testing it again; the next experiment | |
| – | References and appendix | Format | 2 | All 20 runs; how each response is computed; the check against class Example 20 | |

### §5 Data analysis (all from `analyze.py`, repeated for each response)

1. Input data: the 20-run table.
2. Plots first: main-effect plots and interaction plots (AB, AC, AD) with every
   replicate shown, and what they show before any calculation.
3. Calculation matrix: condition means and variances.
4. Effects, pooled variance (8 dof), effect variance, t-test of each effect,
   Pareto chart.
5. Model with only the significant effects, in coded and in real units.
6. Model check: normalized residuals against fitted value and run order, normal
   probability plot.
7. Centre runs against the model's prediction (is a straight line enough?).
8. Sensitivity: the analysis without R02. It is already generated; say whether B is
   still significant and whether anything else becomes significant.
9. Prediction interval at the recommended setting: `doe.py` has
   `prediction_interval`, but `analyze.py` does not report it yet. One table,
   still within the standardized method.

## Presentation (start from the proposal's timing; about 11 slides)

| Time | Slides | Content |
| --- | --- | --- |
| 0:00-0:15 | 1 | Title, name, robot photo |
| 0:15-1:45 | 2-4 | Robots crowd and sit idle (pilot numbers); the system; factors A-D and the two responses as an input/output diagram |
| 1:45-3:00 | 5-6 | 2⁴⁻¹ design and its alias pairs; sessions as blocks; why the standardized method fits (screens 4 factors in 16 runs, replicates give the pooled variance for the t-tests) |
| 3:00-6:00 | 7-10 | Main-effect plots; T90 Pareto (B); completion Pareto (D); models, residuals, centre runs; R02 and R16 |
| 6:00-7:00 | 11 | Coordination did not pay off here; start the robots apart and at full speed; what to change in the explorer |

Speaker notes as short cues, not sentences, because the course says not to read a script.

## Build approach

- Slides: `report/build_deck.py` builds `report/ENTC641_Presentation.pptx` (12 slides,
  cue-style speaker notes timed to about 6:25):

  ```bash
  stat636/.venv/bin/python entc641/report/build_deck.py
  ```
- Report: Word, built by `report/build_report.py` into `report/ENTC641_Final_Report.docx`.
  Every table is computed from `data/responses.csv`. Rebuild after any data change:

  ```bash
  python3 entc641/analyze.py && python3 entc641/analyze.py --exclude R02
  python3 entc641/report/make_figures.py
  PYTHONPATH=entc641/.pylib stat636/.venv/bin/python entc641/report/build_report.py
  ```

  `.pylib/` holds python-docx (installed with `pip install --target`; Node is not
  on this machine). In Word, update the table of contents with right-click → Update Field.
- New figures (system diagram, input/output diagram, coverage curves, merge time
  against T90) are made by `report/make_figures.py`.
