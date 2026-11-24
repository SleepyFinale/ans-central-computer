#!/usr/bin/env python3
"""Build the ENTC 641 final presentation (12 slides, about 6.5 minutes).

Figures come from results/ (analyze.py) and report/figures (make_figures.py);
numbers are computed from data/responses.csv with doe.py, like the report.

  stat636/.venv/bin/python entc641/report/build_deck.py

Writes entc641/report/ENTC641_Presentation.pptx. Speaker notes are short cues,
not a script (the course asks for a presentation, not a reading).
"""

import csv
import sys
from pathlib import Path

import numpy as np
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Emu, Inches, Pt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
import design  # noqa: E402
import doe  # noqa: E402

OUT = HERE / 'ENTC641_Presentation.pptx'
RES = ROOT / 'results'
FIG = HERE / 'figures'

INK = RGBColor(0x1C, 0x2B, 0x33)      # dominant dark slate
TEAL = RGBColor(0x2A, 0x7F, 0x7A)
AMBER = RGBColor(0xE0, 0x8E, 0x1B)    # the one sharp accent: significant results
CARD = RGBColor(0xEE, 0xF2, 0xF3)
GREY = RGBColor(0x5B, 0x68, 0x70)
LIGHT = RGBColor(0xC4, 0xD0, 0xD4)
MIST = RGBColor(0xD5, 0xE8, 0xE6)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
HEAD, BODY = 'Cambria', 'Calibri'
W, H = 13.333, 7.5
AUTHOR = 'Santhosh Chendil'

prs = Presentation()
prs.slide_width, prs.slide_height = Inches(W), Inches(H)
BLANK = prs.slide_layouts[6]
COLUMNS = list(design.FACTORS) + list(design.INTERACTIONS)


# ---------------------------------------------------------------- helpers

def background(slide, color):
    fill = slide.background.fill
    fill.solid()
    fill.fore_color.rgb = color


def text(slide, x, y, w, h, runs, size=16, color=INK, font=BODY, bold=False, align=PP_ALIGN.LEFT,
         anchor=MSO_ANCHOR.TOP, name=None, italic=False, gap=8):
    """runs: a string, or a list of paragraphs, each a string or a list of (text, {opts})."""
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    if name:
        box.name = name
    tf = box.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = anchor
    paras = runs if isinstance(runs, list) else [runs]
    for k, para in enumerate(paras):
        p = tf.paragraphs[0] if k == 0 else tf.add_paragraph()
        p.alignment = align
        parts = para if isinstance(para, list) else [(para, {})]
        for t, opts in parts:
            r = p.add_run()
            r.text = t
            f = r.font
            f.name = opts.get('font', font)
            f.size = Pt(opts.get('size', size))
            f.bold = opts.get('bold', bold)
            f.italic = opts.get('italic', italic)
            f.color.rgb = opts.get('color', color)
        if k < len(paras) - 1:
            p.space_after = Pt(gap)
    return box


def bullets(slide, x, y, w, h, items, size=16, color=INK, gap=10, name=None):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    if name:
        box.name = name
    tf = box.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    for k, item in enumerate(items):
        p = tf.paragraphs[0] if k == 0 else tf.add_paragraph()
        p.space_after = Pt(gap)
        pPr = p._p.get_or_add_pPr()
        pPr.set('marL', str(Emu(Inches(0.25))))
        pPr.set('indent', str(-Emu(Inches(0.25))))
        pPr.append(pPr.makeelement(qn('a:buChar'), {'char': '•'}))
        parts = item if isinstance(item, list) else [(item, {})]
        for t, opts in parts:
            r = p.add_run()
            r.text = t
            r.font.name = BODY
            r.font.size = Pt(opts.get('size', size))
            r.font.bold = opts.get('bold', False)
            r.font.color.rgb = opts.get('color', color)
    return box


def card(slide, x, y, w, h, fill=CARD, name=None, line=None):
    shp = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    shp.adjustments[0] = 0.07
    shp.fill.solid()
    shp.fill.fore_color.rgb = fill
    if line is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = line
        shp.line.width = Pt(1.25)
    shp.shadow.inherit = False
    if name:
        shp.name = name
    return shp


def badge(slide, x, y, label, fill=TEAL, d=0.5, size=16, name=None):
    c = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(x), Inches(y), Inches(d), Inches(d))
    c.fill.solid()
    c.fill.fore_color.rgb = fill
    c.line.fill.background()
    c.shadow.inherit = False
    if name:
        c.name = name
    tf = c.text_frame
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    p.alignment = PP_ALIGN.CENTER
    r = p.add_run()
    r.text = label
    r.font.name = HEAD
    r.font.size = Pt(size)
    r.font.bold = True
    r.font.color.rgb = WHITE
    return c


def title(slide, t, color=INK):
    text(slide, 0.6, 0.42, W - 1.2, 0.8, t, size=32, color=color, font=HEAD, bold=True, name='Title',
         anchor=MSO_ANCHOR.MIDDLE)


def footer(slide, n, dark=False):
    col = LIGHT if dark else GREY
    text(slide, 0.6, 7.02, 7, 0.3, 'ENTC 641 · Two-robot exploration DOE', size=10, color=col,
         name='Footer')
    text(slide, W - 1.6, 7.02, 1.0, 0.3, str(n), size=10, color=col, align=PP_ALIGN.RIGHT,
         name='Slide number')


def picture(slide, path, x, y, w=None, h=None, name=None, alt=''):
    pic = slide.shapes.add_picture(str(path), Inches(x), Inches(y),
                                   Inches(w) if w else None, Inches(h) if h else None)
    if name:
        pic.name = name
    pic._element.nvPicPr.cNvPr.set('descr', alt)
    return pic


def stat(slide, x, y, w, big, label, color=TEAL, big_size=40, name=None):
    text(slide, x, y, w, 0.75, big.replace('-', '−'), size=big_size, color=color, font=HEAD, bold=True, name=name)
    text(slide, x, y + 0.8, w, 0.8, label.replace('= -', '= −'), size=14, color=GREY)


def notes(slide, cues):
    slide.notes_slide.notes_text_frame.text = '\n'.join(f'• {c}' for c in cues)


def new(n, dark=False):
    s = prs.slides.add_slide(BLANK)
    background(s, INK if dark else WHITE)
    footer(s, n, dark)
    return s


# ---------------------------------------------------------------- numbers

with open(design.RESPONSES_CSV, newline='') as fh:
    ROWS = list(csv.DictReader(fh))


def fit(rows, col):
    fact = [r for r in rows if r['type'] == 'factorial']
    return doe.analyze(doe.group_conditions(fact, design.FACTORS, design.columns, col), COLUMNS)


T90 = fit(ROWS, 'T90_s')
RL = fit(ROWS, 'run_length_s')
T90_X = fit([r for r in ROWS if r['run_id'] != 'R02'], 'T90_s')
RL_X = fit([r for r in ROWS if r['run_id'] != 'R02'], 'run_length_s')
t90_same, t90_diff = (T90.predict(design.columns(0, b, 0, 0)) for b in (-1, 1))
rl_slow, rl_fast = (RL.predict(design.columns(0, 0, 0, d)) for d in (-1, 1))
merge_after = sum(float(r['merge_time_s']) > float(r['T90_s']) for r in ROWS)
merge_after_on = sum(float(r['merge_time_s']) > float(r['T90_s']) for r in ROWS if r['A'] == '1')
n_on = sum(r['A'] == '1' for r in ROWS)

# ================================================================ 1. Title
s = new(1, dark=True)
text(s, 0.8, 1.45, 7.4, 0.4, 'ENTC 641 · Final project', size=16, color=MIST, name='Kicker')
text(s, 0.8, 1.95, 7.4, 1.4, 'Does Coordination Pay Off?', size=44, color=WHITE, font=HEAD, bold=True,
     name='Title')
text(s, 0.8, 3.75, 7.2, 1.2, 'A fractional factorial experiment on two-robot exploration with a '
     'TurtleBot3 fleet', size=22, color=MIST, italic=True, name='Subtitle')
text(s, 0.8, 5.45, 7.2, 0.4, f'{AUTHOR} · Texas A&M University · December 2026', size=14,
     color=LIGHT, name='Byline')
card(s, 8.6, 1.2, 4.1, 4.9, fill=WHITE, name='Map card')
picture(s, ROOT / 'data' / 'reference.png', 8.75, 1.35, w=3.8, name='Reference map',
        alt='Occupancy-grid map of the three-room test area')
text(s, 8.75, 5.3, 3.8, 0.6, 'The three-room test area, 42 m²', size=12, color=GREY,
     align=PP_ALIGN.CENTER)
notes(s, ['Name, project title (0:15)',
          'Question: two robots, new coordination term — does it make exploring faster?'])

# ================================================================ 2. Problem
s = new(2)
title(s, 'Two robots, but not twice as fast')
stat(s, 0.6, 1.6, 3.8, '6–80%', 'of a run a robot sat idle with no goal (pilot runs)', color=AMBER)
stat(s, 4.75, 1.6, 3.8, '76%', 'of the time under 5 m apart after the maps merged (one hallway run)',
     color=AMBER)
stat(s, 8.9, 1.6, 3.8, '950–1,430 s', 'hallway runs, none of which ended on their own', color=AMBER)
card(s, 0.6, 3.95, 12.1, 2.7, name='Question card')
text(s, 0.95, 4.2, 11.4, 0.45, 'The problem', size=20, color=TEAL, font=HEAD, bold=True)
bullets(s, 0.95, 4.8, 11.4, 1.8, [
    'The explorer was tuned by trial and error. Nobody could say which settings matter',
    [('A new coordination term ', {'bold': True}),
     ('(steer a robot away from frontiers the other robot is closer to) had never been tested on the robots', {})],
    [('Goal: ', {'bold': True}), ('find which choices significantly change exploration time, and model it', {})],
], size=17)
notes(s, ['Pilot runs: idle time, crowding, long runs',
          'Tuning by trial and error; new coordination term untested',
          'Goal: which choices matter + predictive model (~0:30)'])

# ================================================================ 3. System
s = new(3)
title(s, 'The system that generates the data')
picture(s, FIG / 'fig_system.png', 0.6, 1.45, w=8.0, name='System diagram',
        alt='Two robots send maps to a central PC, which merges them and sends goals back')
bullets(s, 9.0, 1.7, 3.75, 4.6, [
    [('Each robot ', {'bold': True}), ('builds its own map (SLAM) and drives to goals (Nav2)', {})],
    [('Central PC ', {'bold': True}), ('merges the two maps and picks a frontier goal for each robot', {})],
    [('Frontier: ', {'bold': True}), ('the edge between known and unknown space', {})],
    [('Data: ', {'bold': True}), ('ROS bag, explorer events, robot monitors; ~25 files per run', {})],
], size=16, gap=14)
notes(s, ['Inky and Clyde, TurtleBot3 Burger, Raspberry Pi',
          'Central PC: map merge + frontier explorer',
          'Factors A, C in explorer; D on robots; B is placement (~0:30)'])

# ================================================================ 4. Variables
s = new(4)
title(s, 'Four factors in, two responses out')
factors = [('A', 'Coordination weight', '0 (off)  /  1.0 (on)'),
           ('B', 'Start configuration', 'same room, 1 m apart  /  different rooms'),
           ('C', 'Frontier ranking', 'near first (2 : 1)  /  large first (1 : 2)'),
           ('D', 'Max linear speed', '0.15  /  0.22 m/s')]
text(s, 0.6, 1.4, 6.4, 0.4, 'Factors (inputs), coded −1 / +1', size=18, color=TEAL, font=HEAD, bold=True)
for i, (k, name, lv) in enumerate(factors):
    y = 1.95 + i * 1.12
    card(s, 0.6, y, 6.4, 0.95, name=f'Factor {k}')
    badge(s, 0.8, y + 0.2, k, fill=TEAL, d=0.55, size=18)
    text(s, 1.6, y + 0.13, 5.2, 0.4, name, size=17, bold=True)
    text(s, 1.6, y + 0.5, 5.2, 0.4, lv, size=14, color=GREY)
text(s, 7.6, 1.4, 5.1, 0.4, 'Responses (outputs)', size=18, color=TEAL, font=HEAD, bold=True)
for i, (k, name, d) in enumerate([
        ('Y1', 'T90 (s)', 'time until the robots have seen 90% of a reference map of the area'),
        ('Y2', 'Exploration complete (s)', 'until the explorer finds no reachable frontier')]):
    y = 1.95 + i * 1.5
    card(s, 7.6, y, 5.1, 1.3, fill=MIST, name=f'Response {k}')
    text(s, 7.85, y + 0.15, 4.7, 0.4, f'{k}: {name}', size=17, bold=True)
    text(s, 7.85, y + 0.55, 4.7, 0.7, d, size=14, color=GREY)
text(s, 7.6, 5.15, 5.1, 1.5, [
    [('Held fixed / blocked: ', {'bold': True, 'color': INK}),
     ('battery ≥ 11.6 V at start, door open, marked start spots, each replicate a session', {})],
    [('Described only: ', {'bold': True, 'color': INK}), ('idle time, separation, distance, goal success', {})],
], size=13, color=GREY, gap=6)
notes(s, ['A: the new term. B: where I place them. C: near vs large frontiers. D: top speed',
          'Y1 T90 against a reference map (merged map double counts before merge)',
          'Y2: how long the whole job takes',
          'Nuisance: battery, door, sessions as blocks (~0:40)'])

# ================================================================ 5. Design
s = new(5)
title(s, 'Design: a 2⁴⁻¹ fractional factorial, run twice')
rows = []
for s_ in sorted({tuple(int(float(r[f])) for f in design.FACTORS)
                  for r in ROWS if r['type'] == 'factorial'}, key=lambda v: (v[2], v[1], v[0])):
    rows.append(s_)
tbl = s.shapes.add_table(len(rows) + 1, 5, Inches(0.6), Inches(1.5), Inches(5.6), Inches(4.6)).table
for j, hdr in enumerate(['Cond.', 'A', 'B', 'C', 'D = ABC']):
    tbl.cell(0, j).text = hdr
for i, v in enumerate(rows, start=1):
    tbl.cell(i, 0).text = str(i)
    for j, x in enumerate(v, start=1):
        tbl.cell(i, j).text = f'{x:+d}'
for i in range(len(rows) + 1):
    for j in range(5):
        c = tbl.cell(i, j)
        c.fill.solid()
        c.fill.fore_color.rgb = INK if i == 0 else (CARD if i % 2 else WHITE)
        p = c.text_frame.paragraphs[0]
        p.alignment = PP_ALIGN.CENTER
        for r in p.runs:
            r.font.size = Pt(16)
            r.font.name = BODY
            r.font.bold = i == 0 or j == 4
            r.font.color.rgb = WHITE if i == 0 else (AMBER if j == 4 else INK)
        c.vertical_anchor = MSO_ANCHOR.MIDDLE
for k, (big, small) in enumerate([
        ('8 conditions × 2', 'one replicate per session (block), random order within each'),
        ('+ 4 centre runs', 'A, C, D at mid levels: is a straight line enough?'),
        ('Resolution IV', 'main effects clear; AB = CD, AC = BD, AD = BC'),
        ('20 runs, 480 s cap', 'Nov 19–24; failed attempts redone and kept apart')]):
    y = 1.5 + k * 1.17
    badge(s, 6.75, y + 0.12, str(k + 1), fill=AMBER if k == 0 else TEAL, d=0.5)
    text(s, 7.5, y, 5.2, 0.45, big, size=19, bold=True, font=HEAD)
    text(s, 7.5, y + 0.47, 5.2, 0.6, small, size=14, color=GREY)
notes(s, ['Full 2^4 = 32 runs with replicates: too many batteries',
          'Half fraction, D = ABC (course design table)',
          'Cost: two-factor interactions confounded in pairs',
          'Sessions = blocks; centre runs check curvature (~0:35)'])

# ================================================================ 6. Method
s = new(6)
title(s, 'Method: the standardized method')
steps = [('Plot first', 'main-effect and interaction plots, every run shown'),
         ('Effects', 'calculation matrix: E = Σ(sign · ȳ) / 4'),
         ('t-test', 'pooled variance of replicates (8 dof); significant if t beyond ±2.306'),
         ('Model', 'keep only significant effects: ŷ = ȳ + Σ (E/2)·X'),
         ('Check', 'residual plots, normal plot, centre runs, without the outlier')]
for k, (name, d) in enumerate(steps):
    x = 0.6 + k * 2.45
    card(s, x, 1.6, 2.2, 2.6, name=f'Step {k + 1}')
    badge(s, x + 0.2, 1.8, str(k + 1), fill=TEAL, d=0.5)
    text(s, x + 0.2, 2.45, 1.9, 0.45, name, size=19, bold=True, font=HEAD)
    text(s, x + 0.2, 2.95, 1.9, 1.2, d, size=14, color=GREY)
card(s, 0.6, 4.6, 12.1, 2.05, fill=MIST, name='Why card')
text(s, 0.95, 4.8, 11.4, 0.45, 'Why this is the right tool', size=19, color=TEAL, font=HEAD, bold=True)
bullets(s, 0.95, 5.35, 11.4, 1.3, [
    'Screens four factors and their interactions in 16 runs: each run on real robots costs a battery cycle',
    'Replicates give a pure estimate of run-to-run noise, so each effect gets its own significance test',
], size=16, gap=6)
notes(s, ['Five steps, same for both responses',
          'Code checked against class Example 20 first',
          'Why: few expensive runs, many factors, noisy system (~0:30)'])

# ================================================================ 7. Data first
s = new(7)
title(s, 'Data first: starting apart shows up early')
picture(s, FIG / 'fig_coverage.png', 0.5, 1.4, w=7.6, name='Coverage curves',
        alt='Coverage against time for 20 runs; different-room starts reach 90% sooner')
stat(s, 8.6, 1.6, 4.1, '46% vs 35%', 'coverage at the first goal: different rooms vs same room')
stat(s, 8.6, 3.25, 4.1, '≤ 102 s', 'every different-room run reached 90%; same-room runs took up to 199 s')
text(s, 8.6, 4.95, 4.1, 1.5, 'Flat stretches in same-room runs: one robot driving through space the other '
     'has already seen', size=14, color=GREY, italic=True)
notes(s, ['Every run’s coverage curve, raw data',
          'Orange (different rooms) starts higher, crosses 90% sooner',
          'Blue flat stretches = duplicated work (~0:25)'])

# ================================================================ 8. T90 result
s = new(8)
title(s, 'T90: only the start configuration is significant')
picture(s, RES / 'pareto_T90_s.png', 0.5, 1.45, w=7.4, name='Pareto T90',
        alt='Pareto chart: only B exceeds the significance limit of 31.3 s')
stat(s, 8.4, 1.5, 4.3, f'{T90.effects["B"]:+.0f} s',
     f'effect of starting in different rooms (t = {T90.t["B"]:.2f}, limit ±2.306)', color=AMBER, big_size=48)
card(s, 8.4, 3.35, 4.3, 1.35, fill=MIST, name='Model card')
text(s, 8.65, 3.5, 3.9, 1.1, [
    [('ŷ₁ = 98.0 − 21.1·X', {'bold': True, 'font': HEAD}), ('B', {'bold': True, 'font': HEAD, 'size': 13})],
    f'{t90_same:.0f} s same room → {t90_diff:.0f} s different rooms'], size=18, gap=4)
bullets(s, 8.4, 4.95, 4.3, 1.8, [
    f'Coordination: {T90.effects["A"]:+.1f} s, not significant, wrong direction',
    f'Without outlier R02: B still significant (t = {T90_X.t["B"]:.2f})'.replace('-', '−'),
], size=14, gap=6)
notes(s, ['Pareto: B is the only bar over the limit (31.3 s)',
          'Model: 119 s → 77 s',
          'Coordination +11.5 s — slower, not significant; falls to ~0 without R02 (~0:40)'])

# ================================================================ 9. Completion result
s = new(9)
title(s, 'Completion: only top speed is significant')
picture(s, RES / 'pareto_run_length_s.png', 0.5, 1.45, w=7.4, name='Pareto completion',
        alt='Pareto chart: only D just exceeds the significance limit of 61.2 s')
stat(s, 8.4, 1.5, 4.3, f'{RL.effects["D"]:+.0f} s',
     f'effect of 0.22 vs 0.15 m/s (t = {RL.t["D"]:.2f}, limit ±2.306)', color=AMBER, big_size=48)
card(s, 8.4, 3.35, 4.3, 1.35, fill=MIST, name='Model card')
text(s, 8.65, 3.5, 3.9, 1.1, [
    [('ŷ₂ = 217.8 − 30.7·X', {'bold': True, 'font': HEAD}), ('D', {'bold': True, 'font': HEAD, 'size': 13})],
    f'{rl_slow:.0f} s at 0.15 m/s → {rl_fast:.0f} s at 0.22 m/s'], size=18, gap=4)
bullets(s, 8.4, 4.95, 4.3, 1.8, [
    'Just over the limit (61.5 vs 61.2 s)',
    f'Without R02: t = {RL_X.t["D"]:.2f}, not significant. Needs confirming'.replace('-', '−'),
], size=14, gap=6)
notes(s, ['Only D over the limit, barely',
          '47% faster top speed → 25% shorter: plausible, turning/planning time does not scale',
          'Honest caveat: depends on R02 (~0:30)'])

# ================================================================ 10. Model check
s = new(10)
title(s, 'Checking the models')
picture(s, RES / 'residuals_T90_s.png', 0.5, 1.45, w=8.3, name='Residuals T90',
        alt='Residual plots for the T90 model; one point, R02, at +2.9')
text(s, 0.5, 4.3, 8.3, 0.4, 'T90 model residuals: no trend with run order; R02 at +2.9', size=12,
     color=GREY, align=PP_ALIGN.CENTER)
for k, (head, body) in enumerate([
        ('R02 (T90 +2.9)', 'Robots started together and kept meeting in the doorway. No set-up fault: kept'),
        ('R16 (complete +2.7)', 'Nav2 kept failing goals at the far end; redone R01 shows the same spot'),
        ('Centre runs', 'All 4 below the model, mostly within 1 SD: straight line adequate')]):
    y = 1.5 + k * 1.75
    card(s, 9.2, y, 3.55, 1.55, name=f'Check {k + 1}')
    text(s, 9.4, y + 0.15, 3.15, 0.4, head, size=16, bold=True, color=TEAL if k == 2 else AMBER)
    text(s, 9.4, y + 0.55, 3.15, 0.95, body, size=13, color=GREY)
card(s, 0.5, 4.95, 8.3, 1.7, fill=MIST, name='PI card')
text(s, 0.75, 5.1, 7.9, 1.5, [
    [('95% prediction interval for one new run: ', {'bold': True}),
     ('±66 s (T90), ±130 s (complete)', {})],
    'The models pick the better setting reliably, but a single run is noisy: blocking and failed '
    'goals give a long upper tail'], size=15, gap=6)
notes(s, ['Residuals: no drift across sessions; normal plot fine except R02',
          'R02 and R16 investigated from logs, both real behaviour, kept',
          'Centre runs: no strong curvature', 'Wide prediction intervals (~0:35)'])

# ================================================================ 11. Why coordination failed
s = new(11)
title(s, 'Why coordination did not pay off')
picture(s, FIG / 'fig_merge.png', 0.6, 1.4, w=6.4, name='Merge vs T90',
        alt='Map merge time against T90; most points above the diagonal')
stat(s, 7.6, 1.6, 5.1, f'{merge_after_on} of {n_on}',
     'coordination-on runs where the maps merged only after T90', color=AMBER, big_size=44)
bullets(s, 7.6, 3.35, 5.1, 3.3, [
    'The term needs both robots in one frame, so it is inactive until the maps merge',
    f'Maps merged after a median of 105 s; T90 averaged {T90.average:.0f} s',
    'Goals the other robot was closer to: 30.4% off, 31.9% on, no change',
    'Small area: three rooms leave little work to divide',
], size=16, gap=10)
notes(s, ['Key insight from the descriptive data',
          f'{merge_after} of 20 runs merged after T90',
          'Literature gains are in big spaces; this implementation acts too late here (~0:35)'])

# ================================================================ 12. Conclusions
s = new(12, dark=True)
title(s, 'Conclusions', color=WHITE)
for k, (big, body) in enumerate([
        ('Start apart', f'cuts T90 by {abs(T90.effects["B"]):.0f} s ({t90_same:.0f} → {t90_diff:.0f} s); robust'),
        ('Drive faster', f'cuts completion by {abs(RL.effects["D"]):.0f} s; just significant, confirm it'),
        ('Coordination: no', 'no detectable effect; it acts only after the maps merge')]):
    x = 0.6 + k * 4.1
    card(s, x, 1.55, 3.8, 2.3, fill=RGBColor(0x26, 0x3A, 0x44), name=f'Takeaway {k + 1}')
    text(s, x + 0.25, 1.75, 3.3, 0.6, big, size=24, bold=True, font=HEAD, color=AMBER if k < 2 else MIST)
    text(s, x + 0.25, 2.45, 3.3, 1.3, body, size=15, color=WHITE)
text(s, 0.6, 4.25, 12.1, 0.45, 'How the analysis helped, and what comes next', size=19, color=MIST,
     font=HEAD, bold=True)
bullets(s, 0.6, 4.8, 12.1, 2.0, [
    'Replaced trial-and-error tuning with a tested setting: robots in different rooms at 0.22 m/s',
    'Showed the new term is not worth keeping as is: make it act before the merge (known start offsets)',
    'Next: confirmation runs at the chosen setting; repeat in the larger hallway area',
], size=16, color=WHITE, gap=8)
notes(s, ['Three takeaways', 'How data analysis helped: tested setting, redesign target',
          'Next steps; thank you (~0:40)'])

prs.core_properties.title = 'Does Coordination Pay Off?'
prs.core_properties.author = AUTHOR
prs.save(OUT)
print('wrote', OUT)
