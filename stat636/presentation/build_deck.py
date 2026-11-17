#!/usr/bin/env python3
"""Build the STAT 636 project presentation (10 slides, 15 minutes).

Run with the project virtualenv:  .venv/bin/python presentation/build_deck.py
Figures come from presentation/make_figures.py; numbers from results/*.json.
"""
import json

from pptx import Presentation
from pptx.chart.data import CategoryChartData
from pptx.dml.color import RGBColor
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION, XL_LEGEND_POSITION, XL_TICK_LABEL_POSITION
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Emu, Inches, Pt

NAVY = RGBColor(0x1F, 0x2A, 0x36)
TEAL = RGBColor(0x0F, 0x7C, 0x80)
CORAL = RGBColor(0xE4, 0x57, 0x2E)
CARD = RGBColor(0xF2, 0xF4, 0xF6)
GREY = RGBColor(0x5B, 0x66, 0x72)
LIGHT = RGBColor(0xC9, 0xCE, 0xD4)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
ICE = RGBColor(0xCF, 0xE8, 0xE9)
HEAD, BODY = 'Cambria', 'Calibri'
FIG = 'presentation/figures'

prs = Presentation()
prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)
BLANK = prs.slide_layouts[6]
W = 13.333


def background(slide, color):
    fill = slide.background.fill
    fill.solid()
    fill.fore_color.rgb = color


def text(slide, x, y, w, h, runs, size=16, color=NAVY, font=BODY, bold=False, align=PP_ALIGN.LEFT,
         anchor=MSO_ANCHOR.TOP, name=None, italic=False):
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
            p.space_after = Pt(8)
    return box


def bullets(slide, x, y, w, h, items, size=16, color=NAVY, gap=10, name=None):
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
        bu = pPr.makeelement('{http://schemas.openxmlformats.org/drawingml/2006/main}buChar', {'char': '•'})
        pPr.append(bu)
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
    shp.adjustments[0] = 0.08
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


def badge(slide, x, y, label, fill=TEAL, d=0.5, size=16):
    c = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(x), Inches(y), Inches(d), Inches(d))
    c.fill.solid()
    c.fill.fore_color.rgb = fill
    c.line.fill.background()
    c.shadow.inherit = False
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


def title(slide, t, color=NAVY):
    text(slide, 0.6, 0.42, W - 1.2, 0.8, t, size=32, color=color, font=HEAD, bold=True, name='Title',
         anchor=MSO_ANCHOR.MIDDLE)


def footer(slide, n, dark=False):
    col = LIGHT if dark else GREY
    text(slide, 0.6, 7.02, 6, 0.3, 'STAT 636 · Map merging by graph matching', size=10, color=col, name='Footer')
    text(slide, W - 1.6, 7.02, 1.0, 0.3, str(n), size=10, color=col, align=PP_ALIGN.RIGHT, name='Slide number')


def picture(slide, path, x, y, w=None, h=None, name=None, alt=''):
    pic = slide.shapes.add_picture(path, Inches(x), Inches(y),
                                   Inches(w) if w else None, Inches(h) if h else None)
    if name:
        pic.name = name
    pic._element.nvPicPr.cNvPr.set('descr', alt)
    return pic


def style_axis_text(axis, size=12, color=GREY):
    axis.tick_labels.font.size = Pt(size)
    axis.tick_labels.font.name = BODY
    axis.tick_labels.font.color.rgb = color


# ---------------------------------------------------------------- numbers
conf = json.load(open('results/confirm_rule.json'))
RUNS = ['twobot1', 'twobot4', 'small1', 'linktest1', 'hall1', 'hall2', 'hall3', 'hall4', 'hall5']
METHODS = [('akaze', 'AKAZE (fleet merger)'), ('geodesic_clearance', 'FAQ: geodesic + clearance'),
           ('clearance_only', 'Clearance only'), ('partial_euclidean', 'Partial matching, Euclidean'),
           ('partial_geodesic', 'Partial matching, geodesic')]
deltas = [conf[r]['geodesic_clearance']['first_right'] - conf[r]['akaze']['first_right'] for r in RUNS]
totals = {m: (sum(conf[r][m]['right'] for r in RUNS), sum(conf[r][m]['wrong'] for r in RUNS),
              sum(conf[r][m]['gross'] for r in RUNS)) for m, _ in METHODS}
points = sum(conf[r]['akaze']['points'] for r in RUNS)

# ================================================================ 1. Title
s = prs.slides.add_slide(BLANK)
background(s, NAVY)
text(s, 0.8, 1.55, 7.2, 0.4, 'STAT 636 · Final project', size=16, color=ICE, name='Kicker')
text(s, 0.8, 2.0, 7.2, 2.1, 'Aligning two robots’ maps with graph matching', size=40, color=WHITE,
     font=HEAD, bold=True, name='Title')
text(s, 0.8, 4.45, 7.2, 1.0, 'When does the network structure of free space help, and what limits it?',
     size=20, color=ICE, italic=True, name='Subtitle')
text(s, 0.8, 5.75, 7.2, 0.4, 'Santhosh Chendil · Texas A&M University · November 2026', size=14,
     color=LIGHT, name='Byline')
card(s, 8.55, 1.45, 4.2, 4.3, fill=WHITE, name='Figure card')
from PIL import Image
im = Image.open(f'{FIG}/fig_graph.png')
im.crop((0, int(im.height * 0.06), int(im.width * 0.355), im.height)).save(f'{FIG}/fig_graph_map.png')
picture(s, f'{FIG}/fig_graph_map.png', 8.7, 1.8, w=3.9, name='Graph thumbnail',
        alt='A map of three rooms with its free-space graph drawn over it')
s.notes_slide.notes_text_frame.text = (
    "Hi, I'm Santhosh. Our project is about a practical problem from our lab's robot fleet: "
    "two robots map the same building separately, and before they can share the work their maps "
    "have to be aligned. We treat that as a network problem: each map becomes a graph of its free "
    "space, and aligning the maps becomes matching the two graphs. The question is when that graph "
    "structure actually helps, compared with the image-based method the fleet uses now, and what "
    "limits it. (about 30 seconds)")

# ================================================================ 2. Problem
s = prs.slides.add_slide(BLANK)
background(s, WHITE)
title(s, 'Two robots, two maps, two coordinate frames')
bullets(s, 0.6, 1.45, 5.6, 2.6, [
    [('Inky and Clyde ', {'bold': True}), ('(TurtleBot3, 2D LiDAR) each build an occupancy-grid map with SLAM, in their own frame', {})],
    [('To share work, ', {'bold': True}), ('a central computer must find the rotation and shift between the maps', {})],
    [('The fleet’s merger ', {'bold': True}), ('matches image features (AKAZE) on the two grids, and often answers late', {})],
], size=16, name='Context')
card(s, 6.6, 1.5, 6.13, 2.35, fill=ICE, name='Question card')
text(s, 6.95, 1.75, 5.5, 0.4, 'Our question', size=14, color=TEAL, bold=True, name='Question label')
text(s, 6.95, 2.2, 5.5, 1.5,
     'Does matching graphs of the free space align the maps sooner than AKAZE, and is it ever wrong?',
     size=20, color=NAVY, font=HEAD, bold=True, name='Question')
picture(s, f'{FIG}/fig_problem.png', 2.72, 4.1, h=2.75, name='Problem figure',
        alt="Inky's map, Clyde's map in a different frame, and the two aligned by a 270 degree rotation")
footer(s, 2)
s.notes_slide.notes_text_frame.text = (
    "Our lab runs TurtleBot3 robots with 2D LiDAR. Each one builds an occupancy-grid map with SLAM, "
    "but each in its own coordinate frame: as you can see, Clyde's map of the same hallways comes out "
    "rotated relative to Inky's. To coordinate, the central computer has to find the rigid motion, a "
    "rotation and a shift, that lines them up; on the right, Clyde's walls in teal sit on Inky's after "
    "a 270-degree rotation. The fleet does this today with AKAZE, which matches image features on the "
    "two grids. It is careful but slow: it often answers only once the maps overlap a lot. So our "
    "question is: if we turn each map into a graph and match the graphs, do we get the alignment "
    "sooner, and is it ever wrong? Both matter, because a wrong alignment sends robots to the wrong "
    "places. (about 1.5 minutes)")

# ================================================================ 3. Data
s = prs.slides.add_slide(BLANK)
background(s, WHITE)
title(s, 'Data: develop on one set, test on another')
cards = [
    ('260', 'development map pairs', 'Five single-robot runs of a three-room area, paired with each other; early snapshots and crops give partial overlap. All settings chosen here.'),
    ('9', 'two-robot test runs', 'Inky and Clyde exploring together: four in the three-room area, five in a larger area of corridors. Scored after the settings were fixed.'),
    (str(points), 'scored time points', 'Every 15 s of each test run, every method aligns the maps the robots held at that moment.'),
]
for k, (big, label, desc) in enumerate(cards):
    x = 0.6 + k * 4.12
    card(s, x, 1.5, 3.85, 3.55, name=f'Data card {k + 1}')
    text(s, x + 0.35, 1.75, 3.2, 1.0, big, size=60, color=TEAL, font=HEAD, bold=True, name=f'Data number {k + 1}')
    text(s, x + 0.35, 2.85, 3.2, 0.4, label, size=17, color=NAVY, bold=True, name=f'Data label {k + 1}')
    text(s, x + 0.35, 3.35, 3.2, 1.6, desc, size=14, color=GREY, name=f'Data text {k + 1}')
card(s, 0.6, 5.3, 12.13, 1.45, fill=ICE, name='Truth card')
text(s, 0.95, 5.47, 11.4, 1.2, [
    [('Truth: ', {'bold': True, 'color': TEAL}), ('a reference alignment per run from an exhaustive search over rotations and shifts of the two final maps, checked by eye and against the live merger (agree within about 0.1 m).', {})],
    [('Scoring: ', {'bold': True, 'color': TEAL}), ('right if Clyde’s nodes land within 0.30 m on average; ', {}), ('grossly wrong', {'bold': True}), (' if 2 m or more off.', {})],
], size=15, color=NAVY, name='Truth text')
footer(s, 3)
s.notes_slide.notes_text_frame.text = (
    "All the data is our own. For development we used five runs of a single robot in a three-room "
    "area and paired them with each other: 260 map pairs, using early snapshots and crops to get "
    "partial overlap. Every setting of the method was chosen on those. Then we ran nine two-robot "
    "runs with Inky and Clyde together, four in the rooms and five in a larger corridor area, and "
    "scored them only after the settings were frozen. Every 15 seconds we take the maps the robots "
    "had at that moment and ask each method for the alignment: 478 time points in total. The truth "
    "is a reference alignment from an exhaustive search over rotations and shifts of the final maps. "
    "We checked it by eye, and where the live merger merged, it agrees to about 10 cm. An answer counts "
    "as right within 30 cm and grossly wrong beyond 2 m. (about 1.5 minutes)")

# ================================================================ 4. Method: map to graph
s = prs.slides.add_slide(BLANK)
background(s, WHITE)
title(s, 'From a map to a graph to a matrix')
picture(s, f'{FIG}/fig_graph.png', 0.97, 1.3, w=11.4, name='Graph figure',
        alt='A map with its skeleton graph, the sparse adjacency matrix, and the dense geodesic kernel matrix')
card(s, 0.6, 5.45, 5.9, 1.35, name='Kernel card')
text(s, 0.9, 5.58, 5.4, 1.15, [
    [('Geodesic kernel  ', {'bold': True, 'color': TEAL}), ('K', {'font': HEAD, 'italic': True}), ('ij', {'size': 11}),
     (' = exp(−d', {'font': HEAD}), ('G', {'size': 11, 'font': HEAD}), ('(i,j)² / 2σ²)', {'font': HEAD})],
    [('d', {'font': HEAD, 'italic': True}), ('G', {'size': 11}), (' = travel (shortest-path) distance; σ = 1 m', {'color': GREY})],
], size=15, name='Kernel formula')
card(s, 6.83, 5.45, 5.9, 1.35, fill=ICE, name='Latent card')
text(s, 7.13, 5.58, 5.4, 1.15, [
    [('Latent space view: ', {'bold': True, 'color': TEAL}), ('node positions are the latent positions.', {})],
    [('Distances do not change under rotation, so K is the same in both robots’ frames.', {'color': GREY})],
], size=14, name='Latent text')
footer(s, 4)
s.notes_slide.notes_text_frame.text = (
    "Here is the network part. We thin the free space of a map down to its skeleton, the line "
    "equally far from the walls, and put a node every 25 cm, at junctions and at dead ends. Each node "
    "also gets its clearance, its distance to the nearest wall. That gives a graph like the one on the "
    "left. Its adjacency matrix, in the middle, is almost empty: these graphs are sparse, nearly trees, "
    "with a mean degree around two. So instead we use travel distances, the shortest-path length "
    "between every pair of nodes, and turn them into a Gaussian kernel, on the right, which is dense. "
    "The reason this works comes from the latent space models in class: you can read the nodes' real "
    "positions as latent positions, and the kernel is roughly a function of the distance between them. "
    "Distances do not change when you rotate a map, so K is the same matrix in both robots' frames. "
    "The unknown rotation simply drops out. (about 1.5 minutes)")

# ================================================================ 5. Method: matching and decision
s = prs.slides.add_slide(BLANK)
background(s, WHITE)
title(s, 'Match, fit, then decide whether to trust it')
card(s, 0.6, 1.4, 12.13, 1.3, fill=ICE, name='QAP card')
text(s, 0.95, 1.55, 11.4, 1.05, [
    [('Graph matching (a quadratic assignment problem):   ', {'bold': True, 'color': TEAL, 'size': 16}),
     ('max', {'font': HEAD}), (' Q', {'font': HEAD, 'italic': True, 'size': 12}),
     ('  tr(K', {'font': HEAD}), ('A', {'font': HEAD, 'size': 12}), (' Q K', {'font': HEAD}), ('B', {'font': HEAD, 'size': 12}),
     (' Qᵀ) + λ tr(SᵀQ)', {'font': HEAD})],
    [('Q is a permutation matrix (node i → node π(i)); S rewards similar clearance. NP-hard, so FAQ relaxes Q to doubly stochastic matrices and climbs with Frank–Wolfe steps.', {'color': GREY, 'size': 15})],
], size=20, name='QAP text')
steps = [('1', 'Build graphs', 'Skeleton, nodes every 0.25 m, kernel K and clearance'),
         ('2', 'Match nodes', 'FAQ, 8 restarts; or partial (spectral) matching'),
         ('3', 'Fit the motion', 'Procrustes on matched pairs, made robust with RANSAC'),
         ('4', 'Accept or reject', 'Few map conflicts and ≥ 3 m² of shared space'),
         ('5', 'Confirm over time', 'Use only after 2 agreeing answers (merger’s rule)')]
for k, (n, head, desc) in enumerate(steps):
    x = 0.6 + k * 2.47
    card(s, x, 3.0, 2.2, 2.7, name=f'Step card {n}')
    badge(s, x + 0.2, 3.25, n)
    text(s, x + 0.2, 3.85, 1.85, 0.65, head, size=15, bold=True, name=f'Step title {n}')
    text(s, x + 0.2, 4.55, 1.85, 1.1, desc, size=13, color=GREY, name=f'Step text {n}')
    if k < len(steps) - 1:
        a = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(x + 2.22), Inches(4.12), Inches(0.23), Inches(0.3))
        a.fill.solid(); a.fill.fore_color.rgb = LIGHT; a.line.fill.background(); a.shadow.inherit = False
        a.name = f'Arrow {n}'
text(s, 0.6, 5.95, 12.13, 0.9, [
    [('Compared against: ', {'bold': True}), ('AKAZE with the same confirmation rule (the baseline), and clearance only, with no edges (does the graph structure add anything?).', {'color': GREY})],
], size=15, name='Comparison note')
footer(s, 5)
s.notes_slide.notes_text_frame.text = (
    "Once each map is a graph, the question becomes: which node in Clyde's graph is the same place as "
    "which node in Inky's? That is graph matching. The formula at the top just says: pair up the nodes "
    "so that distances between nodes, and how far each node is from a wall, agree as well as possible "
    "in both graphs. Checking every possible pairing would take far too long, so we use a standard "
    "approximation called FAQ that finds a good pairing quickly. "
    "The whole method is these five steps. One, turn each map into a graph. Two, match the nodes. "
    "Three, use the matched nodes to work out how to rotate and shift Clyde's map onto Inky's; some "
    "matches will be wrong, so this step ignores the ones that don't fit. Four, a sanity check: if the "
    "aligned maps contradict each other, say a wall landing on open floor, we reject the answer. Five, "
    "we only trust an answer once we get the same one twice in a row, the same rule the fleet already "
    "uses. To make it a fair test, AKAZE goes through the same last step, and we also run a version "
    "that uses only each node's distance to the wall, with no graph connections, to see whether the "
    "graph structure itself is helping. (about 1.5 minutes)")

# ================================================================ 6. Why it works and when it cannot
s = prs.slides.add_slide(BLANK)
background(s, WHITE)
title(s, 'Matching needs a matrix with no symmetries')
text(s, 0.6, 1.4, 6.1, 1.5, [
    [('For an exact copy, by Cauchy–Schwarz:', {'color': GREY})],
    [('tr(K', {'font': HEAD}), ('A', {'font': HEAD, 'size': 13}), (' Q K', {'font': HEAD}), ('B', {'font': HEAD, 'size': 13}),
     (' Qᵀ) ≤ ‖K', {'font': HEAD}), ('A', {'font': HEAD, 'size': 13}), ('‖²', {'font': HEAD})],
    [('Equality whenever Q is the true matching or an ', {}), ('automorphism', {'bold': True, 'color': TEAL}),
     (': a relabelling that leaves the matrix unchanged. Near-symmetries let noise pick the wrong one.', {})],
], size=17, name='Identifiability text')
stats = [('13–17%', 'of nodes matched using adjacency, even on an exact copy', CORAL),
         ('100%', 'matched using the geodesic kernel on the same copy', TEAL),
         ('62%', 'of nodes have a counterpart in another run’s graph', NAVY)]
for k, (big, desc, col) in enumerate(stats):
    y = 1.4 + k * 1.75
    card(s, 7.1, y, 5.63, 1.5, name=f'Stat card {k + 1}')
    text(s, 7.4, y + 0.2, 2.2, 1.1, big, size=36, color=col, font=HEAD, bold=True, anchor=MSO_ANCHOR.MIDDLE,
         name=f'Stat number {k + 1}')
    text(s, 9.65, y + 0.22, 2.85, 1.1, desc, size=15, color=NAVY, anchor=MSO_ANCHOR.MIDDLE, name=f'Stat text {k + 1}')
bullets(s, 0.6, 3.75, 6.1, 3.1, [
    [('Adjacency: ', {'bold': True}), ('long chains of degree-2 nodes read the same forwards and backwards: many symmetries', {})],
    [('Kernel: ', {'bold': True}), ('distances fix the point layout up to a rigid motion, so a generic layout has none', {})],
    [('Corridors: ', {'bold': True}), ('a loop that looks alike turned 90° is a near-symmetry', {})],
    [('Simulation: ', {'bold': True}), ('dropping 38% of nodes cuts recovery from 100% to 8%', {})],
], size=15, name='Why bullets')
footer(s, 6)
s.notes_slide.notes_text_frame.text = (
    "So when should matching work, and when can't it? Think of both robots' graphs as two imperfect copies "
    "of the same building. The key idea is symmetry. If a graph looks exactly the same after you swap some "
    "of its nodes around, the matching has no way to tell the right pairing from the swapped one: both "
    "score equally well. The formula on the left is the math behind that. So matching only works when the "
    "graph has no symmetries like that. "
    "That explains what we saw. Using only which nodes are connected doesn't work: these graphs are mostly "
    "long chains, and a chain looks the same read in either direction. Even on an exact copy of a graph, that "
    "version matched only 13 to 17 percent of nodes. Using the distances between nodes does work: the "
    "distances pin down the whole shape, so it matched 100 percent. But corridors are risky: a loop of "
    "hallways can look almost the same rotated by 90 degrees, which is nearly a symmetry. "
    "The other limit is that the two robots don't build quite the same graph from the same place: only "
    "62 percent of nodes show up in both. In simulation, losing that many nodes drops matching from 100 "
    "percent to 8. (about 2 minutes)")

# ================================================================ 7. Result: timing
s = prs.slides.add_slide(BLANK)
background(s, WHITE)
title(s, 'Graph matching was right first in 7 of 9 runs')
cd = CategoryChartData()
cd.categories = RUNS
cd.add_series('FAQ minus AKAZE (s)', deltas)
gf = s.shapes.add_chart(XL_CHART_TYPE.BAR_CLUSTERED, Inches(0.6), Inches(1.35), Inches(7.9), Inches(5.45), cd)
gf.name = 'Timing chart'
ch = gf.chart
ch.has_legend = False
ch.has_title = True
ch.chart_title.text_frame.text = 'First correct alignment: graph method minus AKAZE (s)'
tp = ch.chart_title.text_frame.paragraphs[0]
tp.runs[0].font.size = Pt(14); tp.runs[0].font.name = BODY; tp.runs[0].font.bold = False
tp.runs[0].font.color.rgb = GREY
plot = ch.plots[0]
plot.gap_width = 45
plot.has_data_labels = True
dl = plot.data_labels
dl.number_format = '+0" s";−0" s"'
dl.number_format_is_linked = False
dl.position = XL_LABEL_POSITION.OUTSIDE_END
dl.font.size = Pt(12); dl.font.name = BODY; dl.font.color.rgb = NAVY
ser = plot.series[0]
for i, (run, d) in enumerate(zip(RUNS, deltas)):
    pt = ser.points[i]
    pt.format.fill.solid()
    if conf[run]['geodesic_clearance']['gross'] > 0:
        pt.format.fill.fore_color.rgb = CORAL
    elif d < 0:
        pt.format.fill.fore_color.rgb = TEAL
    else:
        pt.format.fill.fore_color.rgb = LIGHT
cat = ch.category_axis
cat.reverse_order = True
cat.tick_label_position = XL_TICK_LABEL_POSITION.LOW
cat.format.line.color.rgb = LIGHT
style_axis_text(cat, 13, NAVY)
val = ch.value_axis
val.minimum_scale, val.maximum_scale, val.major_unit = -90, 45, 30
val.has_major_gridlines = True
val.major_gridlines.format.line.color.rgb = RGBColor(0xE6, 0xE9, 0xEC)
val.format.line.fill.background()
val.tick_labels.number_format = '+0;−0;0'
val.tick_labels.number_format_is_linked = False
style_axis_text(val, 12)
card(s, 8.9, 1.45, 3.83, 1.5, name='Median card')
text(s, 9.2, 1.62, 3.3, 0.7, '60 s', size=36, color=TEAL, font=HEAD, bold=True, name='Median number')
text(s, 9.2, 2.35, 3.3, 0.5, 'median head start over AKAZE', size=14, color=NAVY, name='Median label')
card(s, 8.9, 3.15, 3.83, 1.5, name='Sign test card')
text(s, 9.2, 3.32, 3.3, 0.7, 'p ≈ 0.09', size=32, color=NAVY, font=HEAD, bold=True, name='Sign test number')
text(s, 9.2, 4.02, 3.3, 0.55, 'sign test, 7 of 9 runs (one-sided): consistent, not significant', size=13, color=GREY,
     name='Sign test label')
card(s, 8.9, 4.85, 3.83, 1.95, fill=ICE, name='Key card')
text(s, 9.2, 5.0, 3.3, 1.7, [
    [('■ ', {'color': TEAL}), ('earlier, never wrong first', {})],
    [('■ ', {'color': CORAL}), ('hall2: earlier, but only after holding a wrong alignment for ~9 min', {})],
    [('■ ', {'color': LIGHT}), ('AKAZE earlier', {})],
], size=13, color=NAVY, name='Chart key')
footer(s, 7)
s.notes_slide.notes_text_frame.text = (
    "First result: speed. Each bar is one test run: when the graph method first held a correct alignment, "
    "minus when AKAZE did. Negative means the graph method was first. It was first in 7 of 9 runs, by a "
    "median of 60 seconds, in the rooms and in the corridors. In two runs AKAZE was first, by 15 and 30 "
    "seconds. One bar is coral: in hall2 the graph method was technically first, but only after holding a "
    "wrong alignment for about nine minutes. I'll come back to that. Statistically: the runs are the unit, "
    "not the 478 time points, because consecutive time points see almost the same maps. A sign test on "
    "7 of 9 gives a one-sided p of about 0.09: a consistent advantage, but nine runs are not enough to call "
    "it significant. (about 1.5 minutes)")

# ================================================================ 8. Result: right versus wrong
s = prs.slides.add_slide(BLANK)
background(s, WHITE)
title(s, 'Only the graph methods were ever wrong')
cd = CategoryChartData()
cd.categories = [label for _, label in METHODS]
cd.add_series('Held right', [totals[m][0] for m, _ in METHODS])
cd.add_series('Held wrong', [totals[m][1] for m, _ in METHODS])
gf = s.shapes.add_chart(XL_CHART_TYPE.BAR_CLUSTERED, Inches(0.6), Inches(1.35), Inches(7.9), Inches(5.45), cd)
gf.name = 'Right and wrong chart'
ch = gf.chart
ch.has_title = True
ch.chart_title.text_frame.text = f'Time points holding a right or wrong alignment ({points} in all)'
tp = ch.chart_title.text_frame.paragraphs[0]
tp.runs[0].font.size = Pt(14); tp.runs[0].font.name = BODY; tp.runs[0].font.bold = False
tp.runs[0].font.color.rgb = GREY
ch.has_legend = True
ch.legend.position = XL_LEGEND_POSITION.BOTTOM
ch.legend.include_in_layout = False
ch.legend.font.size = Pt(12); ch.legend.font.name = BODY; ch.legend.font.color.rgb = NAVY
plot = ch.plots[0]
plot.gap_width = 60
plot.overlap = -10
plot.has_data_labels = True
dl = plot.data_labels
dl.position = XL_LABEL_POSITION.OUTSIDE_END
dl.font.size = Pt(12); dl.font.name = BODY; dl.font.color.rgb = NAVY
for ser, col in zip(plot.series, (TEAL, CORAL)):
    ser.format.fill.solid()
    ser.format.fill.fore_color.rgb = col
cat = ch.category_axis
cat.reverse_order = True
cat.format.line.color.rgb = LIGHT
style_axis_text(cat, 13, NAVY)
val = ch.value_axis
val.minimum_scale, val.maximum_scale, val.major_unit = 0, 350, 100
val.has_major_gridlines = True
val.major_gridlines.format.line.color.rgb = RGBColor(0xE6, 0xE9, 0xEC)
val.format.line.fill.background()
style_axis_text(val, 12)
callouts = [('0', 'wrong alignments held by any method in the four three-room runs', TEAL),
            ('0 of 9', 'runs where AKAZE was wrong (95% upper bound: 28% of runs)', NAVY),
            ('35', 'time points 2 m or more off for FAQ, all in one hallway run', CORAL)]
for k, (big, desc, col) in enumerate(callouts):
    y = 1.45 + k * 1.8
    card(s, 8.9, y, 3.83, 1.6, name=f'Callout card {k + 1}')
    text(s, 9.2, y + 0.15, 3.3, 0.7, big, size=32, color=col, font=HEAD, bold=True, name=f'Callout number {k + 1}')
    text(s, 9.2, y + 0.82, 3.3, 0.75, desc, size=13, color=NAVY, name=f'Callout text {k + 1}')
footer(s, 8)
s.notes_slide.notes_text_frame.text = (
    "Second result: correctness. Across all nine runs, this counts the time points at which each method was "
    "holding a right alignment, in teal, or a wrong one, in coral. FAQ held a right alignment slightly more "
    "often than AKAZE, 310 against 297, because it answers sooner. But AKAZE was never wrong in any run, and "
    "the graph methods were. Clearance alone and the two partial-matching variants were worse than FAQ, so "
    "the graph structure does add something over a node feature, though on the development data only about "
    "5 to 9 percentage points. Two things on the right. In the three-room area, no method was ever wrong. "
    "And AKAZE's zero out of nine is not proof of a zero rate: with nine runs the exact upper bound is about "
    "28 percent of runs. All of FAQ's grossly wrong time points come from one hallway run, hall2. "
    "(about 1.5 minutes)")

# ================================================================ 9. Why corridors fail
s = prs.slides.add_slide(BLANK)
background(s, WHITE)
title(s, 'In corridors the wrong answer repeats itself')
picture(s, f'{FIG}/fig_hall2.png', 0.6, 1.4, w=7.45, name='hall2 figure',
        alt='Clyde’s map placed about 90 degrees off along Inky’s corridor, next to the correct placement')
text(s, 0.6, 5.5, 7.45, 0.4, 'hall2, 120 s into the run: the two maps overlap only about 7%', size=13, color=GREY, name='hall2 caption')
card(s, 8.35, 1.35, 4.38, 2.2, fill=ICE, name='Rule card')
text(s, 8.65, 1.5, 3.85, 2.3, [
    [('The confirmation rule assumes errors are independent:', {'bold': True, 'color': TEAL})],
    [('P(commit to a wrong alignment) ≈ q²π  ≪  q', {'font': HEAD, 'size': 17})],
    [('q = chance a wrong answer is accepted, π = chance two wrong answers agree', {'color': GREY, 'size': 13})],
], size=15, name='Rule text')
bullets(s, 8.35, 3.85, 4.38, 2.7, [
    [('Random errors: ', {'bold': True}), ('removed. FAQ was wrong in 7 of 9 runs without the rule, 3 with it', {})],
    [('Systematic errors: ', {'bold': True}), ('the same near-symmetry wins every time, so π ≈ 1 and the rule cannot catch it', {})],
], size=14, name='Rule bullets')
footer(s, 9)
s.notes_slide.notes_text_frame.text = (
    "So what happened in hall2? On the left is Clyde's map at 120 seconds, placed by the graph method in coral, "
    "and on the right the correct placement in teal. Clyde had mapped one corridor with a side opening. The graph "
    "method laid it along Inky's corridor turned 90 degrees, where it fits almost as well, while the truth is that "
    "Clyde's corridor continues Inky's. At that point the maps overlapped only about 7 percent. That is the "
    "near-symmetry from slide 6. Why did the confirmation rule not catch it? The rule only uses an answer after two "
    "in a row agree. If wrong answers are independent and scattered, committing to one needs two of them to agree, "
    "probability about q squared times pi, which is tiny. That works: without the rule FAQ accepted wrong answers in "
    "7 of 9 runs, with it in 3. But here successive answers come from nearly the same maps, so the same wrong "
    "placement wins every time: pi is close to one, and the rule offers no protection. Once committed, it is held "
    "until three answers agree on something else, so it lasted about nine minutes. (about 1.5 minutes)")

# ================================================================ 10. Discussion
s = prs.slides.add_slide(BLANK)
background(s, NAVY)
title(s, 'What we learned, and what remains open', color=WHITE)
text(s, 0.6, 1.4, 5.6, 0.45, 'Answer to our question', size=16, color=ICE, bold=True, name='Answer label')
bullets(s, 0.6, 1.95, 5.6, 4.6, [
    [('Faster where the space is distinctive: ', {'bold': True, 'color': WHITE}), ('first in 7 of 9 runs, never wrong in the rooms', {'color': LIGHT})],
    [('Fooled where it repeats: ', {'bold': True, 'color': WHITE}), ('corridor symmetries give consistent wrong matches that time cannot filter out', {'color': LIGHT})],
    [('The useful structure is distance, not adjacency; ', {'bold': True, 'color': WHITE}), ('the limit is how repeatably the graph is built', {'color': LIGHT})],
], size=15, color=WHITE, gap=14, name='Answer bullets')
opens = [('Corridor ambiguity', 'Let the graph method propose early and AKAZE (or a second check) confirm before use'),
         ('Unstable graphs', 'Build graphs that repeat across runs: junction-only nodes, stable sampling along the skeleton'),
         ('Too few runs', 'More runs and buildings; p ≈ 0.09 and a 28% bound call for more power'),
         ('Estimated truth', 'Measured start poses for a reference that does not come from the maps themselves')]
for k, (head, desc) in enumerate(opens):
    y = 1.4 + k * 1.33
    card(s, 6.6, y, 6.13, 1.17, fill=RGBColor(0x2A, 0x38, 0x47), name=f'Open card {k + 1}')
    badge(s, 6.82, y + 0.3, str(k + 1), fill=TEAL, d=0.5)
    text(s, 7.55, y + 0.12, 5.0, 0.4, head, size=15, color=WHITE, bold=True, name=f'Open title {k + 1}')
    text(s, 7.55, y + 0.5, 5.0, 0.62, desc, size=13, color=LIGHT, name=f'Open text {k + 1}')
footer(s, 10, dark=True)
s.notes_slide.notes_text_frame.text = (
    "To answer our question: graph structure helps where the space is distinctive. The graph method was first in "
    "7 of 9 runs and never wrong in the rooms. It misleads where the space repeats: corridor symmetries produce "
    "consistent wrong matches that confirming over time cannot filter out. And on the network side, the useful "
    "structure was travel distance between nodes, not adjacency, and the real limit is how repeatably we build the "
    "graph from a map. What remains open, and how we would address it: first, the corridor ambiguity. The natural "
    "fix is to let the graph method propose an alignment early and use it only once AKAZE or another independent "
    "check agrees, combining the speed of one with the safety of the other. Second, graph construction: nodes that "
    "repeat across runs, for example junctions only. Third, statistical power: nine runs give a p of 0.09 and a 28 "
    "percent bound, so more runs and more buildings. And fourth, a measured ground truth, from surveyed start poses, "
    "instead of one estimated from the maps. Thank you, happy to take questions. (about 1.5 minutes)")

prs.save('presentation/STAT636_map_matching.pptx')
print('saved presentation/STAT636_map_matching.pptx')
