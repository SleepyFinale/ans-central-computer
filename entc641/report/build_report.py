"""Build the ENTC 641 final report as a Word file.

Every table is computed here from data/responses.csv with the same doe.py code as
analyze.py; figures come from results/ (analyze.py) and report/figures
(make_figures.py). Run those two first.

  PYTHONPATH=entc641/.pylib stat636/.venv/bin/python entc641/report/build_report.py

Writes entc641/report/ENTC641_Final_Report.docx.
"""

import csv
import re
import sys
from itertools import product
from pathlib import Path

import numpy as np
from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
import design  # noqa: E402
import doe  # noqa: E402

OUT = HERE / 'ENTC641_Final_Report.docx'
RES = ROOT / 'results'
FIG = HERE / 'figures'

AUTHOR = 'Santhosh Chendil'
TITLE = 'Does Coordination Pay Off?'
SUBTITLE = 'A Fractional Factorial Experiment on Two-Robot Exploration with a TurtleBot3 Fleet'
COURSE = 'ENTC 641: Data Analysis, Simulation & Experimental Methods for Industry'
DATE = 'December 2, 2026'
VIDEO = '[YouTube link: add before submitting]'

NAVY = RGBColor(0x1F, 0x3A, 0x5F)
COLUMNS = list(design.FACTORS) + list(design.INTERACTIONS)
LABEL = {c: (f'{c}={design.INTERACTIONS[c]}' if c in design.INTERACTIONS else c) for c in COLUMNS}
LEVEL = {
    'A': {-1: 'off (0)', 0: '0.5', 1: 'on (1.0)'},
    'B': {-1: 'same room', 1: 'different rooms'},
    'C': {-1: '2 : 1', 0: '1.5 : 1.5', 1: '1 : 2'},
    'D': {-1: '0.15 m/s', 0: '0.185 m/s', 1: '0.22 m/s'},
}
RESPONSES = [('T90_s', 'Time to 90% coverage (T90)'),
             ('run_length_s', 'Time to exploration complete')]


# ---------------------------------------------------------------- data

def load(exclude=()):
    with open(design.RESPONSES_CSV, newline='') as fh:
        rows = [r for r in csv.DictReader(fh) if r['run_id'] not in exclude]
    return rows


def coded(r):
    return tuple(int(float(r[f])) for f in design.FACTORS)


def analyze(rows, col):
    fact = [r for r in rows if r['type'] == 'factorial']
    conds = doe.group_conditions(fact, design.FACTORS, design.columns, col)
    return conds, doe.analyze(conds, COLUMNS)


def resid_sd(rows, res, col):
    fact = [r for r in rows if r['type'] == 'factorial']
    y = np.array([float(r[col]) for r in fact])
    fit = np.array([res.predict(design.columns(*coded(r))) for r in fact])
    return float((y - fit).std(ddof=1))


# ---------------------------------------------------------------- docx helpers

class Report:
    def __init__(self):
        self.doc = Document()
        self.fig_n = 0
        self.tab_n = 0
        self._styles()

    def _styles(self):
        d = self.doc
        sec = d.sections[0]
        sec.page_width, sec.page_height = Inches(8.5), Inches(11)
        for side in ('left_margin', 'right_margin', 'top_margin', 'bottom_margin'):
            setattr(sec, side, Inches(1))
        st = d.styles['Normal']
        st.font.name = 'Calibri'
        st.element.rPr.rFonts.set(qn('w:eastAsia'), 'Calibri')
        st.font.size = Pt(11)
        st.paragraph_format.space_after = Pt(6)
        st.paragraph_format.line_spacing = 1.15
        for name, size, before in (('Heading 1', 16, 18), ('Heading 2', 13, 12), ('Heading 3', 11.5, 10)):
            h = d.styles[name]
            h.font.name = 'Calibri'
            rf = h.element.rPr.find(qn('w:rFonts'))
            if rf is None:
                rf = OxmlElement('w:rFonts')
                h.element.rPr.append(rf)
            for a in ('w:ascii', 'w:hAnsi', 'w:cs', 'w:eastAsia'):
                rf.set(qn(a), 'Calibri')
            for a in ('w:asciiTheme', 'w:hAnsiTheme', 'w:cstheme', 'w:eastAsiaTheme'):
                if rf.get(qn(a)) is not None:
                    del rf.attrib[qn(a)]
            h.font.size = Pt(size)
            h.font.bold = True
            h.font.italic = False
            h.font.color.rgb = NAVY
            h.paragraph_format.space_before = Pt(before)
            h.paragraph_format.space_after = Pt(6)
            h.paragraph_format.keep_with_next = True

    # text with **bold** and *italic*
    def _runs(self, p, text, size=None, italic=False):
        for part in re.split(r'(\*\*[^*]+\*\*|\*[^*]+\*)', text):
            if not part:
                continue
            if part.startswith('**'):
                run = p.add_run(part[2:-2])
                run.bold = True
            elif part.startswith('*'):
                run = p.add_run(part[1:-1])
                run.italic = True
            else:
                run = p.add_run(part)
            if italic:
                run.italic = True
            if size:
                run.font.size = Pt(size)
        return p

    def h1(self, text):
        self.doc.add_heading(text, level=1)

    def h2(self, text):
        self.doc.add_heading(text, level=2)

    def h3(self, text):
        self.doc.add_heading(text, level=3)

    def p(self, text, align=None, size=None, italic=False, after=None):
        para = self._runs(self.doc.add_paragraph(), text, size=size, italic=italic)
        para.paragraph_format.alignment = align if align is not None else WD_ALIGN_PARAGRAPH.JUSTIFY
        if after is not None:
            para.paragraph_format.space_after = Pt(after)
        return para

    def eq(self, text):
        para = self.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        run = para.add_run(text)
        run.font.name = 'Cambria Math'
        run.font.size = Pt(11)
        return para

    def bullets(self, items, style='List Bullet'):
        for it in items:
            para = self._runs(self.doc.add_paragraph(style=style), it)
            para.paragraph_format.space_after = Pt(3)

    def numbered(self, items):
        self.bullets(items, style='List Number')

    def figure(self, path, caption, width=6.3):
        self.fig_n += 1
        para = self.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        para.paragraph_format.keep_with_next = True
        para.add_run().add_picture(str(path), width=Inches(width))
        cap = self.doc.add_paragraph()
        cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = cap.add_run(f'Figure {self.fig_n}. ')
        r.bold = True
        r.font.size = Pt(9.5)
        self._runs(cap, caption, size=9.5)
        cap.paragraph_format.space_after = Pt(10)
        return self.fig_n

    def table(self, header, rows, caption, widths=None, size=9, align_first_left=True, label=None):
        if label is None:
            self.tab_n += 1
            label = str(self.tab_n)
        cap = self.doc.add_paragraph()
        cap.paragraph_format.keep_with_next = True
        cap.paragraph_format.space_before = Pt(6)
        r = cap.add_run(f'Table {label}. ')
        r.bold = True
        r.font.size = Pt(9.5)
        self._runs(cap, caption, size=9.5)
        t = self.doc.add_table(rows=1 + len(rows), cols=len(header))
        t.style = 'Table Grid'
        t.alignment = WD_TABLE_ALIGNMENT.CENTER
        for i, row in enumerate([header] + rows):
            for j, val in enumerate(row):
                cell = t.cell(i, j)
                cell.text = ''
                para = cell.paragraphs[0]
                para.paragraph_format.space_after = Pt(0)
                para.paragraph_format.line_spacing = 1.0
                para.alignment = (WD_ALIGN_PARAGRAPH.LEFT if (j == 0 and align_first_left)
                                  else WD_ALIGN_PARAGRAPH.CENTER)
                self._runs(para, str(val), size=size)
                if i == 0:
                    for run in para.runs:
                        run.bold = True
                        run.font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF)
                    shade(cell, '1F3A5F')
                elif i % 2 == 0:
                    shade(cell, 'EEF3FA')
        if widths:
            t.autofit = False
            tblPr = t._tbl.tblPr
            layout = OxmlElement('w:tblLayout')
            layout.set(qn('w:type'), 'fixed')
            tblPr.append(layout)
            for j, (gc, w) in enumerate(zip(t._tbl.tblGrid.findall(qn('w:gridCol')), widths)):
                gc.set(qn('w:w'), str(int(w * 1440)))
                for cell in t.columns[j].cells:
                    cell.width = Inches(w)
        for row in t.rows:          # keep a row on one page, the header with the body
            trPr = row._tr.get_or_add_trPr()
            cant = OxmlElement('w:cantSplit')
            trPr.append(cant)
        hdr = t.rows[0]._tr.get_or_add_trPr()
        hdr.append(OxmlElement('w:tblHeader'))
        self.doc.add_paragraph().paragraph_format.space_after = Pt(2)
        return self.tab_n

    def page_break(self):
        self.doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)


def shade(cell, hex_color):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement('w:shd')
    shd.set(qn('w:val'), 'clear')
    shd.set(qn('w:color'), 'auto')
    shd.set(qn('w:fill'), hex_color)
    tcPr.append(shd)


def field(paragraph, instr, placeholder=''):
    run = paragraph.add_run()
    for tag, attr in (('w:fldChar', 'begin'), ('w:instrText', None), ('w:fldChar', 'separate')):
        el = OxmlElement(tag)
        if attr:
            el.set(qn('w:fldCharType'), attr)
        else:
            el.set(qn('xml:space'), 'preserve')
            el.text = instr
        run._r.append(el)
    paragraph.add_run(placeholder)
    end = OxmlElement('w:fldChar')
    end.set(qn('w:fldCharType'), 'end')
    paragraph.add_run()._r.append(end)


def update_fields_on_open(doc):
    settings = doc.settings.element
    el = OxmlElement('w:updateFields')
    el.set(qn('w:val'), 'true')
    settings.append(el)


# ---------------------------------------------------------------- content

def cover(R):
    d = R.doc
    for _ in range(5):
        d.add_paragraph()
    p = d.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run(TITLE)
    r.bold = True
    r.font.size = Pt(28)
    r.font.color.rgb = NAVY
    p = d.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run(SUBTITLE)
    r.font.size = Pt(15)
    r.font.color.rgb = NAVY
    for _ in range(3):
        d.add_paragraph()
    for text, size, bold in ((AUTHOR, 14, True), ('Final Project Report', 12, False),
                             (COURSE, 11, False), ('Texas A&M University', 11, False),
                             (DATE, 11, False)):
        p = d.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.space_after = Pt(2)
        r = p.add_run(text)
        r.font.size = Pt(size)
        r.bold = bold
    for _ in range(3):
        d.add_paragraph()
    p = d.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run('Video presentation: ')
    r.bold = True
    r = p.add_run(VIDEO)
    r.font.highlight_color = 7  # yellow, so the placeholder cannot be missed
    R.page_break()


def front_matter(R, nums):
    R.h1('Abstract')
    R.p('Two robots should map a building faster than one, but in earlier runs our two TurtleBot3 '
        'robots often drove near each other, sat idle and finished at very different times. This '
        'project used a two-level fractional factorial experiment to find out which explorer and '
        'set-up choices actually change how fast the fleet explores. Four factors were varied: a new '
        'coordination term that steers a robot away from frontiers the other robot is closer to (A), '
        'whether the robots start in the same room or in different rooms (B), how the explorer ranks '
        'frontiers (C) and the robots\' top speed (D). The design was a 2⁴⁻¹ with D = ABC, run twice '
        'in two sessions (16 runs) plus 4 centre-point runs, in a 42 m² three-room area. Two responses '
        'were modelled with the course\'s standardized method: the time until the robots had seen 90% '
        'of a reference map (T90) and the time until the explorer declared the area fully explored.')
    R.p(f'Starting the robots in different rooms was the only significant effect on T90: it cut the '
        f'time from {nums["t90_same"]:.0f} s to {nums["t90_diff"]:.0f} s (effect −42.2 s, t = −3.11, '
        f'critical t = 2.306). The higher top speed was the only significant effect on the time to '
        f'exploration complete, from {nums["c_slow"]:.0f} s to {nums["c_fast"]:.0f} s (effect −61.5 s, '
        f't = −2.32), although that effect stops being significant if the one unusual run R02 is left '
        f'out. The coordination term had no detectable effect on either response. In most runs the '
        f'maps were merged only after T90 had been reached, and the term only acts after the merge, '
        f'so it had little chance to help. The recommendation is to start the robots apart and at '
        f'full speed, and to make coordination act before the maps merge before testing it again.')
    R.h1('Contents')
    p = R.doc.add_paragraph()
    field(p, 'TOC \\o "1-2" \\h \\z \\u',
          'Right-click and choose Update Field to build the table of contents.')
    R.page_break()


def introduction(R):
    R.h1('1. Introduction')
    R.p('Mobile robots that build a map of an unknown building are used in search and rescue, '
        'inspection and warehouse set-up. With more than one robot the job should go faster, because '
        'each robot can cover a different part of the building. Whether it does depends on how the '
        'robots are told where to go: if both are sent to the same unexplored area, the second robot '
        'adds nothing, and if they block each other in a doorway, the fleet can be slower than one '
        'robot alone.')
    R.p('This project studies a fleet of two TurtleBot3 Burger robots, named Inky and Clyde, and a '
        'central computer (Figure 1). Each robot runs SLAM Toolbox on its Raspberry Pi to build its '
        'own occupancy-grid map from its LiDAR, and Nav2 to drive to a goal. The central computer '
        'receives both maps over Wi-Fi, merges them into one map once it can align them, and runs a '
        'frontier explorer. A *frontier* is the edge between known free space and unknown space. The '
        'explorer ranks the frontiers it can see, gives each robot a frontier as its next goal, and '
        'stops when no reachable frontier is left.')
    R.figure(FIG / 'fig_system.png',
             'The system that generates the data. Each robot maps and drives itself; the central PC '
             'merges the maps and assigns goals. Factors A and C are explorer settings, factor D is set '
             'in Nav2 on each robot, and factor B is where the robots are placed before a run.')
    R.p('The explorer has many settings, and before this project they were tuned by trial and error. '
        'A new coordination term had just been added to it but had never been tested on the real '
        'robots. The objective of this project is to use the experimental methods of ENTC 641 to find '
        'out which of four controllable choices significantly change how fast the two robots explore, '
        'to build a model that predicts the exploration time from those choices, and to use the '
        'result to decide how the fleet should be run and what to change next.')
    R.p('The rest of the report is organized as follows. Section 2 reviews the literature on '
        'frontier-based and multi-robot exploration. Section 3 states the problem and defines the '
        'factors and responses. Section 4 describes the experimental design and how the data were '
        'collected. Section 5 analyzes the data with the standardized method for two-level factorial '
        'designs. Section 6 discusses the results and Section 7 gives the conclusions.')


def literature(R):
    R.h1('2. Literature Review')
    R.h2('2.1 Frontier-based exploration')
    R.p('Yamauchi [1] introduced frontier-based exploration: the robot keeps an occupancy grid, '
        'finds the boundaries between free and unknown cells, and drives to the nearest one, where '
        'its sensors will see new space. The method is simple, needs no prior map and is still the '
        'basis of most exploration software, including the m-explore package [6] on which the '
        'explorer in this project is built. In m-explore, each frontier gets a cost that trades off '
        'its distance against its size: cost = potential_scale × distance − gain_scale × size. '
        'The ratio of the two weights decides whether the robot prefers near frontiers or large '
        'ones, and it is factor C in this study.')
    R.h2('2.2 Multi-robot exploration and coordination')
    R.p('Yamauchi [2] extended the idea to several robots that share one map but choose their '
        'frontiers independently. Without coordination, robots that see the same map often choose '
        'the same frontier. Burgard et al. [3] addressed this by reducing the utility of frontiers near '
        'a target already assigned to another robot, and showed in simulation and on real robots that '
        'coordinated assignment cuts exploration time considerably compared with uncoordinated '
        'robots, with the benefit growing with the number of robots. Simmons et al. [4] used a similar '
        'bid-based assignment, and market-based methods [5] treat goals as tasks the robots trade. '
        'The coordination term tested here (factor A) follows the same idea in a simple form: it adds '
        'a penalty to a frontier that the other robot is closer to.')
    R.p('Most of these results come from simulation or from large environments with many rooms, '
        'where the robots can spread out. Fewer studies measure the effect of coordination on small '
        'fleets in small spaces, and fewer still vary several settings at once in a designed '
        'experiment. Amigoni [7] compared exploration strategies experimentally and found that their '
        'relative performance depends on the environment, which supports testing the settings on the actual '
        'robots and area rather than relying on published results.')
    R.h2('2.3 Map merging')
    R.p('When robots start at unknown positions relative to each other, their maps are in different '
        'frames and must be aligned before they can be combined. Birk and Carpin [8] framed this as '
        'searching for the transform that best overlaps two occupancy grids. Hörner [6] implemented '
        'feature-based map merging for ROS, which is the basis of the merge node used here. Until the '
        'maps are merged, the central computer cannot place both robots in one frame, which matters for '
        'any coordination rule that compares the robots\' distances to a frontier.')
    R.h2('2.4 Design of experiments for robot settings')
    R.p('Navigation and exploration software exposes many parameters that interact, and runs on real '
        'robots are slow, so testing one factor at a time is costly and misses interactions. Two-level '
        'factorial and fractional factorial designs [9, 10] estimate several main effects and '
        'interactions from few runs, and replicates give an estimate of the run-to-run noise against '
        'which each effect can be tested. A 2⁴⁻¹ fractional factorial studies four factors in eight '
        'conditions, at the cost of confounding the two-factor interactions in pairs. The ENTC 641 '
        'standardized method [11] gives the calculation matrix, the effects, their t-tests from the '
        'pooled variance, and a model of the significant effects. This project applies that method '
        'to the robot fleet.')


def problem(R):
    R.h1('3. Problem Statement and Formulation')
    R.h2('3.1 The problem')
    R.p('Before this project, four two-robot runs with the older explorer showed the problem clearly. '
        'Each robot stood still with no goal for between 6% and 80% of a run; after the maps merged, '
        'the two robots were less than 5 m apart 76% of the time in one hallway run; and the hallway '
        'runs took 950 to 1,430 s and never ended on their own (Table 1). It was not known which '
        'settings caused the slow, uneven runs, or whether the new coordination term would help.')
    R.table(['Run', 'Area', 'Time to 90% of final area', 'Time to 95%', 'Run length',
             'Idle with no goal (Inky / Clyde)'],
            [['small1', 'Three rooms', '208 s', '254 s', '552 s', '33% / 14%'],
             ['hall3', 'Hallways', '540 s', '570 s', '1,429 s', '24% / 32%'],
             ['hall4', 'Hallways', '567 s', '591 s', '952 s', '31% / 80%'],
             ['hall5', 'Hallways', '515 s', '558 s', '1,024 s', '6% / 10%']],
            'Pilot runs with the earlier explorer (before this experiment).',
            widths=[0.7, 1.0, 1.2, 0.9, 0.9, 1.6])
    R.p('The problem this project addresses is therefore: **which of the controllable choices '
        'significantly change how fast the two robots explore, and what model predicts the exploration '
        'time from them?** The hallway runs are too long to repeat 20 times, so the experiment uses the '
        'three-room area, where the pilot run small1 reached 95% coverage in about four minutes.')
    R.h2('3.2 Input variables (factors)')
    R.p('Four factors were chosen because each is a choice the operator controls and each was '
        'suspected of changing how much the robots duplicate each other\'s work. Each has two levels, '
        'coded −1 and +1 (Table 2). Three of them are quantitative and also have a centre level '
        '(coded 0); the start configuration is qualitative and has none.')
    R.table(['Factor', 'Low (−1)', 'High (+1)', 'Centre (0)', 'Type', 'Why it was chosen'],
            [['A: Coordination weight', '0 (off)', '1.0 (on)', '0.5', 'Quantitative',
              'The new term; does penalizing the other robot\'s frontiers help?'],
             ['B: Start configuration', 'Same room, about 1 m apart', 'Different rooms', 'none',
              'Qualitative', 'Robots placed apart should overlap less from the start'],
             ['C: Frontier ranking (potential : gain)', '2 : 1, favors near frontiers',
              '1 : 2, favors large frontiers', '1.5 : 1.5', 'Quantitative',
              'Decides where each robot goes next'],
             ['D: Max linear speed', '0.15 m/s', '0.22 m/s', '0.185 m/s', 'Quantitative',
              'The robot\'s upper limit; low batteries limit it in practice']],
            'Controllable factors and their levels.',
            widths=[1.35, 1.05, 1.05, 0.7, 0.85, 1.5], size=8.5)
    R.h2('3.3 Output variables (responses)')
    R.p('The instructor\'s feedback on the proposal was to model two responses, each with its own '
        'model (Table 3). Both are measured from t0, the explorer\'s first goal, and every run is '
        'scored over its first 480 s.')
    R.table(['Response', 'Unit', 'Definition', 'Source'],
            [['Y1: Time to 90% coverage (T90)', 's',
              'Time from t0 until the robots together have seen 90% of the cells known in the '
              'reference map', 'Each robot\'s own map in the run\'s ROS bag, aligned to the reference map'],
             ['Y2: Time to exploration complete', 's',
              'Time from t0 until the explorer logs that no reachable frontier is left (capped at 480 s)',
              'Explorer log']],
            'The two modelled responses.', widths=[1.5, 0.45, 2.6, 2.0], size=8.5)
    R.p('T90 measures how fast the robots see most of the area. The time to exploration complete '
        'measures how long the whole job takes, including the last small frontiers in corners, which '
        'is when a real deployment would end. A 90% threshold was used rather than 100% because the '
        'last few percent are small, scattered cells whose timing is mostly noise.')
    R.p('**Why a reference map.** Before the two maps are merged, the live merged map counts the '
        'area both robots have seen twice. In the pilot run hall4, which never merged, it reported '
        '153 m² against about 116 m² for similar runs. Instead, Inky alone mapped the whole area once '
        'with no time limit, giving a reference map with 42.4 m² of known cells (Figure 3). In each '
        'run, each robot\'s own map is aligned to the reference by an exhaustive search over shifts and '
        'rotations, and coverage is the share of reference cells that either robot has seen. Every '
        'alignment was checked by eye (Figure 4).')
    R.p('Other measures were recorded but not modelled, and are only described: the share of time a '
        'robot sat with no goal, the median distance between the robots, the share of goals the other '
        'robot was closer to, the distance driven, and goal success (goals reached divided by goals '
        'reached plus goals aborted by Nav2; goals the explorer cancelled to retarget a robot are left '
        'out).')
    R.h2('3.4 Nuisance factors')
    R.table(['Factor', 'Why it matters', 'How it was handled'],
            [['Battery voltage', 'Near 11 V the motors top out at about 0.18 m/s, which would blur factor D',
              'Every run started at 11.6 V or above (lowest start: 11.66 V); start voltage recorded'],
             ['Door between rooms', 'The mappable area changes from about 26.5 m² to 42 m²',
              'Held open for every run'],
             ['Day and session', 'Wi-Fi load, room clutter, people walking through',
              'Each replicate is one session (a block); run order randomized within each session'],
             ['Start positions', 'A different start gives a different first view',
              'Marked spots on the floor, the same for every run of a level']],
            'Nuisance factors and how each was controlled.', widths=[1.3, 2.5, 2.7], size=8.5)
    R.figure(FIG / 'fig_io.png',
             'The experiment as a system: four factors in, two responses out, and the nuisance '
             'factors that were held fixed or blocked.')
    R.figure(ROOT / 'data' / 'reference.png',
             'Reference map of the three-room area, made by one robot with no time limit. White is free, '
             'black is occupied, grey is unknown. Coverage in every run is measured against its known cells.',
             width=3.9)


def design_section(R, rows):
    R.h1('4. Experimental Design and Data Collection')
    R.h2('4.1 The 2⁴⁻¹ fractional factorial design')
    R.p('A full 2⁴ factorial needs 16 conditions, which with replicates would be 32 runs, too many '
        'for the time and batteries available. The design used is the half fraction 2⁴⁻¹: a full 2³ '
        'in A, B and C, with the level of D set by the generator D = ABC, as in the course\'s '
        '2³ fractional factorial design for four factors [11]. The defining relation is I = ABCD, '
        'so the design is resolution IV: each main effect is confounded only with a three-factor '
        'interaction, which is assumed negligible, and the two-factor interactions are confounded in '
        'pairs, AB = CD, AC = BD and AD = BC. Table 5 lists the eight conditions.')
    plan = []
    conds = sorted({coded(r) for r in rows if r['type'] == 'factorial'},
                   key=lambda s: (s[2], s[1], s[0]))
    runs = {}
    for r in rows:
        if r['type'] == 'factorial':
            runs.setdefault(coded(r), []).append(r['run_id'])
    for n, s in enumerate(conds, start=1):
        plan.append([str(n)] + [f'{v:+d}' for v in s] + [LEVEL['A'][s[0]], LEVEL['B'][s[1]],
                     LEVEL['C'][s[2]], LEVEL['D'][s[3]], ', '.join(sorted(runs[s]))])
    R.table(['Cond.', 'A', 'B', 'C', 'D = ABC', 'Coordination', 'Start', 'Ranking', 'Speed', 'Runs'],
            plan, 'Design matrix of the 2⁴⁻¹ design and the runs made at each condition.',
            widths=[0.55, 0.38, 0.38, 0.38, 0.65, 0.9, 1.05, 0.6, 0.75, 0.86], size=8.5,
            align_first_left=False)
    R.h2('4.2 Replicates, blocks, centre points and randomization')
    R.p('Each condition was run twice, once in each of two sessions, giving 16 factorial runs. The '
        'sessions act as blocks: anything that changes from one day to the next (Wi-Fi load, clutter, '
        'battery wear) affects every condition once. Within each session the order was randomized. Four '
        'centre-point runs were added, two per session, with A, C and D at their centre levels; since '
        'B has no centre, two centre runs started in the same room and two in different rooms. The '
        'centre runs check whether a straight-line model is adequate between the levels. The full run '
        'order is in the Appendix (Table A1).')
    R.h2('4.3 Running a trial')
    R.p('A script ran every trial the same way: it printed the settings and the start placement, set '
        'the explorer parameters and the speed limit on each robot, started the fleet, recorded a ROS '
        'bag and a 1 Hz system monitor on each robot, and stopped the run 480 s after the first goal. '
        'Each run ended with a short written note. A run that went wrong for a reason outside the '
        'experiment (a robot off Wi-Fi, a robot placed on the wrong mark, a run stopped by hand) was '
        'repeated, and the failed attempt was kept apart and never analyzed. R01 was redone because '
        'Inky had started about 1 m from its mark and turned by 90°. All 20 runs were completed '
        'between November 19 and November 24, 2026.')
    R.h2('4.4 From logs to responses')
    R.p('Each run produces about 25 files: the bag, explorer events, robot logs and monitors. An '
        'extraction script turns each run into one row of responses. For coverage, each robot\'s map '
        'is taken every 5 s from the bag and aligned to the reference map; Figure 4 shows one run\'s '
        'alignment, where both robots\' walls fall on the reference walls. Figure 5 shows the coverage '
        'curves of all 20 runs, the raw data from which T90 is read. Every run reached at least 96% '
        'coverage within the cap, and every run ended with the explorer declaring the area explored, '
        'so neither response is censored.')
    R.figure(ROOT / 'data' / 'runs' / 'R01' / 'registration.png',
             'Run R01: the two robots\' maps placed on the reference map. The walls line up, so the '
             'coverage measured from them is trustworthy.', width=3.9)
    R.figure(FIG / 'fig_coverage.png',
             'Coverage of the reference map against time for all 20 runs. T90 is where each curve '
             'crosses the dotted line. Runs that started in different rooms (orange) start higher '
             'and cross sooner.', width=6.0)


def input_table(R, rows):
    data = []
    order = sorted([r for r in rows if r['type'] == 'factorial'],
                   key=lambda r: (int(r['condition']), r['session']))
    order += [r for r in rows if r['type'] == 'centre']
    for r in order:
        s = coded(r)
        data.append([r['run_id'], r['session'], r['condition'] or 'centre'] + [f'{v:+d}' for v in s]
                    + [f'{float(r["T90_s"]):.1f}', f'{float(r["run_length_s"]):.1f}'])
    R.table(['Run', 'Session', 'Condition', 'A', 'B', 'C', 'D', 'T90 (s)', 'Exploration complete (s)'],
            data, 'Input data: coded factor levels and the two responses for all 20 runs.',
            widths=[0.55, 0.65, 0.8, 0.45, 0.45, 0.45, 0.45, 0.8, 1.3], size=8.5,
            align_first_left=False)


def analysis(R, rows, rows_x, nums):
    R.h1('5. Data Analysis')
    R.p('The analysis follows the ENTC 641 standardized method for two-level factorial designs [11], '
        'applied separately to each response, as agreed with the instructor: plot the data first, '
        'compute the effects with the calculation matrix, test each effect with a t-test based on the '
        'pooled variance of the replicates, keep only the significant effects in the model, and check '
        'the model with residual plots and the centre runs. The calculations were done in Python; the '
        'code was first checked against class Example 20 and reproduces its effects and t-values '
        'exactly.')
    R.h2('5.1 Input data')
    input_table(R, rows)
    R.p('Across the 16 factorial runs, T90 ranged from 57.9 s to 198.6 s (mean 98.0 s) and the time to '
        'exploration complete from 155.5 s to 390.7 s (mean 217.8 s). The two sessions gave almost the '
        'same averages (T90: 99.5 s and 96.4 s; exploration complete: 216.8 s and 218.9 s), so there is '
        'no sign of drift between the blocks and the two replicates can be pooled.')
    R.h2('5.2 Method')
    R.p('For each of the eight conditions i, the two replicates give a mean ȳᵢ and a variance Sᵢ². Each '
        'effect is the difference between the average response at the + level and at the − level of its '
        'column in the calculation matrix:')
    R.eq('E = Σᵢ (signᵢ · ȳᵢ) / 4')
    R.p('The pooled variance of the replicates estimates the run-to-run noise. With two replicates each '
        'condition contributes one degree of freedom, so there are 8 in total, and with N = 16 runs:')
    R.eq('Sp² = (1/8) Σᵢ Sᵢ²,     S²effect = 4 Sp² / N,     t = E / Seffect')
    R.p('An effect is significant at α = 0.05 when |t| exceeds the two-tailed critical value '
        't(0.025, 8) = 2.306. The model keeps the average plus half of each significant effect times its '
        'coded column, ŷ = ȳ + Σ (E/2)·X. Residuals are normalized by their standard deviation and '
        'plotted against the fitted value, the run order and the normal quantiles. Finally, the centre '
        'runs are compared with the model\'s prediction at the centre.')

    results = {}
    for k, (col, name) in enumerate(RESPONSES):
        conds, res = analyze(rows, col)
        _, res_x = analyze(rows_x, col)
        sd = resid_sd(rows, res, col)
        results[col] = (res, res_x)
        sec = f'5.{3 + k}'
        R.h2(f'{sec} Response Y{k + 1}: {name}')
        R.h3('Plots of the data')
        if col == 'T90_s':
            R.p('Figure 6 shows each response against each factor, with every run as a grey point and the '
                'level means joined. The start configuration stands out: every different-room run reached '
                '90% within about 100 s, while the same-room runs spread from 80 s to 200 s. Speed shows a '
                'smaller drop. Coordination and frontier ranking show almost no change. One run, R02 '
                '(same room, coordination on, slow), sits far above the rest at 198.6 s. The interaction '
                'plots (Figure 7) show mostly parallel lines, apart from the AC = BD pair, whose lines '
                'cross because of R02.')
        else:
            R.p('For the time to exploration complete (Figure 10), speed shows the clearest trend: '
                'slower robots took longer. The start configuration makes little difference here, unlike '
                'for T90. The spread is much larger than for T90: one run, R16 (different rooms, '
                'coordination on, slow), took 390.7 s, and its replicate took 226.0 s. The interaction '
                'plots (Figure 11) show the AD = BC lines diverging.')
        R.figure(RES / f'main_effects_{col}.png',
                 f'Main-effect plots for {name.lower()}. Grey points are runs, blue points are level means, '
                 'orange diamonds are centre runs.')
        R.figure(RES / f'interactions_{col}.png',
                 f'Interaction plots for {name.lower()} (each pair is confounded with the one named after it).')
        R.h3('Calculation matrix and effects')
        mat = []
        for n, c in enumerate(conds, start=1):
            mat.append([str(n)] + [f'{c.signs[k2]:+d}' for k2 in COLUMNS]
                       + [', '.join(f'{y:.1f}' for y in c.ys), f'{np.mean(c.ys):.1f}',
                          f'{np.var(c.ys, ddof=1):.1f}'])
        R.table(['Test', 'A', 'B', 'C', 'D', 'AB=CD', 'AC=BD', 'AD=BC', 'y₁, y₂ (s)', 'ȳ (s)', 'S²'],
                mat, f'Calculation matrix for {name.lower()}.',
                widths=[0.45, 0.4, 0.4, 0.4, 0.4, 0.6, 0.6, 0.6, 1.05, 0.55, 0.65], size=8.5,
                align_first_left=False)
        eff = [[LABEL[c], f'{res.effects[c]:+.1f}', f'{res.t[c]:+.2f}',
                '**yes**' if c in res.significant else 'no'] for c in COLUMNS]
        R.table(['Effect', 'E (s)', 't', 'Significant at α = 0.05'], eff,
                f'Effects and t-tests for {name.lower()}: Sp² = {res.pooled_var:.1f} s² (8 dof), '
                f'Seffect = {res.s_effect:.1f} s, so an effect must exceed '
                f'{res.t_crit * res.s_effect:.1f} s in size.',
                widths=[1.2, 1.0, 1.0, 1.8], size=9)
        if col == 'T90_s':
            R.p(f'The average T90 is {res.average:.1f} s. Only the start configuration is significant: '
                f'starting in different rooms shortened T90 by 42.2 s (t = −3.11). Speed (−22.7 s) and the '
                f'AC = BD pair (+20.9 s) are the next largest, but both are below the limit of '
                f'{res.t_crit * res.s_effect:.1f} s. Coordination is +11.5 s, that is, slightly *slower* with '
                f'the term on, and not significant. Frontier ranking (−2.4 s) and AB = CD (−2.0 s) are '
                f'negligible. The Pareto chart (Figure 8) shows the same ranking.')
        else:
            R.p(f'The average is {res.average:.1f} s. Only speed is significant: the higher top speed shortened '
                f'the time to exploration complete by 61.5 s (t = −2.32). It only just passes the limit of '
                f'{res.t_crit * res.s_effect:.1f} s. The AD = BC pair (−48.0 s, t = −1.81) and coordination '
                f'(+33.0 s, t = +1.24) are the next largest, both not significant. The pooled variance is '
                f'almost four times larger than for T90 (2,813.6 against 735.8 s²), mostly because of '
                f'condition 4, whose two runs (R05 and R16) differ by 165 s.')
        R.figure(RES / f'pareto_{col}.png',
                 f'Pareto chart of the absolute effects on {name.lower()}. The dashed line is the '
                 'significance limit t(0.025, 8)·Seffect; significant effects are red.', width=5.2)
        R.h3('Model')
        if col == 'T90_s':
            R.p('Keeping only the significant effect, the model in coded units (XB = −1 same room, +1 '
                'different rooms) is:')
            R.eq(f'ŷ₁ = {res.average:.1f} − 21.1 · XB   (s)')
            R.p(f'In real terms: T90 ≈ {nums["t90_same"]:.1f} s when the robots start in the same room and '
                f'{nums["t90_diff"]:.1f} s when they start in different rooms. Since B is qualitative, the '
                f'model only applies at those two levels.')
        else:
            R.p('Keeping only the significant effect, the model in coded units (XD = −1 for 0.15 m/s, +1 '
                'for 0.22 m/s) is:')
            R.eq(f'ŷ₂ = {res.average:.1f} − 30.7 · XD   (s)')
            R.p('Speed is quantitative, so with XD = (v − 0.185) / 0.035 for a top speed v in m/s, the model '
                'in real units is:')
            R.eq('ŷ₂ ≈ 380.1 − 877 · v   (s, valid for 0.15 ≤ v ≤ 0.22 m/s)')
            R.p(f'This predicts {nums["c_slow"]:.1f} s at 0.15 m/s and {nums["c_fast"]:.1f} s at 0.22 m/s.')
        R.h3('Residuals')
        big = 'R02 (+2.9)' if col == 'T90_s' else 'R16 (+2.7)'
        R.figure(RES / f'residuals_{col}.png',
                 f'Normalized residuals of the {name.lower()} model against fitted value and run number, '
                 'and normal probability plot.')
        if col == 'T90_s':
            R.p(f'The residual standard deviation is {sd:.1f} s. The residuals show no trend with run order, '
                f'so there is no drift over the two sessions. They fall in two columns because the model '
                f'has only two fitted values. All residuals are within ±2 except {big}, which also bends the '
                f'top of the normal probability plot away from the line; without it, the remaining points '
                f'lie close to the line. R02\'s note says the robots kept meeting and struggled to pass each '
                f'other. No set-up fault was found in its logs, so it is kept as a real outcome, and its '
                f'influence is checked in Section 5.6.')
        else:
            R.p(f'The residual standard deviation is {sd:.1f} s. Again there is no trend with run order. The '
                f'largest residual is {big}: Inky spent about four minutes on goals at the far end of the '
                f'area and Nav2 aborted two of them. The redone R01 shows the same pattern (three aborted '
                f'goals at the far end), so the far end of the area is a recurring trouble spot for Nav2, '
                f'not a one-off. Apart from R16 the normal probability plot is close to a straight line.')
        R.h3('Prediction for a new run')
        pi = []
        sig = sorted({ch for c in res.significant for ch in c})
        for levels in product((-1, 1), repeat=len(sig)):
            fixed = dict(zip(sig, levels))
            signs = design.columns(*[fixed.get(f, 0) for f in design.FACTORS])
            pred, half = res.predict(signs), res.prediction_interval(signs)
            pi.append([', '.join(f'{f} = {LEVEL[f][v]}' for f, v in fixed.items()), f'{pred:.1f}',
                       f'± {half:.1f}', f'{pred - half:.1f} to {pred + half:.1f}'])
        R.table(['Setting', 'ŷ (s)', 'Half width (s)', '95% prediction interval (s)'], pi,
                f'95% prediction intervals for one new run, ŷ ± t(0.025, 8)·√(Sp²(1 + h)) with '
                f'h = (1 + model terms)/N.', widths=[1.9, 0.9, 1.1, 2.0], size=9)
        if col == 'T90_s':
            R.p('The intervals are wide (±66 s) because the run-to-run noise is large compared with the '
                'effects: two runs at the same settings can differ by a minute or more. The model is good '
                'for choosing a setting, but a single run can land far from its prediction.')
        else:
            R.p('These intervals are wider still (±130 s), and the two overlap almost completely. Speed '
                'shifts the average completion time, but in a single run the noise from Nav2 struggling '
                'with particular goals is about as large as the effect.')
        R.h3('Centre points')
        cen = []
        for r in rows:
            if r['type'] != 'centre':
                continue
            pred = res.predict(design.columns(*coded(r)))
            y = float(r[col])
            cen.append([r['run_id'], r['session'], LEVEL['B'][coded(r)[1]], f'{y:.1f}', f'{pred:.1f}',
                        f'{y - pred:+.1f}'])
        R.table(['Run', 'Session', 'Start (B)', 'Observed (s)', 'Model (s)', 'Difference (s)'], cen,
                f'Centre runs (A, C, D at their centre levels) against the {name.lower()} model.',
                widths=[0.7, 0.8, 1.3, 1.1, 1.0, 1.1], size=9, align_first_left=False)
        if col == 'T90_s':
            R.p(f'Three of the four centre runs are within about half a residual standard deviation '
                f'({sd:.1f} s) of the model and R01 is 30.9 s below it, just over one. All four are below '
                f'the model, by 14.3 s on average. This hints that intermediate settings may be slightly '
                f'faster than the corners, but with four runs and this much noise it is not strong evidence '
                f'of curvature, and the straight-line model is adequate for this data.')
        else:
            R.p(f'Three centre runs are within 20 s of the model, well inside one residual standard '
                f'deviation ({sd:.1f} s). R20 is 64.3 s faster than predicted, about 1.2 standard deviations. '
                f'As for T90, all four centre runs are below the model; the straight-line model is adequate, '
                f'with the same weak hint that the intermediate settings are a little faster.')

    R.h2('5.5 Summary of the two models')
    t90, _ = results['T90_s']
    rl, _ = results['run_length_s']
    R.table(['', 'Y1: T90', 'Y2: Exploration complete'],
            [['Average', f'{t90.average:.1f} s', f'{rl.average:.1f} s'],
             ['Significant effects', 'B: −42.2 s (t = −3.11)', 'D: −61.5 s (t = −2.32)'],
             ['Not significant', 'A, C, D, AB=CD, AC=BD, AD=BC', 'A, B, C, AB=CD, AC=BD, AD=BC'],
             ['Model (coded)', 'ŷ = 98.0 − 21.1·XB', 'ŷ = 217.8 − 30.7·XD'],
             ['Pooled SD of replicates', f'{np.sqrt(t90.pooled_var):.1f} s', f'{np.sqrt(rl.pooled_var):.1f} s'],
             ['Largest residual', 'R02, +2.9', 'R16, +2.7'],
             ['Centre runs', 'all below model, mean −14.3 s', 'all below model, mean −27.0 s']],
            'Summary of the analysis of both responses.', widths=[1.7, 2.4, 2.4], size=9)

    R.h2('5.6 Sensitivity to the unusual run R02')
    R.p('R02 has the largest residual for T90. To see how much the conclusions depend on it, the whole '
        'analysis was repeated without it. Condition 6 then has one run instead of two, so the pooled '
        'variance has 7 degrees of freedom and the critical t is 2.365. Table 16 compares the effects.')
    sens = []
    for col, name in RESPONSES:
        res, res_x = results[col]
        for c in COLUMNS:
            sens.append([name.split(' (')[0] if c == 'A' else '', LABEL[c],
                         f'{res.effects[c]:+.1f}', f'{res.t[c]:+.2f}' + (' *' if c in res.significant else ''),
                         f'{res_x.effects[c]:+.1f}',
                         f'{res_x.t[c]:+.2f}' + (' *' if c in res_x.significant else '')])
    R.table(['Response', 'Effect', 'E, all runs (s)', 't, all runs', 'E, without R02 (s)', 't, without R02'],
            sens, 'Effects with and without run R02 (* = significant at α = 0.05).',
            widths=[1.5, 0.8, 1.0, 1.0, 1.1, 1.1], size=8.5)
    _, x_t90 = results['T90_s']
    _, x_rl = results['run_length_s']
    R.p(f'For T90 the conclusion holds without R02: the start configuration is still the only significant '
        f'effect (−30.8 s, t = −3.78), and the model becomes ŷ₁ = {x_t90.average:.1f} − 15.4·XB. The '
        f'coordination effect falls from +11.5 s to +0.1 s, so the small penalty seen for coordination comes '
        f'entirely from R02. For the time to exploration complete the conclusion is weaker: without R02 the '
        f'speed effect is −53.0 s with t = −1.91, below the critical 2.365, and no effect is significant. '
        f'Speed is still the largest effect and its direction is the same, but its significance depends on '
        f'one run, and it should be confirmed with more runs.')

    R.h2('5.7 Other measures (descriptive)')
    R.p('The other measures were not modelled, only summarized. Table 17 gives their means by '
        'coordination level, as planned, and by start configuration, which turned out to be the factor '
        'that matters for T90.')
    fact = [r for r in rows if r['type'] == 'factorial']
    sec_rows = []
    for col, name, unit in (('idle_pct', 'Idle with no goal', '%'),
                            ('separation_m', 'Median robot separation', 'm'),
                            ('other_closer_pct', 'Goals the other robot was closer to', '%'),
                            ('distance_m', 'Distance driven, both robots', 'm'),
                            ('goal_success_pct', 'Goal success', '%'),
                            ('merge_time_s', 'Time until the maps merged', 's')):
        def m(key, val):
            return np.mean([float(r[col]) for r in fact if r[key] == val])
        allv = [float(r[col]) for r in rows]
        sec_rows.append([f'{name} ({unit})', f'{m("A", "-1"):.1f}', f'{m("A", "1"):.1f}',
                         f'{m("B", "-1"):.1f}', f'{m("B", "1"):.1f}',
                         f'{min(allv):.1f}–{max(allv):.1f}'])
    R.table(['Measure', 'A off', 'A on', 'Same room', 'Different rooms', 'Range, all runs'], sec_rows,
            'Mean of the other measures over the 16 factorial runs, by coordination level and by start '
            'configuration.', widths=[2.1, 0.7, 0.7, 0.9, 1.0, 1.1], size=9)
    R.p('Coordination changed none of these much: idle time is slightly lower with it (11.0% against '
        '13.8%), goal success is almost the same, and the share of goals the other robot was closer to, '
        'which the term is meant to reduce, is 30.4% with it off and 31.9% with it on. With the robots in '
        'different rooms they stayed a little further apart (3.2 m against 2.8 m median separation).'),
    R.p('The maps also merged later in the runs with coordination on (130 s on average, against 87 s '
        'with it off). This was not tested, but it is plausible: the merge needs the two maps to overlap, '
        'and anything that keeps the robots apart delays it. Figure 14 shows why coordination could not '
        'do much for T90: the term only works after the two maps are merged, and in 12 of the 20 runs, '
        'including 6 of the 8 runs with coordination on, the maps merged only after T90 had been reached.')
    R.figure(FIG / 'fig_merge.png',
             'Time until the maps merged against T90 for each run. Points above the dotted line merged '
             'after the robots had already seen 90% of the area, so the coordination term was inactive for '
             'most of the T90 window. Merge time is estimated from when the map-to-robot transform settled.',
             width=4.4)


def discussion(R, nums):
    R.h1('6. Discussion')
    R.h2('6.1 Start configuration drives the time to 90% coverage')
    R.p('Starting the robots in different rooms was the strongest and most robust result: it cut T90 '
        'by about 42 s, a third of the same-room average, and stayed significant without R02. Part of '
        'this is a head start. In the first scan, robots in different rooms see two rooms instead of one: '
        f'coverage at t0 averaged {nums["cov0_diff"]:.1f}% for different-room starts against '
        f'{nums["cov0_same"]:.1f}% for same-room starts (Figure 5). At the early rate of roughly one '
        'percentage point per second, that is worth something like 10–15 s. The rest comes from less '
        'duplicated work: robots that start in the same room both see the same frontiers first, and one '
        'of them must later travel through the other\'s area to reach new space. The coverage curves of '
        'same-room runs show exactly this as flat stretches, where coverage stalls at about 55–70% while a '
        'robot drives through known space.')
    R.p('The start configuration did not matter for the time to exploration complete. Once most of the '
        'area is seen, the remaining time is spent on the last small frontiers in corners and at the far '
        'end of the area, and those are reached at the same pace whichever room a robot started in.')
    R.h2('6.2 Speed drives the time to exploration complete')
    R.p('The higher top speed shortened the whole exploration by about 61 s on average. A simple '
        'physical check supports the size of the effect: going from 0.15 to 0.22 m/s is 47% faster, which '
        'would cut pure driving time by 32%. The model predicts a 25% cut (from '
        f'{nums["c_slow"]:.0f} s to {nums["c_fast"]:.0f} s), which is plausible because part of each run '
        'is spent turning, planning and waiting for goals, which speed does not change. Speed mattered '
        'less for T90 (−22.7 s, not significant) because the first 90% is mostly seen from where the '
        'robots already are, with short drives. However, the speed effect is only just significant and '
        'loses significance without R02, so it is the less certain of the two results.')
    R.h2('6.3 Why coordination did not pay off')
    R.p('The coordination term had no detectable effect on either response, and if anything its estimated '
        'effect points the wrong way (+11.5 s on T90, +33.0 s on completion). Three observations explain '
        'this. First, the term is inactive until the two maps are merged, because before that the robots\' '
        'positions are not in one frame. The maps merged after a median of 105 s, and in 6 of the 8 runs '
        'with coordination on, after T90 (Figure 14), so for most of the period that T90 measures the two '
        'levels of factor A ran identical code. Second, the area is small: three rooms with one robot in '
        'each leaves little room for the robots to divide the work differently. Third, the measure the '
        'term targets, the share of goals the other robot was closer to, did not go down. The literature '
        'reports large gains from coordination [3], but in larger environments with more frontiers to '
        'share; this result does not contradict that, it shows that this implementation in this area acts '
        'too late to matter.')
    R.h2('6.4 Interactions and confounding')
    R.p('No interaction was significant. The largest were AC = BD on T90 (+20.9 s) and AD = BC on the '
        'time to exploration complete (−48.0 s, t = −1.81). Because the design is resolution IV, each '
        'of these is the sum of two interactions that cannot be separated with these runs. For example, '
        'AD = BC could be coordination working better at high speed or the start configuration mattering '
        'more for one ranking. The AC = BD value on T90 drops to +9.5 s without R02, so it is mostly that '
        'one run. If a later experiment needs to separate a pair, the other half fraction (D = −ABC) would '
        'de-alias all of them.')
    R.h2('6.5 The unusual runs')
    R.p('R02 and R16 were both investigated from their logs and notes, and neither had a set-up fault. '
        'In R02 the robots started together and kept meeting in the doorway; in R16, Nav2 repeatedly '
        'failed goals at the far end of the area. Both are failures of how the robots interact with the '
        'building, which is part of what the experiment measures, so both were kept. Their large size '
        'compared with the other runs is itself a finding: the exploration time has a long upper tail, '
        'caused by occasional blocking and failed goals rather than by the settings.')
    R.h2('6.6 Limitations')
    R.bullets([
        'One area, three rooms, 42 m². The effects of coordination and start configuration may be '
        'different in larger or more open spaces, such as the hallways of the pilot runs.',
        'Two replicates per condition. The pooled variance has only 8 degrees of freedom, so effects '
        'smaller than about 31 s (T90) and 61 s (completion) cannot be detected.',
        'Resolution IV. Two-factor interactions are confounded in pairs, and three-factor interactions '
        'are assumed negligible.',
        'Part of the start configuration effect is the initial view (head start), not better '
        'exploration. The experiment cannot separate the two.',
        'Merge time is estimated from when the map-to-robot transform settled, so the comparison with T90 '
        'in Figure 14 is approximate.',
    ])


def conclusions(R):
    R.h1('7. Conclusions and Recommendations')
    R.p('A 2⁴⁻¹ fractional factorial experiment with two replicates and four centre points, analyzed with '
        'the standardized method, identified which of four controllable choices change how fast a '
        'two-robot TurtleBot3 fleet explores a three-room area:')
    R.numbered([
        '**Where the robots start matters most for seeing the area quickly.** Starting in different rooms '
        'cut the time to 90% coverage by 42 s, from about 119 s to 77 s. The effect is significant and '
        'stays significant without the most unusual run.',
        '**Top speed matters most for finishing.** Raising the top speed from 0.15 to 0.22 m/s cut the time '
        'to exploration complete by about 61 s, from about 249 s to 187 s. The effect is just significant, '
        'and it depends on one run, so it needs confirmation.',
        '**The new coordination term did not pay off.** Its effect was not significant on either response. '
        'The data suggest why: it only acts after the maps merge, which in most runs happened after the '
        'robots had already seen 90% of the area.',
        '**Frontier ranking did not matter** in this area, and no interaction was significant.',
        '**Run-to-run noise is large.** A single run can land a minute or more from the model, mainly '
        'because of occasional blocking between the robots and goals Nav2 fails to reach.',
    ])
    R.p('The data analysis was useful in two ways. It replaced trial-and-error tuning with numbers: the '
        'fleet should be run with the robots starting apart and at full speed, and the frontier ranking can '
        'be left at its default. It also showed that the coordination term, which had been expected to help, '
        'is not worth keeping in its current form, and the merge-time comparison points to what to change.')
    R.p('**Recommendations for the next steps:**')
    R.bullets([
        'Make coordination act before the maps merge, for example by using the known start positions to '
        'place both robots in one frame from the first goal, then test it again.',
        'Repeat the study in the larger hallway area, where the robots have more space to divide and '
        'coordination should matter more.',
        'Confirm the speed effect with additional runs at the recommended setting (different rooms, '
        '0.22 m/s) and compare them with the prediction intervals.',
        'Investigate the far end of the area, where Nav2 repeatedly aborted goals, since those failures '
        'cause the longest runs.',
    ])


def references(R):
    R.h1('References')
    refs = [
        'B. Yamauchi, "A frontier-based approach for autonomous exploration," in *Proc. IEEE Int. Symp. on '
        'Computational Intelligence in Robotics and Automation (CIRA)*, 1997, pp. 146–151.',
        'B. Yamauchi, "Frontier-based exploration using multiple robots," in *Proc. 2nd Int. Conf. on '
        'Autonomous Agents*, 1998, pp. 47–53.',
        'W. Burgard, M. Moors, C. Stachniss and F. E. Schneider, "Coordinated multi-robot exploration," '
        '*IEEE Transactions on Robotics*, vol. 21, no. 3, pp. 376–386, 2005.',
        'R. Simmons, D. Apfelbaum, W. Burgard, D. Fox, M. Moors, S. Thrun and H. Younes, "Coordination for '
        'multi-robot exploration and mapping," in *Proc. AAAI Conf. on Artificial Intelligence*, 2000, '
        'pp. 852–858.',
        'R. Zlot, A. Stentz, M. B. Dias and S. Thayer, "Multi-robot exploration controlled by a market '
        'economy," in *Proc. IEEE Int. Conf. on Robotics and Automation (ICRA)*, 2002, pp. 3016–3023.',
        'J. Hörner, "Map-merging for multi-robot system," Bachelor\'s thesis, Charles University, Prague, '
        '2016.',
        'F. Amigoni, "Experimental evaluation of some exploration strategies for mobile robots," in *Proc. '
        'IEEE Int. Conf. on Robotics and Automation (ICRA)*, 2008, pp. 2818–2823.',
        'A. Birk and S. Carpin, "Merging occupancy grid maps from multiple robots," *Proceedings of the '
        'IEEE*, vol. 94, no. 7, pp. 1384–1397, 2006.',
        'D. C. Montgomery, *Design and Analysis of Experiments*, 9th ed. Hoboken, NJ: Wiley, 2017.',
        'G. E. P. Box, J. S. Hunter and W. G. Hunter, *Statistics for Experimenters*, 2nd ed. Hoboken, NJ: '
        'Wiley, 2005.',
        'ENTC 641 lecture notes, "Design of Experiments," "Standardized Method for Calculation of Effects" '
        'and "Two-Level Fractional Factorial Designs," Texas A&M University, Fall 2026.',
        'S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the dynamic world," *Journal of Open Source '
        'Software*, vol. 6, no. 61, p. 2783, 2021.',
        'S. Macenski, F. Martín, R. White and J. Ginés Clavero, "The Marathon 2: A navigation system," in '
        '*Proc. IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS)*, 2020.',
    ]
    for i, ref in enumerate(refs, start=1):
        para = R.p(f'[{i}] {ref}', align=WD_ALIGN_PARAGRAPH.LEFT, size=10, after=4)
        para.paragraph_format.left_indent = Inches(0.35)
        para.paragraph_format.first_line_indent = Inches(-0.35)


def appendix(R, rows):
    R.h1('Appendix A. Run Order and Full Data')
    R.doc.paragraphs[-1].paragraph_format.page_break_before = True
    with open(design.PLAN_CSV, newline='') as fh:
        plan = list(csv.DictReader(fh))
    data = []
    by_id = {r['run_id']: r for r in rows}
    for p in plan:
        r = by_id[p['run_id']]
        data.append([p['run_id'], p['session'], p['order'], p['condition'] or 'centre',
                     p['coordination_weight'], p['start'].replace('_', ' '),
                     f'{p["potential_scale"]} : {p["gain_scale"]}', p['max_speed'],
                     r['T90_s'], r['run_length_s'], r['coverage_cap_pct'], f"{float(r['start_voltage_v']):.2f}"])
    R.table(['Run', 'Ses.', 'Ord.', 'Cond.', 'A', 'Start', 'C', 'D (m/s)', 'T90 (s)',
             'Done (s)', 'Cov. (%)', 'V start'],
            data, 'Run order as executed (randomized within each session), actual settings and main '
            'measurements. Done = time to exploration complete; Cov. = coverage at 480 s.', widths=[0.42, 0.42, 0.42, 0.55, 0.4, 1.0, 0.7, 0.55, 0.5, 0.55, 0.55, 0.5],
            size=7.5, align_first_left=False, label='A1')
    R.h2('A.2 Data and code')
    R.p('All data and code are kept in the project repository under entc641/. Each run folder holds the '
        'run\'s settings, logs, robot monitors, coverage curve and alignment image; the ROS bags are kept '
        'locally. extract.py computes the responses, doe.py implements the standardized method, '
        'check_example20.py verifies doe.py against class Example 20, analyze.py produces every analysis '
        'table and figure in Section 5, and report/build_report.py builds this report from the same data.',
        align=WD_ALIGN_PARAGRAPH.LEFT)


def header_footer(R):
    sec = R.doc.sections[0]
    sec.different_first_page_header_footer = True
    hp = sec.header.paragraphs[0]
    hp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    r = hp.add_run(f'{AUTHOR}, ENTC 641 Final Project: {TITLE}')
    r.font.size = Pt(8.5)
    r.font.color.rgb = RGBColor(0x70, 0x70, 0x70)
    fp = sec.footer.paragraphs[0]
    fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    field(fp, 'PAGE', '1')


def main():
    rows = load()
    rows_x = load(exclude=('R02',))
    _, t90 = analyze(rows, 'T90_s')
    _, rl = analyze(rows, 'run_length_s')
    cov0 = {}
    for b in ('-1', '1'):
        v = []
        for r in rows:
            if r['B'] == b:
                with open(design.RUNS / r['run_id'] / 'coverage.csv', newline='') as fh:
                    v.append(float(next(csv.DictReader(fh))['coverage_pct']))
        cov0[b] = float(np.mean(v))
    nums = {
        't90_same': t90.predict(design.columns(0, -1, 0, 0)),
        't90_diff': t90.predict(design.columns(0, 1, 0, 0)),
        'c_slow': rl.predict(design.columns(0, 0, 0, -1)),
        'c_fast': rl.predict(design.columns(0, 0, 0, 1)),
        'cov0_same': cov0['-1'], 'cov0_diff': cov0['1'],
    }
    R = Report()
    header_footer(R)
    cover(R)
    front_matter(R, nums)
    introduction(R)
    literature(R)
    problem(R)
    design_section(R, rows)
    analysis(R, rows, rows_x, nums)
    discussion(R, nums)
    conclusions(R)
    references(R)
    appendix(R, rows)
    update_fields_on_open(R.doc)
    R.doc.core_properties.title = f'{TITLE} {SUBTITLE}'
    R.doc.core_properties.author = AUTHOR
    R.doc.save(OUT)
    print('wrote', OUT)


if __name__ == '__main__':
    main()
