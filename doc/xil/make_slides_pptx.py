#!/usr/bin/env python
"""Regenerate AxleDynoCarMakerCoupling.pptx.

The deck is a build product. This script is its source -- a .pptx does not diff,
so edit here and regenerate rather than editing the binary, unless the change is
one-off presentation polish.

Everything is emitted as native PowerPoint shapes (rectangles, connectors, text
boxes), never as embedded images, so every box and arrow stays editable in
PowerPoint. Content mirrors AxleDynoCarMakerCoupling_slides.html and the section
numbering of AxleDynoCarMakerCoupling.md.

    pip install python-pptx
    python doc/xil/make_slides_pptx.py

Colour semantics, shared with the HTML deck and load-bearing throughout:

    HW    warm amber   hardware, measured, physical
    MODEL cool teal    CarMaker, computed
    INK   near-black   our Simulink bypass -- the fulcrum between the two
"""
from __future__ import annotations

import pathlib

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.dml import MSO_LINE_DASH_STYLE
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Emu, Pt

OUT = pathlib.Path(__file__).resolve().parent / "AxleDynoCarMakerCoupling.pptx"

# --------------------------------------------------------------------------
# tokens
# --------------------------------------------------------------------------

INK       = RGBColor(0x11, 0x18, 0x20)
INK_2     = RGBColor(0x4A, 0x57, 0x64)
INK_3     = RGBColor(0x78, 0x83, 0x8F)
RULE      = RGBColor(0xC7, 0xCE, 0xD5)
PAPER     = RGBColor(0xFF, 0xFF, 0xFF)
PANEL     = RGBColor(0xF4, 0xF6, 0xF7)

HW        = RGBColor(0xA9, 0x61, 0x1A)
HW_SOFT   = RGBColor(0xF2, 0xE4, 0xD3)
MODEL     = RGBColor(0x1B, 0x64, 0x6D)
MODEL_SOFT= RGBColor(0xD9, 0xE9, 0xEA)
WARN      = RGBColor(0x93, 0x37, 0x23)
WARN_SOFT = RGBColor(0xF3, 0xDE, 0xD8)

F_DISPLAY = "Bahnschrift"      # Windows' DIN -- the industrial-standard face
F_BODY    = "Segoe UI"
F_MONO    = "Consolas"

DASH = MSO_LINE_DASH_STYLE.DASH

# 16:9 at 13.333 x 7.5 in
SW, SH = Emu(12192000), Emu(6858000)
IN = 914400


def inch(v: float) -> Emu:
    return Emu(int(round(v * IN)))


# --------------------------------------------------------------------------
# primitives
# --------------------------------------------------------------------------

def textbox(slide, x, y, w, h, *, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP):
    tb = slide.shapes.add_textbox(inch(x), inch(y), inch(w), inch(h))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = anchor
    tf.paragraphs[0].alignment = align
    return tf


def line(tf, text, *, font=F_BODY, size=12, color=INK, bold=False,
         space_before=0, space_after=0, align=None, first=False):
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.space_before = Pt(space_before)
    p.space_after = Pt(space_after)
    if align is not None:
        p.alignment = align
    r = p.add_run()
    r.text = text
    r.font.name = font
    r.font.size = Pt(size)
    r.font.color.rgb = color
    r.font.bold = bold
    return p


def box(slide, x, y, w, h, *, fill=None, edge=RULE, width=1.25, dash=None):
    sh = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, inch(x), inch(y), inch(w), inch(h))
    sh.shadow.inherit = False
    if fill is None:
        sh.fill.background()
    else:
        sh.fill.solid()
        sh.fill.fore_color.rgb = fill
    sh.line.color.rgb = edge
    sh.line.width = Pt(width)
    if dash is not None:
        sh.line.dash_style = dash
    sh.text_frame.word_wrap = True
    tf = sh.text_frame
    tf.margin_left = tf.margin_right = inch(0.10)
    tf.margin_top = tf.margin_bottom = inch(0.06)
    tf.vertical_anchor = MSO_ANCHOR.TOP
    return sh


def labelled_box(slide, x, y, w, h, head, body, *,
                 fill=None, edge=RULE, head_color=None, width=1.25):
    sh = box(slide, x, y, w, h, fill=fill, edge=edge, width=width)
    tf = sh.text_frame
    line(tf, head, font=F_DISPLAY, size=13, color=head_color or edge,
         bold=True, first=True)
    for i, b in enumerate(body):
        line(tf, b, font=F_MONO, size=9.5, color=INK_2, space_before=3 if i == 0 else 1)
    return sh


def arrow(slide, x1, y1, x2, y2, *, color=INK, width=2.0, dash=None):
    cn = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT,
                                    inch(x1), inch(y1), inch(x2), inch(y2))
    cn.line.color.rgb = color
    cn.line.width = Pt(width)
    if dash is not None:
        cn.line.dash_style = dash
    tail = cn.line._get_or_add_ln().find(
        "{http://schemas.openxmlformats.org/drawingml/2006/main}tailEnd")
    if tail is None:
        from pptx.oxml.ns import qn
        tail = cn.line._get_or_add_ln().makeelement(qn("a:tailEnd"), {})
        cn.line._get_or_add_ln().append(tail)
    tail.set("type", "triangle")
    tail.set("w", "med")
    tail.set("len", "med")
    return cn


def plain_line(slide, x1, y1, x2, y2, *, color=RULE, width=1.0, dash=None):
    cn = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT,
                                    inch(x1), inch(y1), inch(x2), inch(y2))
    cn.line.color.rgb = color
    cn.line.width = Pt(width)
    if dash is not None:
        cn.line.dash_style = dash
    return cn


def caption(slide, text, *, y=6.62, color=INK_3, size=9.5, font=F_MONO):
    tf = textbox(slide, 0.62, y, 12.1, 0.6)
    line(tf, text, font=font, size=size, color=color, first=True)


def new_slide(prs, num, eyebrow, title, lede=None):
    s = prs.slides.add_slide(prs.slide_layouts[6])
    bg = s.background.fill
    bg.solid()
    bg.fore_color.rgb = PAPER

    plain_line(s, 0.62, 0.52, 12.72, 0.52, color=RULE, width=1.0)

    tf = textbox(s, 0.62, 0.24, 8.0, 0.26)
    p = tf.paragraphs[0]
    r = p.add_run(); r.text = "%02d / 09" % num
    r.font.name, r.font.size, r.font.bold = F_MONO, Pt(9), True
    r.font.color.rgb = HW
    r = p.add_run(); r.text = "    " + eyebrow.upper()
    r.font.name, r.font.size = F_MONO, Pt(9)
    r.font.color.rgb = INK_3

    tf = textbox(s, 0.62, 0.70, 12.1, 0.62)
    line(tf, title, font=F_DISPLAY, size=30, color=INK, bold=True, first=True)

    if lede:
        tf = textbox(s, 0.62, 1.36, 11.4, 0.5)
        line(tf, lede, font=F_BODY, size=12.5, color=INK_2, first=True)
    return s


def mono_block(slide, x, y, w, h, rows):
    """A fixed-pitch panel. rows = [(text, color, bold), ...]"""
    sh = box(slide, x, y, w, h, fill=PANEL, edge=RULE, width=1.0)
    tf = sh.text_frame
    tf.margin_left = inch(0.16)
    tf.margin_top = inch(0.10)
    for i, (t, c, b) in enumerate(rows):
        line(tf, t, font=F_MONO, size=11, color=c, bold=b,
             space_before=0 if i == 0 else 2, first=(i == 0))
    return sh


# --------------------------------------------------------------------------
# slides
# --------------------------------------------------------------------------

def slide_01(prs):
    s = prs.slides.add_slide(prs.slide_layouts[6])
    bg = s.background.fill; bg.solid(); bg.fore_color.rgb = PAPER

    tf = textbox(s, 0.9, 1.15, 8.0, 0.3)
    line(tf, "FIXS  ·  XIL BENCH  ·  dev_v0.9.0", font=F_MONO, size=10,
         color=HW, bold=True, first=True)

    tf = textbox(s, 0.9, 1.62, 11.2, 1.5)
    line(tf, "Axle Dyno ↔ CarMaker Coupling", font=F_DISPLAY, size=52,
         color=INK, bold=True, first=True)

    tf = textbox(s, 0.9, 3.02, 9.6, 1.0)
    line(tf, "How the working implementation exchanges torque and speed with CarMaker "
             "across a single scalar per corner — where the one-step delay comes from, "
             "why the loop is stable, and what the architecture cannot tell you about itself.",
         font=F_BODY, size=14, color=INK_2, first=True)

    mono_block(s, 0.9, 4.16, 8.2, 1.06, [
        ("w[k] = w[k-1] + (K*Ts / I) * ( Taxl[k] + Trq_T2W[k] )", INK, True),
        ("", INK, False),
        ("K = 1     Ts = 0.001 s     I = Wheel.<i>.I  (CarMaker parameter)", INK_2, False),
    ])

    y = 5.56
    for col, lbl in ((HW, "hardware · measured · physical"),
                     (MODEL, "CarMaker · computed"),
                     (INK, "our Simulink bypass")):
        sw = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, inch(0.9), inch(y + 0.03),
                                inch(0.15), inch(0.15))
        sw.shadow.inherit = False
        sw.fill.solid(); sw.fill.fore_color.rgb = col
        sw.line.fill.background()
        tf = textbox(s, 1.16, y, 6.0, 0.26)
        line(tf, lbl, font=F_MONO, size=10, color=INK_2, first=True)
        y += 0.30
    return s


def slide_02(prs):
    s = new_slide(prs, 2, "State ownership",
                  "One wheel-spin integrator exists in the whole system",
                  "CarMaker's went out with the PowerTrain module. The hardware's physical spin is "
                  "servo-slaved. Ours is the only one — which is why nothing can diverge.")

    labelled_box(s, 0.62, 2.20, 3.5, 2.30, "HARDWARE",
                 ["powertrain", "a torque source only", "",
                  "owns no integrated state:", "its shaft speed is forced", "by the servo"],
                 fill=HW_SOFT, edge=HW, head_color=HW, width=1.5)

    labelled_box(s, 4.92, 2.20, 3.5, 2.30, "SIMULINK — OURS",
                 ["wheel spin  w", "Backward Euler, Ts = 1 ms", "",
                  "THE ONLY INTEGRATOR", "for this state, anywhere", "in the system"],
                 fill=None, edge=INK, head_color=INK, width=2.5)

    labelled_box(s, 9.22, 2.20, 3.5, 2.30, "CARMAKER",
                 ["body · suspension", "tire · road", "",
                  "PowerTrain module removed", "— so its wheel spin DOF", "is gone too"],
                 fill=MODEL_SOFT, edge=MODEL, head_color=MODEL, width=1.5)

    arrow(s, 4.12, 2.72, 4.86, 2.72, color=HW, width=2.25)
    tf = textbox(s, 4.10, 2.44, 1.0, 0.26); line(tf, "Taxl", font=F_MONO, size=10, color=HW, first=True)
    arrow(s, 4.86, 3.92, 4.12, 3.92, color=HW, width=2.0, dash=DASH)
    tf = textbox(s, 4.02, 4.00, 1.3, 0.26); line(tf, "w_cmd", font=F_MONO, size=9, color=HW, first=True)

    arrow(s, 8.42, 2.72, 9.16, 2.72, color=MODEL, width=2.25)
    tf = textbox(s, 8.40, 2.44, 1.0, 0.26); line(tf, "rotv", font=F_MONO, size=10, color=MODEL, first=True)
    arrow(s, 9.16, 3.92, 8.42, 3.92, color=MODEL, width=2.25)
    tf = textbox(s, 8.32, 4.00, 1.4, 0.26); line(tf, "Trq_T2W", font=F_MONO, size=9, color=MODEL, first=True)

    caption(s, "CM DriveLine performs this integration natively — RefMan Fig. 16.83 "
               "“Integration of rotation speeds”. We rebuilt that block with the same inertia "
               "parameter,", y=5.00)
    caption(s, "matching IPG's own UserPowerTrain.mdl. Consequence: CM's tire slip is computed from "
               "exactly the number that commands the dyno — consistent by construction, not by tracking.",
            y=5.24)
    return s


def slide_03(prs):
    s = new_slide(prs, 3, "Signal flow", "What crosses the cut at step k",
                  "The actual dyno shaft speed never reaches CarMaker. It is seen only by the "
                  "dyno's internal speed-tracking controller.")

    labelled_box(s, 0.62, 2.30, 3.1, 1.75, "DYNO + VEHICLE",
                 ["internal speed servo", "tracks w_cmd", "", "w_act stays in here"],
                 fill=HW_SOFT, edge=HW, head_color=HW, width=1.5)

    sh = labelled_box(s, 5.05, 2.10, 3.3, 2.15, "SIMULINK",
                      ["w[k] = w[k-1]", "   + (Ts/I) *", "   ( Taxl[k]", "   + Trq_T2W[k] )"],
                      fill=None, edge=INK, head_color=INK, width=2.5)

    labelled_box(s, 9.70, 2.30, 3.0, 1.75, "CARMAKER",
                 ["tire → body → road", "", "holds vehicle mass"],
                 fill=MODEL_SOFT, edge=MODEL, head_color=MODEL, width=1.5)

    arrow(s, 3.72, 2.70, 4.99, 2.70, color=HW, width=2.5)
    tf = textbox(s, 3.78, 2.40, 1.6, 0.26); line(tf, "Taxl[k]", font=F_MONO, size=10, color=HW, first=True)
    tf = textbox(s, 3.78, 2.80, 1.7, 0.26); line(tf, "measured", font=F_MONO, size=8.5, color=INK_3, first=True)

    arrow(s, 4.99, 3.72, 3.72, 3.72, color=HW, width=2.5, dash=DASH)
    tf = textbox(s, 3.86, 3.80, 1.9, 0.26); line(tf, "w_cmd = w[k]", font=F_MONO, size=9, color=HW, first=True)

    arrow(s, 8.35, 2.70, 9.64, 2.70, color=MODEL, width=2.5)
    tf = textbox(s, 8.42, 2.40, 1.7, 0.26); line(tf, "rotv = w[k]", font=F_MONO, size=10, color=MODEL, first=True)
    tf = textbox(s, 8.42, 2.80, 2.2, 0.26); line(tf, "Trq_Drive = Taxl[k]", font=F_MONO, size=8.5, color=INK_3, first=True)

    arrow(s, 9.64, 3.72, 8.35, 3.72, color=MODEL, width=2.5)
    tf = textbox(s, 8.50, 3.80, 1.7, 0.26); line(tf, "Trq_T2W[k]", font=F_MONO, size=9, color=MODEL, first=True)

    plain_line(s, 0.62, 4.62, 12.72, 4.62, color=RULE)
    caption(s, "Zeroed by us: all CM brake torque, Trq_Supp2WC, Trq_Supp2Bdy1, Trq_Supp2BdyEng.",
            y=4.80, color=HW)
    caption(s, "Trq_Drive therefore has no live consumer — in this build it is bookkeeping only. "
               "The one dynamically active hardware→CM signal is rotv:", y=5.04)
    caption(s, "CarMaker sees the resultant of Taxl and Ttire, never Taxl itself.", y=5.28)
    return s


def slide_04(prs):
    s = new_slide(prs, 4, "Timing · the key diagram",
                  "Where the one-step delay actually lives",
                  "The tire's slip→force map is memoryless. The delay is in the shared variable, "
                  "because Tire is scheduled before PowerTrain and Rim_rotv is written once per cycle.")

    y = 2.05
    h = 0.72
    tf = textbox(s, 0.62, 1.80, 3.0, 0.22)
    line(tf, "cycle k-1", font=F_MONO, size=9, color=INK_3, first=True)
    tf = textbox(s, 4.30, 1.80, 3.0, 0.22)
    line(tf, "cycle k", font=F_MONO, size=9, color=HW, bold=True, first=True)
    tf = textbox(s, 11.10, 1.80, 2.0, 0.22)
    line(tf, "cycle k+1", font=F_MONO, size=9, color=INK_3, first=True)

    def mod(x, w, label, *, fill, edge, width=1.25, dash=None, note=None):
        sh = box(s, x, y, w, h, fill=fill, edge=edge, width=width, dash=dash)
        tf = sh.text_frame
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        line(tf, label, font=F_MONO, size=10.5, color=edge if fill else INK,
             bold=True, align=PP_ALIGN.CENTER, first=True)
        if note:
            line(tf, note, font=F_MONO, size=8, color=HW, align=PP_ALIGN.CENTER)
        return sh

    mod(0.62, 1.72, "… PowerTrain", fill=None, edge=RULE, dash=DASH)
    mod(2.50, 1.55, "Body", fill=MODEL_SOFT, edge=MODEL)
    mod(4.30, 1.62, "Tire", fill=MODEL_SOFT, edge=MODEL, width=2.0)
    mod(6.07, 1.00, "Brake", fill=MODEL_SOFT, edge=MODEL)
    mod(7.22, 1.95, "PowerTrain", fill=None, edge=INK, width=2.5, note="= OUR BLOCK")
    mod(9.32, 1.35, "Body", fill=MODEL_SOFT, edge=MODEL)
    mod(10.95, 1.77, "Tire …", fill=None, edge=RULE, dash=DASH)

    plain_line(s, 4.30, 1.74, 10.67, 1.74, color=HW, width=1.5)

    tf = textbox(s, 0.62, 3.62, 2.0, 0.44)
    line(tf, "shared variable", font=F_MONO, size=8.5, color=INK_3, first=True)
    line(tf, "Rim_rotv", font=F_MONO, size=11, color=HW, bold=True)

    # the lifeline: a step function, drawn as connected segments
    lo, mid, hi = 4.30, 3.98, 3.66
    plain_line(s, 2.34, lo, 2.98, lo, color=HW, width=2.5)
    plain_line(s, 2.98, lo, 2.98, mid, color=HW, width=2.5)
    plain_line(s, 2.98, mid, 7.92, mid, color=HW, width=2.5)
    plain_line(s, 7.92, mid, 7.92, hi, color=HW, width=2.5)
    plain_line(s, 7.92, hi, 12.72, hi, color=HW, width=2.5)

    tf = textbox(s, 3.10, 3.72, 2.4, 0.24)
    line(tf, "holds w[k-1]", font=F_MONO, size=9, color=INK_3, first=True)
    tf = textbox(s, 8.06, 3.40, 2.4, 0.24)
    line(tf, "holds w[k]", font=F_MONO, size=9, color=INK_3, first=True)

    plain_line(s, 5.11, y + h, 5.11, mid, color=MODEL, width=1.5, dash=DASH)
    tf = textbox(s, 5.20, 2.94, 2.2, 0.5)
    line(tf, "READ", font=F_MONO, size=9.5, color=MODEL, bold=True, first=True)
    line(tf, "gets w[k-1]", font=F_MONO, size=9, color=MODEL)

    plain_line(s, 7.92, y + h, 7.92, hi, color=INK, width=1.5, dash=DASH)
    tf = textbox(s, 8.02, 2.94, 2.2, 0.5)
    line(tf, "WRITE", font=F_MONO, size=9.5, color=INK, bold=True, first=True)
    line(tf, "emits w[k]", font=F_MONO, size=9, color=INK)

    sh = box(s, 5.11, 4.72, 2.81, 0.40, fill=HW_SOFT, edge=HW, width=1.5)
    tf = sh.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    line(tf, "this gap IS the z⁻¹", font=F_MONO, size=11, color=HW,
         bold=True, align=PP_ALIGN.CENTER, first=True)
    plain_line(s, 5.11, mid + 0.06, 5.11, 4.72, color=HW, width=1.0, dash=DASH)
    plain_line(s, 7.92, hi + 0.06, 7.92, 4.72, color=HW, width=1.0, dash=DASH)

    tf = textbox(s, 8.20, 4.78, 4.5, 0.3)
    line(tf, "A memoryless block fed a stale", font=F_MONO, size=9, color=INK_3, first=True)
    line(tf, "input still produces a delayed output.", font=F_MONO, size=9, color=INK_3)

    caption(s, "The delay is also REQUIRED: without it, w[k] = w[k-1] + (Ts/I)(Taxl[k] + f(w[k])) is "
               "implicit in w[k] — an algebraic loop", y=5.42)
    caption(s, "needing iteration, which real-time code cannot do. Either the tire's relaxation states "
               "break the loop, or this scheduling delay does.", y=5.66)
    return s


def slide_05(prs):
    s = new_slide(prs, 5, "Two integrations",
                  "The wheel integration is not after PowerTrain — it IS PowerTrain",
                  "Routinely conflated. They happen at different points in the sequence and are "
                  "owned by different parties.")

    labelled_box(s, 0.62, 2.05, 5.9, 1.36, "A · OURS — wheel spin  w",
                 ["At the PowerTrain slot. CM natively does it in the",
                  "DriveLine — “Integration of rotation speeds”, Fig 16.83.",
                  "In our build, our Simulink block IS this step."],
                 fill=HW_SOFT, edge=HW, head_color=HW, width=1.5)

    labelled_box(s, 6.82, 2.05, 5.9, 1.36, "B · CARMAKER — body & suspension",
                 ["At Body Frame, AFTER PowerTrain. Fig 1.23 brackets",
                  "the whole vehicle-model evaluation as",
                  "“One integration step”."],
                 fill=MODEL_SOFT, edge=MODEL, head_color=MODEL, width=1.5)

    mono_block(s, 0.62, 3.62, 12.1, 1.34, [
        ("Tire         reads  Rim_rotv = w[k-1],  emits Trq_T2W[k]      (memoryless map)", INK_2, False),
        ("Brake", INK_2, False),
        ("PowerTrain   INTEGRATION A:  w[k] = w[k-1] + (Ts/I)(Taxl[k] + Trq_T2W[k])", HW, True),
        ("Body Frame   INTEGRATION B:  advances chassis / suspension states", MODEL, True),
    ])

    labelled_box(s, 0.62, 5.18, 5.9, 1.36, "CM's solver is Backward Euler",
                 ["“Since the solver used in CarMaker is backwards-euler,",
                  "the user needs to choose this option.”  Our K*Ts*z/(z-1)",
                  "matches CM's own solver, not just IPG's reference model."],
                 fill=None, edge=RULE, head_color=INK, width=1.25)

    labelled_box(s, 6.82, 5.18, 5.9, 1.36, "THE ONE RULE — add no further delay",
                 ["The ordering supplies exactly one step, free. Any Memory,",
                  "Unit Delay or rate transition on Trq_T2W → integrator →",
                  "rotv makes d = 2, costing a further dzeta ~ 0.14."],
                 fill=WARN_SOFT, edge=WARN, head_color=WARN, width=1.5)
    return s


def slide_06(prs):
    s = new_slide(prs, 6, "Inside the tire",
                  "Almost stateless — and the exception is what saves you")

    tf = textbox(s, 0.62, 2.00, 2.6, 0.3)
    line(tf, "INPUTS", font=F_DISPLAY, size=13, color=MODEL, bold=True, first=True)
    ins = [("P_v0_W[0..2]", INK), ("Rim_rotv   ← our w[k-1]", HW),
           ("Frc_W[2] = Fz", INK), ("InclinAngle", INK), ("muRoad", INK),
           ("Rim_turnv", INK)]
    yy = 2.36
    for t, c in ins:
        tf = textbox(s, 0.62, yy, 3.0, 0.24)
        line(tf, t, font=F_MONO, size=10, color=c, bold=(c is HW), first=True)
        yy += 0.28
    tf = textbox(s, 0.62, yy + 0.14, 3.1, 0.24)
    line(tf, "no torque input exists", font=F_MONO, size=9.5, color=WARN, bold=True, first=True)

    stages = [
        (2.00, 0.72, "1 · kinematics → slip  — algebraic",
         ["s = (Rim_rotv*rBelt_eff - vx) / |…|      alpha = atan(vy/|vx|)"], None, RULE, INK_3),
        (2.86, 0.86, "1.5 · RELAXATION — THE ONLY STATE",
         ["du/dt, dv/dt  (EQ 383-384)    tau = sigma / |Vcx|",
          "2-3 states per wheel · only if USE_MODE >= +10"], HW_SOFT, HW, HW),
        (3.86, 0.62, "2 · pure-slip forces — algebraic",
         ["Fx0(s)        Fy0(alpha)"], None, RULE, INK_3),
        (4.60, 0.86, "3 · COMBINED SLIP — algebraic",
         ["Fx = Gx(alpha) * Fx0(s)      EQ 352",
          "Fy = Gy(s)     * Fy0(alpha)  EQ 360"], MODEL_SOFT, MODEL, MODEL),
        (5.60, 0.68, "4 · spin-axis moment — algebraic",
         ["Trq_T2W  !=  -r*Fx   —  also carries TrqRR (SAE J2452)"], None, RULE, INK_3),
    ]
    for top, hh, head, body, fill, edge, hc in stages:
        sh = box(s, 4.10, top, 5.30, hh, fill=fill, edge=edge,
                 width=2.0 if fill else 1.25)
        tf = sh.text_frame
        line(tf, head, font=F_MONO, size=9.5, color=hc, bold=True, first=True)
        for b in body:
            line(tf, b, font=F_MONO, size=9.5, color=INK_2, space_before=2)

    tf = textbox(s, 9.85, 2.00, 3.0, 0.3)
    line(tf, "OUTPUTS", font=F_DISPLAY, size=13, color=MODEL, bold=True, first=True)
    outs = ["Slp   Alpha", "TurnSlp", "vBelt", "rBelt_eff", "Frc_W  Fx Fy Fz", "Trq_W  Mx My Mz"]
    yy = 2.36
    for t in outs:
        tf = textbox(s, 9.85, yy, 3.0, 0.24)
        line(tf, t, font=F_MONO, size=10, color=INK, first=True)
        yy += 0.28
    tf = textbox(s, 9.85, 4.30, 3.2, 0.3)
    line(tf, "Trq_T2W", font=F_MONO, size=11, color=MODEL, bold=True, first=True)
    tf = textbox(s, 9.85, 4.76, 3.2, 1.0)
    line(tf, "Fz is an INPUT, not a state:", font=F_MONO, size=9, color=INK_3, first=True)
    line(tf, "CPI models carry no", font=F_MONO, size=9, color=INK_3)
    line(tf, "vertical dynamics.", font=F_MONO, size=9, color=INK_3)

    arrow(s, 3.70, 2.90, 4.04, 2.90, color=HW, width=2.5)
    arrow(s, 9.44, 4.40, 9.79, 4.40, color=MODEL, width=2.5)

    caption(s, "The relaxation states set the ~45 Hz mode on the next slide. Without them the tire is an "
               "instantaneous spring from rotv to torque and the", y=6.44)
    caption(s, "delayed loop is far less forgiving. Model.USE_MODE 11/12 gives exactly that stateless "
               "version.", y=6.68)
    return s


def slide_07(prs):
    s = new_slide(prs, 7, "Longitudinal ↔ lateral",
                  "The bench is longitudinal-only, but the load it sees is lateral-aware")

    labelled_box(s, 0.62, 2.20, 2.5, 0.86, "our w[k]", ["→ vBelt → s"],
                 fill=None, edge=HW, head_color=HW, width=2.5)
    labelled_box(s, 0.62, 4.30, 2.5, 0.86, "steer, yaw", ["→ vy(P) → alpha"],
                 fill=None, edge=MODEL, head_color=MODEL, width=1.5)

    labelled_box(s, 4.10, 2.10, 3.3, 1.06, "Fx = Gx(alpha)·Fx0(s)",
                 ["longitudinal force,", "de-rated by sideslip"],
                 fill=HW_SOFT, edge=HW, head_color=INK, width=1.5)
    labelled_box(s, 4.10, 4.20, 3.3, 1.06, "Fy = Gy(s)·Fy0(alpha)",
                 ["lateral force,", "de-rated by long. slip"],
                 fill=MODEL_SOFT, edge=MODEL, head_color=INK, width=1.5)

    labelled_box(s, 8.50, 2.10, 3.0, 1.06, "Trq_T2W",
                 ["back into our", "integrator"],
                 fill=None, edge=WARN, head_color=WARN, width=2.5)
    labelled_box(s, 8.50, 4.20, 3.0, 1.06, "Fz — load transfer",
                 ["pitch → understeer", "balance shifts"],
                 fill=None, edge=RULE, head_color=INK, width=1.25)

    arrow(s, 3.12, 2.63, 4.04, 2.63, color=HW, width=2.5)
    arrow(s, 3.12, 4.73, 4.04, 4.73, color=MODEL, width=2.0)
    arrow(s, 7.40, 2.63, 8.44, 2.63, color=WARN, width=3.0)
    arrow(s, 7.40, 4.73, 8.44, 4.73, color=RULE, width=2.0)

    arrow(s, 6.20, 3.16, 6.20, 4.14, color=HW, width=2.5)
    tf = textbox(s, 6.32, 3.44, 2.0, 0.5)
    line(tf, "A   s → Gy(s)", font=F_MONO, size=10, color=HW, bold=True, first=True)
    line(tf, "     Fy de-rated", font=F_MONO, size=9, color=INK_3)

    arrow(s, 5.30, 4.14, 5.30, 3.16, color=WARN, width=3.0)
    tf = textbox(s, 4.20, 3.52, 1.0, 0.3)
    line(tf, "C", font=F_MONO, size=13, color=WARN, bold=True, first=True)

    arrow(s, 10.00, 4.14, 10.00, 3.16, color=RULE, width=1.5, dash=DASH)
    tf = textbox(s, 7.52, 4.44, 0.9, 0.3)
    line(tf, "B", font=F_MONO, size=13, color=INK_3, bold=True, first=True)

    tf = textbox(s, 11.45, 2.20, 1.8, 1.0)
    line(tf, "PATH C", font=F_MONO, size=10, color=WARN, bold=True, first=True)
    line(tf, "reaches the", font=F_MONO, size=9, color=WARN)
    line(tf, "hardware.", font=F_MONO, size=9, color=WARN)

    tf = textbox(s, 11.45, 4.32, 1.8, 1.0)
    line(tf, "Path B partly", font=F_MONO, size=9, color=INK_3, first=True)
    line(tf, "suppressed:", font=F_MONO, size=9, color=INK_3)
    line(tf, "Supp2WC = 0", font=F_MONO, size=9, color=INK_3)

    caption(s, "In a corner the same axle torque produces a different wheel acceleration — cornering "
               "reaches the bench through the torque, with no lateral", y=5.68)
    caption(s, "signal crossing the interface. All three paths require Model.USE_MODE to select combined "
               "slip; at 11 or 12, paths A and C do not exist.", y=5.92)
    return s


def slide_08(prs):
    s = new_slide(prs, 8, "Stability", "A ~45 Hz mode whose damping grows with speed",
                  "Wheel inertia against tire relaxation gives a second-order system. Natural frequency "
                  "is speed-independent; damping is proportional to speed.")

    mono_block(s, 0.62, 1.98, 12.1, 1.10, [
        ("wn   = r * sqrt( Cs / (I*sigma) )  ~ 286 rad/s ~ 45 Hz   — INDEPENDENT of speed", INK, True),
        ("zeta = vx / (2*sigma*wn)           ~ vx / 28.6           — proportional to speed", INK, False),
        ("delay cost                         ~ wn*Ts/2 = 0.143     — fixed", HW, True),
    ])

    # chart frame
    cx, cy, cw, ch = 0.90, 3.36, 7.10, 2.52
    box(s, cx, cy, cw, ch, fill=None, edge=RULE, width=1.0)
    # zones: standstill (<2 m/s) and exposed band (2 - 4.1 m/s)
    def px(v):  # vx [m/s] -> inches
        return cx + cw * v / 30.0
    def py(z):  # zeta -> inches (0 at bottom, 0.8 at top)
        return cy + ch * (1.0 - z / 0.8)

    z1 = box(s, cx, cy, px(2.0) - cx, ch, fill=MODEL_SOFT, edge=MODEL_SOFT, width=0.5)
    z2 = box(s, px(2.0), cy, px(4.1) - px(2.0), ch, fill=WARN_SOFT, edge=WARN_SOFT, width=0.5)
    for z in (z1, z2):
        z.text_frame.text = ""

    for zt in (0.2, 0.4, 0.6):
        plain_line(s, cx, py(zt), cx + cw, py(zt), color=RULE, width=0.75, dash=DASH)
        tf = textbox(s, cx - 0.46, py(zt) - 0.11, 0.44, 0.22, align=PP_ALIGN.RIGHT)
        line(tf, "%.1f" % zt, font=F_MONO, size=8.5, color=INK_3, first=True)

    plain_line(s, cx, py(0.0), cx + cw, py(0.0), color=INK, width=1.5)
    plain_line(s, px(0), py(0.0), px(30), py(0.8 * 30 / 22.88), color=MODEL, width=2.5, dash=DASH)
    plain_line(s, px(4.09), py(0.0), px(30), py(30 / 28.6 - 0.143), color=HW, width=3.0)

    tf = textbox(s, cx + 4.30, py(0.78), 2.6, 0.24)
    line(tf, "zeta physical", font=F_MONO, size=9.5, color=MODEL, bold=True, first=True)
    tf = textbox(s, cx + 4.30, py(0.52), 2.9, 0.24)
    line(tf, "after 1-step delay", font=F_MONO, size=9.5, color=HW, bold=True, first=True)

    for v in (0, 5, 10, 15, 20, 25, 30):
        tf = textbox(s, px(v) - 0.18, cy + ch + 0.06, 0.4, 0.22, align=PP_ALIGN.CENTER)
        line(tf, str(v), font=F_MONO, size=8.5, color=INK_3, first=True)
    tf = textbox(s, cx + cw - 2.5, cy + ch + 0.28, 2.6, 0.22, align=PP_ALIGN.RIGHT)
    line(tf, "vehicle speed  vx  [m/s]", font=F_MONO, size=9, color=INK_3, first=True)

    tf = textbox(s, cx + 0.06, cy + 0.08, 1.0, 0.9)
    line(tf, "CM stand-", font=F_MONO, size=8, color=MODEL, first=True)
    line(tf, "still model", font=F_MONO, size=8, color=MODEL)
    line(tf, "below 2.0", font=F_MONO, size=8, color=MODEL)
    tf = textbox(s, px(2.05), cy + 0.08, 1.0, 0.9)
    line(tf, "exposed", font=F_MONO, size=8, color=WARN, bold=True, first=True)
    line(tf, "band", font=F_MONO, size=8, color=WARN, bold=True)
    line(tf, "2 – 4.1", font=F_MONO, size=8, color=WARN, bold=True)

    rows = [("vx [m/s]", "zeta phys", "after delay", "verdict"),
            ("20", "0.70", "0.56", "well damped"),
            ("10", "0.35", "0.21", "fine"),
            ("5", "0.175", "0.03", "marginal"),
            ("2", "0.070", "-0.07", "mode switch")]
    tx, ty = 8.35, 3.36
    tbl = s.shapes.add_table(len(rows), 4, inch(tx), inch(ty), inch(4.37), inch(2.0)).table
    for c, wdt in enumerate((1.05, 1.05, 1.20, 1.35)):
        tbl.columns[c].width = inch(wdt)
    for r, row in enumerate(rows):
        for c, val in enumerate(row):
            cell = tbl.cell(r, c)
            cell.text = val
            cell.fill.solid()
            cell.fill.fore_color.rgb = PANEL if r == 0 else PAPER
            cell.margin_left = cell.margin_right = inch(0.07)
            p = cell.text_frame.paragraphs[0]
            p.alignment = PP_ALIGN.LEFT
            f = p.runs[0].font
            f.name = F_MONO
            f.size = Pt(9.5 if r else 8.5)
            f.bold = (r == 0)
            f.color.rgb = INK_3 if r == 0 else (WARN if r == len(rows) - 1 else INK)

    caption(s, "The mode is PHYSICAL — the tire/wheel torsional resonance a real vehicle has. The delay "
               "only removes damping from it. Below 2.0 m/s CarMaker", y=6.10)
    caption(s, "switches to its stand-still tire model. The residual 2–4.1 m/s band is covered on the "
               "bench by switching the dyno to torque mode with Ttire = 0.", y=6.34)
    return s


def slide_09(prs):
    s = new_slide(prs, 9, "Verdict", "Why it works, and the two things it cannot tell you")

    why = [("One integrator",
            ["Divergence needs two states drifting apart.", "There is one. CM's tire slip comes from",
             "exactly the number that commands the dyno."]),
           ("IPG's sanctioned bypass",
            ["Backward Euler at 1 ms with Wheel.<i>.I read", "from CM parameters — block-for-block",
             "identical to UserPowerTrain.mdl."]),
           ("Bounded, not divergent",
            ["A 2 N·m torque bias yields 1e-4 slip offset", "and ~0.24 m/s speed offset: tire slip",
             "stiffness and road-load drag both restore."])]
    for i, (h, b) in enumerate(why):
        labelled_box(s, 0.62 + i * 4.10, 1.80, 3.85, 1.55, h, b,
                     fill=MODEL_SOFT, edge=MODEL, head_color=MODEL, width=1.5)

    blind = [("BLIND SPOT — the servo error is invisible",
              ["CM never sees w_act. If the servo saturates we integrate a torque measured at the",
               "wrong operating point. Nothing rings, nothing diverges — the sim quietly becomes a",
               "different vehicle.  ACTION: log w_act - w_cmd and alarm on it."]),
             ("BLIND SPOT — Taxl cannot be validated against CM",
              ["CarMaker never sees Taxl dynamically, so a torque-measurement error surfaces only",
               "as that ~0.24 m/s bias — indistinguishable from a dozen modelling choices.",
               "Calibration must be established on the bench."])]
    for i, (h, b) in enumerate(blind):
        labelled_box(s, 0.62 + i * 6.20, 3.58, 5.95, 1.45, h, b,
                     fill=WARN_SOFT, edge=WARN, head_color=WARN, width=1.5)

    rest = [("Still open",
             ["Model.USE_MODE in the tire file — decides both", "whether combined slip exists and which",
              "stability analysis applies. Expected 13/14/15."]),
            ("Recorded simplifications",
             ["All CM brake torque zeroed (real braking is inside", "Taxl). Trq_Supp2WC / Supp2Bdy1 / Supp2BdyEng",
              "zeroed — no squat, no engine-mount reaction."]),
            ("Full detail",
             ["doc/xil/AxleDynoCarMakerCoupling.md — k-indexed", "equations, source citations, and the",
              "open-item register."])]
    for i, (h, b) in enumerate(rest):
        labelled_box(s, 0.62 + i * 4.10, 5.26, 3.85, 1.35, h, b,
                     fill=None, edge=RULE, head_color=INK, width=1.25)
    return s


def main() -> None:
    prs = Presentation()
    prs.slide_width, prs.slide_height = SW, SH
    for fn in (slide_01, slide_02, slide_03, slide_04,
               slide_05, slide_06, slide_07, slide_08, slide_09):
        fn(prs)
    prs.save(OUT)
    print("wrote %s  (%d slides, %.0f KB)"
          % (OUT.name, len(prs.slides.__iter__.__self__._sldIdLst), OUT.stat().st_size / 1024))


if __name__ == "__main__":
    main()
