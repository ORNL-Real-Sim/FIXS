#!/usr/bin/env python
"""Regenerate AxleDynoCarMakerCoupling.pptx.

The deck is a build product. This script is its source -- a .pptx does not diff,
so edit here and regenerate rather than editing the binary, unless the change is
one-off presentation polish.

Everything is emitted as native PowerPoint shapes (rectangles, connectors, text
boxes), never as embedded images, so every box and arrow stays editable in
PowerPoint. Content follows AxleDynoCarMakerCoupling.md.

Six slides, in one chain: the loop -> when Trq_T2W[k] is computed within the
integration step -> how the tire model computes it -> what can make the loop
ring -> summary.

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
    r = p.add_run(); r.text = "%02d / 06" % num
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
    s = new_slide(prs, 2, "The loop",
                  "What each part owns, and what crosses between them",
                  "Three parts, four signals, one equation. The wheel speed is computed in "
                  "Simulink and sent to both sides. Nothing else crosses.")

    labelled_box(s, 0.62, 2.02, 3.15, 1.70, "HARDWARE",
                 ["real powertrain on the dyno", "",
                  "The dyno speed controller", "makes the shaft follow w."],
                 fill=HW_SOFT, edge=HW, head_color=HW, width=1.5)

    labelled_box(s, 4.72, 1.86, 3.90, 2.02, "SIMULINK — OUR BLOCK",
                 ["w[k] = w[k-1]",
                  "       + (Ts / I) * ( Taxl[k]",
                  "       + Trq_T2W[k] )",
                  "",
                  "Ts = 0.001 s, same step as CarMaker",
                  "I  = Wheel.<i>.I, read from CarMaker"],
                 fill=None, edge=INK, head_color=INK, width=2.5)

    labelled_box(s, 9.57, 2.02, 3.15, 1.70, "CARMAKER",
                 ["tire, body, suspension, road", "",
                  "Uses w to find tire slip,", "then returns the tire torque."],
                 fill=MODEL_SOFT, edge=MODEL, head_color=MODEL, width=1.5)

    # hardware -> simulink
    arrow(s, 3.77, 2.42, 4.66, 2.42, color=HW, width=2.5)
    tf = textbox(s, 3.80, 2.12, 1.6, 0.24)
    line(tf, "Taxl[k]", font=F_MONO, size=10, color=HW, bold=True, first=True)
    tf = textbox(s, 3.80, 2.62, 1.7, 0.5)
    line(tf, "measured axle", font=F_MONO, size=8.5, color=INK_3, first=True)
    line(tf, "torque [Nm]", font=F_MONO, size=8.5, color=INK_3)

    # simulink -> hardware
    arrow(s, 4.66, 3.42, 3.77, 3.42, color=HW, width=2.5, dash=DASH)
    tf = textbox(s, 3.80, 3.50, 1.8, 0.5)
    line(tf, "w[k] as speed", font=F_MONO, size=8.5, color=HW, first=True)
    line(tf, "command [rad/s]", font=F_MONO, size=8.5, color=HW)

    # simulink -> carmaker
    arrow(s, 8.62, 2.42, 9.51, 2.42, color=MODEL, width=2.5)
    tf = textbox(s, 8.65, 2.12, 1.6, 0.24)
    line(tf, "w[k]", font=F_MONO, size=10, color=MODEL, bold=True, first=True)
    tf = textbox(s, 8.65, 2.62, 1.8, 0.5)
    line(tf, "written to rotv", font=F_MONO, size=8.5, color=INK_3, first=True)
    line(tf, "[rad/s]", font=F_MONO, size=8.5, color=INK_3)

    # carmaker -> simulink
    arrow(s, 9.51, 3.42, 8.62, 3.42, color=MODEL, width=2.5)
    tf = textbox(s, 8.65, 3.50, 1.8, 0.5)
    line(tf, "Trq_T2W[k]", font=F_MONO, size=9, color=MODEL, bold=True, first=True)
    line(tf, "tire torque [Nm]", font=F_MONO, size=8.5, color=INK_3)

    plain_line(s, 0.62, 4.30, 12.72, 4.30, color=RULE)

    facts = [
        ("The same w[k] goes to both sides.", HW,
         "CarMaker is given the computed speed, not the measured one."),
        ("The real shaft speed stays in the dyno controller.", INK,
         "If the servo lags, CarMaker never finds out."),
        ("CarMaker has no wheel speed of its own.", MODEL,
         "Its integrator went with the PowerTrain module, so ours is the only one."),
        ("CarMaker never uses Taxl either.", INK,
         "We send it, but with the support torques zeroed nothing reads it."),
    ]
    yy = 4.46
    for head, col, tail in facts:
        tf = textbox(s, 0.62, yy, 12.1, 0.24)
        p = tf.paragraphs[0]
        r = p.add_run(); r.text = head
        r.font.name, r.font.size, r.font.bold = F_MONO, Pt(10), True
        r.font.color.rgb = col
        r = p.add_run(); r.text = "   " + tail
        r.font.name, r.font.size = F_MONO, Pt(10)
        r.font.color.rgb = INK_2
        yy += 0.30

    caption(s, "Read it as a sentence: add the measured axle torque to the tire torque from CarMaker, "
               "divide by the wheel inertia, integrate.", y=5.82)
    caption(s, "CarMaker does this same integration natively in its DriveLine (RefMan Fig. 16.83). We "
               "rebuilt that block with the same inertia parameter,", y=6.06)
    caption(s, "matching IPG's own UserPowerTrain.mdl. All CarMaker brake torque is zeroed here, because "
               "real braking is already inside Taxl.", y=6.30)
    return s


def slide_03(prs):
    s = new_slide(prs, 3, "When", "One integration step, in a fixed order",
                  "Everything from here on answers one question: how is Trq_T2W[k] produced. "
                  "Start with when CarMaker computes it.")

    # bracket marking the integration step
    plain_line(s, 0.72, 1.94, 0.72, 4.86, color=HW, width=2.0)
    plain_line(s, 0.72, 1.94, 0.94, 1.94, color=HW, width=2.0)
    plain_line(s, 0.72, 4.86, 0.94, 4.86, color=HW, width=2.0)
    tf = textbox(s, 0.36, 2.20, 2.4, 0.24)
    line(tf, "ONE INTEGRATION STEP", font=F_MONO, size=8.5, color=HW, bold=True, first=True)
    tf = textbox(s, 0.36, 2.42, 2.4, 0.24)
    line(tf, "Ts = 1 ms", font=F_MONO, size=8.5, color=HW, first=True)

    tf = textbox(s, 1.10, 1.90, 5.0, 0.24)
    line(tf, "Vehicle", font=F_MONO, size=11, color=INK, bold=True, first=True)
    for i, sub in enumerate(["Steering", "Suspension kinematics & compliance",
                             "Aerodynamics", "Suspension forces"]):
        tf = textbox(s, 1.44, 2.14 + i * 0.24, 5.0, 0.24)
        line(tf, sub, font=F_MONO, size=10, color=INK_2, first=True)

    tire = box(s, 1.28, 3.16, 3.55, 0.62, fill=MODEL_SOFT, edge=MODEL, width=2.0)
    tf = tire.text_frame
    line(tf, "Tire", font=F_MONO, size=11, color=MODEL, bold=True, first=True)
    line(tf, "reads the wheel speed as it stands", font=F_MONO, size=9, color=INK_2, space_before=2)

    tf = textbox(s, 1.10, 3.92, 5.0, 0.24)
    line(tf, "Brake", font=F_MONO, size=11, color=INK, bold=True, first=True)

    pt = box(s, 1.28, 4.22, 3.55, 0.62, fill=None, edge=INK, width=2.5)
    tf = pt.text_frame
    p = tf.paragraphs[0]
    r = p.add_run(); r.text = "PowerTrain"
    r.font.name, r.font.size, r.font.bold = F_MONO, Pt(11), True
    r.font.color.rgb = INK
    r = p.add_run(); r.text = "     <- our block"
    r.font.name, r.font.size = F_MONO, Pt(9)
    r.font.color.rgb = HW
    line(tf, "advances the wheel speed state", font=F_MONO, size=9, color=INK_2, space_before=2)

    bf = box(s, 1.28, 4.98, 3.55, 0.36, fill=MODEL_SOFT, edge=MODEL, width=1.5)
    tf = bf.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    line(tf, "Body Frame   advances the chassis states",
         font=F_MONO, size=9.5, color=MODEL, bold=True, first=True)

    arrow(s, 3.05, 3.78, 3.05, 4.16, color=RULE, width=2.0)
    arrow(s, 3.05, 4.84, 3.05, 4.92, color=RULE, width=2.0)

    arrow(s, 4.83, 3.47, 5.40, 3.47, color=MODEL, width=2.0)
    rb = box(s, 5.46, 3.16, 3.30, 0.62, fill=None, edge=MODEL, width=1.5)
    tf = rb.text_frame
    line(tf, "uses  w[k-1]", font=F_MONO, size=10.5, color=MODEL, bold=True, first=True)
    line(tf, "emits Trq_T2W[k]", font=F_MONO, size=10.5, color=MODEL, bold=True, space_before=2)

    arrow(s, 4.83, 4.53, 5.40, 4.53, color=INK, width=2.5)
    eb = box(s, 5.46, 4.22, 4.60, 0.62, fill=HW_SOFT, edge=HW, width=2.0)
    tf = eb.text_frame
    line(tf, "w[k] = w[k-1] + (Ts / I) *", font=F_MONO, size=10.5, color=INK, first=True)
    line(tf, "           ( Taxl[k] + Trq_T2W[k] )", font=F_MONO, size=10.5, color=INK, space_before=2)

    caption(s, "The tire is evaluated before the wheel speed is advanced, so it uses the value the "
               "state holds at the start of the step.", y=5.56, color=HW)
    caption(s, "That is what advancing an ODE state means. It is not a delay added by our bypass: "
               "CarMaker's own powertrain does the same thing.", y=5.80)

    labelled_box(s, 0.62, 6.10, 3.85, 1.06, "Two states advance here",
                 ["Wheel speed at the PowerTrain slot,", "chassis afterwards at Body Frame."],
                 fill=None, edge=MODEL, head_color=MODEL, width=1.25)
    labelled_box(s, 4.72, 6.10, 3.85, 1.06, "Backward Euler",
                 ["CarMaker's own solver; ours matches.", "Single stage, so this runs once per step."],
                 fill=None, edge=RULE, head_color=INK, width=1.25)
    labelled_box(s, 8.82, 6.10, 3.90, 1.06, "Do not add another step",
                 ["A Memory block or rate transition on", "Trq_T2W -> integrator -> rotv costs damping."],
                 fill=WARN_SOFT, edge=WARN, head_color=WARN, width=1.5)
    return s


def slide_04(prs):
    s = new_slide(prs, 4, "How", "What the tire model does with the wheel speed",
                  "The bench supplies one quantity: the wheel rotational speed. Follow it down "
                  "through the tire model to the torque that comes back.")

    labelled_box(s, 0.62, 1.94, 5.55, 0.86, "FROM THE BENCH",
                 ["w[k-1]  ->  Rim_rotv", "the wheel rotational speed, and nothing else"],
                 fill=HW_SOFT, edge=HW, head_color=HW, width=2.0)
    labelled_box(s, 7.17, 1.94, 5.55, 0.86, "FROM CARMAKER'S BODY MODEL",
                 ["P_v0_W · Fz · camber · muRoad", "contact-point velocity, load, friction"],
                 fill=MODEL_SOFT, edge=MODEL, head_color=MODEL, width=1.5)

    plain_line(s, 3.40, 2.80, 3.40, 3.00, color=HW, width=2.5)
    plain_line(s, 9.95, 2.80, 9.95, 3.00, color=MODEL, width=2.0)
    plain_line(s, 3.40, 3.00, 9.95, 3.00, color=RULE, width=2.0)
    arrow(s, 6.67, 3.00, 6.67, 3.20, color=RULE, width=2.5)

    stage = box(s, 0.62, 3.26, 12.10, 0.90, fill=None, edge=RULE, width=1.5)
    tf = stage.text_frame
    p = tf.paragraphs[0]
    for txt, col, bold, sz in (("1   SLIP        ", INK, True, 10.5),
                               ("longitudinal   ", HW, True, 10),
                               ("s = ( w · rBelt_eff - vx ) / |…|      ", INK, False, 10),
                               ("<- the bench sets this", HW, False, 9)):
        r = p.add_run(); r.text = txt
        r.font.name, r.font.size, r.font.bold = F_MONO, Pt(sz), bold
        r.font.color.rgb = col
    p2 = tf.add_paragraph(); p2.space_before = Pt(2)
    for txt, col, bold, sz in (("                lateral        ", MODEL, True, 10),
                               ("alpha = atan( vy / |vx| )             ", INK, False, 10),
                               ("<- steering and body yaw", MODEL, False, 9)):
        r = p2.add_run(); r.text = txt
        r.font.name, r.font.size, r.font.bold = F_MONO, Pt(sz), bold
        r.font.color.rgb = col
    line(tf, "Both come from one contact-point velocity vector, split into its two components.",
         font=F_MONO, size=9, color=INK_3, space_before=3)

    arrow(s, 6.67, 4.16, 6.67, 4.34, color=RULE, width=2.5)

    relax = box(s, 0.62, 4.40, 12.10, 0.76, fill=HW_SOFT, edge=HW, width=2.0)
    tf = relax.text_frame
    p = tf.paragraphs[0]
    for txt, col, bold, sz in (("2   RELAXATION  ", HW, True, 10.5),
                               ("tau = sigma / vx    sigma ~ 0.05 m      ", INK, False, 10),
                               ("the tire's only ODE states", HW, False, 9)):
        r = p.add_run(); r.text = txt
        r.font.name, r.font.size, r.font.bold = F_MONO, Pt(sz), bold
        r.font.color.rgb = col
    line(tf, "The carcass takes a rolling distance to build force, so slip is low-passed before it "
             "becomes force. Everything else here is algebraic.",
         font=F_MONO, size=9, color=INK_3, space_before=3)

    arrow(s, 6.67, 5.16, 6.67, 5.34, color=RULE, width=2.5)

    forces = box(s, 0.62, 5.40, 12.10, 0.90, fill=MODEL_SOFT, edge=MODEL, width=2.0)
    tf = forces.text_frame
    p = tf.paragraphs[0]
    for txt, col, bold, sz in (("3   FORCES      ", MODEL, True, 10.5),
                               ("Fx = Gx(alpha) · Fx0(s)      ", INK, False, 10),
                               ("longitudinal force, reduced by cornering", INK_3, False, 9)):
        r = p.add_run(); r.text = txt
        r.font.name, r.font.size, r.font.bold = F_MONO, Pt(sz), bold
        r.font.color.rgb = col
    p2 = tf.add_paragraph(); p2.space_before = Pt(2)
    for txt, col, bold, sz in (("                Fy = Gy(s)     · Fy0(alpha)  ", INK, False, 10),
                               ("lateral force, reduced by driving or braking", INK_3, False, 9)):
        r = p2.add_run(); r.text = txt
        r.font.name, r.font.size, r.font.bold = F_MONO, Pt(sz), bold
        r.font.color.rgb = col
    line(tf, "Gx and Gy are the friction ellipse: one contact patch, one friction budget, so the two "
             "directions take from each other.",
         font=F_MONO, size=9, color=INK_3, space_before=3)

    arrow(s, 6.67, 6.30, 6.67, 6.48, color=RULE, width=2.5)

    trq = box(s, 0.62, 6.54, 12.10, 0.44, fill=None, edge=WARN, width=2.5)
    tf = trq.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    for txt, col, bold, sz in (("4   SPIN-AXIS TORQUE   ", WARN, True, 10.5),
                               ("Trq_T2W = -r · Fx  +  rolling resistance      ", INK, False, 10),
                               ("-> back to our integrator", WARN, True, 9)):
        r = p.add_run(); r.text = txt
        r.font.name, r.font.size, r.font.bold = F_MONO, Pt(sz), bold
        r.font.color.rgb = col
    return s


def slide_05(prs):
    s = new_slide(prs, 5, "What can go wrong", "The wheel can oscillate against the tire",
                  "The wheel has inertia. The tire sits between it and the road and acts like a "
                  "spring. That is an oscillator, and it exists on any real vehicle.")

    tf = textbox(s, 0.62, 1.96, 4.0, 0.26)
    line(tf, "WHAT OSCILLATES", font=F_DISPLAY, size=13, color=INK, bold=True, first=True)

    wheel = s.shapes.add_shape(MSO_SHAPE.DONUT, inch(0.90), inch(2.34), inch(1.50), inch(1.50))
    wheel.shadow.inherit = False
    wheel.fill.background()
    wheel.line.color.rgb = INK
    wheel.line.width = Pt(2.5)
    tf = textbox(s, 1.40, 2.98, 0.5, 0.26, align=PP_ALIGN.CENTER)
    line(tf, "I", font=F_MONO, size=13, color=INK, bold=True, first=True)
    tf = textbox(s, 0.72, 3.92, 2.0, 0.24)
    line(tf, "wheel rocks back", font=F_MONO, size=9, color=HW, first=True)
    line(tf, "and forth", font=F_MONO, size=9, color=HW)

    spring = s.shapes.add_shape(MSO_SHAPE.LIGHTNING_BOLT, inch(2.52), inch(2.90),
                                inch(1.20), inch(0.40))
    spring.shadow.inherit = False
    spring.fill.background()
    spring.line.color.rgb = MODEL
    spring.line.width = Pt(2.0)
    tf = textbox(s, 2.46, 2.56, 1.6, 0.24)
    line(tf, "tire carcass", font=F_MONO, size=9, color=MODEL, first=True)
    tf = textbox(s, 2.46, 3.36, 1.7, 0.24)
    line(tf, "acts as a spring", font=F_MONO, size=9, color=MODEL, first=True)

    plain_line(s, 3.92, 2.66, 3.92, 3.58, color=INK, width=3.0)
    tf = textbox(s, 4.02, 2.98, 0.8, 0.24)
    line(tf, "road", font=F_MONO, size=9, color=INK_3, first=True)

    tf = textbox(s, 0.62, 4.34, 4.6, 1.1)
    for t in ["The carcass has to flex before it can pass",
              "force to the road. CarMaker gives that flex a",
              "length: about 5 cm of rolling. A compliance is",
              "a spring, and a spring against the wheel",
              "inertia is an oscillator."]:
        line(tf, t, font=F_MONO, size=9.5, color=INK_2, first=(t.startswith("The carcass")))

    plain_line(s, 5.50, 1.90, 5.50, 5.60, color=RULE, width=1.0)

    tf = textbox(s, 5.80, 1.96, 6.9, 0.26)
    line(tf, "WHERE THE DAMPING COMES FROM", font=F_DISPLAY, size=13, color=INK, bold=True, first=True)
    tf = textbox(s, 5.80, 2.28, 6.9, 0.9)
    line(tf, "As the tire rolls it continuously lays down fresh rubber and renews its grip.",
         font=F_MONO, size=9.5, color=INK_2, first=True)
    line(tf, "That is what settles the oscillation.", font=F_MONO, size=9.5, color=INK_2)
    line(tf, "Faster rolling renews it faster, so damping grows with speed.",
         font=F_MONO, size=9.5, color=HW, bold=True, space_before=4)
    line(tf, "At a standstill there is none at all.", font=F_MONO, size=9.5, color=HW, bold=True)

    tf = textbox(s, 5.80, 3.52, 6.9, 0.26)
    line(tf, "WHAT THE INTEGRATION STEP COSTS", font=F_DISPLAY, size=13, color=INK, bold=True, first=True)
    tf = textbox(s, 5.80, 3.84, 6.9, 1.0)
    line(tf, "The tire is evaluated once per step, from the wheel speed as it stood at",
         font=F_MONO, size=9.5, color=INK_2, first=True)
    line(tf, "the start of it. That always removes a small, fixed amount of damping.",
         font=F_MONO, size=9.5, color=INK_2)
    line(tf, "At road speed there is plenty to spare. Down near walking pace there is",
         font=F_MONO, size=9.5, color=WARN, space_before=4)
    line(tf, "not, and the trace can ring.", font=F_MONO, size=9.5, color=WARN)

    tf = textbox(s, 5.80, 5.02, 6.9, 0.26)
    line(tf, "EXAMPLE — rotv OR Trq_T2W DURING A SLOW CRAWL",
         font=F_DISPLAY, size=12, color=INK_3, bold=True, first=True)
    # healthy trace
    pts = [(5.90, 5.48), (6.60, 5.48), (6.80, 5.42), (7.00, 5.52), (7.20, 5.47), (8.10, 5.48)]
    for a, b in zip(pts, pts[1:]):
        plain_line(s, a[0], a[1], b[0], b[1], color=MODEL, width=2.0)
    tf = textbox(s, 8.20, 5.36, 1.0, 0.24)
    line(tf, "healthy", font=F_MONO, size=9, color=MODEL, first=True)
    # ringing trace
    rp = [(9.30, 5.48), (9.70, 5.48), (9.82, 5.14), (9.94, 5.78), (10.06, 5.26),
          (10.18, 5.66), (10.30, 5.38), (10.42, 5.56), (10.54, 5.46), (11.20, 5.48)]
    for a, b in zip(rp, rp[1:]):
        plain_line(s, a[0], a[1], b[0], b[1], color=WARN, width=2.0)
    tf = textbox(s, 11.30, 5.36, 1.2, 0.24)
    line(tf, "ringing", font=F_MONO, size=9, color=WARN, bold=True, first=True)

    caption(s, "This oscillation is physical, not a modelling artefact. A real car has it: it is the "
               "same mechanism behind brake judder and ABS chatter.", y=5.86)

    labelled_box(s, 0.62, 6.14, 3.85, 1.04, "Why we do not see it",
                 ["Below 2 m/s CarMaker switches to its", "stand-still tire model, and we put the dyno",
                  "in torque mode while stopping."],
                 fill=None, edge=MODEL, head_color=MODEL, width=1.25)
    labelled_box(s, 4.72, 6.14, 3.85, 1.04, "What raises the risk",
                 ["A longer sample time, a smaller wheel", "inertia, a shorter relaxation length,",
                  "or a second delay in the loop."],
                 fill=None, edge=HW, head_color=HW, width=1.25)
    labelled_box(s, 8.82, 6.14, 3.90, 1.04, "The frequency, if you want it",
                 ["Tens of hertz for a passenger wheel; we", "estimate 40-50 Hz from a textbook slip",
                  "stiffness. Read Cs from the tire file."],
                 fill=None, edge=RULE, head_color=INK, width=1.25)
    return s


def slide_06(prs):
    s = new_slide(prs, 6, "Summary", "What we can rely on, and what to check",
                  "The loop is sound. Three reasons, then the two things it cannot report on "
                  "itself, then the open items.")

    rely = [("There is only one wheel speed",
             ["Not one in the model and another on the bench.", "Ours is the only one, so the two cannot",
              "disagree."]),
            ("It is IPG's own bypass",
             ["Backward Euler at 1 ms, wheel inertia read", "from CarMaker, block for block the same as",
              "UserPowerTrain.mdl."]),
            ("Errors stay small",
             ["A 2 N·m torque bias settles at about", "0.24 m/s of speed offset. It does not grow:",
              "slip stiffness and drag both push back."])]
    for i, (h, b) in enumerate(rely):
        labelled_box(s, 0.62 + i * 4.10, 1.90, 3.85, 1.30, h, b,
                     fill=MODEL_SOFT, edge=MODEL, head_color=MODEL, width=1.5)

    blind = [("We cannot see the servo error",
              ["CarMaker is given the commanded speed, so it never learns whether the shaft reached",
               "it. If the servo saturates we integrate a torque produced at a different speed, and",
               "nothing warns us.  Log w_act - w_cmd and set an alarm on it."]),
             ("We cannot check Taxl against the model",
              ["CarMaker never reads the axle torque, so a measurement error shows up only as that",
               "small speed offset, which looks like any other modelling choice. Torque calibration",
               "has to be done on the bench."])]
    for i, (h, b) in enumerate(blind):
        labelled_box(s, 0.62 + i * 6.20, 3.42, 5.95, 1.42, h, b,
                     fill=WARN_SOFT, edge=WARN, head_color=WARN, width=1.5)

    rest = [("To check",
             ["Model.USE_MODE in the tire file. 13, 14 or 15", "is expected. It decides both the lateral",
              "coupling and the low-speed behaviour."]),
            ("Recorded simplifications",
             ["All CarMaker brake torque zeroed, since real", "braking is inside Taxl. Support torques",
              "zeroed too: no squat, no mount reaction."]),
            ("Full detail",
             ["doc/xil/AxleDynoCarMakerCoupling.md has the", "step-indexed equations, the source",
              "references and the open-item list."])]
    for i, (h, b) in enumerate(rest):
        labelled_box(s, 0.62 + i * 4.10, 5.06, 3.85, 1.30, h, b,
                     fill=None, edge=RULE, head_color=INK, width=1.25)
    return s


def main() -> None:
    prs = Presentation()
    prs.slide_width, prs.slide_height = SW, SH
    for fn in (slide_01, slide_02, slide_03, slide_04, slide_05, slide_06):
        fn(prs)
    prs.save(OUT)
    print("wrote %s  (%d slides, %.0f KB)"
          % (OUT.name, len(prs.slides.__iter__.__self__._sldIdLst), OUT.stat().st_size / 1024))


if __name__ == "__main__":
    main()
