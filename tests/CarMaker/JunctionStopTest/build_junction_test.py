"""
FIXS #172 -- minimal 2-edge + 1-junction verification of the CM traffic-light stop mechanism.

Scene (from netconvert -> osc2cm): a signalized junction jC; the ego drives the main road
straight through, W -> jC -> E, on lane-path chain 15 (approach, RL.1) -> 218 (connector) ->
109 (departure, RL.95). The approach edge RL.1 carries the straight head (controller 31).

We force controller 31 to STATIC RED and add a DrvStop on the approach (referencing ctrl 31),
then build three variants that differ ONLY in where the signal head is, to settle definitively
whether the head must be on the approach for the DrvStop to fire:

  approach : straight head on the approach edge RL.1 (osc2cm default)        [baseline]
  nohead   : NO head on the approach (RL.1 heads removed)                    [doc says: still stops]
  farside  : straight head moved to the departure edge RL.95 (across junction)[far-side]

Run:  python build_junction_test.py <approach|nohead|farside>
"""
from __future__ import annotations
import re, pathlib, sys

HERE = pathlib.Path(__file__).resolve().parent
CMPROJ = HERE.parents[2] / "ProprietaryFiles" / "CM13_proj"
BASE_RD = CMPROJ / "Data" / "Road" / "junction.rd5"
TEMPLATE = HERE.parent / "SignalStopTest" / "driver_template.cmtestrun"  # McLaren + Car_Normal

APPR_RL = 1        # approach edge (lane-path 15)
DEP_RL = 95        # departure edge (lane-path 109), across the junction
STRAIGHT_CTRL = 31 # straight-movement controller on the approach
PATH = [15, 218, 109]   # approach -> connector -> departure
STOP_BACK = 12.0


def strip_routes(t):
    """Remove any pre-existing Route definition (e.g. one saved into the rd5 by the CarMaker
    Scenario Editor) so we don't end up with two Route.0 blocks -> CM syntax error."""
    t = re.sub(r"(?m)^Route\.\d+\.Length = .*\n", "", t)
    t = re.sub(r"(?m)^Route\.\d+\.(ID|Name|DrvPath\.ID) = .*\n", "", t)
    t = re.sub(r"(?m)^Route\.\d+\.DrvPath:\n(?:\t.*\n)*", "", t)
    return t


def build(variant):
    t = BASE_RD.read_text(encoding="utf-8", errors="ignore")
    t = strip_routes(t)
    maxid = int(re.search(r"MaxUsedObjId = (\d+)", t).group(1))
    nid = maxid

    def newid():
        nonlocal nid; nid += 1; return nid

    lines = t.split("\n")

    # --- head variant: drop / move the approach (RL.1) heads ---
    far_head = None
    if variant in ("nohead", "farside"):
        if variant == "farside":
            # remember the straight head line (ctrl 31) to re-mount on the departure edge
            for ln in lines:
                if re.match(rf"RL\.{APPR_RL}\.Mount\.0\.\d+ = 1 {STRAIGHT_CTRL} ", ln):
                    far_head = ln.split("=", 1)[1].split()      # [1,ctrl,vOff,hOff,...,facing,type]
        # remove the whole approach mount block (all RL.1.Mount.* lines)
        lines = [ln for ln in lines if not re.match(rf"RL\.{APPR_RL}\.Mount\.", ln)]
    t = "\n".join(lines)

    blocks = ""
    if variant == "farside" and far_head:
        mid, hid = newid(), newid()
        p = far_head
        p[-2] = "-1"                                            # face back toward the junction
        blocks += (f"RL.{DEP_RL}.Mount.0.ID = {mid} {DEP_RL}\n"
                   f"RL.{DEP_RL}.Mount.0 = 1.0 0 -1 -1 1 9 0 0 0 4.2 5.39\n"
                   f"RL.{DEP_RL}.Mount.0.0.ID = {hid}\n"
                   f"RL.{DEP_RL}.Mount.0.0 = " + " ".join(p) + "\n"
                   f"RL.{DEP_RL}.Mount.0.0.Tag = odrSignalId:farside_{STRAIGHT_CTRL}\n")

    # --- force controller 31 to static red ---
    t = re.sub(rf"(Control\.TrfLight\.\d+ = {STRAIGHT_CTRL} \S+) \".*",
               r'\1 "" 3 0 0 0 99999 1', t)

    # --- Route over the ego's path (replace nRoutes=0) ---
    rid, did = newid(), newid()
    blocks += (f"Route.0.ID = {rid}\nRoute.0.Name = TestRoute\nRoute.0.DrvPath.ID = {did}\n"
               "Route.0.DrvPath:\n" + "".join(f"\t{lp}\n" for lp in PATH))

    # --- DrvStop on the approach lane-path (ref ctrl 31), STOP_BACK before the junction ---
    idxs = [int(x) for x in re.findall(rf"(?m)^RL\.{APPR_RL}\.Marker\.(\d+)\.ID = ", t)]
    mk_i = (max(idxs) + 1) if idxs else 0
    blocks += (f"RL.{APPR_RL}.Marker.{mk_i}.ID = {newid()} {PATH[0]}\n"
               f"RL.{APPR_RL}.Marker.{mk_i}.Type = DrvStop\n"
               f"RL.{APPR_RL}.Marker.{mk_i}.Param = {STOP_BACK} 1 1 {STRAIGHT_CTRL} 2 0\n")

    t = re.sub(r"(?m)^nRoutes = \d+", "nRoutes = 1", t, count=1)
    t = t.replace(f"MaxUsedObjId = {maxid}", blocks + f"MaxUsedObjId = {nid}")
    out_rd = CMPROJ / "Data" / "Road" / f"junction_{variant}.rd5"
    out_rd.write_text(t, encoding="utf-8")

    # --- TestRun: clone the proven McLaren+Car_Normal template, point at this road/route ---
    tr = TEMPLATE.read_text(encoding="utf-8", errors="ignore")
    tr = re.sub(r"(?m)^Road\.FName = .*$", f"Road.FName = junction_{variant}.rd5", tr, 1)
    tr = re.sub(r"(?m)^Vehicle\.Routing\.ObjId = .*$", f"Vehicle.Routing.ObjId = {rid}", tr, 1)
    tr = re.sub(r"(?m)^Vehicle\.StartPos\.ObjId = .*$", f"Vehicle.StartPos.ObjId = {rid}", tr, 1)
    tr = re.sub(r"(?m)^Vehicle\.StartPos = .*$", "Vehicle.StartPos = 5.0 0", tr, 1)
    tr = re.sub(r"(?m)^DrivMan\.Global\.EndCond = .*$", "DrivMan.Global.EndCond = rise(Time > 60.00)", tr, 1)
    (CMPROJ / "Data" / "TestRun" / f"junction_{variant}").write_text(tr, encoding="utf-8")

    nheads = len(re.findall(r"Mount\.\d+\.\d+ = 1 \d+ ", t))
    print(f"[{variant}] junction_{variant}.rd5 (ctrl {STRAIGHT_CTRL} static RED, DrvStop on approach "
          f"RL.{APPR_RL}; total heads={nheads}) + TestRun junction_{variant}")


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else "approach")
