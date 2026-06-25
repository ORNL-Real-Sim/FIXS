"""
FIXS signal-stop verification test builder.

Takes the bare straight-road rd5 that osc2cm produced (signal_stop_test.rd5) and
injects a traffic-light setup that proves two things CarMaker-side:

  Q3  the signal HEAD can be placed AFTER the stop line (far side), and the ego
      still stops at the STOP LINE -- because IPGDriver stops at the DrvStop
      *marker* (type 2 = RDST_TrfLight, refObjId=controller), NOT at the head.
  Q1  multiple heads on the SAME lane with DIFFERENT states (one red, one green):
      what does CM do? (built as a second variant road/testrun)

Layout on the 300 m straight road (lane-path 15, lane 8, controller red):
  ego start s=10  ->  DrvStop stop line s=STOP_S=100  ->  head s=HEAD_S=130 (far side)
Expected: ego halts at ~100, head sits at 130.

Run AFTER osc2cm (run_signal_stop_test.bat does both). Writes the modified rd5 +
the IPGDriver TestRun into the CM project.
"""
from __future__ import annotations
import re, pathlib, sys

REPO = pathlib.Path(__file__).resolve().parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
STOP_S = 100.0      # stop line (DrvStop) position along the road
HEAD_S = 130.0      # signal head position (far side, past the stop line)
# Control.TrfLight cycle (IPGRoad doc): "<startCond>" init t0 t1 t2 t3 t4
# phases: 0=off/skip, 1=Green, 2=Yellow, 3=Red, 4=RedYellow (verified vs JunctionNetwork.rd5).
# Static state = start in that phase and give it a ~infinite timer so it never advances.
# NOTE: a controller with only ONE non-zero phase time runs in "manual mode" (state set
# externally). For a self-contained static state we need "automatic mode" (>=2 non-zero
# timers) -- so we start in the wanted phase with a ~infinite timer and give a second phase
# a tiny (never-reached) timer. Within a 60 s test the light never leaves the wanted phase.
RED_CTRL   = '"" 3 0 0 0 99999 1'   # init=3 (Red),   t3=99999 -> red for the whole run
GREEN_CTRL = '"" 1 0 99999 0 0 1'   # init=1 (Green), t1=99999 -> green for the whole run


def inject(rd_name, testrun_name, variant):
    """variant: 'q3' = one far-side red head; 'q1' = two heads (red + green) same lane.

    Idempotent: always starts from the pristine osc2cm base (signal_stop_test_base.rd5),
    so re-running never double-injects. Writes the injected road to <rd_name>.rd5.
    """
    base = CMPROJ / "Data" / "Road" / "signal_stop_test_base.rd5"
    rd_path = CMPROJ / "Data" / "Road" / f"{rd_name}.rd5"
    t = base.read_text(encoding="utf-8", errors="ignore")
    maxid = int(re.search(r"MaxUsedObjId = (\d+)", t).group(1))
    nid = maxid

    def newid():
        nonlocal nid; nid += 1; return nid

    # --- controllers (one per head); 'q1' makes the 2nd green ---
    ctrls = []           # (ctrlObjId, name, timers, headType, headS, tag)
    c0 = newid()
    ctrls.append((c0, "TL_main", RED_CTRL, 0, HEAD_S, f"{rd_name}_0"))   # red, circular, far side
    if variant == "q1":
        c1 = newid()
        ctrls.append((c1, "TL_second", GREEN_CTRL, 0, HEAD_S, f"{rd_name}_1"))

    # --- a Mount (post) on RL.1 carrying the heads, at HEAD_S, off to the right ---
    mount_id = newid()
    head_lines = []
    head_parts = []
    for k, (cid, name, _, htype, hs, tag) in enumerate(ctrls):
        hpid = newid()
        # head part: 1 ctrlObjId vOff hOff dOff rotV rotH rotD latR facing type
        # facing=1 -> light is valid for ROUTE-direction traffic (the ego). road.h: facing
        # is direction *validity*, 1=in route dir, -1=counter. Working demo uses 1.
        head_parts.append(
            f"RL.1.Mount.0.{k}.ID = {hpid}\n"
            f"RL.1.Mount.0.{k} = 1 {cid} {5.0 + k*1.2} 0 0 0 0 0 98 1 {htype}\n"   # vOff stacks heads on the pole
            f"RL.1.Mount.0.{k}.Tag = odrSignalId:{tag}\n")
    mount_block = (
        f"RL.1.Mount.0.ID = {mount_id} 1\n"
        f"RL.1.Mount.0 = {HEAD_S} 0 -5 -1 {len(ctrls)} 9 0 0 0 4.2 5.39\n"
        + "".join(head_parts))

    # --- DrvStop stop marker(s) on RL.1 at STOP_S, type 2 (RDST_TrfLight) ---
    # The marker's dependsOnObjId must be a LanePath/Link/Path id (NOT the RL id) so IPGDriver
    # ties the stop to the lane the ego drives. q3 -> one marker (the red controller). q1 ->
    # one marker PER controller (red AND green on the same lane), to observe which the driver
    # obeys when conflicting stop markers reference different-state lights.
    lane_path_id = re.search(r"(?m)^LanePath\.0 = (\d+)", t).group(1)
    stop_lines = []
    for k, (cid, *_ ) in enumerate(ctrls):
        mk_id = newid()
        stop_lines.append(
            f"RL.1.Marker.{k}.ID = {mk_id} {lane_path_id}\n"
            f"RL.1.Marker.{k}.Type = DrvStop\n"
            f"RL.1.Marker.{k}.Param = {STOP_S} 0 1 {cid} 2 0\n")
    stop_block = "".join(stop_lines)

    # --- a Route over the single lane-path 15 so IPGDriver has a clear path ---
    rid = newid(); did = newid()
    route_block = (
        f"Route.0.ID = {rid}\nRoute.0.Name = TestRoute\nRoute.0.DrvPath.ID = {did}\n"
        "Route.0.DrvPath:\n\t15\n")

    ctrl_block = "".join(
        f'Control.TrfLight.{i} = {cid} "{name}" {timers}\n'
        for i, (cid, name, timers, *_ ) in enumerate(ctrls))

    # splice into the rd5
    t = re.sub(r"(?m)^nRoutes = \d+", "nRoutes = 1", t, count=1)
    t = re.sub(r"(RoadNetworkLength = .*\n)", r"\1Route.0.Length = 285.0\n", t, count=1)
    t = t.replace(f"MaxUsedObjId = {maxid}",
                  mount_block + stop_block + route_block + ctrl_block + f"MaxUsedObjId = {nid}")
    rd_path.write_text(t, encoding="utf-8")
    print(f"[{variant}] rd5 {rd_name}: {len(ctrls)} head(s) at s={HEAD_S}, DrvStop@s={STOP_S} -> ctrl {c0}; route 15")

    # --- IPGDriver TestRun: clone the driver template, repoint to this road / route.
    #     CRITICAL: the template uses Vehicle.DriverTemplate.FName = Car_Normal, NOT Car_Normal_osc.
    #     CarMaker RELOADS the named DriverTemplate at runtime and it overrides the inline Driver
    #     block. osc2cm writes Car_Normal_osc -- a trajectory-REPLAY tune (Acc.axMin=-20, ayMax=20,
    #     dtAccBrake=0, CornerCutCoef=0.01) that does NOT brake for stop markers even with
    #     Consider.TrfLight=1. Car_Normal is the standard autonomous tune (axMin=-4) that stops at
    #     reds. This works with ANY vehicle (incl. Demo_McLaren_F1) -- the vehicle was never the
    #     issue. The template mirrors IPG's signalized-junction example Man_AutonomousJunctions. ---
    tmpl = pathlib.Path(__file__).resolve().parent / "driver_template.cmtestrun"
    tr = tmpl.read_text(encoding="utf-8", errors="ignore")
    for pat, repl in [
        (r"(?m)^Road\.FName = .*$",              f"Road.FName = {rd_name}.rd5"),
        (r"(?m)^Vehicle\.Routing\.ObjId = .*$",  f"Vehicle.Routing.ObjId = {rid}"),
        (r"(?m)^Vehicle\.StartPos\.ObjId = .*$", f"Vehicle.StartPos.ObjId = {rid}"),
        (r"(?m)^Vehicle\.StartPos = .*$",        "Vehicle.StartPos = 5.0 0"),
        (r"(?m)^DrivMan\.Global\.EndCond = .*$", "DrivMan.Global.EndCond = rise(Time > 60.00)"),
    ]:
        tr = re.sub(pat, repl, tr, count=1)
    (CMPROJ / "Data" / "TestRun" / testrun_name).write_text(tr, encoding="utf-8")
    print(f"[{variant}] TestRun {testrun_name} (DriverTemplate Car_Normal; route {rid})")


if __name__ == "__main__":
    variant = sys.argv[1] if len(sys.argv) > 1 else "q3"
    rd = "signal_stop_test" if variant == "q3" else "signal_stop_test_q1"
    tr = "SignalStopTest" if variant == "q3" else "SignalStopTest_Q1"
    inject(rd, tr, variant)
