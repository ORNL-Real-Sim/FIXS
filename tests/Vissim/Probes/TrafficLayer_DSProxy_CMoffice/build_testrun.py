"""
Make the CarMaker road + TestRun for the #168 demo fully reproducible.

osc2cm produces a route-less road and an old-format traffic set that CarMaker 13
rejects. This script, run AFTER osc2cm (see import_road.bat), deterministically:

  1. Adds a closed-loop Route (id 900) to simple_loop.rd5 by chaining the road's
     8 LanePath segments in clockwise driving order (link0 E -> link2 N ->
     link3 W -> link1 S, with the junction connectors between them). The chain
     is validated by `roadutil -rlen 0` (≈820 m, the full loop).
  2. Rewrites the ego TestRun so the maneuver has a real duration (TimeLimit
     120 s, else CarMaker ends the run at t≈0) and the ego follows Route 900.
  3. Replaces the 20 RS_C traffic objects with the CM13 working format
     (Template.FName + AutoDriver.FName + maneuver block + Route 900), so they
     load. At runtime VirtualEnvironment.lib free-motion-teleports them to the
     VISSIM-driven positions; the route is only load-time scaffolding.

Usage:  python build_testrun.py
        (paths are fixed to ProprietaryFiles/CM13_proj)
"""
from __future__ import annotations
import os
import pathlib
import re

from derive_drvpath import derive_drvpath

REPO = pathlib.Path(__file__).resolve().parents[4]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
RD = CMPROJ / "Data" / "Road" / "simple_loop.rd5"
TR_EGO = CMPROJ / "Data" / "TestRun" / "SimpleLoop_VISSIM"
TR_DEMO = CMPROJ / "Data" / "TestRun" / "SimpleLoop_VISSIM_rs"
# Ego is the CM-shipped McLaren MP4 race car (copied into the project Data/Vehicle):
# CG 0.25 m -- cannot roll on the loop corners -- and the McLaren_MP4_2016.mobj visual
# is already wired into the .car. Replaces the osc2cm-generated SimpleLoop_VISSIM_Ego
# (which came out truck-tall at 1.30 m CG and rolled). Its tyres/brake/visual resolve
# from the standard CarMaker install.
EGO_VEHICLE = "Demo_McLaren_F1"
DRIVER_TPL = CMPROJ / "Data" / "Driver" / "Template" / "Car_Normal_osc"

ROUTE_ID = 900
DRVPATH_ID = 901
# The DrvPath (LanePath ids, driving order) is auto-derived from the current rd5
# by derive_drvpath() -- the authored junction loop's 8 LanePaths (4 straights +
# 4 R15 corner-junction connectors) chained 1->5->2->6->3->7->4->8.
ROUTE_LEN = 774.25  # 4x170 straights + 4x23.56 R15 arcs; roadutil -rlen 0 = 774.25
N_TRAFFIC = int(os.environ.get("RS_N_TRAFFIC", "50"))  # RS_N_TRAFFIC overrides (perf sweep).
                        # Size this JUST ABOVE the real VISSIM peak (~40), NOT a big buffer.
                        # The sweep_traffic.py study showed each CM traffic object costs
                        # ~0.73 ms/step in CarMaker's core EVEN WHEN PARKED (idle slots at
                        # z=-5000 are NOT free): per-step wall time is ~linear in N_TRAFFIC
                        # (RTF 7.2 @20, 2.7 @50, 0.94 @150, all at ~13 active vehicles), so
                        # N_TRAFFIC=150 already runs slower than real-time. Too FEW slots =
                        # VISSIM cars (incl. the ego's leader) with no CM object -> ego sees
                        # clear road, won't follow. 50 balances coverage vs cost.
# The loop now has real R15 corner junctions (authored xodr -> osc2cm), so
# IPGDriver follows the full 774 m loop and rounds every corner smoothly. The
# ego runs the whole demo window; co-simulation (ego pose -> VISSIM, VISSIM
# traffic -> CarMaker) runs throughout.
EGO_SPEED_KMH = 18.0     # ego start velocity
EGO_CRUISE_KMH = 50.0    # IPGDriver cruise cap, KM/H. CarMaker stores
                         # Driver.Vel.CruisingSpeed in km/h (shipped Car_Normal=150,
                         # Car_Aggressive=250) -- NOT m/s. Set to the VISSIM loop traffic's
                         # desired speed (50 km/h) so the ego paces the flow; with
                         # Consider.Traffic=1 it still follows leaders at a gap. The
                         # McLaren's 0.25 m CG means the corners are no rollover risk. Used
                         # for BOTH the TestRun and the driver template (kept consistent).
# The ego MANEUVER must never be the run limiter -- set it effectively forever
# (9999 s). The actual run length comes from the CO-SIMULATION, not CarMaker:
# config.yaml's SimulationEndTime drives TrafficLayer's tick budget, and the run
# ends when TrafficLayer closes the co-sim socket (-> CarMaker SIM_END). Tying the
# maneuver to the config duration (the old behaviour) made CarMaker cap the run
# itself at t=SimulationEndTime regardless of VISSIM.
RUN_SECONDS = 9999


def add_route_to_road() -> None:
    # Always re-derive + rewrite (idempotent: prior Route.0.* is dropped below),
    # so the route tracks the current rd5 even on re-runs without a fresh osc2cm.
    lines = RD.read_text(encoding="utf-8").splitlines()
    drvpath, road_order, n_seg, _, closed = derive_drvpath(RD)
    if not closed:
        print(f"[build_testrun] WARNING: derived DrvPath is not a closed loop "
              f"({n_seg} segs, roads {road_order})")
    out = []
    in_drvpath = False
    for l in lines:
        if l.startswith("nRoutes ="):
            out.append("nRoutes = 1")
            in_drvpath = False
            continue
        # drop a prior Route.0.DrvPath block: the header line AND its indented
        # "\t<id>" segment lines (those don't start with "Route.0." so must be
        # dropped explicitly, else they orphan and CarMaker errors "Unknown line").
        if l.startswith("Route.0.DrvPath:"):
            in_drvpath = True
            continue
        if in_drvpath:
            if re.match(r"\s+\d+\s*$", l):
                continue
            in_drvpath = False
        # drop any other prior Route.0.* to stay idempotent
        if re.match(r"Route\.0\.", l):
            continue
        out.append(l)
        # Route.0.Length must sit AFTER the BBox line (matches the .rd5 header
        # ordering CarMaker's loader expects; roadutil is laxer but the GUI/exe
        # loader errors if it's placed right after nRoutes).
        if l.startswith("BBox ="):
            out.append(f"Route.0.Length = {ROUTE_LEN}")
    out.append(f"Route.0.ID = {ROUTE_ID}")
    out.append("Route.0.Name = loop")
    out.append(f"Route.0.DrvPath.ID = {DRVPATH_ID}")
    out.append("Route.0.DrvPath:")
    for seg in drvpath:
        out.append(f"\t{seg}")
    RD.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"[build_testrun] added Route {ROUTE_ID} (loop) to simple_loop.rd5")


def fix_ego_testrun(tr: pathlib.Path) -> None:
    lines = tr.read_text(encoding="utf-8").splitlines()
    out = []
    for l in lines:
        if l.startswith("DrivMan.Man.0.LongStep.0.TimeLimit"):
            out.append(f"DrivMan.Man.0.LongStep.0.TimeLimit = {RUN_SECONDS}.000")
        elif l.startswith("DrivMan.Global.EndCond"):
            out.append(f"DrivMan.Global.EndCond = rise(Time > {RUN_SECONDS}.00)")
        elif l.startswith("DrivMan.Man.0.LongStep.0.Dyn"):
            # Hand the longitudinal to the IPGDriver (not a scripted VelTransition)
            # so the ego adapts its speed to VISSIM traffic -- brakes for leaders
            # and keeps a gap. The lateral already follows Route 900.
            out.append("DrivMan.Man.0.LongStep.0.Dyn = Driver 1 0")
        elif l.startswith("DrivMan.Man.Start.Velocity"):
            out.append(f"DrivMan.Man.Start.Velocity = {EGO_SPEED_KMH:.3f}")
        elif l.startswith("Driver.Vel.CruisingSpeed"):
            out.append(f"Driver.Vel.CruisingSpeed = {EGO_CRUISE_KMH:.1f}")
        elif l.startswith("Driver.Consider.Traffic"):
            out.append("Driver.Consider.Traffic = 1")
        elif l.startswith("Vehicle = "):
            out.append(f"Vehicle = {EGO_VEHICLE}")
        elif l.startswith("Vehicle.Routing.Type"):
            out.append("Vehicle.Routing.Type = Route")
        elif l.startswith("Vehicle.Routing.ObjId"):
            out.append(f"Vehicle.Routing.ObjId = {ROUTE_ID}")
        elif l.startswith("Vehicle.StartPos.RefPnt"):
            continue  # incompatible with Route start
        elif l.startswith("Vehicle.StartPos.Type"):
            out.append("Vehicle.StartPos.Type = Route")
        elif l.startswith("Vehicle.StartPos.ObjId"):
            out.append(f"Vehicle.StartPos.ObjId = {ROUTE_ID}")
        elif l.startswith("Vehicle.StartPos.Orientation.Type"):
            out.append("Vehicle.StartPos.Orientation.Type = Relative")
        elif re.match(r"Vehicle\.StartPos = ", l):
            out.append("Vehicle.StartPos = 0.0 0")
        else:
            out.append(l)
    tr.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"[build_testrun] fixed ego maneuver + routing in {tr.name}")


# CM traffic UpdRate (Hz). KEEP AT 1000. Lowering it FREEZES the teleported FreeMotion
# traffic (verified: UpdRate=200 stalled the whole co-sim -- the cars cluster and the ego
# stops behind them), because below CarMaker's FreeMotion default it stops applying the
# .lib's per-step position. The earlier "UpdRate = free 15x" claim was WRONG -- it measured
# STATIC traffic. RS_UPD_RATE overrides this only for the correctness-checked sweep
# (sweep_updrate.py). MOTION_KIND has no effect (FreeMotion skips the dynamics solver).
UPD_RATE = int(os.environ.get("RS_UPD_RATE", "1000"))
MOTION_KIND = os.environ.get("RS_MOTION_KIND", "").strip()


def traffic_obj(i: int) -> list[str]:
    # Spread the load-time positions EVENLY around the ~774 m loop (no overlap for
    # any N_TRAFFIC). The .lib parks them at z=-5000 at init and only lifts those
    # mapped to a live VISSIM vehicle, so these are just valid load anchors.
    s = (i + 0.5) * ROUTE_LEN / N_TRAFFIC
    lines = [
        f"Traffic.{i}.Name = RS_C{i:03d}", f"Traffic.{i}.Info:",
        f"Traffic.{i}.DetectMask = 1 1", f"Traffic.{i}.UpdRate = {UPD_RATE}",
        f"Traffic.{i}.Lighting = 0", f"Traffic.{i}.FreeMotion = 1",
        f"Traffic.{i}.TrailerName =",
        f"Traffic.{i}.Template.FName = 1_Vehicles/IPG_CompanyCar_2018_Blue",
        f"Traffic.{i}.AutoDriver.FName = Car_HDM_Normal",
        f"Traffic.{i}.Routing.Type = Route", f"Traffic.{i}.Routing.ObjId = {ROUTE_ID}",
        f"Traffic.{i}.StartPos.Type = Route", f"Traffic.{i}.StartPos.ObjId = {ROUTE_ID}",
        f"Traffic.{i}.StartPos = {s:.3f} 0.0",
        f"Traffic.{i}.StartPos.Orientation.Type = Relative",
        f"Traffic.{i}.StartPos.Orientation = 0.0 0.0 0.0",
        f"Traffic.{i}.nMan = 1", f"Traffic.{i}.Man.Start.Velocity = 0.0",
        f"Traffic.{i}.Man.TreatAtEnd = FreezePos",
        f"Traffic.{i}.Man.0.nLongSteps = 1", f"Traffic.{i}.Man.0.nLatSteps = 1",
        f"Traffic.{i}.Man.0.CombinedSteps = 1", f"Traffic.{i}.Man.0.MaxExec = 1",
        f"Traffic.{i}.Man.0.ConsiderDomain = own",
        f"Traffic.{i}.Man.0.Transition.Interrupt = self",
        f"Traffic.{i}.Man.0.Transition.EndCond = end",
        f"Traffic.{i}.Man.0.Transition.SimultanStart = end",
        f"Traffic.{i}.Man.0.LongStep.0.Limit = t 0.0",
        f"Traffic.{i}.Man.0.LatStep.0.Limit = t 0.0",
    ]
    if MOTION_KIND:
        lines.append(f"Traffic.{i}.Motion.Kind = {MOTION_KIND}")
    return lines


def rewrite_traffic(tr: pathlib.Path) -> None:
    lines = tr.read_text(encoding="utf-8").splitlines()
    head = [l for l in lines
            if not (re.match(r"Traffic\.\d+\.", l)
                    or l.startswith("Traffic.N")
                    or l.startswith("Traffic.SpeedUnit"))]
    traffic = [f"Traffic.N = {N_TRAFFIC}", "Traffic.SpeedUnit = ms"]
    for i in range(N_TRAFFIC):
        traffic += traffic_obj(i)
    tr.write_text("\n".join(head + traffic) + "\n", encoding="utf-8")
    print(f"[build_testrun] wrote {N_TRAFFIC} CM13-format RS_C traffic objects in {tr.name}")


def fix_driver_template(tpl: pathlib.Path) -> None:
    # osc2cm regenerates Car_Normal_osc from the .xosc each import, dropping the ego's
    # traffic-respect (Driver.Consider.Traffic) and leaving an uncapped cruise speed --
    # so after a fresh import the ego ignores VISSIM traffic and (with the McLaren) races
    # off. Force the two settings the demo needs so it stays reproducible without a manual
    # GUI re-save: respect traffic (brake for leaders, hold a gap) + a tame cruise cap.
    if not tpl.is_file():
        return
    forced = {
        "Driver.Consider.Traffic": "1",
        "Driver.Vel.CruisingSpeed": f"{EGO_CRUISE_KMH:.1f}",   # km/h (CarMaker unit)
    }
    seen = set()
    out = []
    for l in tpl.read_text(encoding="utf-8").splitlines():
        # match only "Key = value" lines (skip "Key:" table headers and indented data)
        key = l.split("=", 1)[0].strip() if ("=" in l and not l.rstrip().endswith(":")) else None
        if key in forced:
            out.append(f"{key} = {forced[key]}")
            seen.add(key)
        else:
            out.append(l)
    for key, val in forced.items():  # add any key osc2cm's output omitted entirely
        if key not in seen:
            out.append(f"{key} = {val}")
    tpl.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"[build_testrun] forced traffic-respect + cruise cap in {tpl.name}")


def main() -> int:
    if not RD.is_file():
        raise SystemExit(f"ERROR: {RD} missing -- run import_road.bat (osc2cm) first")
    add_route_to_road()
    # ego TestRun (no traffic) + demo TestRun (with traffic) both follow Route 900
    if TR_EGO.is_file():
        fix_ego_testrun(TR_EGO)
    if TR_DEMO.is_file():
        fix_ego_testrun(TR_DEMO)
        rewrite_traffic(TR_DEMO)
    fix_driver_template(DRIVER_TPL)
    print("[build_testrun] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
