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
import pathlib
import re

from derive_drvpath import derive_drvpath

REPO = pathlib.Path(__file__).resolve().parents[4]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
RD = CMPROJ / "Data" / "Road" / "simple_loop.rd5"
TR_EGO = CMPROJ / "Data" / "TestRun" / "SimpleLoop_VISSIM"
TR_DEMO = CMPROJ / "Data" / "TestRun" / "SimpleLoop_VISSIM_rs"

ROUTE_ID = 900
DRVPATH_ID = 901
# The DrvPath (LanePath ids, driving order) is auto-derived from the current rd5
# by derive_drvpath() -- the authored junction loop's 8 LanePaths (4 straights +
# 4 R15 corner-junction connectors) chained 1->5->2->6->3->7->4->8.
ROUTE_LEN = 774.25  # 4x170 straights + 4x23.56 R15 arcs; roadutil -rlen 0 = 774.25
N_TRAFFIC = 20
# The loop now has real R15 corner junctions (authored xodr -> osc2cm), so
# IPGDriver follows the full 774 m loop and rounds every corner smoothly. The
# ego runs the whole demo window; co-simulation (ego pose -> VISSIM, VISSIM
# traffic -> CarMaker) runs throughout.
EGO_SPEED_KMH = 18.0
RUN_SECONDS = 120


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
            out.append(f"DrivMan.Man.0.LongStep.0.Dyn = VelTransition {EGO_SPEED_KMH:.3f} step")
        elif l.startswith("DrivMan.Man.Start.Velocity"):
            out.append(f"DrivMan.Man.Start.Velocity = {EGO_SPEED_KMH:.3f}")
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


def traffic_obj(i: int) -> list[str]:
    # Spread the 20 load-time positions around the full ~774 m loop route. They
    # are free-motion-teleported to the VISSIM-driven positions at runtime, so
    # these are just valid load-time anchors.
    s = 5.0 + (i % 20) * 38.0
    return [
        f"Traffic.{i}.Name = RS_C{i:03d}", f"Traffic.{i}.Info:",
        f"Traffic.{i}.DetectMask = 1 1", f"Traffic.{i}.UpdRate = 1000",
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
    print("[build_testrun] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
