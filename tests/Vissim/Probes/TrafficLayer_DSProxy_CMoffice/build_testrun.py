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

REPO = pathlib.Path(__file__).resolve().parents[4]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
RD = CMPROJ / "Data" / "Road" / "simple_loop.rd5"
TR_EGO = CMPROJ / "Data" / "TestRun" / "SimpleLoop_VISSIM"
TR_DEMO = CMPROJ / "Data" / "TestRun" / "SimpleLoop_VISSIM_rs"

ROUTE_ID = 900
DRVPATH_ID = 901
# Clockwise loop LanePath object-ids (verified against simple_loop.rd5 geometry):
#   link0 E (LP id15) -> jc1 (id161) -> link2 N (id71) -> jc3 (id223)
#   -> link3 W (id99) -> jc2 (id192) -> link1 S (id43) -> jc0 (id130)
DRVPATH = [15, 161, 71, 223, 99, 192, 43, 130]
ROUTE_LEN = 820.0
N_TRAFFIC = 20
# Ego drives one 200 m straight (link 0) at EGO_SPEED_KMH; IPGDriver does not
# navigate the hand-authored loop junctions, so the run is ended GRACEFULLY at
# RUN_SECONDS (before the ego reaches the link-0 end at ~40 s @ 18 km/h). The
# co-simulation (ego pose -> VISSIM, VISSIM traffic -> CarMaker) is fully
# exercised in that window. Looping the ego is a known follow-up (IPGDriver
# junction routing on the synthetic road).
EGO_SPEED_KMH = 18.0
RUN_SECONDS = 30


def add_route_to_road() -> None:
    lines = RD.read_text(encoding="utf-8").splitlines()
    if any(l.startswith(f"Route.0.ID = {ROUTE_ID}") for l in lines):
        # already has our route (idempotent re-run after a fresh osc2cm wipes it)
        if "nRoutes = 1" in lines:
            return
    out = []
    for l in lines:
        if l.startswith("nRoutes ="):
            out.append("nRoutes = 1")
            continue
        # drop any prior Route.0.* (e.g. a half-written one) to stay idempotent
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
    for seg in DRVPATH:
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
    s = 5.0 + (i % 20) * 9.0
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
