"""
Build the co-simulation TestRun for the #172 signal demo: SimpleTrafficLight_Cosim.

This combines the two halves of the demo into one CarMaker TestRun:
  - the EGO from SimpleTrafficLight_Scene (built by add_signal_stops.py): Car_Normal
    driver on the signalized corridor rd5, with the in-road DrvStops that make it
    brake at a red light, following Route 3 (the native corridor loop); and
  - N RS_C traffic slots (CM13 format) so VISSIM's background vehicles have CM
    objects to be teleported onto by VirtualEnvironment.lib at runtime.

The ego pose is sent to VISSIM (id 'egoCm'); VISSIM returns background traffic
(-> RS_C slots) and signal-group states (-> CM traffic lights via the signal
socket + RSsignalTable.csv). The DrvStop + the now VISSIM-driven red is what
stops the ego, so the SAME rd5 serves both the scene-only and the co-sim demos.

Run AFTER add_signal_stops.py (needs SimpleTrafficLight_Scene + the signalstop rd5).
The RS_C slots are load-time anchors only: the .lib parks them at z=-5000 and
lifts only those mapped to a live VISSIM vehicle (identical to #168).
"""
from __future__ import annotations
import os, pathlib, re

CMPROJ = pathlib.Path(__file__).resolve().parents[4] / "ProprietaryFiles" / "CM13_proj"
TR_SRC = CMPROJ / "Data" / "TestRun" / "SimpleTrafficLight_Scene"   # ego-only (scene) TestRun
TR_OUT = CMPROJ / "Data" / "TestRun" / "SimpleTrafficLight_Cosim"   # co-sim TestRun (this script)

# Route anchored in simple_traffic_light.rd5 (the native corridor loop).
ROUTE_ID = 3
ROUTE_LEN = 2861.45            # roadutil Route.0.Length of the signalstop rd5
N_TRAFFIC = int(os.environ.get("RS_N_TRAFFIC", "50"))   # JUST above the VISSIM peak; each CM
                                                        # object costs ~0.7 ms/step even parked (#168)
UPD_RATE = int(os.environ.get("RS_UPD_RATE", "200"))
AUTO_DRIVER = "" if os.environ.get("RS_NO_AUTODRIVER") else "Car_HDM_Normal"


def traffic_obj(i: int) -> list[str]:
    # Spread load-time anchors evenly along the corridor route (no overlap).
    s = (i + 0.5) * ROUTE_LEN / N_TRAFFIC
    lines = [
        f"Traffic.{i}.Name = RS_C{i:03d}", f"Traffic.{i}.Info:",
        f"Traffic.{i}.DetectMask = 1 1", f"Traffic.{i}.UpdRate = {UPD_RATE}",
        f"Traffic.{i}.Lighting = 0", f"Traffic.{i}.FreeMotion = 1",
        f"Traffic.{i}.TrailerName =",
        f"Traffic.{i}.Template.FName = 1_Vehicles/IPG_CompanyCar_2018_Blue",
        f"Traffic.{i}.AutoDriver.FName = {AUTO_DRIVER}",
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
    return lines


def main():
    if not TR_SRC.is_file():
        raise SystemExit(f"missing {TR_SRC} -- run add_signal_stops.py first")
    lines = TR_SRC.read_text(encoding="utf-8", errors="ignore").splitlines()
    # drop any existing traffic block; keep the ego + everything else verbatim
    head = [l for l in lines
            if not (re.match(r"Traffic\.\d+\.", l)
                    or l.startswith("Traffic.N")
                    or l.startswith("Traffic.SpeedUnit"))]
    traffic = [f"Traffic.N = {N_TRAFFIC}", "Traffic.SpeedUnit = ms"]
    for i in range(N_TRAFFIC):
        traffic += traffic_obj(i)
    TR_OUT.write_text("\n".join(head + traffic) + "\n", encoding="utf-8")
    print(f"[build_cosim_testrun] wrote {TR_OUT.name}: ego (Car_Normal + DrvStops) "
          f"+ {N_TRAFFIC} RS_C slots on Route {ROUTE_ID}")


if __name__ == "__main__":
    main()
