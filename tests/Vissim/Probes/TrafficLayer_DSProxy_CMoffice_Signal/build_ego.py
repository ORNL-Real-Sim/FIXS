"""
Finalize the SimpleTrafficLight_import ego (run AFTER osc2cm) for #172:
  - Ego -> Demo_McLaren_F1 (handles the loop geometry; osc2cm ego is truck-tall).
  - Keep the osc2cm FollowTraj polyline (lat+long) -- it IS the SUMO-recorded path
    (lane 0, no lane change, full loop, on the real lanes). The ego follows it.
  - Routing.Type = Lane (drop osc2cm's partial auto-Path so it doesn't fight the
    full-loop trajectory).

A native closed Route + IPGDriver signal-braking is deferred to #173 (eco-driving):
the clean way to build that route is to run CM on this FollowTraj and log the
ego's lane-path via the road API (RoadLaneGetLanePathObjId), then inject it.
"""
from __future__ import annotations
import pathlib

REPO = pathlib.Path(__file__).resolve().parents[4]
TR = REPO / "ProprietaryFiles" / "CM13_proj" / "Data" / "TestRun" / "SimpleTrafficLight_import"
EGO_VEHICLE = "Demo_McLaren_F1"


def main() -> int:
    if not TR.is_file():
        raise SystemExit(f"ERROR: {TR} missing -- run osc2cm first")
    out = []
    for l in TR.read_text(encoding="utf-8").splitlines():
        if l.startswith("Vehicle = "):
            out.append(f"Vehicle = {EGO_VEHICLE}")
        elif l.startswith("Vehicle.Routing.Type"):
            out.append("Vehicle.Routing.Type = Lane")
        elif l.startswith("Vehicle.Routing.ObjId"):
            out.append("Vehicle.Routing.ObjId =")
        else:
            out.append(l)
    TR.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"[build_ego] McLaren + SUMO-path FollowTraj loop + Lane routing in {TR.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
