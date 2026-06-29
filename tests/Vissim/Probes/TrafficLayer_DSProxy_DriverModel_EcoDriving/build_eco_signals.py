"""
Generate signals.json for the eco-driving CAV controller (#173).

Extracts the FIXED through-movement green window for each corridor intersection
from the shared SUMO signal plan (tests/Sumo/networks/simple_traffic_light/
signal_plan.json) -- cycle, offset, green_start, green_dur -- which is reliable.

Positions (where each signal's stop line sits along the corridor travel axis, in
the SAME frame as the DSProxy vehicle readback positionX/Y) CANNOT be derived
from the plan alone: the plan carries OpenDRIVE road-frame `s`, not VISSIM global
coords. So this writes position ESTIMATES (evenly spaced along the corridor) and
marks the file for a one-time calibration -- see README "Calibrate signal
positions". The controller free-flows (no eco) for any signal whose pos is null,
so the demo still runs before calibration; it just won't shape speed until the
positions are set.

Usage:
    python build_eco_signals.py [--plan <signal_plan.json>] [--out signals.json]
"""

from __future__ import annotations

import argparse
import json
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
DEFAULT_PLAN = REPO_ROOT / "tests" / "Sumo" / "networks" / "simple_traffic_light" / "signal_plan.json"

# Corridor intersections, west -> east, with first-guess stop-line positions
# along the travel axis. CALIBRATE these against one readback pass (README).
# 1346 m canonical corridor; evenly-ish spaced as a starting estimate.
CORRIDOR_ORDER = ["int_west", "int_center", "int_east"]
POS_ESTIMATE = {"int_west": 330.0, "int_center": 670.0, "int_east": 1010.0}


def through_window(inter_plan: dict) -> tuple[float, float]:
    """(green_start, green_dur) for the corridor THROUGH movement of one
    intersection. Heuristic: the signal group with the most straight ('s')
    members -- the main-road through phase. Reads the GREEN->(AMBER|RED) span
    from its transitions."""
    best_group = None
    best_through = -1
    for g in inter_plan["groups"]:
        n_through = sum(1 for m in g.get("members", []) if m.get("dir") == "s")
        if n_through > best_through:
            best_through, best_group = n_through, g
    if best_group is None:
        raise ValueError("no signal groups found")

    trans = sorted(best_group["transitions"], key=lambda t: t[0])
    green_t = next((t[0] for t in trans if t[1] == "GREEN"), None)
    if green_t is None:
        raise ValueError("no GREEN transition in through group")
    # green ends at the next transition after GREEN (AMBER or RED)
    after = [t[0] for t in trans if t[0] > green_t]
    cycle = inter_plan["cycle"]
    green_end = after[0] if after else cycle
    return float(green_t), float(green_end - green_t)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", default=str(DEFAULT_PLAN))
    ap.add_argument("--out", default=str(HERE / "signals.json"))
    ap.add_argument("--cav-type", default="100")
    ap.add_argument("--axis", default="x", choices=["x", "y"])
    ap.add_argument("--direction", type=float, default=1.0)
    ap.add_argument("--force", action="store_true",
                    help="overwrite even a calibrated signals.json (DISCARDS calibration)")
    args = ap.parse_args()

    # Never clobber a calibrated signals.json (the launcher calls us every run).
    out_path = pathlib.Path(args.out)
    if out_path.is_file() and not args.force:
        try:
            existing = json.loads(out_path.read_text())
        except (ValueError, OSError):
            existing = {}
        if existing.get("_calibrated"):
            print(f"{out_path.name} is calibrated; keeping it (use --force to regenerate).")
            return 0
        if existing.get("signals"):
            print(f"{out_path.name} exists (uncalibrated); keeping it (use --force to regenerate).")
            return 0

    plan = json.loads(pathlib.Path(args.plan).read_text())

    signals = []
    for name in CORRIDOR_ORDER:
        if name not in plan:
            print(f"  WARN: {name} not in plan; skipping")
            continue
        ip = plan[name]
        gstart, gdur = through_window(ip)
        signals.append({
            "name": name,
            "pos": POS_ESTIMATE.get(name),     # CALIBRATE (see README)
            "cycle": float(ip["cycle"]),
            "offset": float(ip.get("offset", 0)),
            "green_start": gstart,
            "green_dur": gdur,
        })
        print(f"  {name:12s} cycle={ip['cycle']:>4} offset={ip.get('offset',0):>3} "
              f"green=[{gstart:.0f}..{gstart+gdur:.0f}]s  pos~{POS_ESTIMATE.get(name)}")

    out = {
        "_comment": ("Eco-driving corridor signal model (#173). TIMING is from "
                     "signal_plan.json (reliable). POSITIONS are estimates -- "
                     "CALIBRATE against one readback pass, see README."),
        "_calibrated": False,
        "travel_axis": args.axis,
        "direction": args.direction,
        "cav_type": args.cav_type,
        "signals": signals,
    }
    pathlib.Path(args.out).write_text(json.dumps(out, indent=2))
    print(f"\nwrote {args.out}  ({len(signals)} signals)")
    print("NOTE: set _calibrated=true and real positions after the calibration pass.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
