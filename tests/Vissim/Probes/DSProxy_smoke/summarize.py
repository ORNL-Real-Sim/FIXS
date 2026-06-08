"""
Reduce raw smoke-test CSV output to a compact summary JSON suitable for
checking in as a regression baseline.

Exact vehicle counts drift with VISSIM's seeded RNG and per-version car-
following tuning, so the summary captures structural invariants (ranges,
presence, transitions) rather than exact numbers.

Usage:
    python summarize.py <out_dir>
        # writes <out_dir>/summary.json
"""

from __future__ import annotations

import csv
import json
import sys
from collections import defaultdict
from pathlib import Path


def summarize(out_dir: Path) -> dict:
    veh_csv = out_dir / "vehicles.csv"
    sig_csv = out_dir / "signals.csv"
    if not veh_csv.is_file() or not sig_csv.is_file():
        raise FileNotFoundError(f"Missing csv files in {out_dir}")

    veh_count_per_frame: dict[int, int] = defaultdict(int)
    ego_per_frame: dict[int, dict] = {}
    veh_types_seen: set[int] = set()
    leading_resolved = 0
    ds_vehicles_seen: set[int] = set()

    with open(veh_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = int(row["frame"])
            veh_count_per_frame[frame] += 1
            veh_types_seen.add(int(row["vtype"]))
            if int(row["leading_id"]) > 0:
                leading_resolved += 1
            if int(row["create_id"]) == 4711 and int(row["controlled_by_vissim"]) == 0:
                ego_per_frame[frame] = {
                    "vehicle_id": int(row["vehicle_id"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "heading": float(row["heading"]),
                    "speed": float(row["speed"]),
                }
                ds_vehicles_seen.add(int(row["vehicle_id"]))

    sig_states_seen: set[str] = set()
    sig_count_per_frame: dict[int, int] = defaultdict(int)
    sig_controllers: set[int] = set()
    sig_groups_per_controller: dict[int, set[int]] = defaultdict(set)

    with open(sig_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row["state_name"]:
                continue
            frame = int(row["frame"])
            sig_count_per_frame[frame] += 1
            sig_states_seen.add(row["state_name"])
            ctrl = int(row["controller_id"])
            sig_controllers.add(ctrl)
            sig_groups_per_controller[ctrl].add(int(row["signal_group_id"]))

    frames = sorted(veh_count_per_frame)
    veh_counts = [veh_count_per_frame[f] for f in frames]
    sig_counts = [sig_count_per_frame.get(f, 0) for f in frames]
    ego_frames = sorted(ego_per_frame)

    summary = {
        "frames": {
            "count": len(frames),
            "first": frames[0] if frames else None,
            "last":  frames[-1] if frames else None,
        },
        "vehicles": {
            "count_first_frame": veh_counts[0] if veh_counts else 0,
            "count_last_frame":  veh_counts[-1] if veh_counts else 0,
            "count_grows_monotonically_ish":
                veh_counts[-1] > veh_counts[0] if veh_counts else False,
            "types_seen": sorted(veh_types_seen),
            "leading_id_resolved_count": leading_resolved,
        },
        "ego": {
            "registered": bool(ego_frames),
            "first_seen_frame": ego_frames[0] if ego_frames else None,
            "last_seen_frame":  ego_frames[-1] if ego_frames else None,
            "vissim_vehicle_ids_used": sorted(ds_vehicles_seen),
            "x_first": ego_per_frame[ego_frames[0]]["x"] if ego_frames else None,
            "x_last":  ego_per_frame[ego_frames[-1]]["x"] if ego_frames else None,
            "moved_east": (
                ego_per_frame[ego_frames[-1]]["x"] > ego_per_frame[ego_frames[0]]["x"]
                if len(ego_frames) >= 2 else False
            ),
        },
        "signals": {
            "count_per_frame_min": min(sig_counts) if sig_counts else 0,
            "count_per_frame_max": max(sig_counts) if sig_counts else 0,
            "states_observed": sorted(sig_states_seen),
            "num_controllers": len(sig_controllers),
            "num_signal_groups_total": sum(len(g) for g in sig_groups_per_controller.values()),
        },
    }
    return summary


PASS_CRITERIA = {
    "ego_registered":          ("ego.registered",                  True),
    "ego_unique_id":           ("ego.vissim_vehicle_ids_used.len", 1),    # one ID, never reassigned
    "ego_moved_east":          ("ego.moved_east",                  True),
    "signals_present":         ("signals.count_per_frame_min",     ("ge", 1)),
    "signal_state_red_seen":   ("signals.states_observed.contains","Red"),
    "signal_state_green_seen": ("signals.states_observed.contains","Green"),
    "vehicles_grow":           ("vehicles.count_grows_monotonically_ish", True),
    "leading_id_resolved":     ("vehicles.leading_id_resolved_count", ("ge", 1)),
}


def _get_nested(d: dict, path: str):
    parts = path.split(".")
    cur = d
    op = None
    for p in parts:
        if p == "len":
            cur = len(cur)
            continue
        if p == "contains":
            op = "contains"
            continue
        cur = cur[p]
    return cur, op


def verify(summary: dict) -> dict:
    results: dict[str, str] = {}
    all_pass = True
    for name, (path, expected) in PASS_CRITERIA.items():
        actual, op = _get_nested(summary, path)
        if op == "contains":
            ok = expected in actual
            results[name] = f"{'PASS' if ok else 'FAIL'} ({expected!r} in {actual!r})"
        elif isinstance(expected, tuple) and expected[0] == "ge":
            ok = actual >= expected[1]
            results[name] = f"{'PASS' if ok else 'FAIL'} ({actual} >= {expected[1]})"
        else:
            ok = actual == expected
            results[name] = f"{'PASS' if ok else 'FAIL'} (got {actual!r}, expected {expected!r})"
        if not ok:
            all_pass = False
    results["_overall"] = "PASS" if all_pass else "FAIL"
    return results


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: python summarize.py <out_dir>")
        sys.exit(2)
    out_dir = Path(sys.argv[1]).resolve()
    summary = summarize(out_dir)
    verdict = verify(summary)

    summary_path = out_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2))
    print(f"wrote {summary_path}")

    print("\n=== Verification ===")
    for k, v in verdict.items():
        print(f"  {k:30s} {v}")
    sys.exit(0 if verdict["_overall"] == "PASS" else 1)
