"""
Before/after readout for the #173 eco-driving demo.

Compares two cav_controller runs over the SAME corridor + demand:
  - baseline:  out/summary_baseline.json   (run with --penetration 0.0; CAVs free-flow)
  - eco:       out/summary_eco.json        (run with --penetration > 0; GLOSA active)

Reports the eco effect on the DriverModel-hooked CAV population:
  - number of CAVs that stopped at a signal
  - total idle (stopped) time
  - mean deceleration proxy (cumulative |decel| per CAV -- a harshness/energy proxy)

The eco-driving win is FEWER stops / LESS idle / SMOOTHER decel: the GLOSA
controller shapes approach speed so CAVs roll through green instead of braking
to a halt and idling.

Usage:
    # 1) baseline run (in run_eco_demo_headless.bat or by hand):
    python cav_controller.py --penetration 0.0   ... -> out/summary_baseline.json
    # 2) eco run:
    python cav_controller.py --penetration 1.0   ... -> out/summary_eco.json
    # 3) compare:
    python verify_eco.py
"""

from __future__ import annotations

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "out"


def _load(name: str) -> dict | None:
    p = OUT / name
    if not p.is_file():
        return None
    return json.loads(p.read_text())


def _pct(base: float, eco: float) -> str:
    if base == 0:
        return "n/a" if eco == 0 else f"+{eco:.2f} (baseline 0)"
    return f"{(eco - base) / base * 100:+.1f}%"


def main() -> int:
    base = _load("summary_baseline.json")
    eco = _load("summary_eco.json")
    if base is None or eco is None:
        print("ERROR: need both out/summary_baseline.json and out/summary_eco.json.")
        print("  Run the demo twice: --penetration 0.0 (baseline) then > 0 (eco).")
        return 2

    rows = [
        ("CAVs seen",            "n_cavs_seen",          False),
        ("CAVs that stopped",    "cavs_that_stopped",    True),
        ("Total idle time (s)",  "total_idle_s",         True),
        ("Mean decel proxy m/s", "mean_decel_proxy_mps", True),
    ]
    print("\n=== Eco-driving before/after (DriverModel CAVs) ===")
    print(f"{'metric':24s} {'baseline':>12s} {'eco':>12s} {'change':>14s}")
    print("-" * 66)
    improvements = 0
    measured = 0
    for label, key, lower_is_better in rows:
        b, e = base.get(key, 0), eco.get(key, 0)
        change = _pct(b, e) if lower_is_better else ""
        print(f"{label:24s} {b:>12} {e:>12} {change:>14s}")
        if lower_is_better:
            measured += 1
            if e < b:
                improvements += 1

    print("-" * 66)
    print(f"eco-active CAV-ticks: {eco.get('eco_active_cav_ticks', 0)} "
          f"(penetration {eco.get('penetration')})")

    verdict = improvements >= 1 and eco.get("eco_active_cav_ticks", 0) > 0
    print(f"\n=== {'PASS' if verdict else 'INCONCLUSIVE'}: "
          f"{improvements}/{measured} cost metrics improved, "
          f"GLOSA was {'active' if eco.get('eco_active_cav_ticks', 0) > 0 else 'NEVER active'} ===")
    if eco.get("eco_active_cav_ticks", 0) == 0:
        print("  GLOSA never engaged -- signals.json positions are likely uncalibrated")
        print("  (CAVs free-flow when no signal is matched ahead). See README.")

    (OUT / "compare.json").write_text(json.dumps(
        {"baseline": base, "eco": eco, "improvements": improvements,
         "measured": measured, "verdict": verdict}, indent=2))
    print(f"\nwrote {OUT / 'compare.json'}")
    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())
