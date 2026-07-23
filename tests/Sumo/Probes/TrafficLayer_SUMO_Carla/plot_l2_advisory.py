"""Verify L2 ego speed-tracking from the FIXS DataLogger CSV.

L2 (EgoMode 2) feeds the ego's L0 driver an external desired-speed advisory
(CommonLib/EgoSpeedAdvisor). This TEST-SPECIFIC script reads the CSV written by
run_sumo_carla_l2_advisory.bat (_datalog/l2_advisory.csv) -- where `speed` is the
ego's MEASURED speed and `speedDesired` is the COMMANDED advisory -- and shows
how well the measured speed tracks the command, plus the tracking error.

Run:  python plot_l2_advisory.py
"""
from __future__ import annotations
import csv, math, pathlib, sys

HERE = pathlib.Path(__file__).resolve().parent
DL = HERE / "_datalog"
CSV = DL / "l2_advisory.csv"


def load(path):
    t, spd, des = [], [], []
    with open(path) as fh:
        for row in csv.DictReader(l for l in fh if not l.startswith("#")):
            t.append(float(row["simTime"]))
            spd.append(float(row["speed"]))
            des.append(float(row["speedDesired"]))
    return dict(t=t, spd=spd, des=des)


def pct(v, p):
    v = sorted(v)
    return v[max(0, min(len(v) - 1, int(p / 100 * len(v))))]


def main():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available in this interpreter"); return 2
    if not CSV.is_file():
        print(f"missing {CSV} -- run run_sumo_carla_l2_advisory.bat first"); return 2

    d = load(CSV)
    if not d["t"]:
        print(f"{CSV} has no ego rows"); return 2
    err = [m - c for m, c in zip(d["spd"], d["des"])]

    fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("L2 ego speed tracking on SimpleLoop -- measured vs commanded "
                 "advisory (FIXS DataLogger)", fontsize=13, fontweight="bold")

    win = [i for i in range(len(d["t"])) if d["t"][i] <= 120]
    ax[0].plot([d["t"][i] for i in win], [d["des"][i] for i in win],
               "#dc2626", lw=1.6, label="commanded advisory (speedDesired)")
    ax[0].plot([d["t"][i] for i in win], [d["spd"][i] for i in win],
               "#2563eb", lw=1.2, label="measured speed (Carla PhysX)")
    ax[0].set_title("(A) speed vs time (first 120 s)")
    ax[0].set_ylabel("speed [m/s]"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=9)

    ax[1].axhline(0, color="gray", lw=.8)
    ax[1].plot([d["t"][i] for i in win], [err[i] for i in win], "#059669", lw=1.0)
    ax[1].set_title("(B) tracking error (measured - commanded)")
    ax[1].set_xlabel("sim time [s]"); ax[1].set_ylabel("error [m/s]"); ax[1].grid(alpha=.3)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = DL / "l2_advisory.png"
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")

    # numeric summary -- exclude the initial spin-up (first 5 s) from error stats
    settled = [abs(err[i]) for i in range(len(d["t"])) if d["t"][i] > 5.0]
    print("\nL2 speed-tracking summary")
    print(f"  rows                : {len(d['t'])}")
    print(f"  commanded range     : {min(d['des']):.2f} .. {max(d['des']):.2f} m/s")
    print(f"  measured range      : {min(d['spd']):.2f} .. {max(d['spd']):.2f} m/s")
    if settled:
        print(f"  |error| mean (t>5s) : {sum(settled)/len(settled):.3f} m/s")
        print(f"  |error| 95th (t>5s) : {pct(settled, 95):.3f} m/s")
        print(f"  |error| max  (t>5s) : {max(settled):.3f} m/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
