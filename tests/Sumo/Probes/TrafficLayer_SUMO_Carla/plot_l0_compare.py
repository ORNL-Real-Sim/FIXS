"""Compare the two L0 ego drivers (native Carla TM vs our EgoDriver module) from
the FIXS DataLogger CSVs, on the shared SimpleLoop scenario.

This is a TEST-SPECIFIC analysis script (per the FIXS logging split: the logger
is generic infrastructure in CommonLib/DataLogger; the plotting lives here). It
reads the two CSVs produced by run_sumo_carla_l0_tm.bat / _egodriver.bat
(_datalog/l0_tm.csv, _datalog/l0_egodriver.csv) -- both in the SUMO/VISSIM wire
convention -- and writes _datalog/l0_compare.png plus a numeric summary.

Run:  python plot_l0_compare.py
"""
from __future__ import annotations
import csv, math, pathlib, sys

HERE = pathlib.Path(__file__).resolve().parent
DL = HERE / "_datalog"
RUNS = [("EgoDriver (pursuit module)", DL / "l0_egodriver.csv", "#2563eb"),
        ("Native Carla TM",            DL / "l0_tm.csv",        "#dc2626")]


def load(path):
    t, x, y, hdg, spd = [], [], [], [], []
    with open(path) as fh:
        for row in csv.DictReader(l for l in fh if not l.startswith("#")):
            t.append(float(row["simTime"])); x.append(float(row["positionX"]))
            y.append(float(row["positionY"])); hdg.append(float(row["heading"]))
            spd.append(float(row["speed"]))
    return dict(t=t, x=x, y=y, hdg=hdg, spd=spd)


def first_lap(d):
    """Indices of the first full lap: leave the start point, then return to it."""
    x0, y0 = d["x"][0], d["y"][0]
    left = None
    for i in range(len(d["x"])):
        r = math.hypot(d["x"][i] - x0, d["y"][i] - y0)
        if left is None and r > 40:
            left = i
        elif left is not None and r < 5 and i > left + 10:
            return range(0, i + 1)
    return range(len(d["x"]))


def arclen(d, idx):
    s, acc = [], 0.0
    idx = list(idx)
    for k, i in enumerate(idx):
        if k: acc += math.hypot(d["x"][i] - d["x"][idx[k-1]], d["y"][i] - d["y"][idx[k-1]])
        s.append(acc)
    return s


def pct(v, p):
    v = sorted(v); return v[max(0, min(len(v) - 1, int(p / 100 * len(v))))]


def main():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available in this interpreter"); return 2
    for _, p, _ in RUNS:
        if not p.is_file():
            print(f"missing {p} -- run the demo first"); return 2

    data = [(name, load(p), c) for name, p, c in RUNS]

    fig, ax = plt.subplots(2, 2, figsize=(13, 10))
    fig.suptitle("L0 ego drivers on SimpleLoop -- SUMO/VISSIM wire convention "
                 "(FIXS DataLogger)", fontsize=13, fontweight="bold")

    # (A) trajectory, first lap
    for name, d, c in data:
        lap = first_lap(d)
        ax[0, 0].plot([d["x"][i] for i in lap], [d["y"][i] for i in lap], c, lw=1.6, label=name)
    ax[0, 0].set_title("(A) ego path, first lap"); ax[0, 0].set_xlabel("x [m]")
    ax[0, 0].set_ylabel("y [m]"); ax[0, 0].set_aspect("equal"); ax[0, 0].grid(alpha=.3); ax[0, 0].legend()

    # (B) speed vs time, first 200 s
    for name, d, c in data:
        idx = [i for i in range(len(d["t"])) if d["t"][i] <= 200]
        ax[0, 1].plot([d["t"][i] for i in idx], [d["spd"][i] for i in idx], c, lw=1.1, label=name)
    ax[0, 1].axhline(8.33, ls="--", color="gray", lw=.8, label="target 8.33 m/s")
    ax[0, 1].set_title("(B) speed vs time (first 200 s)"); ax[0, 1].set_xlabel("sim time [s]")
    ax[0, 1].set_ylabel("speed [m/s]"); ax[0, 1].grid(alpha=.3); ax[0, 1].legend(fontsize=8)

    # (C) speed vs arc length, first lap (corner slowdowns aligned)
    for name, d, c in data:
        lap = first_lap(d); s = arclen(d, lap)
        ax[1, 0].plot(s, [d["spd"][i] for i in lap], c, lw=1.4, label=name)
    ax[1, 0].set_title("(C) speed vs distance along loop (first lap)")
    ax[1, 0].set_xlabel("arc length [m]"); ax[1, 0].set_ylabel("speed [m/s]")
    ax[1, 0].grid(alpha=.3); ax[1, 0].legend(fontsize=8)

    # (D) heading vs arc length, first lap (cornering: 4 x 90 deg steps)
    for name, d, c in data:
        lap = first_lap(d); s = arclen(d, lap)
        ax[1, 1].plot(s, [d["hdg"][i] for i in lap], c, lw=1.4, label=name)
    ax[1, 1].set_title("(D) heading vs distance along loop (first lap)")
    ax[1, 1].set_xlabel("arc length [m]"); ax[1, 1].set_ylabel("heading [deg, 0=N CW]")
    ax[1, 1].grid(alpha=.3); ax[1, 1].legend(fontsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = DL / "l0_compare.png"
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")

    # numeric summary
    print("\n{:28s} {:>8s} {:>8s} {:>8s} {:>9s} {:>9s}".format(
        "driver", "mean v", "95th v", "min v", "lap len", "laps"))
    for name, d, _ in data:
        lap = first_lap(d); s = arclen(d, lap)
        moving = [v for v in d["spd"] if v > 0.1]
        print("{:28s} {:8.2f} {:8.2f} {:8.2f} {:9.1f} {:9d}".format(
            name, sum(moving)/len(moving), pct(moving, 95), pct(moving, 5),
            s[-1], round(sum(math.hypot(d["x"][i]-d["x"][i-1], d["y"][i]-d["y"][i-1])
                          for i in range(1, len(d["x"]))) / max(s[-1], 1))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
