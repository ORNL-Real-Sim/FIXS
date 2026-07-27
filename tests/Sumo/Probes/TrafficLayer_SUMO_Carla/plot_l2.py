"""Verify L2 ego speed-tracking from the FIXS DataLogger CSV (interactive, plotly).

L2 (EgoMode 2) drives the ego's target from an EXTERNAL desired-speed advisory
streamed over FIXS by py_ego_speed_advisor.py. This TEST-SPECIFIC script reads the
CSV written by run_sumo_carla_l2.bat (_datalog/l2.csv) -- where `speed` is the
ego's MEASURED speed and `speedDesired` is the COMMANDED wire advisory -- and
writes an interactive HTML (_datalog/l2.html) of measured vs commanded + the
tracking error, plus a numeric summary.

Runs under the repo's realsim conda env (plotly; no matplotlib needed):
    <realsim-python> plot_l2.py [csv_path]
"""
from __future__ import annotations
import csv, pathlib, sys

HERE = pathlib.Path(__file__).resolve().parent
DL = HERE / "_datalog"


def load(path):
    t, spd, des = [], [], []
    with open(path) as fh:
        for row in csv.DictReader(l for l in fh if not l.startswith("#")):
            t.append(float(row["simTime"]))
            spd.append(float(row["speed"]))
            des.append(float(row["speedDesired"]))
    return t, spd, des


def pct(v, p):
    v = sorted(v)
    return v[max(0, min(len(v) - 1, int(p / 100 * len(v))))]


def main():
    csv_path = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else DL / "l2.csv"
    if not csv_path.is_file():
        print(f"missing {csv_path} -- run run_sumo_carla_l2.bat first"); return 2
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        print("plotly not available -- use the realsim conda env"); return 2

    t, spd, des = load(csv_path)
    if not t:
        print(f"{csv_path} has no ego rows"); return 2
    err = [m - c for m, c in zip(spd, des)]

    fig = make_subplots(
        rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.09, row_heights=[0.64, 0.36],
        subplot_titles=("measured speed vs commanded advisory", "tracking error (measured - commanded)"))
    fig.add_trace(go.Scatter(x=t, y=des, name="commanded advisory (speedDesired)",
                             line=dict(color="#dc2626", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=spd, name="measured speed (Carla PhysX)",
                             line=dict(color="#2563eb", width=1.4)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=err, name="error", line=dict(color="#059669", width=1.2),
                             showlegend=False), row=2, col=1)
    fig.add_hline(y=0, line=dict(color="gray", width=1), row=2, col=1)
    fig.update_yaxes(title_text="speed [m/s]", row=1, col=1)
    fig.update_yaxes(title_text="error [m/s]", row=2, col=1)
    fig.update_xaxes(title_text="sim time [s]", row=2, col=1)
    fig.update_layout(
        title="L2 ego speed tracking on SimpleLoop -- external advisory over FIXS",
        template="plotly_white", hovermode="x unified", legend=dict(x=0.01, y=0.99))

    out = DL / "l2.html"
    fig.write_html(str(out), include_plotlyjs=True)   # self-contained: opens offline
    print(f"wrote {out}")

    settled = [abs(err[i]) for i in range(len(t)) if t[i] > 5.0]
    print("\nL2 speed-tracking summary")
    print(f"  rows                : {len(t)}")
    print(f"  commanded range     : {min(des):.2f} .. {max(des):.2f} m/s")
    print(f"  measured range      : {min(spd):.2f} .. {max(spd):.2f} m/s")
    if settled:
        print(f"  |error| mean (t>5s) : {sum(settled)/len(settled):.3f} m/s")
        print(f"  |error| 95th (t>5s) : {pct(settled, 95):.3f} m/s")
        print(f"  |error| max  (t>5s) : {max(settled):.3f} m/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
