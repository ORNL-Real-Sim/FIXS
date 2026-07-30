"""L2 ego speed: commanded advisory vs Carla‑measured vs SUMO‑view, from ONE log.

Reads _datalog/l2.csv (run_sumo_carla_l2.bat), which now logs the ego twice per
feed on a SINGLE clock:
  id 'ego'      = Carla readback   (speed = measured, speedDesired = the external command)
  id 'ego_sumo' = SUMO's report    (speed = SUMO getSpeed)
With setSpeed(actual) the SUMO view should EQUAL the Carla speed, lagged ~2 ticks
by the co-sim pipeline (Carla is a step ahead + SUMO getPosition is n-1). We show
both raw and pipeline-aligned. Interactive plotly HTML under the realsim env.

Run:  <realsim-python> plot_l2.py [csv_path]
"""
from __future__ import annotations
import csv, pathlib, sys

HERE = pathlib.Path(__file__).resolve().parent
DL = HERE / "_datalog"
LAG_S = 0.2   # measured co-sim pipeline lag (Carla step-ahead + SUMO n-1)


def load(path):
    ego, sumo = {}, {}
    with open(path) as fh:
        for r in csv.DictReader(l for l in fh if not l.startswith("#")):
            t = round(float(r["simTime"]), 3)
            i = r["id"].strip()
            if i == "ego":
                ego[t] = (float(r["speed"]), float(r["speedDesired"]))
            elif i == "ego_sumo":
                sumo[t] = float(r["speed"])
    return ego, sumo


def main():
    csv_path = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else DL / "l2.csv"
    if not csv_path.is_file():
        print(f"missing {csv_path} -- run run_sumo_carla_l2.bat first"); return 2
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        print("plotly not available -- use the realsim conda env"); return 2

    ego, sumo = load(csv_path)
    if not ego:
        print(f"{csv_path} has no ego rows"); return 2
    t = sorted(ego); spd = [ego[x][0] for x in t]; des = [ego[x][1] for x in t]
    ts = sorted(sumo); ss = [sumo[x] for x in ts]
    ts_aligned = [round(x - LAG_S, 3) for x in ts]   # shift SUMO back onto Carla's timeline

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.09, row_heights=[0.66, 0.34],
                        subplot_titles=("commanded vs Carla‑measured vs SUMO‑view (setSpeed=actual)",
                                        f"SUMO‑view minus Carla speed, pipeline‑aligned (−{LAG_S}s)"))
    fig.add_trace(go.Scatter(x=t, y=des, name="commanded advisory (speedDesired)",
                             line=dict(color="#dc2626", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=spd, name="Carla measured (ego)",
                             line=dict(color="#2563eb", width=1.4)), row=1, col=1)
    if ss:
        fig.add_trace(go.Scatter(x=ts, y=ss, name="SUMO getSpeed (ego_sumo), raw",
                                 line=dict(color="#7c3aed", width=1.1, dash="dot")), row=1, col=1)
        fig.add_trace(go.Scatter(x=ts_aligned, y=ss, name=f"SUMO getSpeed, aligned (−{LAG_S}s)",
                                 line=dict(color="#059669", width=1.1)), row=1, col=1)
        # aligned residual vs Carla
        em = dict(zip(t, spd))
        resid_t = [x for x in ts_aligned if x in em]
        resid = [dict(zip(ts_aligned, ss))[x] - em[x] for x in resid_t]
        fig.add_trace(go.Scatter(x=resid_t, y=resid, name="SUMO(aligned) − Carla",
                                 line=dict(color="#059669", width=1.0), showlegend=False), row=2, col=1)
        fig.add_hline(y=0, line=dict(color="gray", width=1), row=2, col=1)
        if resid:
            import statistics as st
            ar = [abs(r) for r in resid if True]
            print(f"SUMO(aligned) vs Carla: mean|diff| {st.mean(ar):.3f}  max {max(ar):.3f} m/s  (should be ~0 with setSpeed=actual)")
    fig.update_yaxes(title_text="speed [m/s]", row=1, col=1)
    fig.update_yaxes(title_text="Δ [m/s]", row=2, col=1)
    fig.update_xaxes(title_text="sim time [s]", row=2, col=1)
    fig.update_layout(title="L2 ego speed — one clock (Carla vs SUMO)", template="plotly_white",
                      hovermode="x unified", legend=dict(x=0.01, y=0.99))
    out = DL / "l2.html"
    fig.write_html(str(out), include_plotlyjs=True)
    print(f"wrote {out}")
    print(f"  commanded {min(des):.2f}..{max(des):.2f}  Carla {min(spd):.2f}..{max(spd):.2f}"
          + (f"  SUMO {min(ss):.2f}..{max(ss):.2f}" if ss else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
