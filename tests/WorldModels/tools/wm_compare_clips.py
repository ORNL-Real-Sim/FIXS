"""Build a side-by-side Plotly clip from the native and signal-probe NPZ files."""
from __future__ import annotations

import argparse
import pathlib

import numpy as np


LANE = 1.75


def _signal_lanes(half: float = 400.0) -> list[np.ndarray]:
    s = np.arange(-half, half + 1.0, 1.0)
    return [
        np.column_stack((s, np.full_like(s, -LANE))),
        np.column_stack((s[::-1], np.full_like(s, LANE))),
        np.column_stack((np.full_like(s, LANE), s)),
        np.column_stack((np.full_like(s, -LANE), s[::-1])),
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=pathlib.Path, default=pathlib.Path("native_reference.npz"))
    parser.add_argument("--signal", type=pathlib.Path, default=pathlib.Path("signal_probe_stopped.npz"))
    parser.add_argument("--out", type=pathlib.Path, default=pathlib.Path("native_vs_signal.html"))
    args = parser.parse_args()

    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    native = np.load(args.native)
    signal = np.load(args.signal)
    signal_names = ("stop_to_go", "stop_to_none", "none_only")
    labels = ("STOP->GO", "STOP->NONE", "NONE-only")
    colors = ("#2563eb", "#f97316", "#16a34a")
    symbols = ("circle", "x", "diamond-open")

    fig = make_subplots(
        rows=1,
        cols=2,
        subplot_titles=("A — native ClipGT (16.1 s available)", "B — stopped signal queue (30 s)"),
        horizontal_spacing=0.06,
    )
    # Native trajectory context. These traces remain fixed while the markers animate.
    ego = native["ego"]
    fig.add_trace(go.Scattergl(x=ego[:, 0], y=ego[:, 1], mode="lines", line={"color": "#fecaca", "width": 1}, showlegend=False), row=1, col=1)
    for j in range(native["xyz"].shape[1]):
        gt_valid = native["gt_valid"][:, j]
        fig.add_trace(go.Scattergl(x=native["gt"][gt_valid, j, 0], y=native["gt"][gt_valid, j, 1], mode="lines", line={"color": "#d1d5db", "width": 1}, showlegend=False), row=1, col=1)
    for lane in _signal_lanes():
        fig.add_trace(go.Scattergl(x=lane[:, 0], y=lane[:, 1], mode="lines", line={"color": "#e5e7eb", "width": 1}, showlegend=False), row=1, col=2)

    dynamic = []
    dynamic.append(len(fig.data))
    fig.add_trace(go.Scatter(x=[], y=[], mode="markers", marker={"color": "#dc2626", "size": 11}, name="native logged ego"), row=1, col=1)
    dynamic.append(len(fig.data))
    fig.add_trace(go.Scatter(x=[], y=[], mode="markers", marker={"color": "#2563eb", "size": 9}, name="native CATK agents"), row=1, col=1)
    dynamic.append(len(fig.data))
    fig.add_trace(go.Scatter(x=[], y=[], mode="markers", marker={"color": "#9ca3af", "size": 7, "symbol": "circle-open"}, name="native logged agents"), row=1, col=1)
    for name, label, color, symbol in zip(signal_names, labels, colors, symbols):
        dynamic.append(len(fig.data))
        xy = signal[f"{name}_xy"]
        fig.add_trace(go.Scatter(x=xy[0, :, 0], y=xy[0, :, 1], mode="markers", marker={"color": color, "size": 10, "symbol": symbol}, name=label), row=1, col=2)

    signal_t = signal["stop_to_go_t"]
    native_t = native["t"]
    stride = max(1, len(signal_t) // 180)
    frames = []
    for si in range(0, len(signal_t), stride):
        ni = min(int(np.searchsorted(native_t, signal_t[si], side="right") - 1), len(native_t) - 1)
        ni = max(0, ni)
        nvalid = native["valid"][ni]
        gtvalid = native["gt_valid"][ni]
        data = [
            go.Scatter(x=[native["ego"][ni, 0]], y=[native["ego"][ni, 1]]),
            go.Scatter(x=native["xyz"][ni, nvalid, 0], y=native["xyz"][ni, nvalid, 1]),
            go.Scatter(x=native["gt"][ni, gtvalid, 0], y=native["gt"][ni, gtvalid, 1]),
        ]
        data.extend(
            go.Scatter(x=signal[f"{name}_xy"][si, :, 0], y=signal[f"{name}_xy"][si, :, 1])
            for name in signal_names
        )
        light = "red" if signal_t[si] <= float(signal["red_seconds"]) else "green"
        frames.append(go.Frame(
            name=f"{signal_t[si]:.1f}",
            data=data,
            traces=dynamic,
            layout=go.Layout(title=f"CATK isolation — t={signal_t[si]:.1f}s — synthetic signal {light}"),
        ))
    fig.frames = frames
    fig.update_xaxes(range=[-20, 225], title="x (m)", row=1, col=1)
    fig.update_yaxes(range=[-75, 75], title="y (m)", scaleanchor="x", row=1, col=1)
    fig.update_xaxes(range=[-60, 300], title="x (m)", row=1, col=2)
    fig.update_yaxes(range=[-22, 22], title="y (m)", scaleanchor="x2", row=1, col=2)
    fig.update_layout(
        title="CATK isolation: healthy native motion versus absorbing stopped queue",
        width=1500,
        height=650,
        legend={"orientation": "h", "y": -0.12},
        updatemenus=[{"type": "buttons", "buttons": [{"label": "play", "method": "animate", "args": [None, {"frame": {"duration": 65}, "fromcurrent": True}]}, {"label": "pause", "method": "animate", "args": [[None], {"frame": {"duration": 0}, "mode": "immediate"}]}]}],
        sliders=[{"steps": [{"method": "animate", "args": [[frame.name], {"frame": {"duration": 0}, "mode": "immediate"}], "label": frame.name} for frame in frames]}],
    )
    fig.write_html(args.out, include_plotlyjs=True)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
