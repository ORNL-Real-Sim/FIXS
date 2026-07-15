# World Model traffic-agent translator (#189)

A common, config-selectable translator that hands the **ego bubble** (all
vehicles in a square BEV window around the ego) to a learned reactive traffic
**world model**, runs one inference step, and writes the updated surrounding
poses back into SUMO through TrafficLayer.

Structurally this is a `tests/Python/SimpleEchoClient` client with a neural
forward pass as the transform. See [doc/WorldModelDoc.md](../../doc/WorldModelDoc.md)
for the full design.

## Two usages

| | Data flow | Ego driver | Status |
|---|---|---|---|
| **1** | `SUMO -> TrafficLayer -> WM` | SUMO (bubble centre) | **built now** |
| **2** | `SUMO -> TrafficLayer -> controller -> WM` | a controller's planned action | later |

Usage 1 is a pure multi-resolution simulator: SUMO runs the background, the WM
re-simulates the high-fidelity agents surrounding the ego. Usage 2 adds a
controller that plans the ego's action and feeds it to the WM as conditioning
(same `ego_future` field, different source).

## Layout

```
tests/WorldModels/
  bev.py                 square BEV crop, heading conversion, rolling history, ego-future
  wm_translator.py       the loop: recv -> crop -> history -> backend.step -> send back
  backends/
    base.py              WorldModelBackend Protocol (no ABC)
    stub.py              constant-velocity, NO torch/jax  (default CI path)
    __init__.py          registry (lazy import for heavy backends)
    alpasim_backend.py   NVIDIA AlpaSim CATK/SMART (PyTorch)   [to scaffold]
    waymax_backend.py    Waymo Waymax (JAX)                    [to scaffold]
  environment-worldmodel.yml   heavy deps, installed ON DEMAND (never in root env)
  test_wm_translator.py  CI-safe unit tests (no sockets/SUMO/torch/jax)
  Stub/                  runnable end-to-end smoke test (stub backend)
    config.yaml, wm_loop.rou.xml, wm_loop.sumocfg, simple_loop.net.xml, run_wm_stub_demo.bat
```

## Run the CI-safe unit tests (no simulator, no heavy deps)

```
python -m pytest tests/WorldModels/test_wm_translator.py -v
```

Validates the FIXS-side plumbing (BEV crop, rolling history, stub
constant-velocity, heading conversion) with fabricated `VehData`.

## Run the end-to-end stub smoke test (needs TrafficLayer.exe + SUMO)

```
cd tests/WorldModels/Stub
run_wm_stub_demo.bat
```

Launches TrafficLayer (auto-launches SUMO with a 6-vehicle loop), then the
translator with the `stub` backend for 300 steps. Exit code 0 = the full loop
ran end to end: bubble crop -> stub -> `moveToXY` write-back, ego untouched.

## Backends

- **stub** — constant velocity by finite-difference of the last two frames; no
  heavy deps. What CI runs.
- **alpasim** — CATK/SMART tokenized reactive model (PyTorch). Needs a rolling
  >=16-frame history and a const-velocity ego future (usage 1). *To be wired.*
- **waymax** — Waymo Waymax (JAX), `StateDynamics` / sim-agent policies. Global
  frame, needs no ego future. *To be wired.*

Selected via `WorldModelSetup.Backend` in `config.yaml`. Heavy backends install
from `environment-worldmodel.yml` on demand; the root `environment.yml` stays
lightweight.
