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
  vector_map.py          SUMO .net.xml -> lane-centre polylines (parsed once)
  wm_translator.py       the loop: recv -> crop -> history -> backend.step -> send back
  backends/
    base.py              WorldModelBackend Protocol (no ABC)
    constant_velocity.py speed+heading advance, NO torch/jax  (default CI path)
    __init__.py          registry (lazy import for heavy backends)
    alpasim.py           NVIDIA AlpaSim CATK/SMART (PyTorch+CUDA)
    waymax.py            Waymo Waymax IDMRoutePolicy (JAX)
  environment-worldmodel.yml   heavy deps, installed ON DEMAND (never in root env)
  test_wm_translator.py  CI-safe unit tests (no sockets/SUMO/torch/jax)
  Demo/                  runnable endless-loop demo (SimpleTrafficLight loop)
    config.yaml, config_alpasim.yaml, config_waymax.yaml,
    simple_traffic_light.{net,rou,sumocfg},
    run_without_wm.bat, run_with_wm.bat, run_with_wm_alpasim.bat, run_with_wm_waymax.bat
```

## Run the CI-safe unit tests (no simulator, no heavy deps)

```
python -m pytest tests/WorldModels/test_wm_translator.py -v
```

Validates the FIXS-side plumbing (BEV crop, rolling history, constant-velocity,
heading conversion) with fabricated `VehData`.

## Run the endless-loop demo (`Demo/`, needs TrafficLayer.exe + SUMO)

The scene: the ego (`egoCm123`, red) circulates an endless teardrop loop through
3 traffic signals; background traffic transits the corridor and exits at the
ends (so on the loop the ego is alone -- the WM must handle zero neighbours).
One click each, to compare visually:

| launcher | what drives the surrounding bubble traffic |
|---|---|
| `run_without_wm.bat` | **nothing** -- pure SUMO (baseline; traffic obeys the signals) |
| `run_with_wm.bat` | **constant_velocity** baseline WM (no heavy deps) |
| `run_with_wm_alpasim.bat` | **AlpaSim / CATK** tokenized reactive model (PyTorch, GPU) |
| `run_with_wm_waymax.bat` | **Waymax** IDM route-following policy (JAX, CPU on Windows) |

The ego always loops via its SUMO route; only the surrounding vehicles differ.

## Backends (`WorldModelSetup.Backend`)

- **constant_velocity** — advance by reported speed+heading; no torch/jax. CI +
  baseline. (Not signal-aware, by design -- the comparison point.)
- **alpasim** — CATK/SMART tokenized reactive model (PyTorch+CUDA). Builds
  `env_data` directly (bubble history >=16 frames + pinned ego plan + a
  SUMO-derived vector map), runs one inference, takes sub-step 0.
- **waymax** — Waymo Waymax `IDMRoutePolicy` (JAX). Per step builds a
  `SimulatorState` where each agent follows a const-velocity route; IDM does
  car-following. Global frame, no history needed.

## Real-backend environments (installed on demand, outside the repo)

Heavy deps must NOT go in the root `environment.yml`. Dedicated venvs:

- **AlpaSim/CATK** — `C:\src_git\WM_AlpaSim\.venv-catk` (torch 2.8+cu128, PyG
  `torch_cluster/scatter`, omegaconf, hydra, loguru, einops). Deliberately WITHOUT
  `trajdata`/`deltalake` (no Windows wheels) -- CATK *inference* doesn't need them;
  we bypass its map_adapter by building `env_data` ourselves. Weights + tokens at
  `WM_AlpaSim/data/trafficsim-models/{catk_v120,tokens}`.
- **Waymax** — `C:\src_git\WM_Waymax\.venv-waymax` (`git+waymax`, jax). JAX has no
  CUDA build on native Windows, so it runs CPU-only (fine for a small bubble).

The `run_with_wm_{alpasim,waymax}.bat` launchers point the translator at the
matching venv; the plain `realsim` env runs everything else (TrafficLayer client
plumbing + the constant_velocity backend). See `environment-worldmodel.yml` for
the intended single-file layering.
