# World Model traffic-agent translator (#189) — Final Design

FIXS provides a common, config-selectable **orchestrator** that runs a
learned reactive traffic **world model (WM)** on the vehicles *surrounding the
ego* (the "bubble"), inside a SUMO co-simulation. SUMO simulates the
background at low fidelity; the WM re-simulates the bubble neighbours at high
fidelity each step. Two built-in backends behind one interface: **NVIDIA
AlpaSim** (CATK/SMART, PyTorch) and **Waymo Waymax** (JAX), plus a
dependency-free `constant_velocity` baseline.

This is **not** routed through `VirEnvCore` / `IVirEnvBackend` (#174): a WM is a
one-step `states -> inference -> states` transform, not a co-simulated virtual
environment (no spawn pools, sensors, rendering, TLS, pose anchoring, temporal
interpolation, ego A/B orchestration). Structurally it is a
`tests/Python/SimpleEchoClient`-shaped transform.

---

## 1. Roles and formulation

Let `sₖ` = surrounding-traffic state, `xₖ` = ego state, `uₖ:ₖ₊ₙ` = ego
command/plan over horizon N.

- **Controller** (owns the **ego**): `(sₖ, xₖ) → uₖ:ₖ₊ₙ`.
  User-provided; **out of FIXS scope**. FIXS ships only trivial controllers as
  test doubles. The controller emits an ego **plan** (a pose trajectory; or raw
  actions plus an ego-dynamics block that converts them to poses).
- **World model** (owns the **rest**): `(sₖ, ego_plan) → sₖ₊₁`.
  Outputs the **neighbours only**. The ego's next state is **`xₖ₊₁ = plan[k+1]`**
  — supplied by the controller's plan, **never a WM output**.

Why the ego is on the WM's *input* side (this is the crux):

| backend | outputs the ego? | reason |
|---|---|---|
| **CATK** | no | structurally — the ego is a *pinned* input (`freeze_agent_mask`); `inference()` returns `~is_ego` only |
| **Waymax** | no (by config) | it *could* (full simulator), but we set `is_controlled = neighbours only` and overwrite the ego from the plan, so it too returns only `sₖ₊₁` |

Keeping the ego out of the WM output gives **one uniform backend interface**
(`(sₖ, ego_plan) → sₖ₊₁`) and **one unambiguous owner per entity**: controller ⟶
ego, WM ⟶ neighbours. No tie-breaker between two authors of the ego.

The full next bubble written to SUMO is the union of two owners:
```
bubble_{k+1} = sₖ₊₁  (neighbours, from WM)  ∪  xₖ₊₁ = plan[k+1]  (ego, from controller)
```

---

## 2. One unified mechanism (controller is the only variable)

There is a single co-sim loop; "usage 1" and "usage 2" are the *same* loop with
a different controller plugged in:

| | controller | ego motion | status |
|---|---|---|---|
| multi-resolution sim | trivial (`constant_speed`, FIXS test double) | follows a trivial plan | build now |
| closed-loop control | user's real controller/planner | follows the real plan | later, no loop change |

The WM orchestrator code is **identical** in both — it never knows or cares
what kind of controller produced the ego plan.

---

## 3. Per-tick sequence

TrafficLayer services clients **in one ordered `send → recv` pass** (controller
first, WM second), forwarding the controller's plan into the WM's input:

```
each tick k (SUMO --step-length 0.1 = 10 Hz):
  1. sₖ, xₖ = TL reads bubble from SUMO
  ── Stage 1: controller (owns ego) ─────────────────────────────
  2. TL → controller : (sₖ, xₖ)
  3. TL ← controller : uₖ:ₙ                       # ego command / plan
  ── Stage 2: world model (owns neighbours) ─────────────────────
  4. TL → WM : (sₖ, xₖ, uₖ:ₙ)                     # TL FORWARDS the plan
  5. TL ← WM : sₖ₊₁                                # neighbours only
  ── Write back (both owners) ───────────────────────────────────
  6. TL → SUMO : xₖ₊₁ = plan[k+1]  (ego)  +  sₖ₊₁  (neighbours)
  7. SUMO.step()
```

**Zero-lag conditioning** without any side-channel: because servicing is ordered,
the controller's stage-1 output is in hand when TL builds the WM's stage-2 input.

---

## 4. TrafficLayer changes (three, all modest)

1. **Ordered one-loop servicing.** Collapse today's send-all
   ([mainTrafficLayer.cpp:1085](../TrafficLayer/TrafficLayer/mainTrafficLayer.cpp#L1085))
   + recv-all ([:1246](../TrafficLayer/TrafficLayer/mainTrafficLayer.cpp#L1246))
   into one loop that does `send → recv` per client in configured order, and
   forwards a producer client's output into a downstream client's input:
   ```
   from:  { for c: send(c) } ; { for c: recv(c) }
   to:    for c in order: send(c, working); working ⨝ recv(c)
   ```
   Uses the existing `SumoSetup.ExecutionOrder`
   ([ConfigHelper.h:156](../CommonLib/ConfigHelper.h#L156)); folds into the #113
   `mainTrafficLayer` cleanup rather than a bypass. Forwarding is **scoped by
   subscription**, so independent clients (existing CarMaker/CARLA setups) are
   unaffected.
2. **Model-agnostic external-pose write-back.** Generalize the CARLA
   `InterestedIds` `moveToXY`+`setSpeed`(+spawn-if-missing) branch
   ([TrafficHelper.cpp:707](../CommonLib/TrafficHelper.cpp#L707)) into an
   "external region owns these ids" branch, keyed by ownership (controller ⟶
   `EgoId`, WM ⟶ the returned neighbour ids). Requires `SpeedMode=32` (safety
   checks off) so commanded poses take effect. No new `WmHelper.cpp`.
3. **`WorldModelSetup` config.** New `WorldModelSetup_t` in
   `ConfigHelper.{h,cpp,py}`, mirroring `CarlaSetup_t`.

---

## 5. Connection topology (transport-independent)

**Invariant:** FIXS-provided components are **servers** (they listen); the
user's controller and the WM translator are **clients** that dial into TL.
Everything goes through TL — no controller↔WM side channel.

```
              ┌──────────────┐
              │     SUMO     │  in-process (libsumo), not a socket
              └──────▲───────┘
              ┌──────┴───────────────────────────┐
              │        TrafficLayer (TL)          │   SERVER (listens)
              │   ordered: controller → WM        │
              └───▲───────────────────────▲───────┘
        dials in ─┘                       └─ dials in
      ┌───────────┐                    ┌──────────────────┐
      │ Controller│                    │  WM translator   │
      │  (ego)    │                    │  (neighbours)    │
      └───────────┘                    └──────────────────┘
       user-provided                    FIXS-provided
```

| edge | server | client | payload per tick |
|---|---|---|---|
| Controller ↔ TL | TL | Controller | TL→: `(sₖ,xₖ)` · →TL: `uₖ:ₙ` |
| WM ↔ TL | TL | WM | TL→: `(sₖ,xₖ,uₖ:ₙ)` · →TL: `sₖ₊₁` |

**Transport:** each edge is a `Channel` behind an `open/send/recv/close` seam
(today `SocketHelper` TCP). The topology is transport-independent; a later swap
to **shared memory** (or a dSPACE/ring-buffer handler) changes only that edge's
handler, not the graph or the tick logic. Start TCP throughout.

---

## 6. Message types

- **Controller → TL → WM (ego plan `uₖ:ₙ`):**
  - single-step command (accel / target-speed / next pose) fits existing
    `VehData` fields (`speedDesired`, `accelerationDesired`, `heading`, …);
  - a multi-waypoint trajectory needs a small **new "ego plan" message**
    (list of `(x, y, heading, t)`). *Start single-step; add the trajectory
    message when a planner actually emits one.* The WM backend fills any
    remaining horizon by const-velocity extrapolation (`make_ego_future`), so a
    short/absent plan degrades gracefully.
- **WM → TL → SUMO (`sₖ₊₁`):** ordinary `VehData` neighbour states. No schema
  change — `length/width/height` + pose/heading/speed already present.

---

## 7. WM backend contract and backends

Not an `abc.ABC` — a backend is any object with `reset` and `step`
(`typing.Protocol`, structural). A registry maps `constant_velocity | alpasim |
waymax` → class with **lazy imports**, so importing it never pulls torch/jax.

```python
def reset(self, cfg: dict, vector_map=None) -> None: ...
def step(self, bev: BevAgentSet) -> dict[str, AgentPose]: ...   # NON-EGO ids only
```

### constant_velocity — CI baseline, no heavy deps
Advances each neighbour by finite-differencing the last two history frames
(convention-free); identity until a prior frame exists. Drives the full plumbing
with neither torch nor jax — what CI runs. A real cheap baseline, not a
placeholder.

### alpasim — CATK/SMART (PyTorch)
Assembles the CATK `env_data` dict (template:
`WM_AlpaSim/.../tests/test_catk_integration.py::_make_synthetic_env_data`):
neighbour history `(N,T≥16,·)`, ego appended with its **plan** as the pinned
future (`freeze_agent_mask`), and a polyline `map`. Call
`CATK.create_model_input(env_data, filter_map_by_ego=True, filter_distance_th=H)`
then `CATK.inference()`; read `agent_future_{xyz,heading,velocity}[:, 0]`
(`model_predict_step_num = 5`, take sub-step 0). Global frame; velocity is
finite-differenced internally, so the ego plan is **pose waypoints only**.

### waymax — Waymax (JAX)
`SimulatorState` with `Trajectory{x,y,z,vel_x,vel_y,yaw,valid,l,w,h}`
`(…,num_objects,num_timesteps)`; `ObjectMetadata.is_controlled = neighbours
only`, ego overwritten from the plan. `StateDynamics.forward(...)` → next
`Trajectory`; read neighbour `x,y,yaw`, `speed = hypot(vel_x,vel_y)`. Global
frame. Out-of-the-box neighbour reactivity is IDM; a learned policy can be
plugged in later.

---

## 8. Backend I/O specifics

- **Stateful history (translator-owned).** CATK needs a rolling **≥16-frame
  (1.6 s @10 Hz)** window per agent. SUMO delivers one frame/step, so the
  translator owns a `RollingHistory` ring and fills it. Not VirEnvCore — just a
  deque. **Warm-up:** the first ~1.6 s the translator passes neighbours through
  (or const-vel) until the ring is full; harmless at 10 Hz.
- **Resolution — no interpolation.** CATK's *output* is already 0.1 s (10 Hz);
  its 0.5 s is only the token granularity (one token = 5 sub-steps). Take
  **sub-step 0**. Output rate == SUMO rate, so **no VirEnv-style sub-step
  interpolation / pose anchoring**. The 0.5 s only offers a call-cadence choice:
  reactive (inference every 0.1 s, default) vs chunked (every 0.5 s, replay 0–4;
  5× cheaper, neighbours lag ego ≤0.5 s).
- **Ego plan → conditioning.** The controller's plan is the CATK ego future /
  the Waymax ego overwrite. If the controller sends fewer waypoints than the
  horizon, `make_ego_future` const-velocity-extrapolates the rest (in the
  trivial-controller case, from `xₖ` alone).

---

## 9. Config — `WorldModelSetup` (mirrors `CarlaSetup`)

```yaml
WorldModelSetup:
    Enable: true
    Backend: alpasim          # constant_velocity | alpasim | waymax
    EgoId: 'ego'
    BevHalfExtent: 50.0       # square window half-side (m) → crop + CATK filter_distance_th
    StepLength: 0.1           # must match SUMO --step-length
    HistoryFrames: 16         # rolling buffer depth (CATK needs ≥16)
    EgoFutureHorizon: 5       # ego plan / conditioning horizon
    NetFile: './net.net.xml'  # source for the local vector map (parsed once)
    ModelDir: '.../catk_v120' # backend-specific (ckpt/config/tokens)
    Device: cuda
    ip: ['127.0.0.1']
    port: [2445]
```
Subscription reuses `type: ego, radius: [BevHalfExtent·√2]` (circumscribe the
square, crop to box in Python). No new C++ subscription shape.

---

## 10. Local vector map

Road network is static, so parse the SUMO `.net.xml` **once at `reset()`** into
a generic `VectorMap` (lane-centre + boundary polylines); **crop per step** to
the moving bubble. Never parse online. **AlpaSim:** `env_data["map"][layer] =
{"polylines": (P,V,3), "label": [type_id]}` (bypass trajdata; with
`disable_sub_plyline_type=True`, geometry + a road-boundary flag suffice).
**Waymax:** subsampled `RoadgraphPoints`. Prior art: the #172 signal demo parses
`.net.xml` geometry.

---

## 11. Dependency hygiene

Root `environment.yml` stays lightweight and **must not** require torch/jax.
Heavy deps live in `tests/WorldModels/environment-worldmodel.yml`, installed
**on demand** into `realsim` only when a real backend runs
(`conda env update -n realsim -f …`). One file: torch (cu121, self-contained
CUDA) + `jax[cuda12]` (bundled) coexist; per-backend files are the fallback if a
machine hits a CUDA/cuDNN clash. `torch_cluster` must match torch's
version+CUDA. `constant_velocity` imports neither.

---

## 12. Coordinate frames

SUMO delivers heading in **navigational degrees** (0=N, CW); both WMs use **math
radians** (0=+X, CCW). Converted **once** in `bev.py`; write-back converts back
to SUMO degrees for `moveToXY`. Positions are global metres throughout (the
ego-centric square is only a crop).

---

## 13. Tests

- **CI-safe unit tests** (`tests/WorldModels/test_wm_translator.py`) — BEV crop,
  rolling history, `constant_velocity`, heading conversion, with fabricated
  `VehData`. No sockets/SUMO/torch/jax; runs in plain `realsim`. **(12 passing.)**
  `python -m pytest tests/WorldModels/test_wm_translator.py -v`
- **End-to-end smoke** — TL (auto-launches SUMO, multi-vehicle loop) +
  `constant_speed` controller (`tests/controllers/`) + `constant_velocity` WM;
  exit 0 = full unified loop ran (controller → WM → write-back).
- **Gated real-backend tests** — `@pytest.mark.skipif` unless the `worldmodel`
  env is installed; run `alpasim`/`waymax` for real.

---

## 14. Directory layout

```
tests/WorldModels/                 # WM orchestrator (FIXS-provided)
  bev.py                           square crop, heading conv, RollingHistory, make_ego_future
  wm_translator.py                 the loop (WM client): recv → crop → history → backend.step → send sₖ₊₁
  backends/
    base.py                        WorldModelBackend Protocol (no ABC)
    constant_velocity.py           CI baseline (no torch/jax)
    __init__.py                    registry (lazy imports)
    alpasim.py, waymax.py          [to scaffold]
  vector_map.py                    .net.xml → polylines (once)          [to scaffold]
  environment-worldmodel.yml       heavy deps, on demand
  test_wm_translator.py            CI-safe unit tests
  Stub/ (→ rename Demo/)           end-to-end smoke assets
tests/controllers/                 # controller test doubles (out-of-scope stand-ins)
  constant_speed_controller.py     minimal ego client: (sₖ,xₖ) → uₖ     [to scaffold]
CommonLib/
  ConfigHelper.{h,cpp,py}          + WorldModelSetup_t
  TrafficHelper.cpp                generalized external-pose write-back
TrafficLayer/.../mainTrafficLayer.cpp   ordered one-loop client servicing
```

---

## 15. Status / roadmap

- **Done (skeleton, CI green):** `bev.py`, `wm_translator.py`,
  `backends/{base,constant_velocity}`, registry, 12 unit tests,
  `environment-worldmodel.yml`, end-to-end assets. Waymax cloned to
  `C:\src_git\WM_Waymax` for reference.
- **Next (this issue):** rename `stub → constant_velocity`; add
  `tests/controllers/constant_speed_controller.py`; `vector_map.py`;
  `backends/{alpasim,waymax}.py`; `WorldModelSetup_t` wiring; the TL
  ordered-servicing + generalized write-back.
- **Later:** the multi-waypoint ego-plan message (Topology "trajectory");
  shared-memory `Channel` handlers; a learned Waymax neighbour policy.
```
