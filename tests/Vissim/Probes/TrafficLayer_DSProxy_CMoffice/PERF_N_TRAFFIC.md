# CarMaker traffic-object cost in the #168 DSProxy co-sim (N_TRAFFIC perf)

## Question
The CarMaker ↔ VISSIM ↔ FIXS co-sim slows as more CarMaker traffic objects (`Traffic.N`,
the `RS_Cxxx` slots) are added. Is the cost in CarMaker's own traffic engine or in the
FIXS `VirtualEnvironment.lib`? And can it be cut? Long-standing: see **#52** and **#88**.

## Method
RS_DEBUG instrumentation, **VISSIM held fixed** (active vehicle count constant), bucketed
to 0.1 s. `rs_runstep_cm.csv` splits the `.lib`'s own `runStep()` time from the full
CarMaker step. `sweep_traffic.py` sweeps N_TRAFFIC; `sweep_updrate.py` sweeps UpdRate
**with a correctness check** (the ego's travelled distance — see Result 2). Steady state
= simTime > 60 s.

> Granularity: `runStep()` runs once per 1 ms CarMaker step (the recv/send/RTF path is
> gated to the 0.1 s tick), so `rs_runstep_cm.csv` numbers are **per 1 ms step**.

## Result 1 — it's CarMaker core, not the `.lib` (N_TRAFFIC sweep, UpdRate=1000)

Active vehicles were ~13 every run; only the slot count varied.

| N_TRAFFIC | active | **`.lib` µs** | full µs | **CMcore µs** | `.lib` % |
|---:|---:|---:|---:|---:|---:|
| 20  | 13 | **3.9** | 150  | 147  | 2.8% |
| 50  | 13 | **3.8** | 400  | 396  | 1.0% |
| 100 | 13 | **3.9** | 748  | 744  | 0.5% |
| 150 | 13 | **3.7** | 1060 | 1057 | 0.3% |

The `.lib` is **flat at ~3.8 µs** regardless of N_TRAFFIC; CarMaker-core scales linearly
at **~7.4 µs/object/step** — per-object road-network + sensor-envelope geometry, incurred
**even for parked slots**. The `.lib` only loops over the active mapped vehicles, so
optimizing it cannot help: the O(N_TRAFFIC) cost is CarMaker's own traffic engine. (CM
Reference Manual §22.9: even FreeMotion objects recompute road-network state + envelope +
bounding box every update.) The TL/DSProxy side stays small and near-flat.

## Result 2 — UpdRate is NOT a usable lever (it FREEZES the traffic)

> An earlier version of this study claimed lowering UpdRate gave a free ~15×. **That was
> WRONG**: a low UpdRate FREEZES the FreeMotion traffic, and the "cheap" cost was just the
> cost of static cars. The headless harness only checked `vehicles > 0`, never that they
> *move*.

Re-run with a correctness check (the ego's distance from the CarMaker `SIM_END` line — a
moving co-sim lets the ego follow + travel; a frozen one leaves it stalled near 0 m):

| UpdRate (refreshRate=1000) | ego distance | co-sim | cost µs/obj |
|---:|---:|:---:|---:|
| 1000 | **1606 m** | ✅ moving | 7.98 (real) |
| 500  | 3 m | ❌ FROZEN | 0.91 (meaningless) |
| 250  | 3 m | ❌ FROZEN | 0.70 |
| 200  | 3 m | ❌ FROZEN | 0.66 |

**Restriction: `UpdRate` must be ≥ the `.lib`'s position-refresh rate
(`1/TrafficRefreshRate`, default 1000 Hz).** The `.lib` teleports traffic by writing
`TrfObj->t_0` **directly** every refresh (1 kHz, `VirEnvHelper.cpp` ~L937). Below that
rate CarMaker's FreeMotion sub-samples and re-derives the object from its own
velocity-less state, so the car reverts to static and the ego then stalls behind it.
Lowering **both** refreshRate and UpdRate together (200/200, matched) **also froze**,
confirmed by a clean back-to-back A/B on a properly-licensed CarMaker:

| run (refresh/UpdRate) | ego dist | peak veh | `drawnX` vs `targetX` | |
|---|---:|---:|---|:---:|
| baseline 1000/1000 | **1606 m** | 17 | 206 m vs 206 m (tracks) | ✅ moves |
| matched 200/200    | **3 m**    | 1  | (no bg traffic to log)  | ❌ frozen |

The peak collapse is itself a *symptom* of the freeze, not a VISSIM problem: the stalled ego
blocks VISSIM's inflow, so few vehicles ever enter (which is why earlier degraded runs showed
`peak=0/1` — a frozen co-sim, not a license-seat shortage). **Matching the rates is not the
fix — there is no easy UpdRate lever via YAML.**

**Tried — `v_0` does NOT work either.** Publishing `TrfObj->v_0` (the interpolation slope
already computed for `t_0`) was implemented, the `.lib` + headless exe rebuilt, and tested at
UpdRate=200: the traffic **still froze** (ego 3 m, peak 1). FreeMotion objects are pure
teleport — they do not integrate `v_0` — so feeding them velocity is inert, and the freeze is
an *absolute* low-UpdRate effect (1000/1000 moves; both 200/200 matched and 200 + `v_0` freeze),
not a rate-mismatch or a missing-velocity one. **Definitive: `UpdRate` is not a usable lever
for the teleport approach** — CarMaker's FreeMotion needs a high `UpdRate` to render the
external position stream. A genuinely cheaper traffic would require abandoning exact teleport
for CarMaker-integrated motion (an accuracy tradeoff and a much larger `.lib` redesign), not a
config knob.

## Levers (corrected)
1. **Fewer slots — the ONLY confirmed-safe lever.** Size `N_TRAFFIC` to the real VISSIM
   peak (~40–50), not a buffer; each slot costs ~7.4 µs/step in CarMaker's core whether or
   not it's mapped to a live vehicle.
2. **UpdRate: do NOT lower it** (freezes the traffic — Result 2). Keep it at 1000.
3. **Lighter templates / no AutoDriver:** untested; the measured per-step cost is the
   road-network + envelope geometry, which a lighter template may not change. Worth a
   *correctness-checked* experiment, not an assumption.
4. **GUI render (IPGMovie):** a separate per-object cost on top, excluded by this headless
   study — likely the user-visible extra GUI slowdown, stacked on the core cost here.

Reproduce: `python sweep_traffic.py` (N_TRAFFIC, `.lib`-vs-core) and `python
sweep_updrate.py` (UpdRate, **with the ego-distance correctness check — the cost column is
only valid where the ego moved**). Needs the RS_DEBUG builds + a short SimulationEndTime.
