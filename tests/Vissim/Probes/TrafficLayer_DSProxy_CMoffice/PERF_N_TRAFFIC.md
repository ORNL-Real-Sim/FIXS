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

## Result 2 — lowering UpdRate freezes FIXS's *teleported* (`FreeMotion`) traffic

> Note: this freeze is **specific to `FreeMotion=1`** (the mode FIXS needs for the `.lib` to
> teleport VISSIM cars). `UpdRate=200` is fine — in fact CarMaker's *default* — for normal
> (`FreeMotion=0`) traffic; see the stock-CarMaker control in Result 3.

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
not a rate-mismatch or a missing-velocity one (UpdRate=200 is well below the threshold). **But
`UpdRate` is NOT a hard wall at 1000 — there is a sharp threshold, and below it the freeze is
not the `.lib` at all. See Result 3.**

## Result 3 — the real threshold (~550) and the real cause (CarMaker, not the `.lib`)

A matched-pair sweep (`sweep_matched.py`, `TrafficRefreshRate = 1/UpdRate`) shows the freeze is
a **sharp cliff**, not a wall at 1000:

| matched UpdRate | ego dist | moves? |
|---|---:|:---:|
| 1000 / 900 / 800 / 600 / 550 | ~1600 m | ✅ |
| 500 / 400 / 300 / 200 | 3 m | ❌ |

(700 runs with moving traffic — `peak=17` — but has a separate end-of-run parse glitch; it is
not a freeze.) So **matched `UpdRate ≥ 600` is a usable lever** — a ~1.67× cheaper traffic step
than 1000, with the co-sim still moving. Lower `Traffic.<i>.UpdRate` **and** `TrafficRefreshRate`
together to the same value; do not go below ~600.

**Where/why it freezes** (RS_DEBUG probes via `diag_freeze.py` with `RS_DEBUG_BUILD=1` —
`rs_cm_pos.csv`, `rs_freeze.csv`, `rs_slots.csv`):

- In the **moving** case the `.lib` teleport is exact: `t_0` read-back tracks the target to a
  **0.024 m** mean gap. The `.lib` is not the problem.
- In the **frozen** case the `.lib` receives **zero** background traffic (`nMapped=0`) — it is
  completely idle. The freeze is **upstream of the `.lib`**.
- Direct slot reads (`Traffic_GetByTrfId(i)->t_0`, mapped or not): at `UpdRate ≥ 600` the slots
  stay correctly parked; at `≤ 500` they end up on the ego's route and block it. The ego stalls
  at 3 m → chokes VISSIM's inflow → no background is ever released → the `.lib` stays idle (which
  is why `rs_cm_pos`/`rs_freeze` are empty at 500).

**Three control experiments rule out the obvious suspects and pin the cause:**
- **Co-sim OFF** (`diag_vanilla.py`, `.lib` inert via `EnableCosimulation=false`) reproduces the
  freeze *identically*: 200 → 3 m frozen, 600 → 1648 m moves. With the `.lib` not even connecting,
  it cannot be the `.lib` runtime.
- **Dropping the slots' `AutoDriver`** (`RS_NO_AUTODRIVER=1`) does **not** unfreeze 200 (still
  3 m). Not the AutoDriver either.
- **Stock CarMaker, normal traffic, UpdRate=200** (`diag_stock.py`): CarMaker's own `Overtaking`
  example (`Traffic.UpdRate=200`, **`FreeMotion=0`**, 8 cars) run by the **stock install exe**
  (zero FIXS) completes fine — ego travels **5940 m**. `UpdRate=200` is CarMaker's *normal
  default* for normal traffic; it is NOT a CarMaker-core limit.
- **Isolation — flip ONLY `FreeMotion` on that stock example** (`diag_stock_fm.py`): `FreeMotion=0`
  → 5940 m (moves); `FreeMotion=1` → **33 m (frozen) at BOTH UpdRate=200 and 1000**. One field,
  same road/AutoDriver/ego, turns a working run into a frozen one — `FreeMotion=1` is the pivot.
  Note it freezes at 1000 too: `FreeMotion=1` does not self-drive (it is "position set
  externally"), so with no source (vanilla) it is static at any UpdRate. The *UpdRate*-dependence
  appears only **with** a source: the FIXS `.lib` drives it, and CarMaker applies that drive at
  `UpdRate` Hz — fine at ≥600, too coarse at ≤500.

- **CarMaker's own convention** (`SimNet/Highway_3EgoVhcls_Traffic_*` examples): where CarMaker
  itself feeds a `FreeMotion=1` object externally (cross-instance ego vehicles via SimNet), every
  one of them is set to **`UpdRate=1000`** — vs `UpdRate=200` for its normal `FreeMotion=0`
  traffic. CarMaker's own authors treat externally-driven FreeMotion as a 1000-Hz thing. FIXS's
  `FreeMotion=1 @ 1000` matches that; `@ 200` is below it. (Confirms it from the *producer* side,
  independent of any FIXS run.)

So the freeze is **specific to `FreeMotion=1`** — the externally-positioned mode the `.lib`
teleport requires. CarMaker writes a FreeMotion object's position only at its `UpdRate`; below
~600 Hz that is too coarse to hold the teleported cars in place, so they drift onto the ego's
route and stall it. **`UpdRate=200` works for normal (CarMaker-driven) traffic but not for FIXS's
teleported (`FreeMotion=1`) traffic** — it is a property of the teleport model, not the `.lib`
runtime and not CarMaker core. FIXS is bound to `UpdRate ≥ 600` unless it abandons exact teleport
for a CarMaker-driven traffic model (route + occasional correction) — a real redesign.

## Result 4 — UpdRate=200 actually WORKS for driven FreeMotion; the FIXS freeze is a BUG

The cleanest possible test (`diag_mover.py` + a minimal `User.c` mover gated on `RS_TEST_MOVER`,
**no `.lib`, no VISSIM**): move one `FreeMotion=1` object at 13.9 m/s by writing its `t_0` every
step, and log CarMaker's applied read-back vs the target.

| UpdRate | read-back moved | target moved | |
|---|---:|---:|:---:|
| 1000 | 417.0 m | 417.0 m | ✅ tracks |
| **200** | **416.3 m** | 417.0 m | ✅ **tracks** |

**A correct external mover drives FreeMotion perfectly at `UpdRate=200`** — identical to 1000. So
`UpdRate=200` is **not** inherently broken for driven FreeMotion; **Results 2–3's "inherent / bound
to ≥600" framing was wrong** — built on the confounded FIXS co-sim (where the ego stalls first).
**The `.lib`'s 200-freeze is a FIXS implementation bug, and UpdRate=200 is reachable** (the ~5×
cheaper traffic). What the `.lib` does differently from the plain mover — and where the bug must
live: it throttles `t_0` writes to `TrafficRefreshRate` (200 Hz when matched) and
interpolates/parks 50 slots, vs the mover's plain every-step direct write. Isolating that
difference is the path to the fix.

## Levers (corrected)
1. **Fewer slots — the ONLY confirmed-safe lever.** Size `N_TRAFFIC` to the real VISSIM
   peak (~40–50), not a buffer; each slot costs ~7.4 µs/step in CarMaker's core whether or
   not it's mapped to a live vehicle.
2. **UpdRate: matched `≥ 600` is safe (~1.67× cheaper), `≤ 500` freezes** (Result 3). Lower
   `Traffic.UpdRate` and `TrafficRefreshRate` together to the same value; do not go below ~600.
   The freeze below that is CarMaker-side (placeholder slots mis-activate), not the `.lib`.
3. **Lighter templates / no AutoDriver:** untested; the measured per-step cost is the
   road-network + envelope geometry, which a lighter template may not change. Worth a
   *correctness-checked* experiment, not an assumption.
4. **GUI render (IPGMovie):** a separate per-object cost on top, excluded by this headless
   study — likely the user-visible extra GUI slowdown, stacked on the core cost here.

Reproduce: `python sweep_traffic.py` (N_TRAFFIC, `.lib`-vs-core); `python sweep_updrate.py`
(UpdRate, refresh fixed); `python sweep_matched.py` (matched-pair threshold — Result 3); and
`RS_DEBUG_BUILD=1 python diag_freeze.py` (the freeze mechanism — `rs_slots.csv` etc.). The
matched/freeze scripts edit `config.yaml` in place and restore it. Needs the RS_DEBUG builds
(`RS_DEBUG_BUILD=1`, env-gated in `build_headless_exe.bat`) + a short SimulationEndTime.
