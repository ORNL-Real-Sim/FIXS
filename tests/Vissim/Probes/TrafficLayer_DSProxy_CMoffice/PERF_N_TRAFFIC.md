# CarMaker traffic-object cost in the #168 DSProxy co-sim (N_TRAFFIC perf)

## Question
The CarMaker ↔ VISSIM ↔ FIXS co-sim slows down as more CarMaker traffic objects
(`Traffic.N`, the `RS_Cxxx` slots) are added. **Is that cost in CarMaker's own
traffic engine, or in the FIXS `VirtualEnvironment.lib`? And can we cut it?**
Long-standing: see **#52** ("CM11 traffic obj in CM13 → slower than real time")
and **#88** ("FIXS slower than real time as traffic scales up").

## Method
Hold VISSIM **completely unchanged** (same network, same ~50 km/h "Default"
inflow → the *active* mapped-vehicle count is constant) and sweep only the
CarMaker `N_TRAFFIC` slot count: **20 / 50 / 100 / 150**. RS_DEBUG instrumentation,
bucketed to 0.1 s so the CSV writes don't perturb the loop:
- `rs_runstep_cm.csv` — the `.lib`'s own `runStep()` entry→exit time vs the full
  CarMaker step (entry→entry). **This is the key split: `.lib` vs CarMaker-core.**
- `rs_timing_cm.csv` — full per-step wall clock + real-time factor (RTF).
- `rs_timing_tl.csv` — the TrafficLayer / DSProxy side.

Reproduce: `python sweep_traffic.py` (needs the RS_DEBUG builds — VE.lib +
TrafficLayer + headless exe with `-p:RS_DEBUG=1` — and a short
`SimulationEndTime`). Steady-state window = simTime > 60 s, 600 samples/run.

> Granularity note: `runStep()` runs once per **1 ms CarMaker step**; the recv/
> send/RTF path is gated to the 0.1 s FIXS tick. So `rs_runstep_cm.csv` numbers
> are **per 1 ms step**, and `rs_timing_cm.csv` wall is **per 0.1 s tick** (×100).

## Result — active vehicles were **13 in every run** (VISSIM fixed); only slots varied

### Top line (CarMaker per-step wall clock, per 0.1 s tick)
| N_TRAFFIC | active veh | RTF  | wall µs / 0.1 s tick |
|---:|---:|---:|---:|
| 20  | 13 | 7.14 | 15,094  |
| 50  | 13 | 2.52 | 40,034  |
| 100 | 13 | 1.34 | 74,874  |
| 150 | 13 | 0.94 | 106,070 |

Linear in N_TRAFFIC; RTF drops **below 1.0 (slower than real-time) by ~150**.

### The split: FIXS `.lib` vs CarMaker core (per 1 ms CarMaker step)
| N_TRAFFIC | active | **`.lib` µs** | full µs | **CMcore µs** | `.lib` % |
|---:|---:|---:|---:|---:|---:|
| 20  | 13 | **3.9** | 150  | 147  | 2.8% |
| 50  | 13 | **3.8** | 400  | 396  | 1.0% |
| 100 | 13 | **3.9** | 748  | 744  | 0.5% |
| 150 | 13 | **3.7** | 1060 | 1057 | 0.3% |

### TrafficLayer / DSProxy side
| N_TRAFFIC | tick µs | getTraffic µs |
|---:|---:|---:|
| 20  | 15,189  | 4,534 |
| 50  | 39,900  | 5,596 |
| 100 | 74,749  | 6,375 |
| 150 | 106,645 | 9,695 |

(TL just mirrors CarMaker — it waits on it in lockstep. `getTraffic`, the actual
DSProxy/VISSIM round-trip, is small and near-flat.)

## Conclusion — triple-checked

**The O(N_TRAFFIC) cost is CarMaker's own traffic engine, NOT the FIXS `.lib`.**
Six independent angles all agree:

1. **`.lib` time is FLAT** at ~3.8 µs across 20/50/100/150 — it does not scale.
   (`runStep()` only loops over `TrafficSimulatorId2CarMakerId` = the active
   mapped vehicles, which were constant at 13.)
2. **CarMaker-core scales linearly**: 147 → 396 → 744 → 1057 µs, i.e.
   **~7.4 µs per traffic object per 1 ms step** (constant slope, ~zero intercept).
3. **The `.lib`'s share shrinks** 2.8% → 1.0% → 0.5% → 0.3% — vanishing fraction.
4. **Active count was constant (13)** every run, so the scaling is NOT the
   active-vehicle work — it tracks the *slot* count, parked slots included.
5. full step = `.lib` + CarMaker-core, and CarMaker-core ≈ the whole step.
6. TL/DSProxy (`getTraffic` 4.5→9.7 µs) is small + near-flat — not VISSIM either.

So optimizing the `.lib` cannot help (it's ~0.3–3 % of the step). The cost is
CarMaker integrating/checking **every** `Traffic.N` object each step — the
`RS_Cxxx` slots parked at z=−5000 are **not free** (~7.4 µs/object/step).

## Levers
1. **Fewer slots.** Size `N_TRAFFIC` to the real VISSIM peak (~40–50), not a
   buffer. 150 already runs slower than real-time; 50 gives RTF 2.5.
2. **Cheaper slots — the real lever, and what #52 found.** Each `RS_Cxxx` uses a
   full vehicle dynamics template (`IPG_CompanyCar_2018_Blue`) **plus** an
   AutoDriver (`Car_HDM_Normal`). For FIXS-teleported `FreeMotion` objects much
   of that is dead weight CarMaker still spins up. **TODO experiment:** strip the
   AutoDriver / use a lighter template / lower `UpdRate`, re-sweep, and watch the
   ~7.4 µs/object drop. (The ego must still *detect* them → `DetectMask` stays on.)
3. **GUI render (IPGMovie)** adds its own per-object cost on top; this headless
   study excludes it. That is likely the user-visible extra slowdown in the GUI,
   stacked on the core cost measured here.
