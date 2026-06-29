# TrafficLayer_DSProxy_CMoffice_Signal  (FIXS #172)

The SimpleTrafficLight VISSIM↔CarMaker signal demo: a corridor with 3 signalized
intersections and an ego that loops it. The signals are VISSIM-driven; the CarMaker ego
now **physically stops at the red lights**, with the signal heads mounted **far-side**
(across the junction, facing the stop line).

> This dir is the single, consolidated probe (the former `…_SignalStop` variant has been
> folded back in). The base scene comes from SUMO→xodr→osc2cm; the signal-stop layer is
> added on top by `add_signal_stops.py`.

## Build + run (one click)

The VISSIM↔CarMaker co-simulation demo (full architecture in [DEMO.md](DEMO.md)):

```bat
run_signal_demo_gui.bat        :: build + launch TrafficLayer + CarMaker GUI + IPGMovie
run_signal_demo_headless.bat   :: build + run headless + assert PASS/FAIL (CI-style)
```

Both build the assets (`add_signal_stops.py` → `build_signal_table.py` →
`build_cosim_testrun.py`), then run the VISSIM↔CM co-sim on `SimpleTrafficLight_Cosim`: the
ego loops the corridor and stops at the **VISSIM-driven** reds, with VISSIM background traffic
rendered as `RS_C` cars. Needs CarMaker 13.1.3 + VISSIM 2022 (CodeMeter) + Python 3 on `PATH`.

`import_road.bat` (re)generates the base road via osc2cm — only needed if the committed
`simple_traffic_light.rd5` is missing (osc2cm wipes the hand-made ego Route, so re-inject it
from `WORKING_ROUTE_reference.txt` afterwards).

## What makes the ego respect the signals

The osc2cm import renders the signal heads but the ego drives through every red, because
(a) osc2cm imports heads but no **stop markers**, and IPGDriver only brakes for a light via a
`DrvStop` on the route, and (b) the osc2cm TestRun uses `DriverTemplate.FName = Car_Normal_osc`,
a trajectory-**replay** tune that doesn't execute stops (CarMaker reloads the named template at
runtime, overriding the inline `Driver` block). Full investigation:
`../../../CarMaker/SignalStopTest/README.md`.

`add_signal_stops.py` post-processes the committed base road (it does **not** re-run osc2cm,
which would wipe the Route) into `simple_traffic_light.rd5`:

1. **One straight `DrvStop` per crossing** (6 = 3 intersections × both directions), on the
   ego's lane-path, anchored to the lane-path's downstream end (`lonR=1, s=12`) so the ego
   stops ~12 m before the junction — i.e. ~12 m **before the signal head** (which osc2cm mounts
   at the junction, at the approach edge's downstream end). It references the **straight-movement
   controller** on that approach (the ego goes straight). One marker per route→movement, never
   one per head — a green movement-head would otherwise cancel a red one (the Q1 finding).
2. Spreads each approach mount's overlapping heads laterally so straight/left/right are
   visually distinct (cosmetic; heads stay on the approach edge).

and writes the TestRun `SimpleTrafficLight_ego` = the working `SimpleTrafficLight_import` run with the new
road and `DriverTemplate.FName = Car_Normal` (McLaren + Route unchanged).

### Head placement & far-side (verified on a minimal junction scene)

The complete stop rule (verified on `../../../CarMaker/JunctionStopTest/`, a clean
2-edge + 1-junction scene with the controller forced red, varying only the head location):

- a **`DrvStop` marker AND a signal head referencing the same controller** are both required —
  a `DrvStop` with **no head** runs the red (the doc's "controller + stop marker" is incomplete);
- the head's **location is free**: head on the **approach** → stops; head on the **departure
  edge across the junction** (true US far-side, approach has none) → **still stops** at the
  approach `DrvStop`.

So **far-side heads are achievable and the ego still stops.** This dir currently keeps the heads
at the approach edge's downstream end (at the junction, beyond the stop line). To render true
far-side, relocate each ego-crossing head to the **route's next edge** (`route[i+2]`) — using
the route, not geometric collinear matching (an earlier geometric version mis-placed heads on
the undivided 2-way corridor and dropped some, which is what made the ego run reds — an
implementation bug, not a CM limit).

## Verified (headless)

```
PASS: 110/110 stops occurred at a red referenced controller (ego respects the signals).
  t= 38.7-> 62.3s  sRoad= 508.9  brake on  RED   (repeats every loop)
```

## Signal-stop files (added on top of the base probe)

| File | Role |
|---|---|
| `add_signal_stops.py` | builds `simple_traffic_light.rd5` (straight DrvStops + far-side heads + head spread) + the `SimpleTrafficLight_ego` TestRun (`Car_Normal`) |
| `build_signal_table.py` | renames the CM controllers to the VISSIM convention + emits `simple_traffic_light_RSsignalTable.csv` (runs after `add_signal_stops.py`) |
| `verify_signalstop.py` | parses the ERG, reports each stop episode + whether the referenced controller was red |
| `parse_erg.py` | minimal CarMaker ERG reader |
| `run_signal_demo_gui.bat` | one-click co-sim: TrafficLayer + CarMaker GUI + IPGMovie |
| `run_signal_demo_headless.bat` | one-click co-sim headless + PASS/FAIL assertion |

Base probe files (`gen_*.py`, `build_*.py`, `import_*.py`, SUMO/VISSIM nets, `signal_plan.json`,
`WORKING_ROUTE_reference.txt`) are unchanged.

## Signal naming + table (Q2)

`build_signal_table.py` wires each CM traffic-light object to the VISSIM signal
group that drives it, and renames the controllers so the rd5 is self-documenting.

The runtime contract (read from the code, not guessed): `DSProxyMode::toTlsData()`
sends **one message per (controller, signal group)** with `name = "<SCno>_<sg>"`
and a **single-char** state; `VirEnvHelper::runStep` then sets
`TrfLight.Objs[CmTrafficLightIndex].State = tlsChar2CmState(state.at(SignalHeadId))`.
Because each name carries one char, `SignalHeadId` is always `0`.

`RSsignalTable.csv` columns → values:

| Column | Value | Meaning |
|---|---|---|
| `SignalControllerId` | `"<SCno>_<sg>"` e.g. `2_1` | the runtime match key (`tlsId`); SC numbering `int_west=1, int_center=2, int_east=3` |
| `SignalGroupId` | `-1` | unused by the reader |
| `SignalHeadId` | `0` | single-char state per name |
| `CmTrafficLightIndex` | `0..43` | the `Control.TrfLight.<i>` array index |
| `CmControllerId` | `"<tls_id>_<head_id>"` e.g. `int_center_10` | SUMO-canonical head id (= the `odrSignalId` tag, `<intersection>_<linkIndex>`); also written as the `Control.TrfLight` name so the rd5 is portable across SUMO/VISSIM |

The CM-head → VISSIM-group join uses the `odrSignalId` tag osc2cm leaves on each
head part (`<intersection>_<linkIndex>`), resolved to a group via `signal_plan.json`.
All 44 heads map across the 7 groups (`1_1..3_3`); several heads share one group
(a SUMO signal group controls several movements), so the sync sets them together.

## Still open
- Automated route re-derivation after an osc2cm re-import (currently the route is committed).
- Wire `SignalTableFilename` into the CM-side co-sim config for the live VISSIM↔CM run.
