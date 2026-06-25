# TrafficLayer_DSProxy_CMoffice_Signal  (FIXS #172)

The SimpleTrafficLight VISSIM↔CarMaker signal demo: a corridor with 3 signalized
intersections and an ego that loops it. The signals are VISSIM-driven; the CarMaker ego
now **physically stops at the red lights**, with the signal heads mounted **far-side**
(across the junction, facing the stop line).

> This dir is the single, consolidated probe (the former `…_SignalStop` variant has been
> folded back in). The base scene comes from SUMO→xodr→osc2cm; the signal-stop layer is
> added on top by `add_signal_stops.py`.

## Build + run (one click)

```bat
run_cm_scene_only.bat            :: build signal-stop road/TestRun, open CarMaker + IPGMovie
run_cm_scene_only.bat verify     :: build + run headless + print the stop-at-red report
```

Press START in CarMaker: standalone, the 44 controllers cycle on their own (green/red) and
the looping ego stops at the reds. (Full VISSIM co-sim drives the same controllers live; the
stopping mechanism is identical.) Needs CarMaker 13.1.3 + any Python 3 on `PATH`.

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
which would wipe the Route) into `simple_traffic_light_signalstop.rd5`:

1. **One straight `DrvStop` per crossing** (6 = 3 intersections × both directions), on the
   ego's lane-path, anchored to the lane-path's downstream end (`lonR=1, s=12`) so the ego
   stops ~12 m before the junction — i.e. ~12 m **before the signal head** (which osc2cm mounts
   at the junction, at the approach edge's downstream end). It references the **straight-movement
   controller** on that approach (the ego goes straight). One marker per route→movement, never
   one per head — a green movement-head would otherwise cancel a red one (the Q1 finding).
2. Spreads each approach mount's overlapping heads laterally so straight/left/right are
   visually distinct (cosmetic; heads stay on the approach edge).

and writes the TestRun `SimpleTL_SignalStop` = the working `SimpleTL_VISSIM` run with the new
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
| `add_signal_stops.py` | builds `simple_traffic_light_signalstop.rd5` (straight DrvStops + far-side heads + head spread) + the `SimpleTL_SignalStop` TestRun (`Car_Normal`) |
| `verify_signalstop.py` | parses the ERG, reports each stop episode + whether the referenced controller was red |
| `parse_erg.py` | minimal CarMaker ERG reader |
| `run_cm_scene_only.bat` | one-click build + GUI + IPGMovie (or headless `verify`) |

Base probe files (`gen_*.py`, `build_*.py`, `import_*.py`, SUMO/VISSIM nets, `signal_plan.json`,
`WORKING_ROUTE_reference.txt`) are unchanged.

## Still open (Q2)
- `build_signal_table.py` — emit the VISSIM↔CM `RSsignalTable.csv` (format confirmed:
  `signalControllerName, signalGroupId, signalHeadId, CmTrafficLightIndex, CmControllerId`).
- Rename signals to the FIXS controller_group convention.
- Automated route re-derivation after an osc2cm re-import (currently the route is committed).
