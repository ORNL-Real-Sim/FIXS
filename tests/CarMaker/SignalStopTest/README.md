# SignalStopTest — CarMaker traffic-light stop verification (FIXS #172)

Minimal CarMaker scenario that proves **how to make the ego respect a traffic light**,
and answers two questions raised while building the #172 VISSIM↔CarMaker signal demo:

- **Q3** — Can the signal **head** be placed *past* the intersection (far side, like real
  US far-side signals / Carla auto-placement) while the ego still stops at the **stop line**?
- **Q1** — If one lane carries **multiple heads with different states** (e.g. a red and a
  green), what does CarMaker do?

The road is a plain straight 300 m / 1-lane link so the behaviour is unambiguous:

```
ego start (s=5) ───drive──▶  STOP LINE (DrvStop, s=100)   ····   HEAD on pole (s=130, far side)
```

## One-click run

```bat
run_signal_stop_test.bat            :: build Q3, open CarMaker GUI — press Start and watch
run_signal_stop_test.bat q1         :: build Q1, open CarMaker GUI
run_signal_stop_test.bat verify     :: build Q3, run headless, print PASS/FAIL + speed profile
run_signal_stop_test.bat verify q1  :: build Q1 headless
```

(If Python isn't on `PATH`, set `PYTHON=<...>\python.exe` first. Edit `CM_EXE` in the
`.bat` if your CarMaker isn't at `C:\IPG\carmaker\win64-13.1.3`.)

## What the verification shows

Q3 (`verify`): the ego brakes from cruising speed and **comes to rest at s≈99 m — the stop
line — while the head sits 30 m downstream at s=130 m**. The far-side head does *not* pull the
stop point past the intersection. PASS.

Q1 (`verify q1`): two heads on the lane, `TL_main` **red** + `TL_second` **green**, with a
`DrvStop` for *each*. The ego **does not stop** — it drives straight through. **The
green-referenced stop marker clears the stop; a conflicting red on the same lane does not
force a stop (green wins).** Practical rule for the demo: a lane's `DrvStop` must reference
**only the controller for the ego's own movement** (the head it actually obeys). If you
naively emit one `DrvStop` per head, a permissive (green) movement-head will override a red
one and the ego will run the light.

## Findings (why it works the way it does)

Established empirically (headless ERG: `Vhcl.sRoad`, `Car.v`, `Brake.Hyd.Sys.pMC`,
`TrfLight.<name>.State`) and against `road.h` + the IPGRoad InfoFile doc:

1. **A `DrvStop` marker is REQUIRED.** A traffic-light *head* alone does **not** brake
   IPGDriver — the existing `SimpleTrafficLight_Cosim` demo renders all 44 lights and drives straight
   through a full red. `road.h` flags head-based stop-line detection as *deprecated*; the
   active path is `DrvStop` markers evaluated along the **Route**. osc2cm imports heads but
   **not** stop markers, so they must be added.

2. **Far-side head + near `DrvStop` ⇒ stop at the marker, not the head.** The marker carries
   the stop position (`s`), independent of where the head is mounted. (Q3 = yes.)

3. **Use `DriverTemplate.FName = Car_Normal`, NOT `Car_Normal_osc`.** This is the real reason
   the demo never braked — *not* the vehicle and *not* Driver Knowledge. CarMaker **reloads the
   named `Vehicle.DriverTemplate.FName` at runtime and it overrides the inline `Driver.*` block.**
   osc2cm writes `Car_Normal_osc` — a trajectory-**replay** tune (`Acc.axMin=-20`, `ayMax=20`,
   `Long.dtAccBrake=0`, `Course.CornerCutCoef=0.01`, `Vel.CruisingSpeed=50`) built to follow a
   recorded path exactly, which does **not** execute a stop even with `Consider.TrfLight=1`.
   `Car_Normal` is the standard autonomous tune (`axMin=-4`, `dtAccBrake=0.5`) and stops at the
   red. Verified single-variable: same vehicle (`Demo_McLaren_F1`), same road/route/markers,
   flip only this one line → fail→stop. So **any vehicle works (incl. the McLaren)** — just use
   `Car_Normal`. (My earlier "it's the vehicle" / "needs Knowledge" guesses were wrong: each
   prior test changed the template *together with* the vehicle.)

### CarMaker traffic-light cheat-sheet (from `road.h` / IPGRoad doc)

- Controller: `Control.TrfLight.<i> = <objId> <name> "<startCond>" <init> t0 t1 t2 t3 t4`
  — `init` = initial **phase**: `0`=off, `1`=green, `2`=yellow, **`3`=red**, `4`=red+yellow;
  `t0..t4` = phase durations. State channel `TrfLight.<name>.State` uses the same codes.
  A controller with only **one** non-zero timer runs in **manual mode** (state set
  externally — this is the FIXS/VISSIM-driven case); ≥2 non-zero ⇒ automatic cycling.
- Head (on a Mount): `RL.<rl>.Mount.<m>.<p> = 1 <ctrlObjId> vOff hOff dOff rotV rotH rotD latR facing type`
  — `facing=1` = valid for route-direction traffic (the ego); `type` per IPGRoad Table 2.10
  (0=RYG round). The Mount's `dependsOnObjId` is the **RL id**.
- Stop marker: `RL.<rl>.Marker.<m>.Type = DrvStop` /
  `.Param = s lonR val refObjId 2 0` (type `2` = `RDST_TrfLight`). The marker's
  `dependsOnObjId` must be a **LanePath/Link/Path id on the ego's Route** (not the RL id).
  `lonR`: `0` = `s` from element start, `1` = from element end.
- Driver: `Driver.Consider.TrfLight = 1` and `Consider.StopMarker = 1`.

## Files

| File | Role |
|---|---|
| `signal_stop_test.xodr` / `.xosc` | minimal straight road + ego, fed to `osc2cm` to make the base rd5 |
| `build_signal_stop_test.py` | injects head + DrvStop + controller(s) + Route into the base rd5, writes the TestRun (clones `driver_template.cmtestrun`). Idempotent — always starts from `signal_stop_test_base.rd5`. `q3` = far red head; `q1` = red+green heads, one DrvStop each |
| `driver_template.cmtestrun` | IPGDriver template mirroring IPG's `Man_AutonomousJunctions` — `Demo_McLaren_F1` + `DriverTemplate.FName = Car_Normal` (the autonomous tune that brakes for stops; NOT osc2cm's `Car_Normal_osc`) |
| `parse_erg.py` | reads the CarMaker ERG, prints the speed / position / brake / light-state profile and a PASS/FAIL verdict |
| `run_signal_stop_test.bat` | one-click build + GUI (or headless `verify`) |

Generated into the CM project (`ProprietaryFiles/CM13_proj`, not committed here):
`Data/Road/signal_stop_test*.rd5`, `Data/TestRun/SignalStopTest*`.
