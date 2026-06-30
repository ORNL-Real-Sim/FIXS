# TrafficLayer_DSProxy_DriverModel_EcoDriving — #173 eco-driving CAV demo

Heuristic GLOSA eco-driving on a CAV penetration rate, on the **same signalized
corridor as Demo 2** (`TrafficLayer_DSProxy_CMoffice_Signal`), now with **DSProxy
and the FIXS DriverModel co-loaded** in one VISSIM (the #158 Stage C co-load).
This is the validation demo for Stage C — precedent: #168 validates Stage B.

- **DSProxy** = the read/inject bus: injects the CarMaker XIL ego into VISSIM and
  reads the whole network back out (to CarMaker for rendering, to the controller
  for thinking).
- **DriverModel** = the write channel: applies the eco-driving controller's
  per-CAV desired speed to the DriverModel-hooked Cars (Wiedemann integrates it).
- **cav_controller.py** = the brain: GLOSA speed advice from each CAV's distance
  to the next signal + that signal's **fixed** timing.

## Topology — four sockets, TrafficLayer is the hub

```
 CarMaker.win64.exe --ego pose-------> :2444  TrafficLayer (DSProxy) :2444 --readback--> CarMaker (RS_C cars)
                                                                     :2445 --signals---> CarMaker (traffic lights)
 cav_controller.py  <--readback------- :1338  TrafficLayer          :1338 <--CAV cmds--- cav_controller.py
 DriverModel.dll (in VISSIM) <--CAV cmds------ :1337  TrafficLayer  (DriverModel connects out to 1337)
```

| who | connects to TL port | carries |
| --- | --- | --- |
| CarMaker `.lib`        | 2444 | ego pose up / whole-network readback down |
| CarMaker `.lib`        | 2445 | VISSIM signal state down |
| `cav_controller.py`    | 1338 | readback down / per-CAV `speedDesired` up |
| `DriverModel.dll`      | 1337 | TL relays CAV cmds to it (the DLL connects out) |

Per tick: ego in → DSProxy advances VISSIM → readback fans out to CarMaker **and**
the controller → controller runs GLOSA → `speedDesired` back to TL → TL relays to
the DriverModel → Wiedemann integrates the CAV speed (takes effect next tick).

## Vehicles

| group | DSProxy | DriverModel | type |
| --- | --- | --- | --- |
| ego (1, CarMaker XIL)     | injected + read back | — | **200** (HGV — off the hooked type) |
| eco CAVs (penetration×)   | read back → rendered | **controlled** | 100 (Car, `extDriver`) |
| plain background          | read back → rendered | — | — |

`setup_eco.py` hooks Car type 100 → `DriverModel_RealSim.dll` + `eco_par.yaml`
(`SubEgoOnly: false`, `EnableRealSim: true`). The ego uses type 200 so the hook
does not grab the XIL vehicle. **Penetration** is applied controller-side: a
stable per-id fraction eco-drives; the rest get free-flow desired speed
(≈ plain Wiedemann). A true demand-composition split (a dedicated CAV type vs a
plain type) is the faithful follow-up; this controller-side knob keeps the demo
one-network.

## Run it (one-click)

```
run_eco_demo_gui.bat          REM eco run     (penetration 1.0)
```
Wait for the TrafficLayer window to print `VISSIM_Connect OK`,
`CAV-controller listener bound on port 1338`, and `DriverModel connected`, then in
the CarMaker GUI load `SimpleTrafficLight_Cosim` and press START. Watch the
controller window for `eco_active` counts.

### Before/after readout

```
run_eco_demo_gui.bat 0.0      REM baseline run (penetration 0.0 -> CAVs free-flow)
run_eco_demo_gui.bat 1.0      REM eco run
python verify_eco.py          REM compares out/summary_baseline.json vs summary_eco.json
```
`verify_eco.py` reports the eco effect on the CAV population: fewer stops, less
idle time, smoother deceleration (a harshness/energy proxy).

## Prerequisites

1. **Demo 2 works** — the eco demo depends on it (shares the corridor, CM scene,
   signal table). The one-click bat builds the shared Demo 2 assets for you.
2. **DriverModel built** — `scripts\dispatch\3_vissim_components.bat`
   (`ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim.dll`).
3. **Relay actuation smoke test passed once** — confirm a DriverModel-hooked CAV
   actually responds to a commanded speed under `SubEgoOnly: false`, via the
   `TrafficLayer_DSProxy_cav_xil` probe, BEFORE trusting eco numbers here. This is
   the one capability the eco layer is built on; see "Status" below.
4. **VISSIM 2022 healthy** — the bat runs the Win11-24H2 COM self-heal preflight
   (see CLAUDE.md soft reset if it still wedges).

## Calibrate signal positions (one-time)

`signals.json` carries **reliable fixed timing** (green windows) from the shared
`signal_plan.json`, but its `pos` values (each signal's stop-line position along
the travel axis, in the DSProxy readback frame) are **estimates**. The controller
free-flows (no eco shaping) for any signal it can't match ahead of a CAV, so:

1. Run the demo once and watch the controller log (CAV positions / `eco_active`).
2. Note where CAVs actually are as they cross each intersection (the readback
   `positionX`), set each signal's `pos` in `signals.json`, set
   `"_calibrated": true`, and re-run.
3. If the corridor travels along Y or in −axis, set `travel_axis` / `direction`.

`verify_eco.py` prints an explicit "GLOSA never active → calibrate" hint when
positions are wrong.

## Files

| file | role |
| --- | --- |
| `eco_driving.py`      | pure GLOSA math (ported from #120, no I/O). Has a self-check. |
| `cav_controller.py`   | readback → GLOSA → per-CAV `speedDesired`; writes `out/summary_*.json` |
| `build_eco_signals.py`| extracts fixed green windows from `signal_plan.json` → `signals.json` |
| `signals.json`        | corridor signal model (timing reliable; positions to calibrate) |
| `eco_par.yaml`        | DriverModel par-file (`SubEgoOnly:false`, port 1337) |
| `config.yaml`         | demo config (relay on, CAV port 1338, ego type 200) |
| `setup_eco.py`        | stage network + hook DriverModel + write runtime config + register CM |
| `run_eco_demo_gui.bat`| one-click launcher (optional arg = penetration) |
| `verify_eco.py`       | before/after comparison of two runs |

## Status / honest caveats (empirical — from headless runs against live VISSIM+CarMaker)

The Stage C co-load was driven end-to-end with `run_eco_demo_headless.bat`. What
that revealed, in order (each a real blocker the harness caught):

1. **Multithreading abort — FIXED.** VISSIM aborts `VISSIM_Connect` with *"external
   driver model DLL does not support multithreading"* when the FIXS DriverModel is
   co-loaded. VISSIM-under-DSProxy runs multi-core regardless of the `.inpx` core
   count, and the DriverModel returned `DRIVER_DATA_ALLOW_MULTITHREADING = 0`. Fix:
   `setup_eco.py` forces single-core **and** the DriverModel now returns `1`
   (`DriverModel_FIXS_Common.h`, a **ProprietaryFiles** change — pending the PF PR).
   ✅ After this, **`VISSIM_Connect OK`** and **all four sockets connect**: DSProxy
   ego, signals (2445), DriverModel (1337), CAV controller (1338).

2. **Tick-0 stall — FIXED.** The DriverModel's `DRIVER_DATA_TIME` does no I/O on its
   first step (`isVeryFirstStep`, empty network), but DSProxyMode recv'd from it every
   tick → blocked at tick 0. Fix: the `tick >= 1` guard in DSProxyMode. ✅ The loop
   now advances past tick 0 (`tick 0: vehicles=0 signals=7 egos=1`).

3. **Tick-1 relay deadlock — OPEN.** The loop then stalls at tick 1. Root cause is
   structural: the FIXS DriverModel does its **send→recv inside `DRIVER_DATA_TIME`**,
   which VISSIM fires **during** TL's `setDriverVehicles`. So the DriverModel blocks
   on its recv (waiting for TL) while TL is still inside `setDriverVehicles` (waiting
   for the callback to return) and hasn't reached its own DM send/recv → circular
   deadlock. DriverModelLog confirms it: 3 lines (`Real-Sim connected`), no send/recv.
   **Resolving this is the open Stage C relay-protocol work** (cav_xil README's
   "~½ day, restructure the inner loop"): co-design the TL↔DriverModel callback
   interleaving so TL's DM send is available when the in-callback recv runs (e.g.
   move TL's DM exchange to straddle `setDriverVehicles`, or change the DriverModel's
   in-callback ordering). Touches both FIXS (DSProxyMode) and PF (DriverModel).

So: **the co-load CONNECTS and the loop starts** — the multithreading + tick-0 fixes
are real and validated — but **CAVs are not yet actuated** because the per-tick relay
handshake deadlocks at tick 1. Everything else (GLOSA math, signal extraction, CAV
channel, signal sync, ego inject) is in place and exercised.

Other caveats:
- **Signal positions need one calibration pass** (above). Timing is reliable.
- **Penetration is controller-side** (free-flow for non-selected hooked CAVs), not a
  VISSIM demand-composition split — a documented approximation.
- **VISSIM 2022 COM dispatch is fragile on Win11 24H2** (CLAUDE.md): it wedges between
  runs and sometimes needs an admin FlexNet bounce / reboot, not just a zombie reap.
