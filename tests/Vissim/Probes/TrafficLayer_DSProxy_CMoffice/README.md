# TrafficLayer_DSProxy_CMoffice — CarMaker(office) ↔ VISSIM co-simulation demo (#168)

A **click-to-run demo** of CarMaker Office driving an ego through a VISSIM
network via the FIXS DSProxy pipeline, on the small **SimpleEcho** loop.
Vehicles only — **no traffic-signal sync** (SimpleEcho has no signals).

This promotes the Python `fake_carmaker.py` stand-in
([../TrafficLayer_DSProxy_xil/](../TrafficLayer_DSProxy_xil/)) to **real
CarMaker**: the ego pose now comes from CarMaker's `User.c` →
`VirtualEnvironment.lib`, not a Python script.

```
CarMaker.win64.exe ──[ego "egoCm" VehFullData @ port 2444]──▶ TrafficLayer.exe
   (custom office exe,                                          (DSProxy mode)
    User.c → VirtualEnvironment.lib)                                  │
        ▲                                                  VISSIM_SetDriverVehicles
        │                                                             ▼
        └──[all background traffic, ego restamped "egoCm"]──  VISSIM (via DrivingSimulatorProxy.dll)
                                                              SimpleEcho 4-edge loop, 700 veh/h
```

## What you see

- **CarMaker → VISSIM:** exactly one ego (`egoCm`). CarMaker owns its dynamics
  (built-in IPGDriver, no Simulink). It appears in VISSIM at the CarMaker-driven
  pose.
- **VISSIM → CarMaker:** the background vehicles VISSIM spawns (grows to ~8 over
  30 s on this small loop), rendered as the `RS_C000…RS_C019` traffic objects.
  The ego is **not** echoed back (TrafficLayer re-stamps it `egoCm` so the .lib
  skips its own ego).

## ✅ Verified working (2026-06-10)

The full pipeline runs **headless, end-to-end**, the REAL custom CarMaker exe +
`VirtualEnvironment.lib` against TrafficLayer+VISSIM. `verify_demo.py` asserts:
CarMaker `RealSim Initialized` + `SIM_END`, and TrafficLayer per-tick
`vehicles=N … egos=1` (peak vehicles=8, 300 ticks, graceful end). Repeatable.

### Run it — two ways

**A) Headless, self-checking (recommended for a quick proof / CI):**
Double-click **`run_demo_headless.bat`** (calls `verify_demo.py`). It stages the
network, launches TrafficLayer (which spawns VISSIM), then the real CarMaker exe,
captures both sides, and prints **PASS/FAIL** with evidence. No GUI, no clicking.

**B) GUI, for the 3D view / sponsor video:**
Double-click **`run_cm_office_demo.bat`**. It registers the custom exe into the
CM project + launches TrafficLayer (waits) + opens CarMaker Office. Then in the
GUI: load TestRun **`SimpleLoop_VISSIM_rs`** and press the green **Start**. The
GUI runs the *same exe and produces the same exchange* as (A) — it just adds the
live 3D scene. Stop order: **Stop in CarMaker → close VISSIM → Ctrl+C
TrafficLayer.** (Never `taskkill` VISSIM while it holds an open `.inpx` — that
leaks CodeMeter license sessions.)

The co-simulation is **lockstep and CarMaker-triggered**: TrafficLayer spawns
VISSIM, then *blocks* until CarMaker connects; that connect (mode A automatically,
mode B on Start) begins the run.

### Known limitation — ego loop

The ego drives one 200 m straight (link 0) at 18 km/h; the run ends **gracefully
at 30 s** before it reaches the link end. IPGDriver does not navigate the
hand-authored loop junctions (the route is valid — `roadutil` walks all 820 m —
but driver junction-routing on the synthetic OpenDRIVE road is a follow-up). The
co-simulation (ego→VISSIM, VISSIM traffic→CarMaker) is fully exercised in that
window. Looping the ego is tracked as future work.

## Run at another site (ORNL) — no build

For a quick out-of-box test, the `feature/168_cm_office_xil` branch **ships the two
built binaries** (`CarMaker.win64.exe` in ProprietaryFiles, `TrafficLayer.exe`) so
you do **not** need Visual Studio or the build chain. The road, TestRun, ego, and
DS-network are already committed too.

**On the test box you still need installed (cannot be shipped):**
- **CarMaker 13.1.3** office at the default `C:\IPG\carmaker\win64-13.1.3\`. The
  shipped exe is **version-locked to 13.1.3** — a different CarMaker version will not
  load it and you must rebuild (see *Build prerequisites* below).
- **VISSIM 2022** with a working CodeMeter license (provides `DrivingSimulatorProxy.dll`).
- **Python 3** on PATH (used only to stage files — no packages needed).

**Steps:**
```cmd
git clone <FIXS repo> && cd FIXS
git checkout feature/168_cm_office_xil
git submodule update --init --recursive
tests\Vissim\Probes\TrafficLayer_DSProxy_CMoffice\run_cm_office_demo.bat
```
Then in the CarMaker GUI: wait for the TrafficLayer window to print
`VISSIM_Connect OK`, load TestRun `SimpleLoop_VISSIM_rs`, press the green **Start**.

> The committed binaries are **temporary** for this cross-site test and are stripped
> before the ProprietaryFiles PR merges (no multi-MB blob on `PF/main`). If CarMaker
> is not 13.1.3, ignore them and build per *Build prerequisites*.

## First-time setup

The CarMaker road + TestRun are generated headlessly from the VISSIM network's
OpenDRIVE export. If they're missing (or SimpleEcho changed), run once:

```cmd
import_road.bat
python build_testrun.py
```

- **`import_road.bat`** runs `osc2cm.exe` (OpenDRIVE → `simple_loop.rd5` +
  `SimpleLoop_VISSIM` TestRun + ego) then `RealSimCarMakerSetup.py` (seeds 20
  `RS_Cxxx` traffic objects).
- **`build_testrun.py`** then makes it actually load + run in CarMaker 13:
  osc2cm produces a **route-less** road and an old-format traffic set the CM13
  loader rejects. The script (a) adds a closed-loop **Route 900** to the `.rd5`
  by chaining the 8 LanePath segments around the loop (validated by `roadutil
  -rlen 0` ≈ 820 m), (b) gives the ego maneuver a real duration + Route 900, and
  (c) rewrites the 20 traffic objects in the working CM13 format (Template +
  AutoDriver + maneuver + Route 900). At runtime `VirtualEnvironment.lib`
  free-motion-teleports the `RS_C` objects to the VISSIM-driven positions — the
  route is only load-time scaffolding.

Outputs land in `ProprietaryFiles/CM13_proj/Data/` (Road, TestRun, Vehicle).

## Build prerequisites

All three binaries must be built (Release x64):

| Artifact | Build command |
| --- | --- |
| `CommonLib/yaml-cpp` | `scripts\dispatch\1_external_libraries.bat` |
| `VirtualEnvironment.lib` | `scripts\dispatch\4_virtual_environment.bat` |
| `TrafficLayer.exe` | `scripts\dispatch\2_core_components.bat` |
| custom `CarMaker.win64.exe` | `msbuild ProprietaryFiles\CM13_proj\src\CarMaker.sln /t:CarMaker /p:Configuration=Release /p:Platform=x64` |

Environment: VISSIM 2022, CarMaker **13.1.3**, VS 2022. (The issue says CM
13.1.2; the box and the vcxproj target 13.1.3.)

## Verified state (2026-06-10)

The full pipeline is **verified working end-to-end, headless**, with the REAL
custom CarMaker exe + `VirtualEnvironment.lib` (not a Python stand-in). Re-run
`python verify_demo.py` to reproduce; it asserts PASS/FAIL.

| Step | Status |
| --- | --- |
| yaml-cpp / VirtualEnvironment.lib / TrafficLayer.exe / custom CarMaker.win64.exe build | ✅ all built Release x64 |
| osc2cm: `simple_loop.xodr` → `simple_loop.rd5` + TestRun + ego | ✅ |
| `build_testrun.py`: Route 900 + 20 CM13-format `RS_C` traffic + ego routing | ✅ |
| `patch_ds_inpx.py`: DS-enabled copy of `simple_loop.inpx` (committed as `simple_loop_ds.inpx`) | ✅ |
| `VISSIM_Connect` (DSProxy spawns VISSIM 2022) | ✅ `VISSIM_Connect OK` (~14–45 s) |
| **Real CarMaker `.lib` connects** | ✅ `RealSim Initialized` / `All Clients Connected!` |
| **Ego `egoCm` → VISSIM every tick** | ✅ TrafficLayer `egos=1` |
| **VISSIM background traffic → CarMaker `RS_C` objects** | ✅ TrafficLayer `vehicles` grows to 8 |
| Run completes gracefully | ✅ CarMaker `SIM_END` at 30 s, 300 ticks, no socket drop mid-stream |

`verify_demo.py` launches both processes, captures both sides, and checks the
above automatically (this is what `run_demo_headless.bat` calls). `smoke_ego.py`
remains as a lighter Python-only protocol stand-in.

### Two bugs fixed to get here (both real, beyond #168 scope)

1. **`ProprietaryFiles/CM13_proj/src/User.c` SCState guard.** It gated
   `VirEnv_initialization` on `SimCore.State >= SCState_StartWait` (4), but
   `User_TestRun_Start_atEnd` runs in `SCState_Start` (3) — so the FIXS socket
   **never connected** and every real CarMaker co-sim silently no-op'd. Fixed to
   `>= SCState_Start` (matches `doc/CarMakerDoc.md`), plus a `RS_configFile !=
   NULL` guard so running the exe without `-f` doesn't crash. **This is a
   ProprietaryFiles change** (PF PR per the submodule workflow).
2. **`TrafficLayer/.../DSProxyMode.cpp` ego id.** VISSIM reassigns the ego an
   integer `VehicleID`, losing the client's `egoCm` handle; we re-stamp it back
   to `egoCm` on publish so the `.lib` recognizes and skips its own ego (wire-
   identical to the SUMO path).

### Driving-simulator mode (why `patch_ds_inpx.py` exists)

DSProxy's `VISSIM_Connect` requires the network to have VISSIM's
**driving-simulator** mode on — `<netPara drivSimActive="true" …>` in the
`.inpx`. PTV's shipped `driving_simulator_test.inpx` has it; the SimpleEcho
`simple_loop.inpx` (a normal-sim network) does **not**, so a raw connect fails
with *"connection got cancelled by VISSIM"*. `run_cm_office_demo.bat` runs
`patch_ds_inpx.py` to write a DS-enabled copy into `stage_network/` — the
committed source `.inpx` is left untouched (it's shared with the echo probe).

### If VISSIM won't start (CodeMeter license)

If `VISSIM_Connect` fails and the VISSIM log (`%TEMP%\VISSIM\vissim_msgs.txt`)
shows *"Cannot update the certified time … CodeMeter error code: 200"*, that's a
**CodeMeter** license issue (VISSIM 2022 licenses via CodeMeter/Wibu, not
FlexNet). It happens when the system clock is somewhere CodeMeter's anti-rollback
"certified time" can't reconcile. Fix: open **CodeMeter Control Center**
(`C:\Program Files\CodeMeter\Runtime\bin\CodeMeterCC.exe`) → WebAdmin → sync
certified time with the CodeMeter Time Server, confirm the VISSIM 2022 container
is loaded, and correct the system clock if it's wrong. (This was hit and resolved
on 2026-06-10; not a defect in the probe.)

Then verify VISSIM dispatches:
```cmd
"C:\Users\yshao\miniconda3\envs\realsim_dev\python.exe" -c "import pythoncom,win32com.client; pythoncom.CoInitialize(); win32com.client.Dispatch('VISSIM.Vissim.2200'); print('OK')"
```
Once it prints `OK`, re-run `run_cm_office_demo.bat` — no code changes needed.

## Files

| File | Purpose |
| --- | --- |
| `run_demo_headless.bat` | **One-click headless demo** → runs `verify_demo.py`, prints PASS/FAIL |
| `verify_demo.py` | Self-checking end-to-end: launches TL + real CarMaker, asserts the round-trip |
| `run_cm_office_demo.bat` | GUI launcher (stage → config → register exe → TrafficLayer → CarMaker Office) |
| `config.yaml` | TrafficLayer config: DSProxy + `CarMakerSetup`, `egoCm` on port 2444, signals off |
| `simple_loop_ds.inpx` / `.layx` | **Committed** DS-enabled SimpleEcho network (driving-simulator mode on) |
| `simple_loop.xosc` | OpenSCENARIO wrapper for the osc2cm import (one ego on road 40) |
| `import_road.bat` | Headless road + TestRun + seed traffic (osc2cm + RealSimCarMakerSetup.py) |
| `build_testrun.py` | Adds Route 900 to the `.rd5` + rewrites ego/traffic to the working CM13 format |
| `patch_ds_inpx.py` | Regenerates `simple_loop_ds.inpx` (DS mode on) if the source network changes |
| `smoke_ego.py` | Lighter Python-only CarMaker stand-in; verifies the socket round-trip |
| `config.runtime.yaml` | Generated at launch (absolute network path); gitignored |
| `stage_network/` | DS-enabled copy of the SimpleEcho `.inpx`; gitignored |

## How the ego identity round-trips

CarMaker only ever knows its own handle **`egoCm`**. TrafficLayer brokers the
map `egoCm ↔ VISSIM VehicleID`: on first sight of `egoCm` it injects to VISSIM
with a create-tag, learns the VISSIM-assigned integer `VehicleID`, and from then
re-stamps that vehicle's published id back to `egoCm`. CarMaker never sees or
picks a VISSIM id (VISSIM assigns it, collision-free). This makes the DSProxy
publish stream wire-identical to the SUMO path, so the `VirtualEnvironment.lib`
ego-skip logic works unchanged.

## Scope / not covered

- No traffic signals (SimpleEcho has none; signal sync deferred to #156).
- No Simulink, no CM4SL, no dSPACE — office IPGDriver only.
- Single ego. Lockstep start only (deferred-connect / `SimulationMode` wait —
  already a SUMO-path feature — is future work for the DSProxy loop).

## Known issue: loop corners are too sharp for the ego (geometry redesign)

The SimpleEcho square loop has **~2–3 m radius corners** (in VISSIM the corner
connectors, links 5–8, are ~2×2 m and turn ~76°; in the xodr the corner
connectors are 5.19 m / ~3 m radius). A passenger car's **minimum turning radius
is ~5–6 m**, so the IPGDriver ego *physically cannot* track these corners and
drives off the road. Slowing the ego does **not** help — the corner is below the
car's turning limit. (The committed testrun therefore ends at 35 s, before the
ego reaches the first corner at ~40 s, so the demo runs clean on the bottom
straight.)

**Fix — not yet applied; needs a coordinated geometry redesign of *both*
network representations:**

1. Replace each 90° sharp corner with a quarter-circle **arc of radius R ≥ ~10–15 m**
   (a "rounded rectangle"): shorten each 200 m straight to `(200 − 2R)` and add a
   radius-R 90° arc at each corner. Loop perimeter becomes `4·(200−2R) + 4·(πR/2)`.
2. Apply it to **both** the VISSIM network **and** the xodr, kept **coordinate-
   matched** — the co-sim exchanges absolute X/Y, so any mismatch puts the ego
   off the VISSIM road (and the traffic off the CarMaker road).
3. The **xodr** side is mechanical (`<arc curvature="1/R">` geometry, then
   `osc2cm` → rd5 + update the route `DrvPath`). The **VISSIM** side is the
   blocker: rewriting the existing network's link/connector/lane geometry by hand
   is error-prone and can't be visually verified headlessly. Cleanest path:
   **redraw the loop with curved corners in the VISSIM GUI** (or import a
   rounded-rectangle OpenDRIVE so VISSIM and CarMaker derive from one source),
   then regenerate the rd5 from the matching xodr and re-validate the route.

Once corners are widened, the ego can complete the loop and the run can be
extended past 35 s (raise `DrvMan.Global.EndCond` / the maneuver `TimeLimit` in
the testrun).
