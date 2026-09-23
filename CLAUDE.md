# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Real-Sim FIXS (Flexible Interface for XIL Simulation) is a multi-resolution X-in-the-loop (XIL) simulation framework for connected and automated vehicle testing. It provides transparent connections between traffic simulators (VISSIM, SUMO), vehicle dynamics tools (CarMaker, Simulink), and virtual environments (Carla).

**Current Software Versions**: CarMaker 13.1.2, Carla 0.9.15, VISSIM 2022, SUMO 1.21, dSPACE/Matlab 2024a

## Architecture

### Core Components

- **TrafficLayer**: Main interface executable that orchestrates connections between traffic simulators and other components. Entry point: [TrafficLayer/TrafficLayer/mainTrafficLayer.cpp](TrafficLayer/TrafficLayer/mainTrafficLayer.cpp)
- **ControlLayer**: Application-level controllers (e.g., CoordMerge for coordinated merging)
- **VehicleClient**: Dummy clients for testing (numbered 1-20 for multi-vehicle scenarios)
- **VirtualEnvironment**: Interface to virtual environments like Carla
- **CommonLib**: Shared libraries and helpers used across all components

### Key Helper Classes

- **ConfigHelper**: Parses config.yaml files and manages simulation setup ([CommonLib/ConfigHelper.h](CommonLib/ConfigHelper.h))
- **SocketHelper**: Handles TCP socket communication between components ([CommonLib/SocketHelper.h](CommonLib/SocketHelper.h))
- **TrafficHelper**: Manages connections to VISSIM/SUMO traffic simulators ([CommonLib/TrafficHelper.h](CommonLib/TrafficHelper.h))
- **MsgHelper**: Message serialization/deserialization for vehicle data ([CommonLib/MsgHelper.h](CommonLib/MsgHelper.h))

### Communication Architecture

The system uses a client-server socket architecture:
- TrafficLayer acts as central hub, connecting to traffic simulators (VISSIM via DLL, SUMO via libsumo)
- Applications/XIL systems connect to TrafficLayer as clients
- Messages contain vehicle state data defined in [CommonLib/VehDataMsgDefs.h](CommonLib/VehDataMsgDefs.h)
- Configuration driven by config.yaml files (see test directories for examples)

### Simulator-Specific Implementations

- **VISSIM**: Uses driver model DLL ([ProprietaryFiles/VISSIMserver](ProprietaryFiles/VISSIMserver)) that hooks into VISSIM's driver model API
- **SUMO**: Uses libsumo embedded library ([CommonLib/libsumo](CommonLib/libsumo)) for in-process communication
- **CarMaker**: Uses custom HIL integration with Simulink ([CarMaker](CarMaker) utilities)

## Build System

Real-Sim FIXS uses a modular script-based build system with automated tool detection and version management. For comprehensive documentation, see [doc/BUILD.md](doc/BUILD.md).

### Prerequisites

- **Visual Studio 2022** (Community, Professional, or Enterprise)
- **CMake 3.10+** for building external libraries
- **dependencies.yaml** - Central configuration file defining tool versions and paths
- Optional: MATLAB 2024a, CarMaker 13.1.3/11.1.2, dSPACE ConfigurationDesk 2024-A

### First-run setup

After cloning, initialize the checkout. `dispatch.bat` also runs this as its step 1, so a fresh clone can go straight to `dispatch.bat`:
```
powershell -ExecutionPolicy Bypass -File scripts\initialize_fixs.ps1
```
It is idempotent and does: ProprietaryFiles submodule (optional) -> native deps -> yaml-cpp, then prints a per-step summary. Not to be confused with `scripts/update_fixs.ps1` / `scripts/update_fixs.sh`, which are consumer-side (install a published release zip into an application checkout; see #272).

**Native deps are not in git** (#109, #238). `CommonLib/libsumo` and `CommonLib/libcarla` are gitignored and fetched as SHA-256-verified, version-named zips from the public rolling release `fixs-native-deps`. libsumo is **required** (TrafficLayer links `libsumocpp.lib` directly, so a missing fetch is a hard error); libcarla is optional (VirCarlaEnv only). Bumping the SUMO version in `dependencies.yaml` requires publishing a matching asset first — `scripts\dispatch\pack_native_deps.ps1 -Component sumo -Publish` — or every clone breaks. Both fetch and pack load-test `libsumo/bin` (`libsumo_verify.ps1`) because the old vendored copy silently lacked `geos_c.dll`/`geos.dll` for months (#70).

yaml-cpp can also be built alone; it uses CMake with the Visual Studio 17 2022 generator and builds both Release and Debug:
```
scripts\dispatch\1_external_libraries.bat
```

### Building Components

**Release Build (Full):**
```
dispatch.bat
```

Automatically builds all components and copies to `build/` directory:
1. Clone initialization (submodules, native deps, yaml-cpp) via `scripts/initialize_fixs.ps1`
2. TrafficLayer.exe, CoordMerge.exe, VirtualEnvironment.lib
3. DriverModel_RealSim.dll (default, int API, VISSIM 2021+) and DriverModel_RealSim_legacy.dll (frozen, long API, VISSIM ≤ 2020)
4. CarMaker executables for all versions in dependencies.yaml (CM11, CM13)
5. dSPACE libraries (if dSPACE detected)
6. RealSimSocket.mexw64 (if MATLAB detected)
7. BUILD_INFO.txt with version metadata

**Development Build (Individual Components):**

For faster iteration, build specific components:
```
# Core components only
scripts\dispatch\2_core_components.bat

# VISSIM driver models only
scripts\dispatch\3_vissim_components.bat

# CarMaker (auto-generates BuildConfig files based on dependencies.yaml)
powershell -ExecutionPolicy Bypass -File scripts\dispatch\4a_carmaker_components.ps1

# dSPACE libraries
powershell -ExecutionPolicy Bypass -File scripts\dispatch\4b_carmaker_dspace.ps1

# MATLAB MEX file
powershell -ExecutionPolicy Bypass -File scripts\dispatch\5_mex_realsim_socket.ps1
```

### Build System Features

- **Automated tool detection**: [scripts/dispatch/detect_tool_paths.ps1](scripts/dispatch/detect_tool_paths.ps1) finds Visual Studio, MATLAB, dSPACE, and CarMaker installations
- **Version management**: [dependencies.yaml](dependencies.yaml) defines all tool versions; scripts parse this to determine what to build
- **BuildConfig auto-generation**: CarMaker BuildConfig Python files are dynamically generated for each CarMaker/MATLAB version combination
- **Modular scripts**: Each subsystem has dedicated build script in [scripts/dispatch/](scripts/dispatch/)
- **Build logs**: Detailed logs in `scripts/dispatch/build.log` and `scripts/dispatch/build_summary.log`
- **Debug/Release**: Set `RS_BUILD_CONFIG` environment variable to switch between Debug and Release builds

## Configuration

### config.yaml Structure

All simulations are configured via YAML files with these main sections:

- **SimulationSetup**: Core settings including which simulator (VISSIM/SUMO), IP/ports, message fields to exchange, synchronization mode
- **ApplicationSetup**: Application layer subscriptions for vehicle/detector/signal data
- **XilSetup**: XIL system configuration (Simulink, CarMaker)
- **VirtualEnvironmentSetup**: Virtual environment (Carla) settings

Key configuration parameters:
- `SelectedTrafficSimulator`: 'VISSIM' or 'SUMO'
- `EnableExternalDynamics`: Allow external control of vehicle dynamics (SUMO)
- `VehicleMessageField`: Array of fields to exchange (see [README.md](README.md) Appendix for full field list)
- `WarmUpUntilEgoEntry` / `WarmUpTime`: warm-up triggers (see below)

### Warm-up (#86, replaced SimulationMode)

`SimulationMode` / `SimulationModeParameter` were removed in 0.9.0. They were documented
as a bitfield but never were one — the code tested `4||5` and `1||2` as identical
branches, and 3/6/7 silently meant 0. No config in FIXS or FIXS_Applications set them, so
they were dropped outright with no compatibility shim.

Two mutually exclusive keys in `SimulationSetup`, both SUMO-only:

- `WarmUpUntilEgoEntry: true` — tick until the first subscribed ego is in the network.
  The watched ids are the **union** of the by-id vehicle subscriptions in
  `ApplicationSetup` and `XilSetup` (`ConfigHelper::WarmUpEgoIds`), deliberately not
  `vehicleSubscribeId_v`, which is either/or and also drives message routing.
- `WarmUpTime: <absolute sim time>` — one batch `Simulation::step(T)`.

While the warm-up runs, the main loop returns to the top before any client I/O **and the
clients are not accepted yet** (`SocketHelper::DeferAcceptClients` / `acceptClients()`),
so client start-up overlaps the warm-up. Measured on MLK's 185 s warm-up: batch step
2.70 s, tick loop + ego poll 3.29 s — both against minutes for a fully synced warm-up,
because what dominates is client I/O, not stepping. A batch `step(T)` from TrafficLayer
was verified to coexist with a second TraCI client stepping 1:1 (both land exactly on
the target time), though SUMO then advances only as fast as that other client steps.

VISSIM parity is not implemented: TrafficLayer does not own the VISSIM clock, and the
DriverModel DLL still has its own hardcoded `ENABLE_WARMUP`. A VISSIM config that sets
either key is rejected at startup.

## Running Tests

Test scenarios are in [tests](tests) directory, each with .bat files to run:

Example test execution:
```
cd tests/CoordMerge
runCoordMergeSUMO.bat
```

Tests are organized by scenario (CoordMerge, DelayedConnection, Elevation, etc.) and typically include:
- config.yaml for that scenario
- Batch files to launch simulation
- Network files for VISSIM/SUMO
- Simulink models if applicable

## Development Workflow

1. **Modifying simulator interfaces**: Edit TrafficHelper or simulator-specific code in CommonLib
2. **Adding message fields**: Update VehDataMsgDefs.h and corresponding pack/unpack logic in MsgHelper
3. **Creating new controllers**: Add to ControlLayer following CoordMerge pattern
4. **Testing changes**: Use existing test scenarios or create new one with config.yaml

### Branch Strategy

- All development branches off **`dev`**, not `main`
- PRs target **`dev`**; `main` is updated only via release PRs from `dev`
- Branch naming: `<type>/<issue_number>_<short_desc>` — e.g., `maintenance/128_unify_fixs_branding`

### ProprietaryFiles Submodule Workflow

`ProprietaryFiles` ([ORNL-Real-Sim/ProprietaryFiles](https://github.com/ORNL-Real-Sim/ProprietaryFiles)) is a **private** submodule containing CarMaker/VISSIM/dSPACE integration code that cannot be made public. The public FIXS repo stores only a commit hash pointer — no proprietary content is exposed.

**Invariant:** every merged FIXS commit must point to a `ProprietaryFiles/main` commit, never to a feature-branch tip. This way anyone cloning FIXS gets a stable, reviewed PF state.

**Branching:** PF uses `main` only (no `dev` branch in this flow). Feature branches target `main` directly. FIXS keeps its own `dev_v0.X.0` → `main` line because FIXS bundles multiple issues per release; PF is small and atomic per change, so it doesn't need a staging branch.

**When your changes touch `ProprietaryFiles`:**

```bash
# Step 1: Create matching branch in the private repo, commit, push
cd ProprietaryFiles
git checkout -b maintenance/NNN_short_desc
# ... make changes, commit ...
git push origin maintenance/NNN_short_desc

# Step 2: Bump the FIXS submodule pointer to your in-progress PF branch tip
cd ..
git add ProprietaryFiles
git commit -m "chore: bump ProprietaryFiles to maintenance/NNN"
git push origin maintenance/NNN_short_desc

# Step 3: Open BOTH PRs in parallel, cross-referencing each other:
#   - ProprietaryFiles PR  →  PF/main
#   - FIXS PR              →  dev_v0.X.0

# Step 4: Review both together. When both have approval:
#   (a) Merge the ProprietaryFiles PR first
#   (b) Fast-forward the FIXS submodule pointer to the new PF/main HEAD:
cd ProprietaryFiles && git fetch origin && git checkout main && git pull
cd ..
git add ProprietaryFiles
git commit -m "chore: bump ProprietaryFiles pointer to merged main"
git push
#   (c) Merge the FIXS PR
```

**Rules:**
- Never commit directly to `ProprietaryFiles/main` — branch-protected (requires PR + review; admins may bypass)
- Always use a PF branch name that mirrors the FIXS issue branch
- Step 4b is mandatory: do not merge a FIXS PR whose submodule pointer is at a PF feature-branch tip
- If a FIXS PR is rejected *after* its companion PF PR was merged, open a revert PR in PF — don't leave orphan code on `PF/main`
- Use the same PR template as FIXS (identical `.github/pull_request_template.md`)

## Important Notes

- **Error logs**: TrafficLayer.exe writes errors to TrafficLayer.err in its directory
- **VISSIM logs**: DriverModelError.txt and DriverModelLog.txt appear in VISSIM network directory
- **Verbose logging**: Enable with `EnableVerboseLog: true` in config.yaml for debugging
- **Multi-vehicle numbering**: Client/controller components numbered 1-20 for running multiple instances
- **Platform**: Windows-only (uses WinSock, VISSIM is Windows-only)

## VISSIM 2022 dispatch on Win11 24H2 — fragility + soft reset

On Windows 11 24H2 the system `ucrtbase.dll` (10.0.26100.x) is incompatible
with VISSIM 2022's CRT init. `VISSIM220.exe` crashes a meaningful fraction
of COM dispatches at startup, leaving zombie processes / stale FlexNet
license tokens / temp lock files behind. Symptoms accumulate over a
session:

- `actxserver('VISSIM.Vissim.2200')` and Python `Dispatch('VISSIM.Vissim.2200')`
  both work right after reboot, then start returning
  `Server execution failed` (HRESULT `0x80080005`).
- The same dispatch may work from one process and fail from another at
  the same moment (whichever runs into the zombie first).

**Stick with the 2022 ProgID** — do not silently bump scripts to 2600.
The repo target and the pinned `.mat` references are 2022-bound; switching
breaks cross-machine reproducibility.

### Soft reset (try before rebooting)

Run from any shell. Step 2 needs admin; the others don't.

```cmd
REM 1. Reap zombie/half-dead processes
taskkill /F /IM VISSIM220.exe
taskkill /F /IM TrafficLayer.exe
REM (don't kill your interactive MATLAB)

REM 2. Bounce FlexNet (returns leaked license tokens) — admin only
net stop "FlexNet Licensing Service 64"
net start "FlexNet Licensing Service 64"

REM 3. Clean stale VISSIM temp/lock files
del /Q "%TEMP%\VISSIM\*.lock"
del /Q "%TEMP%\VISSIM\vissim_msgs*.txt"
del /Q "%TEMP%\VISSIM\CommonDialogs.log"
```

Verify dispatch is healthy by running a one-off Python probe:
```python
import pythoncom, win32com.client
pythoncom.CoInitialize()
v = win32com.client.Dispatch('VISSIM.Vissim.2200')
print('OK')
```

If still failing after soft reset, reboot. Long-term fix is repair-installing
VISSIM 2022 from PTV's installer so a healthy SxS VC runtime gets dropped
next to `VISSIM220.exe`.

### Automated self-heal in the CMoffice probes

The DSProxy CarMaker probes now **automate** the dispatch check + soft reset so a
single bad dispatch no longer wedges every run. `vissim_dispatch_probe.py` tests
the `VISSIM.Vissim.2200` dispatch (exit 0 healthy / 1 poisoned, and self-closes
its throwaway instance); `run_cm_office_demo.bat` runs it as a preflight and, only
when it reports the zombie state, reaps `VISSIM220.exe` + clears `%TEMP%\VISSIM`
locks and retries (×3). The headless `verify_demo.py` seed-sweep loop does the
same check-and-clean between runs — that's why it can drive dozens of VISSIM runs
unattended where the old bat would stall. The probe needs pywin32, so point
`%PYTHON%` at the `realsim_dev` env; it skips harmlessly otherwise. The reap only
fires when dispatch is *already* broken (a corpse, no live co-sim), so it does not
leak CodeMeter sessions — which is why the blanket VISSIM kill stays off in the
signal probe's bat.

### Other operational notes

- **Never `taskkill /F` a process that holds an open VISSIM COM reference**
  while the .inpx is loaded — that's the primary way zombies are created.
  Let MATLAB/Python exit cleanly when possible.
- **VISSIM 2026 is installed side-by-side on dev boxes that have it**, but
  it is not the supported target. `VISSIM.Vissim.2600` is a fallback only,
  used in emergencies and reverted as soon as 2022 is healthy again.
- **Spawning a second MATLAB from a primary MATLAB and having the child
  call `actxserver`** is the most fragile pattern (the spawned MATLAB
  inherits half-initialized COM state from the parent and contends for
  the floating PSL license). When refactoring tests, prefer having
  Python — not a child MATLAB — run the VISSIM bootstrap.

## SimpleLoop ego junction off-road (CarMaker IPGDriver) — demos are seed-pinned

The CMoffice co-sim demos (SUMO and VISSIM) on the shared **SimpleLoop** scenario
have a **stochastic ego off-road** at a junction curve: on some traffic
realizations the ego leaves the road (around x≈8–23, y≈0–1) and CarMaker aborts
with `Vehicle leaves road…` or `Embedded FARoadSensor … not found on Road`.

Established by direct evidence on the #174 branch (don't re-derive — it's settled):

- **Root cause is CarMaker-side, not the traffic feed.** When background traffic
  halts the ego *on* the junction curve and it restarts from ~0 m/s, IPGDriver
  fails to re-acquire its course — heading stays frozen, it drives nearly straight,
  a tire crosses the inner edge (measured in `rs_ego.csv`). A *moving* ego tracks
  the same corner fine. This is the #168 "drives straight off the link end."
- **Stochastic + backend-agnostic.** Reproduced on BOTH backends at comparable
  rates, driven only by the RNG seed (the traffic realization), NOT the simulator
  or car-following model: SUMO Krauss **2/13**, SUMO W99 **4/13**, VISSIM **5/51**.
  The traffic data CarMaker receives is clean at the abort. Do **not** chase this
  as a SUMO-feed / sync / phantom-velocity bug — those were measured out.
- `Driver.Course.CornerCutCoef=0` (TestRun `SimpleLoop_rs`) **reduces** it (fixed
  the SUMO fails) but does **not** eliminate it (VISSIM still off-roaded) — corner
  cutting contributes; the real fix is on the CarMaker junction/route side
  (junction road topology in `simple_loop.rd5`), tracked separately.

**Because of this the demos pin a verified-clean realization** (same seed ⇒ same
outcome, deterministic):

- **SUMO-CM:** `run_sumo_cm_demo.bat` launches SUMO with `--seed 5`;
  `verify_sumo_cm.py` defaults to `RS_SUMO_SEED=5`. (The *unseeded* SUMO default is
  the realization that crashes — that's why a seed must be pinned.)
- **VISSIM-CM:** the committed default `randSeed=42` is already clean; no change.
- **Reproduce a failure:** SUMO `RS_SUMO_SEED=10` (or `=none`); VISSIM
  `RS_VISSIM_SEED=32` (also 25/31/21/40) — `setup_gui.py`/`verify_demo.py` rewrite
  only the *staged* inpx randSeed, committed source untouched.

## VISSIM ↔ CarMaker traffic-signal co-simulation (#172)

The signal demo lives in
[tests/Vissim/Probes/TrafficLayer_DSProxy_CMoffice_Signal/](tests/Vissim/Probes/TrafficLayer_DSProxy_CMoffice_Signal/)
(`DEMO.md` there is the entry point). It drives CarMaker traffic lights from VISSIM
signal state over the DSProxy co-sim, and the CM ego brakes at the VISSIM-driven red.
One-click: `run_signal_demo.bat` (headless, self-checking) or
`run_cm_office_signal_demo.bat` (GUI). The hard-won, non-obvious facts (a future
agent WILL otherwise re-chase these — we spent a long time on signal placement):

**Generation pipeline (one geometry, two simulators).** `gen_loop_net.py` writes the
SUMO nodes/edges → `netconvert` → `simple_traffic_light.net.xml` → `netconvert
--opendrive-output` → `simple_traffic_light.xodr` (the CM source; osc2cm → rd5) AND
`import_to_vissim.py` (`ImportOpenDrive`) → the VISSIM `.inpx`. The co-sim exchanges
ABSOLUTE X/Y, so VISSIM and CM **must** derive from the same geometry. If the VISSIM
`.inpx` is stale (e.g. a 2000 m net while CM is the 1346 m loop), traffic/ego land
off-road — regen VISSIM from the xodr, **never edit the CM side to match**. `parse_signals.py`
+ `build_demand.py` must read the PROBE-LOCAL `simple_traffic_light.net.xml`, not the
stale `tests/Python/SimpleTrafficLight/` copy.

**The turnaround geometry is shared, and lives in
[`Carla/utils/sumo_uturn.py`](Carla/utils/sumo_uturn.py) (#327).** `gen_loop_net.py`
imports `curved_uturn()` from it rather than carrying its own copy of the curvature
integration. Do NOT re-inline that math here: the regression that keeps it honest is
that `gen_loop_net.py` still regenerates the committed `nodes.nod.xml`/`edges.edg.xml`
byte-for-byte, and two copies would drift silently.

Why a curved loop and not SUMO's own turnaround: SUMO's point turnaround is a short
internal link that pivots the vehicle 180° on the spot (on MLK, 4.97 m at 3.73 m/s —
about a 1.6 m turning radius). That is fine for queue statistics and undrivable for
anything tracking the OpenDRIVE export, so a CM/CARLA ego stalls at the terminus or
leaves the road. The teardrop replaces it with a tangent-continuous path at a 45 m
radius.

`sumo_uturn.py` also runs standalone, to put that same loop on a network that ALREADY
exists — a real digital twin (MLK and the rest of the Digital-Twin-Library) has no
plain-XML sources to regenerate from. It adds edges and never renames or removes any,
so route files stay valid; it emits node/edge/connection patches for `netconvert -s`
instead of hand-editing the net; and it attaches to the LANE CENTRELINES that carry the
traffic, not to the junction centroid. That last point is not cosmetic — at MLK's west
end the arriving lane is 8.5 m to one side of the junction centre, and anchoring on the
centre makes netconvert bridge the gap with a connector that crabs the vehicle sideways
(measured: a ±37° S-jog at an 11 m radius), reintroducing exactly the geometry the loop
exists to remove.

**Signal identity is faithful SUMO → xodr → CM (1:1).** netconvert emits one
`<signal id="<tls_id>_<linkIndex>">` per SUMO controlled connection, with the turn
direction in `subtype` (10=left, 20=right, 30=straight). osc2cm stamps that id on each
CM head as the `odrSignalId` tag and maps subtype → head type (1=straight,2=left,3=right).
So the CM head NAME is the SUMO-canonical `<tls_id>_<head_id>` (== the tag); parse it by
the LAST underscore (head_id = trailing int). THE ARROW TYPE IS ALWAYS CORRECT FROM THE
SUBTYPE — never "fix" arrows by swapping the type.

**Signal-head lateral placement — the bug we chased for a long time.** `add_signal_stops.py`
relocates each approach's head mount to the ACROSS edge (past the junction) so the ego
sees the heads ahead, then places each head over its SUMO lane via
`hOff = t + gantry_len` (gantry_len = the mount beam length). CM mounts heads on a gantry
and renders INCREASING hOff from the RIGHT lane to the LEFT; SUMO lateral `t` is negative
and most-negative for the rightmost lane, so `t + gantry_len` puts the rightmost lane at
the smallest hOff (CM right). The original `-t + 1.0` had the wrong sign → it MIRRORED the
lane assignment (left-turn head on the right lane, etc.), which LOOKED like flipped arrows
but was a flipped LANE mapping. Do NOT flip `facing` to "face the ego" — CM measures hOff
in the facing frame, so flipping facing pushes heads OFF-ROAD.

**Runtime sync (VISSIM is per-SignalGroup, NOT per-head).** The VISSIM driver DLL emits
one state char per SignalGroup; `DSProxyMode::toTlsData` sends `name="<SCno>_<sg>"`,
state = a single char. So `RSsignalTable.csv` (built by `build_signal_table.py`):
`SignalControllerId=<SCno>_<sg>` (the runtime match key), `SignalHeadId=0` (single-char
state), `CmTrafficLightIndex` = the `Control.TrfLight.<i>` array index, `CmControllerId` =
the SUMO-canonical name. One SG fans out to several CM heads (multiple rows share the key).
VISSIM hierarchy = SignalController(=tls) → SignalGroup(state unit) → SignalHead(unique No,
NOT used by the sync). Today's net is one-head-per-light (the simple case).

**Two co-sim gotchas:**
- **Plan A:** with `SynchronizeTrafficSignal: true` the CM `.lib` opens a SECOND socket to
  `TrafficSignalPort` (must differ from the vehicle port); `DSProxyMode` (TrafficLayer)
  serves it and relays per-(controller,SG) `TlsData` there. The `.lib` is unchanged.
- **Pass the signal table to CarMaker's `-s` WITHOUT the `.csv` extension** — the CM GUI/HIL
  auto-parses a `.csv` argument as an InfoFile and FATALS at startup. `readSignalTable`
  appends `.csv` itself, so the file stays `*.csv` on disk.

**Interactive VISSIM zoom briefly stalls BOTH sides — inherent, not a bug (don't re-chase).**
Zooming/panning the VISSIM window during the co-sim makes the whole lockstep hitch (cjffly
flagged it on PR #170). We instrumented the DSProxy loop (`RS_DEBUG` → `rs_timing_tl.csv`,
splitting the per-tick proxy calls): the cost is entirely in `proxy.getTrafficVehicles()`,
which spikes from ~5–10 ms to **100–450 ms** while `setDriverVehicles`/`getSignalStates` stay
flat. PTV's *Driving Simulator Interface* manual (`…\API\DrivingSimulator_DLL\doc\`) documents
it: `VISSIM_GetTrafficVehicles` *"Blocks while the calculation of the time step in Vissim
hasn't finished yet"* (§3, p.11), and §2 (p.6) says *"the visualization in Vissim should be
switched off in order to achieve the highest possible simulation speed."* VISSIM runs the sim
step and the network window on ONE thread, so an interactive zoom delays step completion and
TL blocks longer. **Quick Mode does NOT fix it** (it only removes the autonomous per-step
redraw — cuts *idle* `getVeh` ~6 ms→~2 ms — but can't offload an interactive zoom); **VISSIM
2026 behaves identically** and no 2022/2026 release note touches it. Fix direction (NOT "don't
touch VISSIM" — we want to watch it; and CarMaker can't pause in XIL/HIL real-time): automate
the VISSIM launch to open already framed on the ego vehicle and follow it, so manual zoom/pan
(the spike trigger) isn't needed — investigate VISSIM's COM camera/viewpoint API at startup, or
bake the follow-viewpoint into the saved `.layx`. To re-measure: build TL with `-p:RS_DEBUG=1` to an isolated
`OutDir`/`IntDir` (e.g. `x64\Release_dbg`, gitignored) so the shipped exe is untouched.

## Carla / XIL ego co-simulation design notes

Forward-looking decisions for unifying the virtual-environment bridges (CarMaker
`VirEnvHelper`/`VirtualEnvironment.lib` and the standalone Carla `mainVirCarla`):

- **Road surface fidelity is deliberately kept simple (near-term).** Carla cannot
  supply the road properties a tire model actually needs — it exposes macro
  geometry (grade/bank/elevation via OpenDRIVE waypoints) but **no queryable
  friction (μ) map and no road-roughness profile** (friction is author-only via
  wheel `tire_friction` + `static.trigger.friction` patches, never readable at a
  point). So for any external-dynamics (Simulink/XIL) ego, **assume a simple road
  in the dynamics model** (constant μ ≈ dry asphalt, smooth surface). If spatial
  surface variation is ever needed, it comes from a **scenario dataset**
  (μ/roughness keyed to road position) or CarMaker's native IPGRoad — **never**
  from a Carla→Simulink contact query. Carla stays out of the dynamics ground loop
  entirely; it is perception + visualization only.

- **Ego dynamics ownership is a per-ego mode**, mirroring the two CarMaker
  diagrams: mode A "backend owns ego" (CarMaker office / Carla-PhysX → bridge
  *reads* ego pose back and sends out) vs mode B "external owns ego" (Simulink/
  dSPACE → ego arrives on a 2nd FIXS connection and is *teleported in*). L2/L4 are
  both mode A; they differ only in who controls (TM desired-speed vs external CAV
  client + sensors). This switch belongs in `config.yaml` `CarlaSetup`.

## Documentation

- VISSIM-specific: [doc/VISSIMdoc.md](doc/VISSIMdoc.md)
- SUMO-specific: [doc/SUMOdoc.md](doc/SUMOdoc.md)
- CarMaker-specific: [doc/CarMakerDoc.md](doc/CarMakerDoc.md)

---

## Repo Health Observations (Claude-internal)

*Honest notes for Claude to reason about when assisting — not for user-facing docs.*

### High-priority debt
- **`TrafficHelper.cpp` (~1743 lines)** — SUMO and VISSIM logic is fully interleaved; tracked in milestone 0.8.0. Avoid adding to it; route new functionality to helper sub-files instead.
- **`mainTrafficLayer.cpp` (~1396 lines)** — The main() function does too much directly. Issue #113 tracks a refactor. Prefer adding logic in helper classes, not directly in main.
- **`ConfigHelper.cpp` (~1169 lines)** — Has dead `// TODO: add code` branches and commented-out error blocks; safe to clean up but not urgent.
- **`MsgHelper.py` (~28k lines)** — Almost certainly has dead code paths. Any changes there should be narrow and tested; full audit deferred.

### No automated CI
- Python unit tests run locally only (`tests/Python/unit/`). Issue #57 tracks GitHub Actions setup. When suggesting test commands, always direct to `python -m pytest tests/Python/unit/ -v` — this is the one simulator-free path.

### Branding transition in progress
- `EnableRealSim` → `EnableFIXS` config key rename is backward-compatible (both parsed); contributors should use `EnableFIXS` in new configs. `FIXS_VERSION_*` macros coexist with `REALSIM_VERSION_*` aliases — use `FIXS_VERSION_*` going forward.

### ProprietaryFiles submodule
- `DriverModel_FIXS_Common.h` has 8 `// TODO #129:` stubs for unimplemented message handlers (signal table, connection-failed paths, speed-limit errors). These are in ProprietaryFiles/main which is branch-protected; changes require a PR to ProprietaryFiles/dev first.
- External contributors see `ProprietaryFiles/` as an inaccessible submodule — don't ask them to modify it.

### Documentation gaps (issue #137)
- RTD `doc/BUILD.md` and `doc/CarlaDoc.md` are orphaned from the TOC — they build but aren't linked in `index.rst`.
- Python CommonLib (`ConfigHelper.py`, `MsgHelper.py`, `SocketHelper.py`) has zero docstrings; Sphinx autodoc will produce empty pages until added.
- C++ headers (`ConfigHelper.h`, `SocketHelper.h`, `MsgHelper.h`) have no Doxygen comments; C++ API doc is deferred until issue #137 is addressed.

### Build system
- `generate_version.ps1` requires git tags to exist; falls back to a placeholder version string otherwise. Mention this when helping with release tagging.
- Script numbering in `scripts/dispatch/` (4a, 4b, 5, 6) is non-linear — a known rough edge, not a bug.
