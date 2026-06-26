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

The external library (yaml-cpp) is automatically built by the dispatch system or can be built manually:
```
scripts\dispatch\1_external_libraries.bat
```

yaml-cpp uses CMake with Visual Studio 17 2022 generator and builds both Release and Debug configurations.

### Building Components

**Release Build (Full):**
```
dispatch.bat
```

Automatically builds all components and copies to `build/` directory:
1. External libraries (yaml-cpp)
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
- `SimulationMode`: Bitfield controlling sync behavior (0=sync at start, 1=wait for ego entry, 4=wait for specified time)

### Simulation Modes

SimulationMode is a bitfield:
- 0 (binary 000): Sync from simulation start
- 1 (binary 001): Wait mode until ego vehicle enters network, then sync
- 4 (binary 100): Wait mode until SimulationModeParameter seconds, then sync

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
