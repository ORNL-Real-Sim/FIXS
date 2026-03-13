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
3. DriverModel_RealSim.dll and DriverModel_RealSim_v2021.dll (VISSIM interface)
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

**When your changes touch `ProprietaryFiles`:**

```bash
# Step 1: Create a matching branch in the private repo
cd ProprietaryFiles
git checkout -b maintenance/NNN_short_desc
# make your changes, commit, push
git push origin maintenance/NNN_short_desc

# Step 2: Back in the FIXS worktree, stage the updated submodule pointer
cd ..
git add ProprietaryFiles
git commit -m "chore: bump ProprietaryFiles to maintenance/NNN"
git push origin maintenance/NNN_short_desc

# Step 3: Open PR in FIXS (public) targeting dev — shows only the hash change
# Step 4: Open companion PR in ProprietaryFiles targeting dev
# Step 5: After both devs are merged, release PRs merge dev→main in both repos together
```

**Rules:**
- Never commit directly to `ProprietaryFiles/main` or `ProprietaryFiles/dev` — both are branch-protected (require PR + review; admins may bypass)
- Always use a named branch in ProprietaryFiles that mirrors the FIXS issue branch
- Feature PRs in ProprietaryFiles target **`dev`** (not `main`); `main` is updated only via release PRs from `dev`
- The FIXS PR description should reference the ProprietaryFiles companion PR for context (internal reviewers can check it; external contributors see only the hash)
- Use the same PR template as FIXS (identical `.github/pull_request_template.md`)

## Important Notes

- **Error logs**: TrafficLayer.exe writes errors to TrafficLayer.err in its directory
- **VISSIM logs**: DriverModelError.txt and DriverModelLog.txt appear in VISSIM network directory
- **Verbose logging**: Enable with `EnableVerboseLog: true` in config.yaml for debugging
- **Multi-vehicle numbering**: Client/controller components numbered 1-20 for running multiple instances
- **Platform**: Windows-only (uses WinSock, VISSIM is Windows-only)

## Documentation

- VISSIM-specific: [doc/VISSIMdoc.md](doc/VISSIMdoc.md)
- SUMO-specific: [doc/SUMOdoc.md](doc/SUMOdoc.md)
- CarMaker-specific: [doc/CarMakerDoc.md](doc/CarMakerDoc.md)
