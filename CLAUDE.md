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

### Prerequisites

External libraries must be compiled first using:
```
compileExternalLibraries.bat
```

This builds:
- libevent (event-driven socket library) in [CommonLib/libevent](CommonLib/libevent)
- yaml-cpp (YAML parser) in [CommonLib/yaml-cpp](CommonLib/yaml-cpp)

Both libraries use CMake with Visual Studio 16 2019 generator and build both Release and Debug configurations.

### Building Components

Build all components:
```
cd tests
compileCodes.bat
```

This builds (in order):
1. TrafficLayer.exe
2. CoordMerge.exe (control layer)
3. DriverModel_RealSim.dll and DriverModel_RealSim_v2021.dll (VISSIM interface)
4. VirtualEnvironment.exe
5. CarMaker projects (CM9, CM10, CM11)

All builds use msbuild with Release configuration. The msbuild command must be in PATH (typically `%ProgramFiles(x86)%\Microsoft Visual Studio\2019\<EDITION>\MSBuild\Current\Bin`).

### Release Dispatch

Create a distributable release build:
```
dispatch.bat
```

Requires:
- Conda environment named `realsimdev` with Python >= 3.8
- Runs [dispatchRealSim.py](dispatchRealSim.py) which compiles all components and copies executables/libraries to `build/` folder

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
