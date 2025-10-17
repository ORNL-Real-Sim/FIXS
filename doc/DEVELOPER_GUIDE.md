# Developer Guide

This guide provides comprehensive information about the FIXS repository structure, build process, and usage for both developers/contributors and end-users.

## Table of Contents
- [Repository Structure](#repository-structure)
- [Build Scripts](#build-scripts)
- [Setup Scripts](#setup-scripts)
- [Usage Guide](#usage-guide)

---

## Repository Structure

### Root Level Folders

#### **build/**
Output directory for compiled executables and libraries, including:
- `TrafficLayer.exe` - Central co-simulation synchronization framework executable
- `DriverModel_RealSim.dll` / `DriverModel_RealSim_v2021.dll` - VISSIM driver model DLLs
- MATLAB/Simulink interface files (`.m`, `.mexw64`)

#### **CarMaker/**
CarMaker-specific integration scripts and configuration files for dSPACE/CarMaker co-simulation.

**Python Scripts:**
- `RS_CM_BuildConfig_r2021b.py` - Build configuration script for CarMaker with Matlab R2021b. Configures dSPACE ConfigurationDesk to compile CarMaker HIL executables with Real-Sim libraries. Handles DDE communication with CarMaker GUI.
- `RS_CM_BuildConfig_r2019b.py` - Build configuration script for CarMaker with Matlab R2019b (older version compatibility)
- `RealSimCarMakerSetup.py` - Prepares CarMaker testrun files for Real-Sim co-simulation. Creates traffic objects and signal synchronization tables for SUMO/CarMaker coordination.
- `RealSimSetCarMakerConfig.py` - Parses `config.yaml` and generates CarMaker-specific configuration files for HIL setup

#### **VISSIM/**
VISSIM-specific utility scripts for network setup and data extraction.

**Python Scripts:**
- `setVissimStaticRouteExemption.py` - Exempts specific vehicle classes (default: class 70) from static route decisions, allowing vehicles to drive straight without following routing decisions
- `getVissimSignalTable.py` - Extracts traffic signal information from VISSIM network and exports to CSV table with signal head/group/controller mapping
- `getVissimNetworkInfo.py` - Generates speed limit tables and link geometry data from VISSIM network files. Creates desired speed distribution mappings and adds user-defined attributes (UDA) for position tracking

#### **CommonLib/**
Shared library code and utilities used across all simulators.

**C++ Classes/Headers:**
- `ConfigHelper` - YAML configuration file parser. Reads `config.yaml` and populates simulation, application, XIL, CarMaker, SUMO, and CARLA setup structures
- `SocketHelper` - Cross-platform TCP socket communication wrapper (Windows/Linux)
- `MsgHelper` - Message serialization/deserialization for vehicle, signal, and detector data
- `VehDataMsgDefs` - Vehicle message data structure definitions
- `DeviceV2X` - V2X communication device interface *(not actively maintained)*
- `RealSimCmHelper` - CarMaker-specific helper functions
- `VirEnvHelper` / `VirEnv_Wrapper` - Virtual environment integration helpers
- `TrafficHelper` - Traffic simulation utilities
- `MathLibrary.h` - Mathematical utility functions *(legacy - not currently used)*

**MATLAB/Simulink Files:**
- `RealSimPack.m` - Simulink block that packs vehicle data into bytes and transmits to traffic simulator
- `RealSimDepack.m` - Simulink block that unpacks received traffic data into bus signals
- `RealSimSocket.cpp/.mexw64` - MEX S-function for TCP socket communication in Simulink
- `RealSimInitSimulink.m` - Initialization script for Simulink models
- `RealSimInterpSpeed.m` - Speed interpolation utilities
- `RealSimMsgClass.m` - Message class definition for MATLAB
- `RealSimTemplate.slx` - Simulink template for Real-Sim integration
- `autoRealSimBatchCall.m` / `main_autoRealSim.m` - Batch simulation automation scripts
- `startVissim.m` - MATLAB script to launch VISSIM

**Python Utilities:**
- `ConfigHelper.py` - Python version of configuration parser
- `MsgHelper.py` - Python message handling utilities
- `SocketHelper.py` - Python socket communication
- `VehDataMsgDefs.py` - Vehicle data definitions in Python

**Configuration:**
- `config.yaml` - Main configuration file for all Real-Sim parameters

#### **VirtualEnvironment/**
Virtual environment library for CarMaker integration. Contains Visual Studio solution for building the virtual environment interface used by CarMaker simulations.

#### **TrafficLayer/**
Central co-simulation synchronization framework. Contains Visual Studio solution for building `TrafficLayer.exe`, which serves as the main synchronization framework coordinating message passing between traffic simulators (VISSIM/SUMO) and XIL systems.

#### **VirCarlaEnv/**
CARLA-specific virtual environment implementation.

#### **tests/**
Test scenarios and validation cases. Intended for future automated testing infrastructure.

#### **doc/**
Documentation files including simulator-specific guides:
- `VISSIMdoc.md` - VISSIM interface documentation
- `SUMOdoc.md` - SUMO interface documentation
- `CarMakerDoc.md` - CarMaker interface documentation
- `CARLAdoc.md` - CARLA interface documentation

#### **ProprietaryFiles/**
Proprietary or licensed files not for public distribution. Includes:
- `buildRS_2024a.bat` - Builds Real-Sim MEX files for MATLAB R2024a

#### **.github/**
GitHub-specific configuration (workflows, issue templates, CI/CD).

---

## Dependency Management

### `dependencies.yaml`
Central configuration file tracking external simulator and library versions used by RS_FIXS:
- **SUMO** - Traffic simulation (currently v1.22.0)
- **CARLA** - Virtual environment simulator
- **CarMaker** - IPG Automotive HIL platform
- **yaml-cpp** / **libevent** - C++ libraries

This file serves as the single source of truth for external dependency versions.

### `build_libsumo.ps1`
PowerShell script to build libsumo and libtraci libraries from SUMO source code.

**What it does:**
1. Clones the SUMO repository at the specified version tag
2. Configures the build using CMake with required dependencies
3. Compiles libsumo, libtraci, and all required DLLs
4. Copies built libraries, headers, and DLLs to `CommonLib/libsumo/`
5. Organizes files into `bin/`, `include/`, and `lib/` directories

**Usage:**
```powershell
# Build for SUMO v1.22.0 (default from dependencies.yaml)
.\build_libsumo.ps1

# Build for a specific SUMO version
.\build_libsumo.ps1 -SumoVersion "v1_22_0"

# Build with custom installation location
.\build_libsumo.ps1 -SumoVersion "v1_22_0" -InstallPrefix "C:\custom\path"

# Build with multiple parallel jobs (faster compilation)
.\build_libsumo.ps1 -Jobs 8
```

**Requirements:**
- CMake ≥3.13
- Visual Studio 2022 (or compatible C++ compiler)
- Git
- Python ≥3.8

**Output Structure:**
```
CommonLib/libsumo/
├── bin/           # All runtime DLLs (libsumocpp.dll, libtracicpp.dll, etc.)
├── include/       # Header files (libsumo/*.h, libtraci/*.h)
└── lib/           # Import libraries (*.lib)
```

**Note:** The script preserves the SUMO source clone in `tmp/sumo_build/` for faster rebuilds. Delete this directory to force a clean clone on the next build.

---

## Build Scripts

### `compileExternalLibraries.bat`
Compiles required external dependencies:
- **yaml-cpp** - YAML configuration parser library (builds both Release and Debug versions)
- ~~libevent~~ (commented out - not currently used)

**Requirements:** CMake, Visual Studio 2022

**Usage:**
```batch
compileExternalLibraries.bat
```

### `dispatch.bat`
Main build dispatcher script for creating release builds.

**What it does:**
1. Activates `realsimdev` conda environment
2. Runs `dispatchRealSim.py` to orchestrate the complete build process
3. Deactivates conda environment

**Usage:**
```batch
dispatch.bat
```

### `dispatchRealSim.py`
Python orchestration script that automates the complete build process. Compiles all C++ projects using MSBuild, copies executables to `build/` folder, and prepares the release package.

---

## Setup Scripts

### `checkRequirements.ps1` / `runCheckRequirements.bat`
PowerShell script to verify installation requirements and dependencies. Checks for required software installations (Visual Studio, CMake, MATLAB, etc.).

**Usage:**
```batch
runCheckRequirements.bat
```

### `env.check.py`
Python environment validation script. Verifies that required Python packages and software dependencies are properly installed.

**Usage:**
```bash
python env.check.py
```

### `addVehicleMessage.py`
Utility script to add or modify vehicle message field definitions across the codebase. Updates message structures in C++, Python, and MATLAB implementations.

---

## Usage Guide

### Initial Setup (For Developers)

1. **Install Prerequisites:**
   - Visual Studio 2022 (with C++ development tools)
   - CMake (≥3.15)
   - Python ≥3.8
   - MATLAB (version depending on your needs: R2019b, R2021b, or R2024a)

2. **Setup Python Environment:**
   ```batch
   conda create -n realsimdev python=3.8
   conda activate realsimdev
   pip install -r requirements.txt
   ```

3. **Compile External Libraries:**
   ```batch
   compileExternalLibraries.bat
   ```

4. **Build Project:**
   ```batch
   dispatch.bat
   ```

5. **Verify Build:**
   - Check `build/` folder for executables
   - Ensure `TrafficLayer.exe` and simulator-specific DLLs are present

---

### VISSIM Users

#### 1. Extract Signal Tables
Use this script to generate traffic signal synchronization tables from your VISSIM network:
```bash
python VISSIM/getVissimSignalTable.py
```
- Update `absolute_folder_path` and `file_name` variables in the script
- Output: `SignalTable.csv` with signal head/group/controller mappings

#### 2. Generate Network Information
Extract speed limits and link geometry data:
```bash
python VISSIM/getVissimNetworkInfo.py
```
- Generates:
  - `LinkGeometryTable.csv` - Link topology and geometry
  - `DesSpdDistr2SpeedLimitMap.csv` - Speed distribution to speed limit mapping
  - `SpeedLimitTable.csv` - Complete speed limit table

#### 3. Configure Route Exemptions
Exempt ego vehicle class from static routing decisions:
```bash
python VISSIM/setVissimStaticRouteExemption.py
```
- Default: Exempts vehicle class 70
- Modify `VEHICLE_CLASS_TO_EXEMPT` variable to change target class

---

### CarMaker Users

#### 1. Configure Settings
Edit `CommonLib/config.yaml` with your CarMaker parameters:
- CarMaker project path
- Port numbers
- Ego vehicle ID and type
- Traffic refresh rate

#### 2. Prepare Testrun Files
Generate traffic objects and signal tables for CarMaker:
```bash
python CarMaker/RealSimCarMakerSetup.py --cm-project-path <path> --testrun <name> --car 30 --truck 10
```
- Creates traffic objects that sync with SUMO/VISSIM
- Generates signal synchronization tables
- Options:
  - `--car` / `--truck`: Number of each vehicle type
  - `--no-random-traffic`: Use fixed vehicle models
  - `--overwrite-testrun`: Overwrite original testrun file

#### 3. Generate CarMaker Configuration
Parse config.yaml and create CarMaker-specific settings:
```bash
python CarMaker/RealSimSetCarMakerConfig.py --configFile <path> --cm-project-path <path>
```

#### 4. Build HIL Executable
In dSPACE ConfigurationDesk, run the appropriate build configuration script:
- For MATLAB R2021b: `CarMaker/RS_CM_BuildConfig_r2021b.py`
- For MATLAB R2019b: `CarMaker/RS_CM_BuildConfig_r2019b.py`

This configures compiler settings, search paths, and libraries for Real-Sim integration.

---

### Configuration

The main configuration file is `CommonLib/config.yaml`. Key sections include:

#### SimulationSetup
- `SelectedTrafficSimulator`: Choose VISSIM, SUMO, CarMaker, or CARLA
- `SimulationEndTime`: Total simulation duration
- `EnableVerboseLog`: Enable detailed logging
- `VehicleMessageField`: Define which vehicle data fields to exchange

#### XilSetup
- `EnableXil`: Enable XIL system integration
- `VehicleSubscription`: Define which vehicles to sync
- `SignalSubscription`: Traffic signal synchronization settings

#### ApplicationSetup
- `EnableApplicationLayer`: Enable custom application layer
- `ApplicationPort`: Port numbers for applications
- Vehicle/detector/signal subscriptions

#### CarMakerSetup
- `EnableCosimulation`: Enable CarMaker co-simulation
- `CarMakerIP` / `CarMakerPort`: Network settings
- `EgoId` / `EgoType`: Ego vehicle identification
- `SynchronizeTrafficSignal`: Enable signal sync

#### CarlaSetup
- `EnableCosimulation`: Enable CARLA co-simulation
- `CarlaServerIP` / `CarlaServerPort`: CARLA server connection
- `CarlaMapName`: Map to load
- `InterestedIds`: Vehicle IDs to track

#### SumoSetup
- `EnableAutoLaunch`: Enable automatic SUMO startup (true/false)
- `SumoConfigFile`: Path to SUMO `.sumocfg` file (relative to config.yaml location)
- `NumClients`: Number of TraCI client connections (default: 1)
- `RuntimeLibraryPath`: Optional override for SUMO DLL directory
- `SpeedMode`: SUMO speed control mode (bitfield, see SUMO documentation)
- `ExecutionOrder`: Simulation step execution order for multi-client coordination

**Auto-Launch Example:**
```yaml
SumoSetup:
  EnableAutoLaunch: true
  SumoConfigFile: "./simple_loop.sumocfg"
  NumClients: 1
  SpeedMode: 0
  ExecutionOrder: 0
```

**Note:** When `EnableAutoLaunch: false`, you must manually start SUMO before running TrafficLayer.

Refer to comments in `config.yaml` for detailed parameter descriptions.

---

## Typical Workflow

### Running a Co-Simulation

1. **Configure** `config.yaml` for your scenario
2. **Start TrafficLayer** (central synchronization framework):
   ```
   build/TrafficLayer.exe
   ```
3. **Launch Traffic Simulator** (VISSIM/SUMO) or **Start CarMaker HIL**
4. **Run Simulink Model** (if using XIL with MATLAB)
5. **Monitor** outputs and logs for debugging

### Troubleshooting

- Enable `EnableVerboseLog` in `config.yaml` for detailed diagnostics
- Check `TrafficLayer.err` for error messages
- Review simulator-specific logs (DriverModelError.txt for VISSIM)
- Verify port numbers are not in conflict
- Ensure all components are using the same `config.yaml`

---

## Additional Resources

For simulator-specific details, see:
- [VISSIM Interface](VISSIMdoc.md)
- [SUMO Interface](SUMOdoc.md)
- [CarMaker Interface](CarMakerDoc.md)
- [CARLA Interface](CARLAdoc.md)

For general setup and configuration:
- [Configuration Guide](../ConfigSetup.md)
- [Setup Guide](../setupGuide.md)
