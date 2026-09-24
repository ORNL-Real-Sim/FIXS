# Flexible Interface for XIL Simulation (FIXS)

## FIXS Contacts
**Primary Maintainer**: University of Georgia\

Oak Ridge National Lab PIs
Max Chen\
chenm@ornl.gov\
Anye Zhou\
zhoua@ornl.gov


## About
This interface is part of the **Real-Sim** project funded by Department of Energy (DOE)-Vehicle Technology Office (VTO)-Energy Efficient Mobility Systems (EEMS) program. It is part of the core tools of EEMS.

Real-Sim develops a multi-resolution X-in-the-loop (XIL) simulation framework to support inclusive testing and evaluation of emerging technologies such as connected and automated vehicles. The Real-Sim framework connects different simulation components and integrates various traffic and vehicle simulation tools, virtual environments, and XIL systems through a Flexible Interface for XIL Simulation (FIXS). Connections among different tools are transparent to users and handled by the interface behind the scenes. Co-simulation of various vehicle and traffic simulation tools with different XIL systems can be easily achieved and become a transparent "plug-and-play" process to users. 

**Real-Sim Project Lead**: Oak Ridge National Lab\
**Real-Sim Project Team**: University of Georgia, University of Texas-Dallas

**Links to current project presentation**\
https://www1.eere.energy.gov/vehiclesandfuels/downloads/2024_AMR/EEMS101_Chen_2024_o.pdf \
https://www1.eere.energy.gov/vehiclesandfuels/downloads/2023_AMR/eems101_Shao_2023_o_v2%20-%20Yunli%20Shao.pdf_ADW373C.tmp.pdf

**Links to older project presentation**\
https://www1.eere.energy.gov/vehiclesandfuels/downloads/2022_AMR/eems101_deter_2022_o_5.1_651pm_LS.pdf
https://www1.eere.energy.gov/vehiclesandfuels/downloads/2022_AMR/eems067_deter_2022_o_5.12_1157am_TM.pdf
https://www.energy.gov/sites/default/files/2021-06/eems101_deter_2021_o_5-19_645pm_LR_ML.pdf

Publication list is upcoming...

# Other links
This README contains general information of the interface. For specific documentation of different simulators, check the following links:
* ### [Build Instructions](doc/BUILD.md) - Comprehensive build system documentation
* ### [Developer Guide](doc/DEVELOPER_GUIDE.md) - Repository structure, build scripts, and usage guide
* ### [VISSIM Interface](doc/VISSIMdoc.md)
* ### [SUMO Interface](doc/SUMOdoc.md)
* ### [CarMaker](doc/CarMakerDoc.md)
* ### [CARLA](doc/CARLAdoc.md)

Current software versions used in FIXS development
CM13.1.3, Carla 0.9.15, VISSIM 2022, SUMO 1.21, dSPACE/Matlab 2024a


# Table of Contents
* [Preface](#preface)
* [Build](#build)
   * [Prerequisite](#prerequisite)
   * [Dispatch a Release](#dispatch-a-release)
* [General Setups](#general-setups)
* [Compatibility to Previous Versions](#compatibility-to-previous-versions)
    * [RealSimDepack Output](#realsimdepack-output)
* [VISSIM and SUMO comparison](#vissim-and-sumo-comparison)
* [Troubleshooting](#troubleshooting)
* [Appendix](#appendix)
    * [Vehicle Message Field Specifications](#vehicle-message-field-specifications)

## Preface
Before testing the Real-Sim interface, please 
- read through this README
- review the [configuration reference](ConfigSetup.md) for a full description of every YAML option
- check comments in config.yaml for quick reminders on common fields
- check annotations in the Simulink template which specifies how to properly use each block
- RealSimPack.m and RealSimDepack.m files are currently open-source. check the comments before modifying.


## General Setups
The interface runs the connections to different software, simulators by itself to provide plug-in-and-play experience for the users. A config.yaml file is critical to setup the interface parameters and configure different scenarios. 

By default the executable looks for the SUMO runtime under `CommonLib/libsumo/bin` relative to the repository. When running from `TrafficLayer/x64/<Config>` or `build`, keep that folder with the executable or set `SumoSetup.RuntimeLibraryPath` in the configuration file.

`CommonLib/libsumo` is **not** checked into git — it is fetched during first-run setup (`scripts\initialize_fixs.ps1`, also run automatically by `dispatch.bat`). If the directory is missing, run that first. See [doc/BUILD.md — First-Run Setup](doc/BUILD.md#first-run-setup-fresh-clone).

### Warm-up (SUMO only)

XIL testing usually cares about the simulation only from the moment the ego is on the
road, but reaching that moment means simulating minutes of background traffic first. A
**warm-up** runs the traffic simulator up to that moment with the FIXS boundary closed:
no message reaches any client, and no client is accepted yet, so CARLA's map load and
CarMaker's start-up overlap the warm-up instead of queueing ahead of it. The sockets are
bound and listening throughout, so a client's `connect()` behaves exactly as before — it
just receives nothing until the warm-up ends.

Two mutually exclusive triggers, chosen by who knows the entry time:

| Key | Warm-up ends when | Use when |
| --- | --- | --- |
| `WarmUpUntilEgoEntry: true` | the first subscribed ego is in the network | a controller inserts the ego — it keeps owning the entry time, and no number is duplicated in the config |
| `WarmUpTime: <sim time>` | simulation time reaches that **absolute** value, reached in one batch step | nothing else inserts the ego, i.e. the CarMaker path where TrafficLayer injects it |

Setting neither means no warm-up (sync from the first step). Setting both warns and uses
ego entry — a batch step cannot also watch for an ego arriving part-way through.

For `WarmUpUntilEgoEntry`, the ids watched are the union of the by-id vehicle
subscriptions in `ApplicationSetup` and `XilSetup`, and with several egos the *first* to
arrive ends the warm-up: an ego on the road while the boundary is still closed would be
driven by the traffic model instead of by its XIL.

See [doc/ConfigSetup.md](doc/ConfigSetup.md#warm-up) for the full reference, including
the migration from the removed `SimulationMode` / `SimulationModeParameter` keys (#86).

## Build

Real-Sim FIXS uses a modular build system with automated tool detection. For detailed build instructions, see **[doc/BUILD.md](doc/BUILD.md)**.

### Quick Start

**Prerequisites:**
- Visual Studio 2022 (Community, Professional, or Enterprise)
- CMake (version 3.10 or higher)
- Optional: MATLAB 2024a, CarMaker 13.1.3/11.1.2, dSPACE ConfigurationDesk 2024-A

**Full Release Build:**
```batch
dispatch.bat
```

This single command automatically:
- Detects installed tools (Visual Studio, MATLAB, dSPACE, CarMaker)
- Reads versions from `dependencies.yaml`
- Builds all components (TrafficLayer, VISSIM DLLs, CarMaker executables, dSPACE libraries, MEX files)
- Packages everything into the `build/` directory

**Development Builds:**

For faster iteration during development, build individual components:
```batch
# Core components only
scripts\dispatch\2_core_components.bat

# VISSIM components only
scripts\dispatch\3_vissim_components.bat

# CarMaker components only (auto-generates BuildConfig files)
powershell -ExecutionPolicy Bypass -File scripts\dispatch\4a_carmaker_components.ps1
```

For complete build system documentation including Debug/Release configurations, troubleshooting, and architecture details, see **[doc/BUILD.md](doc/BUILD.md)**.

## Compatibility to Previous Versions
Here are potential compatibility issues with different versions

### RealSimDepack Output
Instead of multiple outputs, now RealSimDepack will output one single bus signal,

## VISSIM and SUMO comparison
To be filled...

## Troubleshooting
- If in any case an issue occurs when using the interface, first try to restart VISSIM and MATLAB and try again.

- If issue persists, turn on the EnableVerboseLog in the config.yaml, enable log of the RealSimSocket s-function, repeat the simulation that causes the issue. Then locate the following logs and error reports and check them (some of the logs will be overwritten when a new simulation starts): 
    - TrafficLayer.err in the directory of TrafficLayer.exe
    - Copy/Screenshot the outputs displaying from the cmd window running the TrafficLayer.exe
    - DriverModelError.txt and DriverModelLog.txt in the directory of the VISSIM network file
    - Copy/Screenshot outputs from the 'Diagnostic Viewer' of the Simulink. 

## Appendix
### Vehicle Message Field Specifications
This is a full list of vehicle message field
```cpp
std::string id; // VISSIM id is integer and will be converted to string, e.g., (int) 8 ==> (string) "8"
std::string type; // VISSIM type is integer and will be converted to string, e.g., (int) 100 ==> (string) "100"
std::string vehicleClass; // vehicle class, such as car, truck in VISSIM; private, passenger, truck in SUMO
float speed; // m/s
float acceleration; // m/s^2
float positionX; // m, position based on the coordinates defined in VISSIM/SUMO
float positionY; // m
float positionZ; // m
float heading; // heading in degree, north is 0 degree, then increasing clockwise. i.e., east is 90 degree.
uint32_t color;  // combined bits in order of r,g,b,a, each is 8-bit, i.e., leftmost 8-bit is r
std::string linkId; // VISSIM linkId is integer and will be converted to string, e.g., (int) 10001 ==> (string) "10001"
int32_t laneId; // lane id convention is consistent to VISSIM, with rightmost lane to be 1
float distanceTravel; // m, cumulative driving distance of each vehicle since entering VISSIM network
float speedDesired; // m/s
float accelerationDesired; // m/s^2
int8_t hasPrecedingVehicle; // 1: has preceding vehicle, 0: no preceding vehicle
std::string precedingVehicleId; // 
float precedingVehicleDistance; // m, distance to preceding vehicle (front pump of ego to rear pump of front), -1 if no preceding vehicle
float precedingVehicleSpeed; // m/s, absolute speed of preceding vehicle, -1 if no preceding vehicle
string signalLightId; // id of signal controller
int32_t signalLightHeadId; // index of signal head
float signalLightDistance; // m, distance to next signal light, no nearby signal light = -1;
int8_t signalLightColor; // color of next signal light, red = 1, amber = 2, green = 3, red / amber = 4, amber flashing = 5, off = 6, other = 0, no nearby signal light = -1;
float speedLimit; // m/s, VISSIM speed limit is defined in terms of reduced speed area, desired speed decision. If neither exists, this value will be -1. Similarly for speedLimitNext.
float speedLimitNext; // m/s, -1 if no speed limit ahead. 
float speedLimitChangeDistance; // m, -1 if no speed limit ahead.
std::string linkIdNext; // 
float grade; // angle in radians, positive climbing up hill
int8_t activeLaneChange; // 1 to the left, -1 to the right, 0 stay on the lane, 
```

<!--
<img src="docs/images/Flowchart.PNG" align="middle" width="500"/>
--[]([url](url))>
