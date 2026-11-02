# RealSim dSPACE Documentation

This page covers running a Real-Sim FIXS client on dSPACE real-time hardware. The
CarMaker side of the same setup (User.c changes, TestRuns, signal-light tables) is in
[CarMakerDoc](CarMakerDoc.md).

## Table of Contents
<!-- TOC -->
* [Scope](#scope)
* [Versions](#versions)
* [RealSim dSPACE library](#realsim-dspace-library)
  * [When you need to build it](#when-you-need-to-build-it)
  * [Build with 5b_carmaker_dspace.ps1](#build-with-5b_carmaker_dspaceps1)
  * [Build by hand with dsmake](#build-by-hand-with-dsmake)
* [CarMaker BuildConfig for dSPACE](#carmaker-buildconfig-for-dspace)
* [Prepare the CarMaker project](#prepare-the-carmaker-project)
* [Configuration file on the target](#configuration-file-on-the-target)
* [ConfigurationDesk project](#configurationdesk-project)
* [SCALEXIO Ethernet for RS Simulink blocks](#scalexio-ethernet-for-rs-simulink-blocks)
* [Run a dSPACE simulation](#run-a-dspace-simulation)
* [Troubleshooting](#troubleshooting)
<!-- TOC -->

## Scope

- **Hardware/tooling:** SCALEXIO, built with ConfigurationDesk. Other dSPACE
  platforms are not covered.
- **What runs on the target:** the RealSim dSPACE library
  (`libRealSimDsLib_<dSPACE>_CM<CarMaker>.a`). It is the same FIXS client code that
  the CarMaker Office/Simulink builds use (`SocketHelper`, `MsgHelper`,
  `VirEnvHelper`, `VirEnv_Wrapper` from `CommonLib/`), cross-compiled for SCALEXIO
  with `RS_DSPACE` defined.
- **What runs on the host PC:** TrafficLayer and the traffic simulator. The target
  connects to TrafficLayer over Ethernet.
- **CarMaker dependency:** `VirEnvHelper.cpp` includes `<CarMaker.h>`, so the
  library is compiled against a specific CarMaker version's headers. There is no
  CarMaker-free dSPACE build today; one library is built per CarMaker version.

The parts that do not depend on CarMaker are the ConfigurationDesk overrun setting,
the target file-system path of the configuration file, and the SCALEXIO Ethernet
setup for RS Simulink blocks.

## Versions

Tool versions come from `dependencies.yaml`:

| Key | Value | Used for |
|---|---|---|
| `development_tools.dspace.version` | `2024a` | Library and BuildConfig file names |
| `development_tools.dspace.product` / `release` | ConfigurationDesk 2024-A / 24.1 | Informational |
| `simulators.carmaker.versions` | `13.1.3`, `11.1.2` | One library per version |
| `development_tools.matlab.version` | `2024a` | Simulink models |

## RealSim dSPACE library

### When you need to build it

A CarMaker-for-dSPACE application that defines `REALSIM` in `User.c` always links
this library. You do **not** need to build it yourself if your release package
already contains it:

- `CommonLib/libRealSimDsLib_<dSPACE>_CM<CarMaker>.a`
- `CarMaker/CM<major>/libRealSimDsLib_<dSPACE>_CM<CarMaker>.a` (a copy next to the
  matching CarMaker executables)

These are only in a release built on a workstation with ConfigurationDesk
installed; `dispatch.bat` skips the step otherwise, and always skips it when
`RS_FIXS_AUTOMATION` is set.

Build it yourself when you change any of the four `CommonLib` sources above, need a
CarMaker version the release does not include, or use a different dSPACE release.

### Build with 5b_carmaker_dspace.ps1

```batch
powershell -ExecutionPolicy Bypass -File scripts\dispatch\5b_carmaker_dspace.ps1
```

The same script runs as step 5b of `dispatch.bat`. What it does:

1. Reads `development_tools.dspace.version` and `simulators.carmaker.versions`
   from `dependencies.yaml`.
2. Finds ConfigurationDesk via `detect_tool_paths.ps1 -Tool dspace`: the
   alphabetically last folder named `dSPACE ConfigurationDesk*` under Program Files.
   The version in `dependencies.yaml` only names the output; it does not select the
   installation.
3. Requires `CFD_vars.bat` and `SCALEXIO\DsBuildLibrary.mk` inside that folder.
4. Uses `C:\IPG\carmaker` as the CarMaker root, overridden by
   `RS_FIXS_CARMAKER_BASE` or `CARMAKER_ROOT` (the latter wins). A CarMaker version
   without `win64-<version>\include` is skipped.
5. For each CarMaker version, runs `dsmake` from `CommonLib/`:

   ```batch
   "<ConfigurationDesk>\CFD_vars.bat" && dsmake -f "<ConfigurationDesk>\SCALEXIO\DsBuildLibrary.mk" ^
     output_filename=RealSimDsLib_<dSPACE>_CM<CarMaker> ^
     source_files="SocketHelper.cpp MsgHelper.cpp VirEnvHelper.cpp VirEnv_Wrapper.cpp" ^
     custom_cpp_options="-std=c++11 -I\"<CarMaker>\include\" -DDSRTLX -DRS_DSPACE -DRS_CAVE -DRS_DEBUG" ^
     target=Dsx86_32
   ```

   Dots in versions become underscores, e.g. `libRealSimDsLib_2024a_CM13_1_3.a`.
   `RS_DEBUG` is always defined; `RS_BUILD_CONFIG` does not apply to this library.
6. Decides success by whether the `.a` file exists, because `dsmake` can return
   non-zero on warnings. The library is moved to `build/CommonLib/`.
7. If CarMaker's `win64-<version>\CM4SL\CM_BuildConfig.py` exists, writes
   `CarMaker/RS_CM<CarMaker>_BuildConfig_<dSPACE>.py` (see
   [CarMaker BuildConfig for dSPACE](#carmaker-buildconfig-for-dspace)).

The script exits 1 if any CarMaker version failed. Run standalone, a missing
ConfigurationDesk or CarMaker version list is an error; run from `dispatch.bat`
(`-RunMode inline`) it is logged as `SKIPPED` in `build_summary.log`. During a full
`dispatch.bat`, each library is also copied to `build/CarMaker/CM<major>/`.

A successful build ends with:

```text
...
x86_64-linux-gnu-ar.exe: creating libRealSimDsLib_2024a_CM13_1_3.a
a - SocketHelper.o64
a - MsgHelper.o64
a - VirEnvHelper.o64
a - VirEnv_Wrapper.o64
library successfully built.
```

### Build by hand with dsmake

Open a command prompt in `CommonLib/`, run `CFD_vars.bat` from the ConfigurationDesk
installation (for 2024-A: `C:\Program Files\dSPACE ConfigurationDesk 2024-A (24.1)`),
then run the `dsmake` command from step 5 with your own paths. dSPACE describes
building user libraries in its FAQ:
https://www.dspace.com/en/inc/home/support/kb/faqs/faq012.cfm.

## CarMaker BuildConfig for dSPACE

CarMaker for dSPACE reads compiler defines and libraries from a `CM_BuildConfig.py`.
`5b_carmaker_dspace.ps1` derives `CarMaker/RS_CM<CarMaker>_BuildConfig_<dSPACE>.py`
from CarMaker's own `CM4SL\CM_BuildConfig.py` with three edits:

- `CARMAKER_DIR` is set to `C:/IPG/carmaker/win64-<version>`. This is fixed and does
  not follow `RS_FIXS_CARMAKER_BASE`/`CARMAKER_ROOT`; edit it if CarMaker is
  installed elsewhere.
- `"RS_CAVE", "RS_DEBUG", "RS_DSPACE"` are prepended to every `CM_DEFINES` list:

  ```python
  if ARCH == 'dsrtlx':
      CM_DEFINES = ["RS_CAVE", "RS_DEBUG", "RS_DSPACE", "DSPACE", "DSRTLX", "_DSRTLX"]
  elif ARCH == 'dsrt64':
      CM_DEFINES = ["RS_CAVE", "RS_DEBUG", "RS_DSPACE", "DSPACE", "DSRT64", "_DSRT64"]
  ```

- The library is appended to the `libs` list:

  ```python
  libs = [ "libdscandrv.so", "libRealSimDsLib_2024a_CM13_1_3.a" ]
  ```

Everything else is CarMaker's template unchanged, including `CM_SRC_DIR`
(`"src_cm4sl"`). Point it at the folder that holds your dSPACE `User.c` and Simulink
model, e.g.:

```python
CM_SRC_DIR = "src_cm4sl_ds"
```

The generated files for the versions in `dependencies.yaml` are committed under
`CarMaker/`; re-running the script overwrites them. In the CM11 example project the
BuildConfig used by ConfigurationDesk is `CM11_proj/ConfigDesk/Python Scripts/CM_BuildConfig.py`.

Instead of editing the Python file you can set the same values in
ConfigurationDesk -> Build Configuration -> Properties:
![CM_DS_BuildConfig_24a.png](img/CM_DS_BuildConfig_24a.png)

## Prepare the CarMaker project

1. Copy `CommonLib/VirEnv_Wrapper.h` and the library for your CarMaker version
   (e.g. `libRealSimDsLib_2024a_CM13_1_3.a`) into `<your CM project>\include`.
2. Apply the `User.c` changes from
   [CarMakerDoc: Compile Source Codes](CarMakerDoc.md#compile-source-codes). The
   only dSPACE difference is the configuration-file path, below.

## Configuration file on the target

With `RS_DSPACE` defined the library does not parse `config.yaml` (no YAML parser on
the target). It reads a flat text file instead, generated on the host by
`RealSimSetCarMakerConfig.py`:

```batch
python CarMaker\RealSimSetCarMakerConfig.py --configFile <config.yaml> --cm-project-path <CM project> [--signal-table-path <RSsignalTable.csv>]
```

This writes `<CM project>\RS_tmp\RealSimCarMakerConfig.txt` (and copies the signal
table next to it). The `TrafficLayerIP` in that file is the address the target
connects to, so it must be the host PC's address on the network SCALEXIO is on.

In `User.c`, `RS_configFile` must point to that file **on the SCALEXIO Linux file
system** (root is `/`):

```cpp
#define REALSIM

#ifdef REALSIM
#include "VirEnv_Wrapper.h"

struct VirEnvHelper* VirEnv_c;

// Replace <your project> with your project location on the target
char* RS_configFile = "/CM_Projects/<your project>/CM13_proj/RS_tmp/RealSimCarMakerConfig.txt";
char* RS_signalTable;
#endif
```

To check the path, use the `msys` tool that ships with CarMaker to telnet into the
SCALEXIO.

## ConfigurationDesk project

1. Set the number of accepted overruns to `-1`:
   ![](img/DS_SCLX_NO_OVERRUN.png)
2. Build the application in ConfigurationDesk.

Example projects (in the private `ProprietaryFiles` submodule): ConfigurationDesk
projects `RS_DS_CM11_SimulinkRS`, `RS_DS_CM11_noSimulinkRS` and
`RS_DS_CM11_noSimulinkRS_useManeuver` under `CM11_proj/ConfigDesk`, with the matching
Simulink models under `CM11_proj/src_cm4sl_ds`.

## SCALEXIO Ethernet for RS Simulink blocks

**Only needed if the Simulink model uses the RealSim Simulink blocks.**

The model needs an initialization script, best called as the model's `InitFcn`:

```matlab
%% define constants
curFilePath = fileparts(mfilename('fullpath'));
RealSimAppPath = fullfile(curFilePath,'..\RealSimApps\SumoIpg');
RealSimPath = fullfile(curFilePath,'..\RealSimRelease');
configFilename = fullfile(RealSimAppPath,'config_SUMO.yaml');

%% add path of RealSim tools
addpath(genpath(RealSimPath))

%% initialize RealSim for Simulink, read yaml file
[VehicleMessageFieldDefInputVec, VehDataBus, TrafficLayerIP, TrafficLayerPort] = RealSimInitSimulink(configFilename);
RealSimPara = struct;
RealSimPara.speedInit = 13.5; % initial speed of the ego vehicle when entering SUMO network
RealSimPara.tLookahead = 0.1; % use 0.1 for external control, recommend to use tLookahead >= 0.2 for SUMO driver
RealSimPara.smoothWindow = 1; % number of moving average data point, 1 essentially mean no moving average
```

![](img/SimulinkInitFcn.png)

Set up the SCALEXIO TCP connections in ConfigurationDesk as below. The
`RS_DS_CM11_SimulinkRS` project and `RS_DS_CM11_SimulinkRS.slx` model show a complete
setup.

![](img/DS_SCLX_Eth_1.png)
![](img/DS_SCLX_Eth_2.png)
![](img/DS_SCLX_Eth_3.png)
![](img/DS_SCLX_Eth_4.png)
![](img/DS_SCLX_Eth_5.png)

## Run a dSPACE simulation

1. Check that `RS_configFile` in `User.c` is correct for your environment.
2. Open the ConfigurationDesk project `RS_DS_CM11_noSimulinkRS_useManeuver` in
   `CM11_proj/ConfigDesk`. Its Simulink model is
   `CM11_proj/src_cm4sl_ds/RS_DS_CM11_noSimulinkRS_useManeuver.slx`. Build the
   project.
3. Start one of the SUMO applications under `tests`:
   - **SignalIpg** shows signal-light synchronization between SUMO, CarMaker and
     dSPACE. Fix the paths in `runShallowfordRd_release.bat` and run it.
   - **SumoIpg** shows CarMaker maneuvers that ramp up and down for HIL use. Fix the
     paths in `runSumoIpg_release.bat` and run it.
4. Start CarMaker and select project `CM11_proj`:
   - **SignalIpg**: TestRun `RS_Shallowford_sumo_signal`
   - **SumoIpg**: TestRun `merge_RS_noSimulinkRS_useManeuver`
5. Start the CarMaker simulation.

After building once you can switch SUMO applications or CarMaker TestRuns without
rebuilding, as long as the Simulink model is unchanged.

## Troubleshooting

- **`Could not auto-detect dSPACE installation`** — no `dSPACE ConfigurationDesk*`
  folder under Program Files. With several installed, the alphabetically last one
  is used.
- **`dSPACE environment file not found` / `dSPACE makefile not found`** — the
  detected installation lacks `CFD_vars.bat` or `SCALEXIO\DsBuildLibrary.mk`.
- **`CarMaker include path not found ... skipping`** — CarMaker `win64-<version>`
  is not under the CarMaker root; install it or set `RS_FIXS_CARMAKER_BASE`.
- **`build FAILED ... output file not found`** — `dsmake` produced no `.a`. When run
  through `dispatch.bat` the compiler output is in `scripts/dispatch/build.log`;
  standalone it is printed to the console.
- **Link errors for `libRealSimDsLib_*`** — the library's CarMaker version must match
  the CarMaker version of the project, and the `.a` must be listed in `libs` of the
  BuildConfig and present in `<your CM project>\include`.
- **CarMaker log shows `RealSim: Read RealSimCarMakerConfig.txt failed`** —
  `RS_configFile` does not point to an existing file on the target; check it with
  `msys`/telnet.
- **No connection to TrafficLayer** — `TrafficLayerIP` in `RealSimCarMakerConfig.txt`
  is not reachable from SCALEXIO (e.g. `127.0.0.1`), or the host firewall blocks
  TrafficLayer.
- **Overrun errors** — set the accepted overruns to `-1` in ConfigurationDesk.
