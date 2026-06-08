# DSProxy_DriverModel_coexist — Stage 2 coexistence test for #156

Answers the load-bearing question for the **B/B′ design path**: can a
single VISSIM 2022 instance run both `DrivingSimulatorProxy.dll` (DSProxy)
and a per-vehicle-type `DriverModel.dll` against the same `.inpx`?

Scope: **VISSIM 2022 only.** Same scope narrowing as the other Stage probes.

## Modes

The test driver patches PTV's shipped DS example `.inpx` to flag vehicle
type 100 (Car) with an `ExtDriver` hook, then runs the Stage 1 DSProxy
loop against it.

| `--mode`            | What's attached to Car (type 100)                                          | Purpose |
| ------------------- | -------------------------------------------------------------------------- | --- |
| `no_drivermodel`    | nothing (control)                                                          | confirms our test infra produces the same result as Stage 1 when nothing is patched |
| `ptv_stock`         | PTV's shipped `DriverModel.cpp` sample, built for x64                      | isolates whether **any** working DriverModel DLL can coexist with DSProxy |
| `bogus_dll`         | `extDriver=true` with a path that points to a non-existent DLL             | isolates whether the `ExtDriver` attribute alone breaks DS, or whether it's the load failure |
| `real_drivermodel`  | FIXS-built `DriverModel_RealSim.dll`                                       | the actual question — does FIXS's DriverModel coexist? |

## Empirical results (last run: 2026-06-06, VISSIM 2022)

| Mode                | `VISSIM_Connect` | Time  | Interpretation |
| ------------------- | ---------------- | ----- | -------------- |
| `no_drivermodel`    | **True**         | 13.3s | baseline — same as Stage 1 |
| `ptv_stock`         | **True**         | 13.5s | **DSProxy + DriverModel CAN coexist in principle** |
| `bogus_dll`         | False            | 14.0s | missing DLL causes VISSIM to abort the DS handshake |
| `real_drivermodel`  | False            | 20.0s | FIXS DriverModel's startup blocks the DS handshake (~7s overhead) |

## Diagnosis of the FIXS-specific failure

In `ProprietaryFiles/VISSIMserver/Common/DriverModel_FIXS_Common.h`, the
`DRIVER_COMMAND_INIT` handler (~line 1280) runs:

```cpp
// THIS IS UNCONDITIONAL — NOT GATED BY ENABLE_REALSIM
selfServerPortUserInput.push_back(Config_c.SimulationSetup.TrafficSimulatorPort);
Sock_c.socketSetup(serverAddr, selfServerPortUserInput);
Sock_c.disableServerTrigger();

if (ENABLE_REALSIM) {           // only THIS block is gated
    if (Sock_c.initConnection(...) < 0) {
        MessageBox(...);
        ...
    }
}
```

So with `EnableRealSim: false` set in the par file, the modal `MessageBox`
on connect-failure is correctly avoided — but `Sock_c.socketSetup(...)`
runs unconditionally. That call sets up TCP machinery against the
configured `TrafficSimulatorPort`. When there's no TrafficLayer
listening, that setup blocks/retries long enough (~7 s) for VISSIM's
own DS-handshake watchdog to give up. VISSIM then cancels the DSProxy
connection, which is what we observe from the Python side.

`DriverModelError.txt` confirms this read: it contains exactly one entry
("Simulation Starts at …"), written before `socketSetup` returns. No
"Error: get configuration file failed", no "Error: initialize connection
to Traffic Layer failed" — Config_c parsed our par file cleanly, the
failure happens in the unconditional socket block.

## Fix shape (for #156 / #101 implementation)

Single-file change in `DriverModel_FIXS_Common.h`:

```cpp
if (ENABLE_REALSIM) {
    selfServerPortUserInput.push_back(Config_c.SimulationSetup.TrafficSimulatorPort);
    Sock_c.socketSetup(serverAddr, selfServerPortUserInput);
    Sock_c.disableServerTrigger();

    if (Sock_c.initConnection(...) < 0) {
        ...
    }
}
```

Once that's in, B/B′ becomes empirically achievable on VISSIM 2022:
- FIXS DriverModel attached to behavior-modifier vehicle types
- DSProxy attached to the FIXS-side TrafficLayer for the ego(s)
- Both coexist in one VISSIM instance against one `.inpx`

(That fix lives in ProprietaryFiles, so it's a dual-PR per the repo
CLAUDE.md workflow — not done here; that's the #156 implementation work
in `ProprietaryFiles/dev`.)

## Run

```powershell
conda activate realsim_dev
cd tests\Vissim\Probes\DSProxy_DriverModel_coexist

# all four modes
python coexist_test.py --mode no_drivermodel  --frames 50
python coexist_test.py --mode ptv_stock       --frames 50
python coexist_test.py --mode bogus_dll       --frames 50
python coexist_test.py --mode real_drivermodel --frames 50
```

Each run writes to `out_2022_<mode>/`. The `network/` subdir contains
the patched `.inpx` for that mode plus the `DriverModelLog.txt` /
`DriverModelError.txt` that VISSIM accumulates.

## Files

- `coexist_test.py` — multi-mode test driver, reuses `DSProxy_smoke/dsproxy_wrapper.py`
- `coexist_par.yaml` — par-file for the FIXS DriverModel, `EnableRealSim: false`
- `PTV_stock_drivermodel.dll` — built from PTV's shipped sample (see "Building the stock DLL" below)
- `results_matrix.json` — checked-in numeric snapshot of the four-mode matrix
- `.gitignore` — bulky per-run outputs stay local

## Building `PTV_stock_drivermodel.dll`

PTV ships `DriverModel.cpp/.h/.vcxproj` under
`C:\Program Files\PTV Vision\PTV Vissim 2022\API\DriverModel_DLL\`.
The shipped `.vcxproj` is `Win32` only, so we build x64 manually:

```powershell
$src = "C:\Program Files\PTV Vision\PTV Vissim 2022\API\DriverModel_DLL"
$out = (pwd).Path  # this probe dir

Copy-Item "$src\DriverModel.cpp" .
Copy-Item "$src\DriverModel.h"   .

# x64 dev shell required (vcvars64.bat)
cl /LD /O2 /DDRIVERMODEL_EXPORTS DriverModel.cpp `
   /Fe:PTV_stock_drivermodel.dll user32.lib gdi32.lib advapi32.lib
```

The DLL is committed because it's a small (~100 KB), benign,
PTV-redistributable passthrough; rebuilding it from a different VS
toolset shouldn't change behavior, but if you do, drop the binary in
this directory next to the test driver.
