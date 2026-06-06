# TrafficLayer_DSProxy_coexist_xil — integrated B+C regression test for #158

The first probe that exercises the **full FIXS stack at once**:

```
python_ego.py            (Python ego — deterministic egoctrl trajectory)
   │
   │ VehFullData_t @ port 2444 (FIXS binary protocol)
   ▼
TrafficLayer.exe         (DSProxy mode, Stage A+B code path)
   │
   │ VISSIM_SetDriverVehicles + GetTrafficVehicles + GetSignalStates
   ▼
VISSIM 2022
   │
   ├── ego (Stage B): pushed via DSProxy, VISSIM-registered with assigned VehicleID
   └── Car type 100 (Stage C): hooked to FIXS DriverModel_RealSim.dll with par-file
       EnableRealSim: false. With PF#6 (ProprietaryFiles#6) merged, the DLL loads,
       processes every callback, returns USE_INTERNAL_MODEL=1 (so VISSIM keeps
       Wiedemann), and opens no socket — the empirical coexistence proof.
```

## What it validates that earlier probes didn't

| Earlier probe | What it proved | What it didn't |
| --- | --- | --- |
| `DSProxy_smoke/` (PR #159) | DSProxy DLL contract on VISSIM 2022/2026 | nothing about TrafficLayer |
| `DSProxy_egoctrl/` (PR #159) | DSProxy honors XIL-style ego control | nothing about TrafficLayer / FIXS protocol |
| `DSProxy_DriverModel_coexist/` (PR #159) | DSProxy + **PTV stock** DriverModel coexist; FIXS DriverModel root cause | nothing about FIXS DriverModel after the patch, nothing end-to-end |
| `TrafficLayer_DSProxy/` (PR #160) | Stage A plumbing — TL spawns VISSIM via DSProxy | no FIXS client, no DriverModel |
| `TrafficLayer_DSProxy_xil/` (PR #161) | Stage B XIL pipeline — Python ego ↔ TL ↔ DSProxy | no DriverModel coexistence; constant-velocity ego only; no invariant checks |
| **this probe** (PR #162) | **Stage B + Stage C, end-to-end, with actual FIXS DriverModel** | — |

## Run

```cmd
scripts\dispatch\2_core_components.bat     REM TrafficLayer.exe
scripts\dispatch\3_vissim_components.bat   REM DriverModel_RealSim.dll
cd tests\Vissim\Probes\TrafficLayer_DSProxy_coexist_xil
run_coexist_xil.bat
```

The `.bat` orchestrates:

1. `patch_inpx.py` mirrors PTV's shipped DS example into a writable `stage_network\` and patches the `.inpx` XML to attach `DriverModel_RealSim.dll` + `coexist_par.yaml` on vehicle type 100 (Car).
2. `TrafficLayer.exe -f config.yaml` launches in DSProxy mode (output captured to `tl.log`).
3. `python_ego.py` connects on port 2444, runs an 85-frame egoctrl trajectory (east/hold/reverse/recovery), checks invariants, writes `out/summary.json`.

## Invariants (all PASS in last run)

| # | Check | Pass criterion | Stage |
| --- | --- | --- | --- |
| 1 | `B_ego_visible_near_pushed_pose` | ≥75% of on-link frames (after the 5-frame snap-in window) have a vehicle within 5m of the pushed ego pose | B |
| 2 | `B_ego_registered_per_tl_log` | TrafficLayer's stdout contains "ego registered: VISSIM VehicleID=..." | B |
| 3 | `B_background_traffic_grew` | vehicle count at last tick > first tick | B |
| 4 | `B_tls_received` | ≥50 ticks completed without a protocol break (any tick with a sane recv_data success proves the wire format aligned) | B |
| 5 | `C_drivermodel_flagged_type_present` | at least one Car (type 100, with FIXS DriverModel hooked) appears in vehicle readback on ≥1 tick — proves the DLL loaded without aborting DSProxy | **C** |
| 6 | `C_drivermodel_init_evidence` | `DriverModelError.txt` exists with non-zero size in `stage_network/` — the DLL writes "Simulation Starts at" at `DRIVER_COMMAND_INIT` | **C** |
| 7 | `C_no_traffic_layer_socket_error` | `DriverModelError.txt` does NOT contain "Error: initialize connection to Traffic Layer" — proves PF#6 successfully gates `socketSetup` on `ENABLE_REALSIM` | **C** |

## Empirical baseline (last run: 2026-06-06)

```
Stage B contracts:
  PASS B_ego_visible_near_pushed_pose      60/60 on-link frames
  PASS B_ego_registered_per_tl_log
  PASS B_background_traffic_grew           vehicles 45 -> 76
  PASS B_tls_received                      85 ticks

Stage C contracts:
  PASS C_drivermodel_flagged_type_present  85/85 ticks had Car (type 100) in readback
  PASS C_drivermodel_init_evidence         DriverModelError.txt size 319 B (multiple entries)
  PASS C_no_traffic_layer_socket_error     no TL connection failure logged

OVERALL: PASS
```

**Why this matters**: prior to this probe, Stage C's PF#6 patch was only empirically validated against PTV's stock DriverModel sample. This probe is the first time the **actual FIXS-built `DriverModel_RealSim.dll`** coexists with DSProxy end-to-end, with TrafficLayer relaying state to a Python client over the FIXS binary protocol. The B′ doctrine is now empirically grounded in the real production DriverModel binary.

## Empirical baseline — VISSIM 2026 (last run: 2026-06-06)

Run via `run_coexist_xil_2026.bat` (stages PTV's shipped DS example from
the 2026 install dir, drives `TrafficLayer.exe` against `config_2026.yaml`
with `VissimDSProxySetup.VissimVersion: 2026`).

```
Stage B contracts:
  PASS B_ego_visible_near_pushed_pose      60/60 on-link frames
  PASS B_ego_registered_per_tl_log         ego registered as VehicleID 11
  PASS B_background_traffic_grew           vehicles 9 -> 96
  PASS B_tls_received                      85 ticks

Stage C contracts:
  PASS C_drivermodel_flagged_type_present  85/85 ticks had Car (type 100) in readback
  PASS C_drivermodel_init_evidence         DriverModelError.txt size 99 B
  PASS C_no_traffic_layer_socket_error     no TL connection failure logged

OVERALL: PASS
```

Baseline JSON: [`out/summary_2026.json`](out/summary_2026.json).

**Conclusion**: the entire #158 stack (Stages A/B/C) runs end-to-end on
VISSIM 2026 with no source changes — only `VissimVersion: 2026` in the
config (selects `versionNo=2600` + the 2026-shipped DSProxy.dll) and a
re-stage from the 2026 install dir's `.inpx`. Both 2022 and 2026 runs
use the same `TrafficLayer.exe` binary and the same FIXS
`DriverModel_RealSim.dll` (PF#6 patch active in both). Vehicle counts
differ across versions because VISSIM's RNG seed and car-following
parameters drift across patch releases — this matches the 2022 vs 2026
observation in the Stage 1 smoke probe
([../DSProxy_smoke/](../DSProxy_smoke/)).

## Files

| File | Purpose |
| --- | --- |
| `config.yaml` | TrafficLayer config for the 2022 run |
| `config_2026.yaml` | TrafficLayer config for the 2026 run |
| `coexist_par.yaml` | DriverModel par-file with `EnableRealSim: false` (same for both versions) |
| `patch_inpx.py` | Stages PTV's `.inpx` (CLI arg `--vissim-version 2022|2026`) and patches `extDriver*` attrs on Car type |
| `python_ego.py` | Ego client + invariant validator + `summary.json` writer (CLI arg `--config`) |
| `run_coexist_xil.bat` | Orchestrates the 2022 run |
| `run_coexist_xil_2026.bat` | Orchestrates the 2026 run |
| `out/summary.json` | 2022 regression baseline — checked in |
| `out/summary_2026.json` | 2026 regression baseline — checked in |
| `stage_network/`, `stage_network_2026/` | Writable mirrors (gitignored, regenerated each run) |
| `tl.log`, `tl_2026.log` | TrafficLayer stdout captures (gitignored) |

## When to re-run

- After any change to `CommonLib/VissimDSProxyHelper.{h,cpp}` (regression guard for Stage A)
- After any change to `TrafficLayer/.../DSProxyMode.cpp` (Stage B publish/receive)
- After any change to ProprietaryFiles' `DriverModel_FIXS_Common.h`, especially anything near `DRIVER_COMMAND_INIT` (Stage C)
- After a VISSIM patch / dependency bump
- Before tagging a 0.9.x release
