# TrafficLayer_DSProxy — Stage A probe for #158

Drives `TrafficLayer.exe` against a fresh VISSIM 2022 instance via PTV's
`DrivingSimulatorProxy.dll`. No CarMaker, no DriverModel, no FIXS-side
socket traffic. The point is to prove the new
[`CommonLib/VissimDSProxyHelper`](../../../../CommonLib/VissimDSProxyHelper.h)
+ [`TrafficLayer/.../DSProxyMode.cpp`](../../../../TrafficLayer/TrafficLayer/DSProxyMode.cpp)
plumbing end-to-end so Stages B–D (CarMaker, DriverModel coexistence,
signal source) can build on it.

## What it exercises

1. ConfigHelper parses the new `VissimSetup:` block (issue #158).
2. `mainTrafficLayer` dispatches to `FIXS::DSProxy::runDSProxyMode` when
   `VissimSetup.EnableDSProxy: true`.
3. `VissimDSProxy::load` resolves the PTV DLL path from `VissimVersion`,
   loads it via `LoadLibrary`, and binds all entry points.
4. `VissimDSProxy::connect` spawns VISSIM via COM and opens the shared-
   memory channel.
5. Per-tick loop pushes an empty ego array, reads back all vehicles +
   signals.
6. `VissimDSProxy::disconnect` closes VISSIM cleanly on exit.

## Run

```cmd
REM From a developer command prompt (VS 2022 build env is fine):
scripts\dispatch\2_core_components.bat     REM build TrafficLayer.exe

cd tests\Vissim\Probes\TrafficLayer_DSProxy
run_dsproxy_smoke.bat
```

The `.bat` first mirrors PTV's shipped DS example into a writable
`stage_network\` subdirectory (VISSIM writes lockfiles next to the
`.inpx` and the PTV install tree is read-only without admin), then
launches `TrafficLayer.exe -f config.yaml`.

Expected output (≈30 seconds wall clock for 300 ticks):

```
=== DSProxy mode (Stage A, issue #158) ===
VissimVersion:      2022
...
VISSIM_Connect OK
Running 300 ticks (end_time=30.0s at 10 Hz)
tick     0: vehicles=  N signals= 20
tick    25: vehicles=  M signals= 20
...
=== DSProxy mode done (rc=0) ===
```

A VISSIM 2022 GUI window opens; close it manually only after the probe
exits cleanly.

## Empirical baseline (last run: 2026-06-06)

| Check | Result |
| --- | --- |
| `VissimSetup:` parsed | true |
| Dispatch to `runDSProxyMode` reached | true |
| `VissimDSProxy::load` | bound all 9 PTV entry points |
| `VISSIM_Connect` (versionNo=2200) | OK |
| Ticks executed | 300 / 300 |
| Vehicles at tick 0 / 275 | 6 / 264 (VISSIM-internal background traffic grew) |
| Signals per tick | constant 20 |
| `VISSIM_Disconnect` | OK |
| Exit code | 0 |

This matches the Stage 1 `DSProxy_smoke` probe — the C++ port behaves
identically to the Python ctypes probe, as expected.

## Files

| File | Purpose |
| --- | --- |
| `config.yaml` | TrafficLayer config with `VissimSetup.EnableDSProxy: true` |
| `run_dsproxy_smoke.bat` | Stage shipped .inpx + launch TrafficLayer |
| `stage_network/` | Writable mirror of PTV's shipped DS example (gitignored) |

## What is NOT covered yet

Stage A intentionally skips:
- Publishing vehicle/signal data on FIXS application sockets — Stage B
- Ego pose injection from CarMaker / Simulink — Stage B
- Coexistence with FIXS `DriverModel_FIXS.dll` (the line-1280 patch is
  Stage C's ProprietaryFiles change)
- Replacing TrafficLayer's existing COM-based signal enumeration —
  Stage D

When those stages land, this probe stays as the **regression test that
the DSProxy plumbing keeps working in isolation**, independent of the
CarMaker / DriverModel / consumer pipelines.
