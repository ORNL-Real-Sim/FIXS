# DSProxy_smoke — Stage 1 probe for issue #156

Plain Python ↔ PTV `DrivingSimulatorProxy.dll`, no FIXS dependencies.
Empirically characterizes the DLL contract before any TrafficLayer
integration. Output of this probe drives the design decision recorded
in [#156](https://github.com/ORNL-Real-Sim/FIXS/issues/156).

## What it does

1. Loads `DrivingSimulatorProxy.dll` from a chosen VISSIM install
   (2022 or 2026) via `ctypes`.
2. Calls `VISSIM_Connect(versionNo, ...)` against PTV's shipped
   `driving_simulator_test.inpx` (which already has the
   "driving simulator" network-settings option enabled, so we're
   not debugging .inpx config in this probe).
3. Replays the shipped ego trajectory file (`driving_simulator_test.fzp`)
   for N frames, pushing one DS-controlled ego per frame.
4. Reads back `VISSIM_GetTrafficVehicles` and `VISSIM_GetSignalStates`
   every frame, writing both to CSV.
5. Calls `VISSIM_Disconnect` cleanly.

## Run

```powershell
conda activate realsim_dev

cd tests\Vissim\Probes\DSProxy_smoke

# VISSIM 2022 (versionNo=2200)
python smoke_test.py --version 2022 --frames 200 --out-dir out_2022

# VISSIM 2026 (versionNo=2600)
python smoke_test.py --version 2026 --frames 200 --out-dir out_2026
```

VISSIM is started by the DLL via COM; it will open a GUI window.
Per PTV docs §2, the DLL spawns a fresh VISSIM unless one is already
running with `-automation`. The script does **not** explicitly launch
VISSIM in advance — let the DLL spawn it.

## What we're verifying

| # | Check | Pass criterion |
|---|---|---|
| 1 | License has DrivingSimulator add-on | `VISSIM_Connect` returns true |
| 2 | DLL ↔ VISSIM shared-memory works on this box | First frame produces a vehicle list |
| 3 | Ego injection round-trips | A `VISSIM_Veh_Data` entry appears with `CreateID==4711` and `ControlledByVissim==False`, and `VehicleID` becomes non-zero |
| 4 | Signal channel emits | `VISSIM_GetSignalStates` returns ≥1 entry on at least one frame |
| 5 | 2022 ↔ 2026 parity | Both versions produce structurally identical output (same field shapes, same heading convention, comparable vehicle counts) |
| 6 | Clean shutdown | `VISSIM_Disconnect` returns true; no zombie `VISSIM220.exe` / `VISSIM260.exe` in Task Manager |

## Outputs

Under `out_<version>/`:
- `run.log` — per-frame summary (gitignored, regenerated each run)
- `vehicles.csv` — every vehicle returned by `VISSIM_GetTrafficVehicles`, all frames (gitignored)
- `signals.csv` — every signal returned by `VISSIM_GetSignalStates`, all frames (gitignored)
- `network/` — copy of PTV's shipped `.inpx` + companion files, used as VISSIM's working network directory (gitignored)
- **`summary.json`** — compact regression-comparison artifact (**checked in**, see "Regression baseline" below)

Then `python summarize.py out_<version>` reduces the CSVs to `summary.json`
and prints PASS/FAIL on a small set of structural invariants.

## Regression baseline

The `summary.json` files under `out_2022/` and `out_2026/` are
checked-in reference snapshots from the last known-good run. Exact
vehicle counts drift with VISSIM's seeded RNG and per-version car-
following tuning, so the regression criteria in `summarize.py`
test **structural invariants** rather than exact numbers:

| Check | Pass criterion |
| --- | --- |
| `ego_registered` | DS ego appeared in `VISSIM_GetTrafficVehicles` with `ControlledByVissim==False` |
| `ego_unique_id` | VISSIM assigned exactly one `VehicleID` to the ego, never re-assigned |
| `ego_moved_east` | ego X coordinate at last frame > first frame |
| `signals_present` | every frame returned ≥1 signal |
| `signal_state_red_seen` / `_green_seen` | both Red and Green states appeared during the run |
| `vehicles_grow` | total vehicle count at last frame > first (VISSIM spawned background traffic) |
| `leading_id_resolved` | ≥1 vehicle had a non-zero `LeadingVehicleID` (VISSIM is computing leader relations on the DS-controlled vehicle's surroundings) |

Re-run after any FIXS-relevant change (build-system, ProprietaryFiles
update, VISSIM version bump) to catch regressions; rebaseline only after
explicit verification.

## Empirical baseline (last run: 2026-06-06)

| | VISSIM 2022 | VISSIM 2026 |
| --- | --- | --- |
| `VISSIM_Connect` returned | True (19s) | True (25s) |
| ego VISSIM `VehicleID` | 1 | 1 |
| signals per frame | 20 (5 controllers × 4 SGs) | 20 (5 × 4) |
| signal states observed | Red, RedAmber, Green | Red, RedAmber, Green, Amber |
| vehicles at frame 0 / 99 | 7 / 88 | 10 / 91 |
| vehicle types seen | 100, 200, 300, 400 | 100, 200, 300, 400 |
| leading IDs resolved | 2419 / 100 frames | 3970 / 100 frames |
| clean shutdown | yes | yes |

**Same Python wrapper, same script. Only the DLL path and `versionNo`
(2200 vs 2600) differ — empirically confirms the doc-only nature of the
header diff between the two installs.**

## Troubleshooting

If anything fails the soft-reset playbook in repo
[CLAUDE.md](../../../../CLAUDE.md) ("VISSIM 2022 dispatch on Win11 24H2")
likely applies — the DSProxy path uses the same COM machinery.

## Why this directory exists

The dock-point [`tests/Vissim/Ipg/`](../../Ipg/) is reserved for the eventual
CarMaker-VISSIM integrated scenario (#101 Stage 3). This probe directory is
intentionally separate and FIXS-independent so it stays a useful smoke test
even after #101 lands.
