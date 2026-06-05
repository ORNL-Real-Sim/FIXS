# Vissim/SpeedLimitLite

Python-only twin of `../SpeedLimit/`. Same VISSIM network, same ego, same
config — but no MATLAB, no Simulink, no `.mat` golden-reference
comparison.

**Status:** working end-to-end against `TrafficLayer.exe` +
`DriverModel_RealSim.dll` on VISSIM 2022. Ego (type=1000) injected at
t=11.5 s, streams 100 ms FIXS samples for the remainder of the
simulation to `speed_limit_lite_trace.{csv,parquet}`.

## Differences from `../SpeedLimit/`

| Concern | SpeedLimit (existing) | SpeedLimitLite |
|---|---|---|
| VISSIM bootstrap | `startVissim.m` (MATLAB COM) | `start_vissim.py` (Python `win32com.client`) |
| Test orchestration | `runSpeedLimit_test1.m` | `run_speed_limit_lite.bat` |
| Client | `speedLimitClient.slx` (Simulink + MEX socket) | `speed_limit_client.py` |
| Validation | byte-exact compare to `speedLimitTest1_orig.mat` | `compare_traces.py` against `*.parquet` |
| Dependencies | MATLAB + Simulink + VISSIM | Python (`pywin32`, `pandas`, `pyarrow`) + VISSIM |
| Wall time | ~5+ min | ~90 s |

## What's shared with SpeedLimit

- VISSIM network files (`../networks/speedLimit/speedLimit.{inpx,layx}`)
  are loaded by relative path. **Do not duplicate** — they are network
  artifacts owned by `tests/Vissim/networks/`.
- Speed-limit/signal-table CSVs (`LinkGeometryTable.csv`,
  `SpeedLimitTable.csv`, `SignalTable.csv`, `DesSpdDistr2SpeedLimitMap.csv`,
  `Routes.yaml`) also live in `../networks/speedLimit/` — the FIXS
  driver-model DLL reads them from the .inpx's directory.
- Ego injection parameters (type 1000, link 1, lane 1, entry time
  11.5 s, initial 18 m/s, desired 20 m/s) match `startVissim.m` exactly.
- `VehicleMessageField` set matches `config_test1.yaml`.

## Run

```
run_speed_limit_lite.bat
```

By default the bat hooks `start_vissim.py` at VISSIM 2022 (ProgID
`VISSIM.Vissim.2200`). On a dev machine that only has VISSIM 2026
installed, run the Python helper directly with the ProgID override:

```
python start_vissim.py --progid VISSIM.Vissim.2600
```

or set `VISSIM_PROGID=VISSIM.Vissim.2600` in the environment.

Runtime outputs (gitignored, in this folder):

- `speed_limit_lite_trace.csv` — human-readable trace
- `speed_limit_lite_trace.parquet` — typed, compressed (~10× smaller)

## Compare against the blessed baseline

Each VISSIM major version produces a different trace (different RNG,
different per-step processing order, sometimes a different ego
trajectory through the network). Two blessed CSVs are committed in
this folder:

- `speed_limit_lite_orig_v2022.csv` — produced by VISSIM 2022.00-13
- `speed_limit_lite_orig_v2026.csv` — produced by VISSIM 2026

Pick the one matching the VISSIM you just ran with:

```
conda activate realsim_dev
python compare_traces.py --vissim-version 2022   # default
python compare_traces.py --vissim-version 2026
```

If you need to compare against the *historical* SpeedLimit Simulink
references for any reason (different stack — Simulink ego controller
at 1 ms vs our Python observer at 100 ms), point `--ref` explicitly:

```
python compare_traces.py --ref ../SpeedLimit/speedLimitTest1_orig.parquet
```

## Reference data: why Parquet, not .mat

The committed `speedLimitTest{1,2,3}_orig.mat` files are
`Simulink.SimulationOutput` opaque objects:

- **Unreadable from Python.** `scipy.io.loadmat`, `h5py`, `hdf5storage`,
  and `pymatreader` all fall back to a `MatlabOpaque` blob you can't
  introspect without MATLAB.
- **One ecosystem only.** Diffing two `.mat` files in `git`, comparing
  values across machines, or building a CI gate on the trace is hard.

The matching `speedLimitTest{1,2,3}_orig.parquet` files (committed
alongside the `.mat`) were generated from the `.mat` by
`archive/export_mat_to_csv.m` running in MATLAB, then converted to
Parquet from Python. The conversion preserves the full 34-column
`VehDataBus` schema and all 90 000–120 000 sample rows.

| File | `.mat` | `.parquet` | ratio |
|---|---:|---:|---:|
| speedLimitTest1 | 619 KB | 452 KB | 1.4× |
| speedLimitTest2 | 467 KB | 343 KB | 1.4× |
| speedLimitTest3 | 462 KB | 340 KB | 1.4× |

(The `.mat` files are already compressed by MATLAB; Parquet's win is
*readability*, not size — both are small for these traces. For raw CSV
the Parquet wins are 45×.)

Parquet was chosen because:

- **Typed**: floats stay floats, ints stay ints (`signalLightHeadId` is
  not a stringified float). Loss-free unlike CSV.
- **Compressed**: zstd column compression. Cheap to read partially
  (column-pruning during `compare_traces.py`).
- **Language-agnostic**: native pandas (`read_parquet`), MATLAB
  R2024a+ (`parquetread`/`parquetwrite`), Julia, R, DuckDB.
- **`git diff`-friendly via `parquet-tools cat`**: text-inspectable
  without re-encoding the binary.

## Why bit-exact reproduction of the `.mat` is hard

I ran the original MATLAB test (`runSpeedLimit_test1.m`) and it fails
on a Simulink port-dimension mismatch:

```
'Output Port 1' of 'speedLimitClient/Signal Specification' is [200x1].
'Input Port 4' of 'speedLimitClient/S-Function' is 1024 elements.
```

The `.slx` model's hardcoded message-buffer width has drifted from the
current `VehicleMessageField` length. The reference `.mat` files were
generated in Nov 2022 from a working state that no longer compiles
unchanged. SpeedLimitLite reproduces the **environment** (same .inpx,
same ego config, same routing tables) but produces a different ego ID
(15 instead of 6 because of background traffic differences across
VISSIM versions). Speed-limit *values* agree exactly (13.89, 22.22,
19.44 m/s correspond to 50/80/70 km/h zones); *timing* of transitions
shifts because the two egos travel different routes through the
network.

`compare_traces.py` will report per-field max/mean diffs — useful for
spotting structural regressions even when the runs aren't bit-equal.
Tightening tolerances is appropriate once the original MATLAB test is
restored to a passing state and we can re-bless the reference.

## Prerequisites

- VISSIM 2022 installed and licensed
- `TrafficLayer.exe` + `DriverModel_RealSim.dll` built
  (`scripts/dispatch/{2_core_components,3_vissim_components}.bat`)
- Conda env `realsim_dev` with `pywin32`, `pandas`, `pyarrow`
  (`pip install pywin32 pandas pyarrow`)
