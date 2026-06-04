# Vissim/SpeedLimitLite

Python-only twin of `../SpeedLimit/`. Same VISSIM network, same ego, same
config — but no MATLAB, no Simulink, no `.mat` golden-reference comparison.

**Status:** intentionally ugly. Meant as a fast iteration loop while
working on FIXS↔VISSIM interface changes. Will eventually be merged with
`../SpeedLimit/` (or absorbed into a unified Python/MATLAB runner).

## Differences from `../SpeedLimit/`

| Concern | SpeedLimit (existing) | SpeedLimitLite |
|---|---|---|
| VISSIM bootstrap | `startVissim.m` (MATLAB COM) | `start_vissim.py` (Python `win32com.client`) |
| Test orchestration | `runSpeedLimit_test1.m` | `run_speed_limit_lite.bat` |
| Client | `speedLimitClient.slx` (Simulink) | `speed_limit_client.py` |
| Validation | byte-exact compare to `speedLimitTest1_orig.mat` | dumps CSV trace, manual inspection |
| Dependencies | MATLAB + Simulink + VISSIM | Python (`pywin32`) + VISSIM |
| Wall time | ~5+ min | ~2 min |

## What's shared

- VISSIM network files (`../networks/speedLimit/speedLimit.{inpx,layx}`)
  are loaded by relative path. **Do not duplicate** — they are shared with
  `../SpeedLimit/`.
- Ego injection parameters (type 1000, link 1, lane 1, entry time 11.5s,
  initial 18 m/s, desired 20 m/s) match `startVissim.m` exactly.
- `VehicleMessageField` set matches `config_test1.yaml`.

## Run

```
run_speed_limit_lite.bat
```

Output: `speed_limit_lite_trace.csv` with per-step ego speed, speedLimit,
speedLimitNext, signal state, preceding-vehicle distance.

## Prerequisites

- VISSIM 2022 installed and licensed
- `TrafficLayer.exe` built
- Conda env `realsim_dev` with `pywin32` (`pip install pywin32`)
