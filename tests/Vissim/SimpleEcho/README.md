# Vissim/SimpleEcho

VISSIM counterpart of `tests/Python/SimpleEchoClient/`. Minimal sanity
check for the FIXS↔VISSIM interface: vehicles flow in a square loop,
TrafficLayer relays their state to a Python subscriber that echoes
the messages back.

```
VISSIM (driver-model DLL) <--1337--> TrafficLayer.exe <--2444--> simple_echo_client.py
       ^
       | loaded via COM by start_vissim.py
```

## Files

| File | Role | Mirrors SUMO file |
|---|---|---|
| `simple_loop.xodr` | OpenDRIVE geometry (4-edge square loop, from netconvert) | `simple_loop.net.xml` |
| `simple_loop.inpx` / `.layx` | VISSIM network + layout with 600 veh/h input baked in. **Not committed** — generate locally via `archive/setup_network.py`. | — |
| `config.yaml` | TrafficLayer + application-layer config | `config.yaml` |
| `start_vissim.py` | Launches VISSIM via COM, loads `.inpx`, runs simulation | (TrafficLayer auto-launches sumo) |
| `simple_echo_client.py` | Subscribes to TrafficLayer, echoes vehicle data | `simple_echo_client.py` |
| `run_simple_echo.bat` | Orchestrates the three pieces | `run_simple_echo_client.bat` |
| `archive/` | Local-only one-time setup scripts (gitignored) | — |

## Setup (one-time, fully headless from your terminal)

`simple_loop.xodr` is checked in. `simple_loop.inpx` + `.layx` are **not**
committed — generate them locally:

```
conda activate realsim_dev
python tests/Vissim/SimpleEcho/archive/setup_network.py
```

That script:
1. Dispatches VISSIM 2022 via COM (`VISSIM.Vissim.2200`)
2. `vissim.New()` → blank network
3. `vissim.ImportOpenDrive(simple_loop.xodr)` → 16 links from the SUMO loop
4. Adds a vehicle input (600 veh/h, composition 2 = *Car only, 50 km/h*)
   on the first non-connector link
5. `SaveNetAs(simple_loop.inpx)` + `SaveLayout(simple_loop.layx)`

The `archive/` folder is gitignored (`**/archive/*` in root `.gitignore`)
so `setup_network.py` stays local — it's scaffolding, not a test artifact.

**Why not commit the generated .inpx?** The .inpx is the
PTV-VISSIM-version-stamped output of `SaveNetAs`, so generating it with
2026 produces a file that 2022 will refuse to load. Generating with
2022 produces a file that loads in **both** 2022 and 2026 (VISSIM is
backward-compatible but not forward-compatible). Until we either
standardize the dev install or build a version-aware regenerator into
CI, it's cleaner to keep the .inpx out of the repo and have each dev
regen locally with their installed VISSIM.

To regenerate `simple_loop.xodr` from the SUMO source:

```
cd tests/Vissim/SimpleEcho
netconvert -s ../../Python/SimpleEchoClient/simple_loop.net.xml \
           --opendrive-output simple_loop.xodr
```

## Running

```
run_simple_echo.bat
```

Or manually in three shells:

```
..\..\..\TrafficLayer\x64\Release\TrafficLayer.exe -f config.yaml
python start_vissim.py
python simple_echo_client.py
```

Expected: echo client prints `Step N | Time: T.TTs | Vehicles: K` every
100 steps with K > 0 once vehicles populate the loop.

## Prerequisites

- VISSIM 2022 installed and licensed (2026 also supported — see Notes)
- Built `TrafficLayer.exe` (`dispatch.bat` at repo root)
- Conda env `realsim_dev` with `pywin32` installed
- For network regeneration: SUMO with `netconvert` on PATH

## Notes

- **VISSIM version & ProgID.** Repo targets VISSIM 2022 (ProgID
  `VISSIM.Vissim.2200`). Scripts default to 2200 so the generated `.inpx`
  loads in both 2022 and 2026. If you only have 2026, switch to
  `VISSIM.Vissim.2600` in `start_vissim.py` + `archive/setup_network.py`
  and regenerate — the resulting `.inpx` will be 2026-only.
- `VehicleSubscription` in `config.yaml` uses `id: ['All']` to subscribe
  to every vehicle — no ego vehicle is injected.
- The OpenDRIVE round-trip preserves geometry; vehicle types, inputs,
  and routes don't exist in `.xodr` so demand is added on the VISSIM side
  by `setup_network.py`.
