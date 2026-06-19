# TrafficLayer_SUMO_Carla — SUMO ↔ Carla co-simulation (#174)

A **click-to-run** scaffold of **Carla** visualizing SUMO traffic through the
FIXS pipeline, on the shared **SimpleLoop** scenario. Vehicles only — **no
traffic-signal sync** (SimpleLoop has no signals).

Sibling of [../TrafficLayer_SUMO_CMoffice/](../TrafficLayer_SUMO_CMoffice/):
same SUMO source + same shared net, but the virtual environment is **Carla**
(`VirCarlaEnv.exe`) instead of CarMaker. The pair drives the
(to-be-consolidated) `VirEnvCore` from one pipeline into two backends — the
cross-simulator differential of #174.

```
SUMO ──[TraCI states]──▶ TrafficLayer.exe ──[VehFullData @ port 440]──▶ VirCarlaEnv.exe
 (SimpleLoop,             (SUMO path,                                    (Carla bridge)
  port 1337)              libsumo/TraCI)                                       │
                                                                     spawn / SetTransform
                                                                               ▼
                                                                        Carla server (PhysX off)
```

By default this runs **visualization-first** (`EnableExternalControl: false`):
Carla renders the SUMO vehicles (ego included) and does not read anything back.
Flip `EnableExternalControl: true` in `config.yaml` for the **mode A readback**
path (Carla reads the ego pose/velocity back and publishes it to FIXS — the
"L4-as-today" behavior).

## Run it

One-time: copy `Carla\carla.env.example` → `Carla\carla.env` and edit it for your
box (UE4_ROOT, CARLA_UPROJECT = your `C:\src_git\Carla_0915\…`, PY with carla 0.9.15).

**One-click:** double-click **`run_sumo_carla_demo.bat`**. It brings CARLA up from
the source build (`Carla\launch_carla.bat` `-game`), waits for RPC, loads
`simple_loop.xodr` as the world (`Carla\load_opendrive_world.py` — no map package,
coordinate-matched), then launches SUMO + TrafficLayer + VirCarlaEnv. If CARLA is
already running with the world loaded, `set SKIP_CARLA=1` to skip those steps.

**Headless self-check:** `python verify_sumo_carla.py` — runs the stack, records
`_logs/*.log`, and reasons about the bridge's decisions (connected to CARLA,
spawns ≥ 1, no spawn-failures/exceptions) → PASS/FAIL. **SKIPs** cleanly if no
CARLA server is reachable.

Stop order: **close VirCarlaEnv → close SUMO → Ctrl+C TrafficLayer → close CARLA.**

## Files

| File | Purpose |
| --- | --- |
| `run_sumo_carla_demo.bat` | **One-click launcher** (CARLA + world → SUMO → TrafficLayer → VirCarlaEnv), via `../../../../Carla/` tooling |
| `verify_sumo_carla.py` | **Headless self-check** — records `_logs/*.log`, reasons PASS/FAIL/SKIP |
| `config.yaml` | TrafficLayer config: `SelectedTrafficSimulator: SUMO` + `CarlaSetup`, ego `ego`, Carla client on port 440, **verbose on** |
| `traffic_light_table.csv` | TLS map header (junction_id,link_id,x,y,z,heading). **Header-only** — SimpleLoop has no signals; the bridge now tolerates this (see below) |
| (shared) `../../SimpleLoop/simple_loop.{net,rou,sumocfg}.xml` | The shared SUMO scenario (not duplicated here) |
| (shared) `../../../../Carla/` | Launch / wait-for-RPC / OpenDRIVE-world-load tooling |

## Validated state + remaining

Validated on the dev box **up to the bridge handshake**: SUMO 1.18 + the built
`TrafficLayer.exe` run the SUMO path cleanly with this config — parses, libsumo
loads, connects to SUMO, opens the Carla-bridge server on **port 440**, waits for
VirCarlaEnv. The CARLA half is **not** runnable here (the source build at
`C:\src_git\Carla_0915` isn't compiled), so the spawn/pose audit runs on your box.

Resolved during bring-up:

- **Empty-TLS hard-fail — FIXED in source.** `mainVirCarla.cpp` used to
  `return -1` on an empty traffic-light table; SimpleLoop has no signals, so that
  aborted the bridge. It now logs a notice and **runs vehicles-only**.
  ⚠️ **Requires rebuilding `VirCarlaEnv.exe`** — the committed
  `tests/SumoCarla/VirCarlaEnv.exe` predates the fix and will still abort. Build
  `VirCarlaEnv\VirCarlaEnv.sln` (Release x64) before running.
- **Geometry — handled via OpenDRIVE standalone.** `load_opendrive_world.py`
  loads `simple_loop.xodr` directly, so CARLA's road **is** the xodr — coordinate-
  matched to SUMO's 0–200 frame and to the CarMaker side. No town placement, no
  map package.
- **Ego — OK.** `simple_loop.rou.xml`'s `ego` is the SUMO-driven vehicle Carla
  visualizes (`InterestedIds: ['ego']`, `EnableExternalControl: false`). Set it
  `true` for the mode-A readback path.

Remaining to confirm on a Carla box: the SimpleLoop `car` vType resolves to a
valid Carla blueprint (`UseVehicleTypeAsBlueprint: false` → mapped via
`BridgeHelper::map_Sumo_vClass_to_Carla_blueprintId`). Traffic mimics the VISSIM
counterpart: ~40 background vehicles circulating endlessly (yellow in SUMO) + the
red ego — see `tests/Sumo/SimpleLoop/`.

## Build prerequisites

| Artifact | Build command |
| --- | --- |
| `CommonLib/yaml-cpp`, libsumo, libcarla | `scripts\dispatch\1_external_libraries.bat` (+ Carla client libs) |
| `TrafficLayer.exe` | `scripts\dispatch\2_core_components.bat` |
| `VirCarlaEnv.exe` | build `VirCarlaEnv\VirCarlaEnv.sln` (Release x64) |

Also installed: **SUMO 1.21** (`sumo-gui` on PATH), **Carla 0.9.15** server.
A prebuilt `VirCarlaEnv.exe` is committed at `tests/SumoCarla/VirCarlaEnv.exe`
and used as a fallback if the build output is absent.
