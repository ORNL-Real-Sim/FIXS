# TrafficLayer_SUMO_CMoffice — SUMO ↔ CarMaker(office) co-simulation (#174)

A **click-to-run** scaffold of CarMaker Office driving an ego through **SUMO**
background traffic via the FIXS pipeline, on the shared **SimpleLoop** scenario.
Vehicles only — **no traffic-signal sync** (SimpleLoop has no signals).

This is the **SUMO sibling** of
[../../../Vissim/Probes/TrafficLayer_DSProxy_CMoffice/](../../../Vissim/Probes/TrafficLayer_DSProxy_CMoffice/),
which drives the *same* CarMaker side from VISSIM via DSProxy. Run both and you
exercise the **same `VirtualEnvironment.lib`** pipeline from two traffic sources
— the cross-simulator differential the #174 consolidation is built to protect.

```
CarMaker.win64.exe ──[ego "egoCm" VehFullData @ port 2444]──▶ TrafficLayer.exe
   (custom office exe,                                          (SUMO path,
    User.c → VirtualEnvironment.lib)                            libsumo/TraCI)
        ▲                                                             │
        │                                                    TraCI moveToXY / inject
        │                                                             ▼
        └──[background traffic, ego skipped]──────────────  SUMO (SimpleLoop net, port 1337)
```

CarMaker owns ego dynamics (**mode A**): it sends its ego pose out; TrafficLayer
injects/teleports it in SUMO and streams SUMO's other vehicles back as the
CarMaker `RS_Cxxx` traffic objects.

## Run it — two ways

**A) Headless self-check (recommended):** `python verify_sumo_cm.py`. It launches
SUMO + TrafficLayer + the real headless CarMaker exe, records every line to
`_logs/`, and reasons about the round-trip (`.lib` connected, steps ran, clean
end, no errors) → prints PASS/FAIL. Auto-(re)builds the headless harness exe.

**B) GUI:** double-click **`run_sumo_cm_demo.bat`**. It registers the FIXS config
into the CM project, launches SUMO + TrafficLayer, and opens CarMaker Office.
Then in the GUI: load TestRun **`SimpleLoop_rs`** and press **Start**.
Stop order: **Stop in CarMaker → close SUMO → Ctrl+C TrafficLayer.**

## Known issue: stochastic IPGDriver junction off-road (demo is seed-pinned)

On some traffic realizations the ego **leaves the road at a junction curve**
(around x≈8–23, y≈0–1) and the run aborts with `Vehicle leaves road…` (or
`Embedded FARoadSensor Geo … not found on Road`). Root cause and scope, established
by direct measurement (`rs_ego.csv`) and a seed campaign on this branch:

- **Mechanism:** when background traffic halts the ego *on* the junction curve and
  it restarts from ~0 m/s, CarMaker's IPGDriver fails to re-acquire the course —
  its heading stays frozen, it drives nearly straight, and a tire crosses the inner
  edge. A *moving* ego tracks the same corner fine. This is the #168 "drives
  straight off the link end," a CarMaker-side junction-navigation defect.
- **Stochastic + backend-agnostic.** It reproduces on **both** SUMO and VISSIM at
  comparable rates (~10–30 % of random seeds) and is **independent of the SUMO
  car-following model** (Krauss 2/13, W99 4/13 — both off-road; VISSIM 5/51). It is
  **not** a SUMO-feed bug: the data CarMaker receives is clean at the abort.
  `Driver.Course.CornerCutCoef=0` *reduces* it (fixed the SUMO fails) but does not
  eliminate it (VISSIM still failed) — corner-cutting contributes; the real fix is
  on the CarMaker junction/route side, tracked separately.
- **So the demo pins a verified-clean realization:** `run_sumo_cm_demo.bat` launches
  SUMO with **`--seed 5`** (and `verify_sumo_cm.py` defaults to seed 5) — that
  realization runs the full 1000 s cleanly. Same seed ⇒ same outcome (deterministic).
- **To reproduce the off-road** for study: `set RS_SUMO_SEED=10` (or `=none` for
  SUMO's unseeded default, which also fails) before `verify_sumo_cm.py`.

## Files

| File | Purpose |
| --- | --- |
| `verify_sumo_cm.py` | **Headless self-check** — runs the stack, records `_logs/*.log`, reasons PASS/FAIL (env: `RS_SUMO_SEED` default 5, `RS_TESTRUN`, `RS_SUMOCFG`, `RS_RUN_TIMEOUT`) |
| `run_sumo_cm_demo.bat` | **GUI one-click** (register exe+config → SUMO → TrafficLayer → CarMaker Office), mirrors the DSProxy `run_cm_office_demo.bat` |
| `setup_gui.py` | Patches `Data/Config/GUI`: `CM.Exe = src/CarMaker.win64.exe`, `CM.Args = -f config.yaml` (pure stdlib) |
| `config.yaml` | TrafficLayer config: `SelectedTrafficSimulator: SUMO` + `CarMakerSetup`, ego `egoCm` on port 2444, signals off |
| (shared) `../../SimpleLoop/simple_loop.{net,rou,sumocfg}.xml` | The shared SUMO scenario (not duplicated here) |
| (shared) CM road/TestRun in `ProprietaryFiles/CM13_proj` | `SimpleLoop_rs`, built by the VISSIM probe's `import_road.bat` + `build_testrun.py` |

## How this differs from the VISSIM/DSProxy probe

| | VISSIM probe | This (SUMO) probe |
| --- | --- | --- |
| Traffic source | VISSIM via `DrivingSimulatorProxy.dll` (DSProxy bypass) | SUMO via libsumo/TraCI (normal FIXS path) |
| `EnableRealSim` | `false` (DSProxy owns the path) | **`true`** |
| Config delivery to CM | `setup_gui.py` registers exe + `-f config` into the CM GUI config | **identical** — `setup_gui.py` registers exe + `-f config.yaml` (no VISSIM network staging) |
| Ego identity | `egoCm` round-trips via VISSIM VehicleID | `egoCm` injected into SUMO by the interface |
| CarMaker side | **identical** — same custom exe + `VirtualEnvironment.lib`, same TestRun | **identical** |

The CarMaker side is deliberately the same so the only variable is the traffic
backend — that's what makes the two probes a differential test.

## Validated state

Validated on the dev box **up to the CarMaker handshake**: SUMO 1.18 + the built
`TrafficLayer.exe` run the SUMO path cleanly — config parses, libsumo loads,
TrafficLayer connects to SUMO over TraCI (`vehicles TOT 1`), opens the XIL server
on **port 2444**, and waits for CarMaker. The CM connect + run is exercised by
`verify_sumo_cm.py` (needs the CM project + the headless harness exe it builds).

Resolved during bring-up (were open scaffold items):

- **Ego handling — OK.** Confirmed FIXS *creates* the ego on the SUMO path
  (`TrafficHelper::addEgoVehicleFromXY` → `Vehicle::add` + `moveToXY`), it does
  not attach to an existing one. So `config.yaml` ego `egoCm` is injected by the
  interface and teleported to CarMaker's pose; the shared route's self-driven
  `ego` simply becomes the one background vehicle CarMaker renders. No route edit
  needed; no duplicate ego.
- **Geometry — unified.** `tests/Sumo/SimpleLoop/simple_loop.net.xml` is now
  **generated from the same `simple_loop.xodr`** that builds the CarMaker road
  (`netconvert --opendrive-files`), so SUMO and CarMaker share one geometry — no
  more square-vs-rounded corner mismatch. Verified headless: the ego traces the
  full R15 loop in the 0–200 frame. (Route edges are `-1 -2 -3 -4` from
  netconvert; see `tests/Sumo/SimpleLoop/README.md`.)
- **Ports — OK.** 2444 (CarMaker) confirmed served; SUMO TraCI on 1337.

Traffic mimics the VISSIM counterpart: a 700 veh/h background flow fills to ~40
vehicles (yellow) that **circulate endlessly** (no exit), plus the red ego —
matching VISSIM's ~40-persistent-after-240 s. See `tests/Sumo/SimpleLoop/`.

## Build prerequisites

| Artifact | Build command |
| --- | --- |
| `CommonLib/yaml-cpp`, libsumo | `scripts\dispatch\1_external_libraries.bat` |
| `VirtualEnvironment.lib` | `scripts\dispatch\4_virtual_environment.bat` |
| `TrafficLayer.exe` | `scripts\dispatch\2_core_components.bat` |
| custom `CarMaker.win64.exe` | `msbuild ProprietaryFiles\CM13_proj\src\CarMaker.sln /t:CarMaker /p:Configuration=Release /p:Platform=x64` |

Also installed (cannot be shipped): **SUMO 1.21** (`sumo-gui` on PATH,
`%SUMO_HOME%` set), **CarMaker 13.1.3** office, **Python 3 + PyYAML** in a conda
env (`realsim_dev` by default; override with `set CONDA_ENV=...`).
