# SUMO &harr; CARLA co-simulation

Table-based SUMO &rarr; CARLA traffic-light synchronization plus full SUMO &harr; CARLA
vehicle co-simulation, built on CARLA's `sumo_integration` bridge.

## Layout

`Carla/` is a **self-contained component** &mdash; it is shipped whole in the FIXS
release zip, so `fetch + unzip` leaves `FIXS/Carla/` ready to run. The **tests**
live separately under `tests/`.

```
Carla/                        <- self-contained co-sim component (shipped in the zip)
  sumo/                       <- the SUMO <-> CARLA co-sim runtime
    run_synchronization/      full CARLA<->SUMO co-sim (vehicles + TL table)
      sumo_integration/       CARLA's bridge (bridge_helper, sumo/carla simulation, ...)
    sumo_carla_tl_sync.py     standalone SUMO->CARLA TL mirror
    auto_place_tls.py         headless TL-actor placement (run inside the UE4 editor)
    unreal_placing_tls.py     spawns BP_TrafficLight actors from the table
    set_spectator_view.py     move the CARLA spectator over the junctions
  utils/                      <- co-sim helpers (kept inside Carla/ so it ships self-contained)
    extract_sumo_tls_as_table.py   generate traffic_light_table.csv from a SUMO net
    trafficlight_helper.py         SUMO<->CARLA<->Unreal coordinate transforms
    unreal_remove_tl.py            remove placed TL actors
  carla_env_setup.py          one-time/reconfigure CARLA env picker (saves ~/.fixs/carla.json)
  setup_carla.bat / setup_carla.sh  thin per-OS wrappers for the picker
  run_cosim.py                cross-platform launcher (Windows/Linux)
  run_cosim.bat / run_cosim.sh  thin per-OS wrappers
  README.md                   (this file)

tests/Sumo/Carla/             <- tests only (no runtime code)
  test_tl_logic.py            Tier-1 TL logic tests (no CARLA server / GPU / map asset)
  test_carla_env.py           Tier-1 env/config + launch-resolution tests (no GUI/server)
  verify_demo.py              gated end-to-end smoke test (needs CARLA_ROOT)
  fixtures/                   tiny SUMO grid net + table (Tier-1 test data, no assets)
```

(`sumo/` is named for the partner simulator &mdash; a future CARLA co-sim with another
tool would slot in as e.g. `Carla/vissim/`.)

## Environment

Use the `realsim` conda env (`environment.yml` at the repo root): Python 3.10 +
`carla` (client wheel from PyPI, no GPU/server needed to import) + `traci` +
`sumolib` + `pytest`. SUMO &ge; 1.20 must be on `PATH`.

## Testing (two tiers)

**Tier 1 - logic tests, run anywhere** (no CARLA server, no GPU, no map asset):

```bash
pytest tests/Sumo/Carla/test_tl_logic.py tests/Sumo/Carla/test_carla_env.py
```

`test_tl_logic.py` validates junction&harr;net consistency, link-index bounds,
coordinate transforms, and SUMO-char &rarr; CARLA-state mapping against the tiny
grid fixture under `tests/Sumo/Carla/fixtures/`. `test_carla_env.py` validates the
`~/.fixs/carla.json` config round-trip and the packaged/source launch-command
resolution (no GUI, no server). These are the portable automated tests.

**Tier 2 - end-to-end smoke test, gated on `CARLA_ROOT`** (skips cleanly if unset):

```bash
CARLA_ROOT=/path/to/carla python tests/Sumo/Carla/verify_demo.py
```

Launches CARLA headless, loads stock **Town01**, runs the co-sim on CARLA's
bundled `Co-Simulation/Sumo/examples/Town01.sumocfg` for a few hundred ticks,
and asserts SUMO vehicles transfer into CARLA. No custom map/FBX required.

## Choosing which CARLA to use (one-time setup)

`run_cosim.py` does **not** hard-code a CARLA path or rely on `CARLA_ROOT`.
Instead it reads a per-machine config at `~/.fixs/carla.json` written by
`carla_env_setup.py`. That config is outside any repo (never git-tracked), so it
is set once per computer and reused by every FIXS app on it.

**You don't have to run setup by hand** &mdash; the first time `run_cosim.py`
launches CARLA on a fresh clone and finds no config, it auto-invokes the setup
prompt, remembers your answer, and continues. Every run after that is seamless.

Run setup explicitly only to **switch CARLA** (packaged &harr; source build, or a
different install/version):

```bash
# Windows
Carla\setup_carla.bat
# Linux/macOS
Carla/setup_carla.sh
# or directly, on either OS:
python Carla/carla_env_setup.py          # interactive picker
python Carla/carla_env_setup.py --show   # print the current config
```

Setup asks **packaged** vs **source build**, then opens a native folder picker:

- **Packaged** &mdash; pick the folder holding `CarlaUE4.exe` (Windows) /
  `CarlaUE4.sh` (Linux). Launched directly.
- **Source build** &mdash; pick the CARLA source folder (with
  `Unreal/CarlaUE4/CarlaUE4.uproject`) and the Unreal Engine root (`UE4_ROOT`).
  Launched through the editor as `UE4Editor <uproject> -game`.

(No display? The picker falls back to typing the path. Headless CI instead sets
`CARLA_ROOT` and runs `verify_demo.py`, which bypasses the interactive setup.)

## Running a co-sim

Once CARLA is configured (or on the auto-prompting first run), `run_cosim.py`
launches the saved CARLA, loads the map, and runs the synchronization:

```bash
# launch CARLA + run on stock Town01 (first run prompts for CARLA, then remembers)
python Carla/run_cosim.py \
    --sumocfg tests/Sumo/Carla/fixtures/grid_tls.sumocfg --map Town01

# pick a different CARLA for this run (re-runs setup first)
python Carla/run_cosim.py --reconfigure --sumocfg ... --map Town01

# CARLA already running, RoadRunner-imported map (vehicles need --no-net-offset)
python run_cosim.py --no-launch --map RP_Ver0529 \
    --sumocfg path/to/roosevelt.sumocfg \
    --tl-table path/to/traffic_light_table.csv \
    --no-net-offset --sumo-gui
```

### Notes
- `--no-net-offset` zeroes the SUMO net offset for **RoadRunner-imported maps**
  (which sit in the SUMO-local frame). Stock CARLA towns do **not** need it.
- `--no-launch` skips both the launch and the env config &mdash; use it when CARLA
  is already running.
- Traffic lights for a RoadRunner map are placed once with `auto_place_tls.py`
  (run via the UE4 editor `-ExecutePythonScript`); see `sumo/`.
- Large map assets (FBX / cooked content) are **not** stored here - the tests use
  stock Town01; application maps ship separately.
