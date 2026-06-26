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
  run_cosim.py                cross-platform launcher (Windows/Linux)
  run_cosim.bat / run_cosim.sh  thin per-OS wrappers
  README.md                   (this file)

tests/Sumo/Carla/             <- tests only (no runtime code)
  test_tl_logic.py            Tier-1 logic tests (no CARLA server / GPU / map asset)
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
pytest tests/Sumo/Carla/test_tl_logic.py
```

Validates the junction&harr;net consistency, link-index bounds, coordinate
transforms, and SUMO-char &rarr; CARLA-state mapping against the tiny grid fixture
under `tests/Sumo/Carla/fixtures/`. This is the portable automated test.

**Tier 2 - end-to-end smoke test, gated on `CARLA_ROOT`** (skips cleanly if unset):

```bash
CARLA_ROOT=/path/to/carla python tests/Sumo/Carla/verify_demo.py
```

Launches CARLA headless, loads stock **Town01**, runs the co-sim on CARLA's
bundled `Co-Simulation/Sumo/examples/Town01.sumocfg` for a few hundred ticks,
and asserts SUMO vehicles transfer into CARLA. No custom map/FBX required.

## Running a co-sim

`run_cosim.py` resolves the CARLA server from `CARLA_ROOT` (OS-aware:
`CarlaUE4.exe` on Windows, `CarlaUE4.sh` on Linux), launches it, loads the map,
and runs the synchronization:

```bash
# launch CARLA + run on stock Town01
CARLA_ROOT=/opt/carla python Carla/run_cosim.py \
    --sumocfg tests/Sumo/Carla/fixtures/grid_tls.sumocfg --map Town01

# CARLA already running, RoadRunner-imported map (vehicles need --no-net-offset)
python run_cosim.py --no-launch --map RP_Ver0529 \
    --sumocfg path/to/roosevelt.sumocfg \
    --tl-table path/to/traffic_light_table.csv \
    --no-net-offset --sumo-gui
```

### Notes
- `--no-net-offset` zeroes the SUMO net offset for **RoadRunner-imported maps**
  (which sit in the SUMO-local frame). Stock CARLA towns do **not** need it.
- Traffic lights for a RoadRunner map are placed once with `auto_place_tls.py`
  (run via the UE4 editor `-ExecutePythonScript`); see `helper_scripts/`.
- Large map assets (FBX / cooked content) are **not** stored here - the tests use
  stock Town01; application maps ship separately.
