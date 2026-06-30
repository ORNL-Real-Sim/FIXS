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
  import_map.py               import a RoadRunner/OpenDRIVE map into a source build
  import_map.bat / import_map.sh    thin per-OS wrappers for the importer
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
  `Unreal/CarlaUE4/CarlaUE4.uproject`) and the Unreal Engine root. `UE4_ROOT` is
  used automatically when it points at a real engine; otherwise you're prompted.
  Launched through the editor as `UE4Editor <uproject> -game`.

**Windows note:** importing a *custom* map into a packaged CARLA is unsupported by
CARLA (map ingestion is Linux + Docker only &mdash; there is no `ImportAssets.bat`),
so on Windows setup offers **source build only** for custom-map apps. Pass
`--allow-packaged-windows` if you only need stock maps (Town01, ...) from a package.

Setup also resolves the **python env** that runs the co-sim and stores it in the
config, so the launcher works on any machine no matter what the env is named:

1. the canonical env from `environment.yml` (`realsim`) if it exists;
2. else, if conda is found, it offers to create it from `environment.yml`;
3. else any conda env that already imports `carla + traci + sumolib`, or a manual
   pick. For a **source build** the matching client wheel is auto-resolved from
   `PythonAPI/carla/dist` (manual pick as fallback).

`run_cosim.py` then re-executes itself under that interpreter before importing
`carla`, so the `.bat`/`.sh` stay trivial. A config written by an older setup
(missing the python env) is repaired automatically on the next run.

(No display? Pickers fall back to typing the path. Headless CI instead sets
`CARLA_ROOT` and runs `verify_demo.py`, which bypasses the interactive setup.)

## Importing a custom map (source build only)

A custom RoadRunner/OpenDRIVE map must be **cooked into a source build** before
CARLA can load it (packaged CARLA cannot import custom maps). `import_map.py`
generalizes CARLA's `Util/BuildTools/Import.py --package=<name>` primitive: it
stages the package (the `<name>.json` descriptor + its fbx/xodr/fbm assets) under
`<carla_root>/Import` and runs the cook, reading `carla_root`/`ue4_root` from the
saved env config.

Large map packages are hosted as **release assets** (private repo), not in git.
Because a consumer may not have the GitHub CLI, the import is **download-and-point**:
`--package-url` is auto-fetched via `gh` when available, otherwise you're prompted
to point at a copy you downloaded by hand from the release page (browser access is
all that's needed):

```bash
# pass the release URL: auto-download via gh if present, else prompt for a local copy
python Carla/import_map.py --package RP_Ver0529 --package-url https://.../RP_Ver0529_carla_import.zip
# or point straight at a downloaded copy (the prompt also accepts a .zip or folder)
python Carla/import_map.py --package RP_Ver0529 --package-dir C:/Downloads/RP_Ver0529_carla_import.zip
```

(`--package-dir` and the prompt both accept either the downloaded `.zip` or an
already-extracted folder.)

An app declares its map in a small text file and points the **generic** importer
at it, so the URL lives in one place (not hard-coded in wrappers):

```
# roosevelt_map.txt
package=RP_Ver0529
url=https://.../RP_Ver0529_carla_import.zip
```
```bash
import_map.bat --package-pick --map-config apps/roosevelt/roosevelt_map.txt
```

`run_cosim.py` does this automatically: in source mode it checks whether the
map's `.umap` is cooked and, with `--auto-import [--map-config <txt>]`, imports it
before launching (auto-configuring the CARLA env first if needed); otherwise it
exits with a clear "import it first" message rather than an opaque `load_world`
error. A **re-import** (`--force` / `--reimport`, or answering the prompt) moves
the old cooked content aside and imports fresh - CARLA's `ImportAssets` cooks
cleanly into an empty destination but often fails to *replace* existing content;
the backup is restored if the cook fails, so a working map is never lost.

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
- The spectator auto-frames the **busiest intersection** from the TL table so the
  signal sync is legible (`--spectator-junction <id>` for a specific one,
  `--spectator-all` for the whole network, `--no-spectator` to leave it).
- Traffic lights for a RoadRunner map are placed once with `auto_place_tls.py`
  (run via the UE4 editor `-ExecutePythonScript`); see `sumo/`.
- Large map assets (FBX / cooked content) are **not** stored here - the tests use
  stock Town01; application maps ship separately.

### Timestep & speed
`--step-length` is the **shared** timestep: SUMO's `--step-length` *and* CARLA's
`fixed_delta_seconds` (default 0.05 s, matching CARLA's official
`run_synchronization`; the hard max with default physics substepping is 0.1 s).
The loop is paced to **real time** by default. Speed levers, none of which change
the defaults:
- `--carla-timeout S` &mdash; client connect timeout (default 10 s; raise for heavy
  source-build maps that are slow to answer the first RPC).
- `--quality-level Low` &mdash; cheaper rendering on heavy maps.
- `--fast` &mdash; drop real-time pacing; run as fast as the hardware allows.
