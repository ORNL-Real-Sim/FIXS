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
  place_tls.py                place SUMO traffic lights into a cooked map (editor)
  place_tls.bat / place_tls.sh      thin per-OS wrappers for the placer
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

A **re-import** (`--force` / `--reimport`, or answering the prompt) moves the old
cooked content aside and imports fresh - CARLA's `ImportAssets` cooks cleanly into
an empty destination but often fails to *replace* existing content; the backup is
restored if the cook fails, so a working map is never lost.

## Traffic lights (maps without dynamic signals)

If a map's OpenDRIVE has no dynamic signals, CARLA spawns no `traffic.traffic_light`
actors and the SUMO->CARLA TL sync has nothing to drive. `place_tls.py` places them
from the table into the map's level (running `sumo/auto_place_tls.py` in the full
editor) and writes a marker so it isn't repeated:

```bash
place_tls.bat --map-config apps/roosevelt/roosevelt_map.txt \
              --tl-table apps/roosevelt/Roosevelt_Sumo_Scenario/traffic_light_table.csv
```

A re-import wipes the map's content (and the marker), so the lights are re-placed
automatically on the next run.

## One-click: run_cosim does it all, idempotently

In source mode `run_cosim.py` is a true one-click: it configures the CARLA env if
needed, then **imports the map if it isn't cooked** (`--auto-import`), then
**places the traffic lights if they aren't placed** (when a `--tl-table` is given),
then loads + runs. Each step is **skipped when already done** and re-done after a
re-import. The first load of a freshly cooked map compiles shaders and can take a
couple of minutes (`--load-timeout`, default 300 s, covers it; later loads are
fast). Without `--auto-import` it instead prints a clear "import / place first"
hint rather than failing opaquely. Apps keep one-click shims (`import_<app>_map`,
`place_tls_<app>`) for explicit re-import / re-place.

## Applications (`apps/apps.json`) and saved run setups

FIXS is the enabler, not the owner: it defines the **application manifest schema**
and the tooling that reads it, and hardcodes no application. An app repo that sits
above a fetched bundle (`<repo>/FIXS/Carla/...`, hence `<repo>/apps/`) declares its
applications in one file, `apps/apps.json`, parsed by
[`app_catalog.py`](app_catalog.py) &mdash; the same split as the Digital-Twin-Library
`catalog.json` that `import_map` already consumes. **No manifest = the previous
behaviour, unchanged.**

```jsonc
{
  "schema": 1,
  "apps": [
    { "id": "roosevelt", "title": "Roosevelt Rd co-sim" },
    { "id": "mlk_eco_driving", "title": "MLK arterial eco-driving",
      "maps": ["mlk"],
      "configs": [
        { "path": "MLK_Sumo_Scenario/config_Sumo_Carla_dSPACE.yaml",
          "title": "dSPACE XIL", "engine": "cpp" }
      ],
      "defaults": { "engine": "cpp" } }
  ]
}
```

- **`maps` defaults to the app's `id`.** An app named after its location matches the
  library entry of the same name and declares nothing. A name that matches neither a
  library entry nor a cooked map is simply *not offered* &mdash; the picker falls
  through to the full library / cooked / local-file menu, so a stale name costs a
  shortcut, never a run.
- **The library section narrows to the app; the cooked section never does.** An app
  pinned to Roosevelt is not offered Atlanta as if they were interchangeable, so
  `Online` lists only the app's library map(s) &mdash; but `Local (already imported
  into CARLA)` still lists **every** cooked map, because what is cooked is a
  property of the machine, not of the application. Pick `none` at the application
  prompt to browse the whole library. If the app's map is not in the library at all
  there is nothing to narrow to, so the full library is listed and `Enter` instead
  lands on that map down in the cooked section.
- **`configs` are staged, not read in place.** Each is copied to
  `~/.fixs/apps/<id>/` on first use, because a committed yaml carries machine
  -specific values (CARLA server IP, dSPACE ports) that must not go back into the
  repo. Your copy is never silently clobbered: an upstream change refreshes it only
  while you have not edited it, otherwise it lands beside it as `<name>.yaml.new`.

### Where `~/.fixs` puts things

The tree splits on **can FIXS re-create this?**, not on which entity produced it:

```
~/.fixs/
  carla.json  catalog.json  run_profiles.json
  apps/                          <- EDITED. Every scenario yaml lives here.
    <app>/<name>.yaml                app-owned, staged from the repo, map-independent
    <app>/maps/<map>/config.yaml     generated for this app on this map
    _generic/...                     a run with no application selected
  maps/                          <- RE-CREATABLE. Safe to delete to reclaim disk.
    <map>/{<bundle>.zip, carla/, sumo/, tl_table.csv}
```

Scenario yamls are **app-bounded, never map-bounded**, for two reasons. They are
edited, and `maps/` is a cache a user should be able to delete wholesale (it is
gigabytes) without destroying a tuned config. And one yaml per map is not enough:
the same map serves several apps, and one app runs several versions of a location,
so the host/ports/subscriptions belong to *(app, map)* &mdash; keyed by map alone,
two apps on `roosevelt_full` silently overwrite each other. Pre-app yamls left in
`maps/<map>/` are **moved** into the app tree on first use, never abandoned.
- **`engine` on a config** says which bridge that yaml is written for. Without it a
  hand-written native-stack yaml reads as `py`, because `ConfigHelper` defaults
  `CarlaSetup.EnablePythonBackend` to true &mdash; it would silently run the wrong
  stack. `--engine` still overrides.

### Named run setups

A run is six choices: application, map, scenario yaml, bridge, CARLA, SUMO. They
are saved as a **named setup** in `~/.fixs/run_profiles.json`, and every later run
starts from the list of them rather than from a blank prompt:

```
[cosim] Saved run setups:
    1) roosevelt_default   roosevelt | roosevelt_full | config.yaml | py | gui   <- last run
    2) roosevelt_headless  roosevelt | roosevelt_full | config.yaml | py | headless
    3) mlk_dspace          mlk_eco_driving | mlk_full | ..._dSPACE.yaml | cpp | gui
    N) new setup
    D) delete a setup
[cosim] Which? [1-3 / N / D], Enter = 1 (roosevelt_default):

[cosim] Run setup 'roosevelt_default'  (last run 2026-07-29 13:21):
   1) app       roosevelt
   2) map       roosevelt_full            (Digital-Twin-Library)
   3) scenario  config.yaml               (generated, this app on this map)
   4) engine    py                        (run_synchronization.py)
   5) CARLA     source  C:/src_ext/Carla  ->  127.0.0.1:2000
   6) SUMO      gui, step 0.05

[cosim] Enter = run it | 1-6 = change (e.g. "2 4") | S = switch setup | N = new | Q = quit:
```

**It loops.** After a change the summary is redrawn, so you see the result and can
keep editing; nothing runs until you press Enter. All six are pure decisions, so no
map is downloaded, cooked or loaded while you are still deciding &mdash; quitting out
leaves nothing half-done.

**Every line is editable.** `4` picks the bridge, `5` switches the CARLA install or
sets the RPC endpoint, `6` sets gui/headless and the timestep (validated against
CARLA's 0.1 s ceiling). Dependent slots follow automatically: changing the app
re-asks the map and scenario; changing the map re-asks the scenario only if it was
that map's *generated* yaml, since an app yaml belongs to the app.

**Editing offers to fork.** Run a setup you did not touch and it is saved back under
its own name silently. Change something and you are asked to name it &mdash; Enter
overwrites, typing a name forks a new one &mdash; so "try it with a different map"
never quietly destroys the setup you meant to keep.

Explicit flags outrank a saved setup, so `run_cosim --map atlanta` changes exactly
one thing and prompts for nothing. `--profile <name>` opens one directly, `--app
<id>` opens that app's most recent, `--fresh` starts a new one, `--no-app` ignores
the manifest. The store is one file (`{last, setups{}}`) so the whole list can be
read, shown and switched in one place &mdash; including by a future front-end, which
is a swap of the prompts, not of the flow.

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
