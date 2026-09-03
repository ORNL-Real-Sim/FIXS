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
    (run_synchronization moved to ../standalone/ -- #330: it speaks no FIXS)
      sumo_integration/       CARLA's bridge (bridge_helper, sumo/carla simulation, ...)
    sumo_carla_tl_sync.py     standalone SUMO->CARLA TL mirror
    auto_place_tls.py         headless TL-actor placement (run inside the UE4 editor)
    unreal_placing_tls.py     spawns BP_TrafficLight actors from the table
  utils/                      <- co-sim helpers (kept inside Carla/ so it ships self-contained)
    extract_sumo_tls_as_table.py   generate traffic_light_table.csv from a SUMO net
    trafficlight_helper.py         SUMO<->CARLA<->Unreal coordinate transforms
    unreal_remove_tl.py            remove placed TL actors
  run_cosim.py                cross-platform launcher (Windows/Linux) - the entry point
  carla_env_setup.py          one-time/reconfigure CARLA env picker (saves ~/.fixs/carla.json)
  import_map.py               import a RoadRunner/OpenDRIVE map into a source build
  place_tls.py                place SUMO traffic lights into a cooked map (editor)
  place_signs.py              place the RoadRunner sign meshes CARLA's import culled
  unreal_place_signs.py       the editor-side half of place_signs.py
  app_catalog.py              the apps/ manifest contract for FIXS application repos
  run_profile.py              named run setups (~/.fixs/run_profiles.json) + review loop
  props.py                    placement props and their manifest, from the map library
  doctor.py                   check this machine can run a co-sim, before it tries
  peer.py                     control channel between the halves of a distributed co-sim
  set_spectator_view.py       point the CARLA spectator at a scene (top-down, or --follow)
  load_opendrive_world.py     mesh an .xodr into a running server - no map package, no cook
  launch_carla.bat            hand-launch a server from carla.env, for that no-cook path
  carla.env.example           copy to carla.env and edit; read by launch_carla.bat
  wait_for_rpc.ps1            block until that server accepts RPC
  README.md                   (this file)

tests/Sumo/Carla/             <- tests only (no runtime code)
  test_tl_logic.py            Tier-1 TL logic tests (no CARLA server / GPU / map asset)
  test_carla_env.py           Tier-1 env/config + launch-resolution tests (no GUI/server)
  verify_demo.py              gated end-to-end smoke test (needs CARLA_ROOT)
  fixtures/                   tiny SUMO grid net + table (Tier-1 test data, no assets)
```

(`sumo/` is named for the partner simulator &mdash; a future CARLA co-sim with another
tool would slot in as e.g. `Carla/vissim/`.)

**Every entry point here is run as `python Carla/<script>.py`**, on either OS. There
are deliberately no `.bat`/`.sh` wrappers beside them: each script re-execs itself
under the configured interpreter (see [Environment](#environment)), so the python
you start with does not matter and a wrapper would add nothing. An application repo
built on a fetched FIXS bundle exposes its own front door &mdash; `run_cosim.bat` /
`run_cosim.sh` at the repo root &mdash; and that is what users should type; it sets
the app's `FIXS_ENV_NAME` and bootstraps the bundle before calling in here. A
same-named wrapper in this directory only shadowed it (ORNL-Real-Sim/FIXS#287).

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
python Carla/carla_env_setup.py                   # interactive picker
python Carla/carla_env_setup.py --show            # print the current config
python Carla/carla_env_setup.py --update-python   # rebind ONLY the python env
```

Setup asks **packaged** vs **source build**, then opens a native folder picker:

- **Packaged** &mdash; pick the folder holding `CarlaUE4.exe` (Windows) /
  `CarlaUE4.sh` (Linux). Launched directly.
- **Source build** &mdash; pick the CARLA source folder (with
  `Unreal/CarlaUE4/CarlaUE4.uproject`) and the Unreal Engine root. `UE4_ROOT` is
  used automatically when it points at a real engine; otherwise you're prompted.
  Launched through the editor as `UE4Editor <uproject> -game`.

**What a packaged build cannot do:** cook or place anything &mdash; that needs the
Unreal editor, which a package does not ship. So it runs a Digital-Twin-Library map
only if the library publishes it *precooked* (`<map>_cooked.tar.gz`), exactly as it
was cooked; traffic lights and signs are whatever the asset already contains.
`run_cosim` installs that asset for you, and names the missing asset up front for a
map that has none. To cook a map yourself, use a source build. *Installing* a
precooked asset works the same on Windows and Linux &mdash; it is a plain extract,
done here in `tarfile` rather than through CARLA's bash-only `Util/ImportAssets.sh`.

**Windows note:** the *assets* are not yet OS-neutral. Every `*_cooked.tar.gz` the
library publishes today is a Linux cook &mdash; its materials carry SPIR-V and no
Direct3D shaders &mdash; and a packaged build has no shader compiler, so on Windows
those materials fall back to the default one: correct geometry and traffic lights,
grey road surface. Setup therefore still offers **source build only** on Windows;
pass `--allow-packaged-windows` if you only need stock maps (Town01, ...) from a
package. `run_cosim` also warns before launching when the installed map has no
shaders for the platform it is about to run on. Tracked in
ORNL-Real-Sim/FIXS_Applications#29.

Setup also resolves the **python env** that runs the co-sim and stores it in the
config, so the launcher works on any machine no matter what the env is named:

1. the canonical env from `environment.yml` (`realsim`) if it exists;
2. else, if conda is found, it offers to create it from `environment.yml`;
3. else any conda env that already imports `carla + traci + sumolib`, or a manual
   pick. For a **source build** the matching client wheel is auto-resolved from
   `PythonAPI/carla/dist` (manual pick as fallback).

Whichever env is bound, setup says so when it is not the one that was asked for
(`FIXS_ENV_NAME`) and when it cannot import the co-sim's runtime modules — those
never stop a run, they degrade it under a different name (no `yaml` makes every
scenario setting read as its default, including `CarlaServerIP`).

**Every entry point runs under that one interpreter.** `carla_env_setup.
reexec_under_configured` is each script's first act — `run_cosim.py`,
`import_map.py`, `place_tls.py`, `place_signs.py` and the world/spectator helpers
— so which script you happen to start with cannot change the env you end up in,
and the `.bat`/`.sh` wrappers stay trivial `exec python <script>` one-liners.
(`sumo/auto_place_tls.py` is deliberately excluded: it runs inside the Unreal
editor's embedded python, which is not this env and must not be replaced.)

A config written by an older setup (missing the python env), or one whose env has
since been deleted or rebuilt without the carla client, is repaired automatically
on the next run. To change the binding on purpose — you created the env setup
asked for, or picked the wrong one from the list — use

```
run_cosim --update-python          # or: setup_carla --update-python
```

which re-resolves only the interpreter and keeps the CARLA / UE4 paths. Since
every entry point follows the config, that one command moves them all.

(No display? Pickers fall back to typing the path. Headless CI instead sets
`CARLA_ROOT` and runs `verify_demo.py`, which bypasses the interactive setup.)

## Importing a custom map

A custom RoadRunner/OpenDRIVE map must be **cooked into a source build** before
CARLA can load it &mdash; cooking runs the Unreal editor, which a packaged build does
not ship. (A packaged CARLA is served instead by `import_map.install_cooked`, which
extracts an already-cooked package; see the setup section above.) `import_map.py`
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

### Raw RoadRunner exports

A published bundle carries its own `<name>.json`. A raw export straight out of
RoadRunner does not — it is just `.fbx` + `.xodr` (+ `.fbm`, `.rrdata.xml`,
`.geojson`) — so `import_map` writes the descriptor for you: it stages the export
under `Import/<name>/`, pairs each `.xodr` with its `<x>.fbx` or its
`<x>_Tile_<i>_<j>.fbx` set, and emits `Import/<name>.json` with
`use_carla_materials: false` (RoadRunner ships its own materials) and, for a
tiled map, `tile_size` from the export's `TilesInfo.txt` or 2000.

**The name you import as wins.** RoadRunner names an export after the author's
working file (`MLK_no_signal_0805`, `Atl_R2024b_final`), so the staged geometry
is renamed to the map name you asked for:

```bash
python Carla/import_map.py --package mlk_no_signal --package-dir C:/Downloads/MLK_no_signal.zip
# -> Import/mlk_no_signal.json, Import/mlk_no_signal/mlk_no_signal.fbx
```

Renaming rather than adopting the export's name is deliberate:

- the cook writes the walker navmesh as `Maps/<map>/Nav/<fbx_stem>.bin` while the
  runtime loads `<MapName>.bin`, so a mismatched `.fbx` costs the map its
  pedestrian navigation, silently;
- a re-export would otherwise cook a *different* CARLA map every time and orphan
  the saved run setups, app manifests and catalog entries pointing at the old name.

Only the `.fbx` (and its `.fbm`) is renamed. The `.xodr` keeps the export's name —
CARLA copies it to `<map>.xodr` during the cook — and `.geojson` / `.rrdata.xml`
stay untouched, so `Import/<name>/` still shows which export the map came from.
The descriptor records it too, as `exported_as`.

**One package holds one map** — a tiled map is still one map. That is assumed of
a Digital-Twin-Library bundle and of a folder you pick by hand, and it is what
lets the importer resolve a package without asking anything. A package that
breaks the rule is reported, not guessed at:

| in the package | result |
|---|---|
| `.fbx` named after the `.xodr` | imported |
| one `.fbx` (or one tile set) named *differently* | imported, with a warning naming what it adopted |
| several `.xodr` — two maps, or one staged twice | refused |
| several unrelated `.fbx` (a layer-split export) | refused |
| `.xodr` with no `.fbx`, or `.fbx` with no `.xodr` | refused |
| both `<name>.fbx` **and** `<name>_Tile_*.fbx` | refused (a map is one or the other) |

The warning case is deliberate rather than silent: a stem mismatch usually means
the package was assembled by hand, and the lone `.fbx` staged might be scenery
rather than the road network — which the cook would happily accept.

An app declares its map in a small text file and points the **generic** importer
at it, so the URL lives in one place (not hard-coded in wrappers):

```
# roosevelt_map.txt
package=RP_Ver0529
url=https://.../RP_Ver0529_carla_import.zip
```
```bash
python Carla/import_map.py --package-pick --map-config apps/roosevelt/roosevelt_map.txt
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
python Carla/place_tls.py --map-config apps/roosevelt/roosevelt_map.txt \
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
  apps/                          <- EDITED. Every scenario yaml lives here, flat.
    <app>/<name>.yaml                app-owned, staged from the repo, map-independent
    <app>/<map>.yaml                 generated for this app on this map
    _generic/...                     a run with no application selected
  maps/                          <- RE-CREATABLE. Safe to delete to reclaim disk.
    <map>/{<bundle>.zip, carla/, sumo/, tl_table.csv}
```

`~/.fixs/apps/<app>/` mirrors `apps/<app>/` in the repo: one folder per
application, its yamls directly inside. The generated one is **named for its map**,
so two maps under one app each get their own file without a subfolder.

Scenario yamls are **app-bounded, never map-bounded**, for two reasons. They are
edited, and `maps/` is a cache a user should be able to delete wholesale (it is
gigabytes) without destroying a tuned config. And one yaml per map is not enough:
the same map serves several apps, and one app runs several versions of a location,
so the host/ports/subscriptions belong to *(app, map)* &mdash; keyed by map alone,
two apps on `roosevelt_full` silently overwrite each other. Yamls left by an older
FIXS in `maps/<map>/` or in `apps/<app>/maps/<map>/` are **moved** here on first
use, never abandoned; a variant is prefixed with its map (`config_fast.yaml` &rarr;
`<map>_config_fast.yaml`) so it cannot outrank the map's own yaml as the default.
- **`engine` on a config** says which bridge that yaml is written for. Without it a
  hand-written native-stack yaml reads as `py`, because `ConfigHelper` defaults
  `CarlaSetup.EnablePythonBackend` to true &mdash; it would silently run the wrong
  stack. `--engine` still overrides.
- **The yaml owns the bridge and the CARLA endpoint.** Neither is ever copied into
  a saved setup: the summary derives them from the chosen yaml every time it is
  drawn, and editing slot 4 or 5 writes them back into that yaml (a targeted line
  rewrite, so comments survive). Anything else gives two sources of truth &mdash;
  hand-editing the yaml stops taking effect, and a stale remembered endpoint makes
  `run_cosim` report a healthy CARLA while VirCarlaEnv, which reads
  `CarlaSetup.CarlaServerIP` straight from the yaml, dials a different one and
  times out.

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
   4) engine    py                        (Python VirEnvCore: VirEnv/mainVirCarla.py)
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
There is no shared timestep any more &mdash; there are **three** cadences, and only one
of them is yours to choose:

| cadence | where | value |
| --- | --- | --- |
| FIXS feed / handshake, = SUMO `--step-length` | `fixs::kFeedPeriodS`, `CommonLib/FixsProtocol.h` | **0.1 s, fixed** |
| CARLA world step (`fixed_delta_seconds`) | `CarlaSetup.CarlaTimeStep` in the scenario yaml | 0.1 / 0.05 / 0.025 / 0.02 / 0.01 |
| traffic pose re-apply | `CarlaSetup.TrafficRefreshRate` | &ge; the tick; omit for every tick |

The feed is a property of the protocol, not a preference: every VirEnvCore host tests
its FIXS send/recv boundary against a 0.1 s grid on its own sim clock, and TrafficLayer
steps the traffic simulator exactly once per exchange. Give SUMO a different step and
the two clocks diverge &mdash; a 0.05 s SUMO step means CARLA advances two ticks per SUMO
step, so every sample is drawn twice and the scene plays at half speed. Both bridges
therefore get `--step-length 0.1` from one builder (`sumo_launch_cmd`).

A tick finer than the feed makes the bridge **interpolate** traffic across the
sub-steps &mdash; position *and* heading, on the shortest arc &mdash; which is exactly what the
CarMaker host has always done at 0.001 s. `--carla-tick` sets it for one run and is
written through to the yaml. (`--step-length` on the CLI is accepted and taken as the
tick, with a note.)

Anything that changes **how the traffic behaves** comes from one table in `run_cosim`
(`SUMO_CONTRACT` / `SUMO_CONVENTION`), injected on the SUMO command line for both
bridges and printed with each flag's origin. The map's `.sumocfg` is used as it ships
and never edited: it is app-independent, so a co-sim requirement does not belong in it.
Precedence is `contract > app sumo_args > convention > the map's cfg`, and an app
deviates via `sumo_args` in `apps.json`. Nothing is generated or cached: the flags are
passed and printed, so what a run gave SUMO is in its own log.

`--start` is passed explicitly as `true`/`false` rather than omitted, because omitting
it does not turn it off &mdash; a cfg declaring `<start value="t"/>` (roosevelt's does)
would start stepping anyway, which is why `SumoSetup.AutoStart: false` used to do
nothing on that map.

The loop is paced to **real time** by default (`CarlaSetup.RealtimePacing`, honoured by
both bridges). Speed levers, none of which change the defaults:
- `--carla-timeout S` &mdash; client connect timeout (default 10 s; raise for heavy
  source-build maps that are slow to answer the first RPC).
- `--quality-level Low` &mdash; cheaper rendering on heavy maps.
- `--fast` &mdash; drop real-time pacing; run as fast as the hardware allows.
