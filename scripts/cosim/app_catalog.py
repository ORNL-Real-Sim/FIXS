"""
app_catalog.py - the application manifest contract for FIXS consumer repos.

FIXS is the enabler, not the owner: this module defines and parses the canonical
application manifest, but hardcodes no application name. A repo that sits above a
fetched FIXS bundle declares everything run_cosim needs to know about its
applications in ONE file, looked up in this order:

    $FIXS_APPS_JSON  ->  <repo>/fixs.json  ->  <repo>/apps/apps.json

If none exists, every entry point here degrades to "no apps", and run_cosim
behaves exactly as it did before app awareness existed. Same shape as the
Digital-Twin-Library `catalog.json` that import_map already consumes: FIXS owns
the schema + tooling, the repo owns the data.


Two schemas, both supported
---------------------------
schema 2 (fixs.json) is the generic form. It sits beside the front door and
addresses app folders by a path relative to ITSELF, at any depth - so a repo can
keep its applications wherever they already live and does not need a directory
called apps/ at all. This is what lets FIXS be dropped into a repo that is not
laid out like FIXS_Applications.

schema 1 (apps/apps.json) is the original: `dir` is a bare folder NAME under the
repo's apps/ folder. It keeps working exactly as before and is not deprecated
here - FIXS_Applications has one committed, and a schema-1 manifest must load
byte-for-byte identically and without a word of warning.

The schema is taken from the manifest's own `schema` key. When that is absent it
is inferred from the filename (fixs.json -> 2, apps.json -> 1), which is the only
place the filename means anything; an explicit declaration always wins.


Schema 2 (fixs.json)
--------------------
{
  "schema": 2,
  "fixs": { "repo": "ORNL-Real-Sim/FIXS",             # engine source + pin, read by
            "version": "v0.9.0-alpha" },              #   the front door before any
                                                      #   engine code exists on disk
  "maps": { "repo": "ORNL-Real-Sim/Digital-Twin-Library",
            "tag_prefix": "" },
  "apps": [ ... same entries as below, except that "dir" is a PATH relative to
            this file ("apps/mlk_eco_driving", "src/controllers/eco") rather than
            a name under apps/ ... ]
}

The "fixs" and "maps" blocks are declared here so one file describes the whole
integration, but nothing in this module reads them: the front door regex-reads
fixs.repo / fixs.version before python exists, and import_map owns the map source.


Schema 1 (apps/apps.json)
-------------------------
{
  "schema": 1,
  "apps": [
    {
      "id":     "roosevelt",            # required, unique. Also the default folder
                                        #   name under apps/ and the profile key.
      "title":  "Roosevelt Ave co-sim", # optional display name (default: id)
      "dir":    "roosevelt",            # optional folder under apps/ (default: id)
      "note":   "...",                  # optional, printed when the app is picked
      "maps":   ["roosevelt", ...],     # optional, DEFAULT ["<id>"]; first = default pick
      "configs":[ <config>, ... ],      # optional app-owned scenario yamls
      "launch": "run_my_app",           # optional: a command run alongside the stack
                                        #   (the app's controller / XIL host), which
                                        #   may also report the scenario to run. See
                                        #   below - run_cosim does not read its args.
      "defaults": {                     # optional per-app run defaults (CLI wins)
        "engine": "py"|"cpp", "sumo_gui": true
      },                                # no timestep: the scenario yaml owns the
                                        # cadence, and SUMO's step is the FIXS feed
      "sumo_args": {                    # optional: this app's deviations from the
        "--lateral-resolution": "0",    #   co-sim SUMO convention (run_cosim's
        "--time-to-teleport": "-1",     #   SUMO_CONVENTION). null drops a flag.
        "--step-length": null           #   Contract flags cannot be overridden.
      }
    }
  ]
}

Why `sumo_args` and not an edit to the map's .sumocfg: a Digital-Twin-Library map
ships ONE app-independent scenario, and a co-sim requirement does not belong in a
shared artifact - otherwise every new map needs the same edit before it works. So
run_cosim injects the convention on the SUMO command line (printed, with each flag's
origin) and an app deviates HERE, tracked and reviewed next to its declaration,
valid on every map that app runs against.

`launch` is how an application gets to run under the one entry point. run_cosim starts
SUMO, TrafficLayer and the bridge; an app that also has a controller names it here, and
the user keeps typing `run_cosim` and nothing else. Deliberately opaque: run_cosim
resolves the command in the app folder (extensionless -> .bat on Windows, .sh
elsewhere, the convention run_cosim / import_map / place_tls already use) and passes
every argument after the first token through UNTOUCHED. It never adds, removes or reads
one, so what a controller needs to be told is the app's business and adding an app
costs no engine change.

It is started FIRST, before SUMO, and it may report the scenario to run. run_cosim
gives it a path in FIXS_HANDOFF and waits for a json object to appear there:

    {"sumocfg": "<abs path>"}       run this scenario instead of the bundle's
    {}                             nothing to report; carry on

That inverts who owns the SUMO scenario, which is the point. An application whose
scenario is GENERATED per run - a run directory, its own demand, output paths written
into the config - cannot declare a static path for it, and a config file is the only
place some SUMO outputs can be redirected at all (<timedEvent dest=> has no command
line flag). So the app builds the scenario and says where it put it.

Reporting one also switches the SUMO CONVENTION off. The convention exists because a
Digital-Twin-Library .sumocfg is a SHARED artifact that must not carry one consumer's
co-sim requirement; an app that generated its own has no shared artifact and no such
problem, and imposing sublane lane-changing on it would be changing a scenario behind
its author's back. The CONTRACT (--step-length: one SUMO step per FIXS exchange) still
applies - that is the protocol, not a preference - and `sumo_args` still adds whatever
else an app wants, so what SUMO got is still one printed list either way.

FIXS_HANDOFF is also how the command knows the stack is not its to start: SUMO and
TrafficLayer are already being launched, and a second copy would fight for the TraCI
and bridge ports. An app that ignores the variable entirely still works - it reports
nothing, keeps the convention, and runs the bundle's scenario - so `launch` is usable
without knowing any of this.

A map is just a NAME - the word the picker matches against what already exists:
a Digital-Twin-Library location / cooked map name / release tag (catalog_entry
matches all three), or the name of a map already cooked into this CARLA. Nothing
is downloaded on an app's say-so and nothing is hardcoded per app; the name is a
hint that hoists that map to the top of the picker and makes it the Enter-default.

"maps" defaults to the app's own id, which is the whole point of naming an app
after its location: `roosevelt` and `atlanta` match the library entries of the
same name and declare no maps at all. Declare "maps" only when the map is named
differently from the app (a study app on a shared map, or several maps per app).

A name that matches nothing is simply not offered in the app section - the picker
falls through to the library list, the cooked list, and the local-file option,
i.e. exactly the menu you get with no app selected. So a wrong or not-yet-published
name costs a missing shortcut, never a failed run.

<config> is either a plain string (a path under the app folder) or an object:

    "MLK_Sumo_Scenario/config_Sumo_Carla_default.yaml"
    {
      "path":   "MLK_Sumo_Scenario/config_Sumo_Carla_dSPACE.yaml",   # required
      "title":  "dSPACE XIL",                                        # optional label
      "engine": "cpp"                   # optional: which bridge this yaml is written
                                        #   for. Declared because a hand-written yaml
                                        #   that omits CarlaSetup.EnablePythonBackend
                                        #   otherwise reads as the python bridge by
                                        #   default - silently running the wrong stack.
    }


Why app yamls are staged into ~/.fixs
-------------------------------------
A committed yaml carries machine-specific values (CARLA server IP, dSPACE ports).
Editing it in place makes every user's working tree dirty and invites those
values into the repo. So a declared config is COPIED to

    ~/.fixs/apps/<app_id>/<basename>.yaml

on first use and read from there. Edits are yours and never tracked. The copy is
never silently clobbered: stage_configs() records the source hash, refreshes the
copy only while you have not touched it, and otherwise drops the new upstream
version beside it as <basename>.yaml.new and says so.
"""
import hashlib
import json
import os
import sys

import carla_env_setup as env
import fixs_paths

# Manifest schemas this FIXS understands. 2 is the generic form (fixs.json, app
# folders addressed by a path relative to the manifest); 1 is the original
# (apps/apps.json, app folders named under apps/). BOTH are supported, and a
# schema-1 manifest must behave exactly as it did before schema 2 existed -
# FIXS_Applications has one committed and is not required to convert.
#
# A membership test, not `!= SCHEMA`: the old equality warned on every load the
# moment this constant moved ahead of a manifest, so bumping it would have made
# every existing repo print "schema 1, this FIXS understands 2. Reading it anyway"
# on every single launch. Only a schema we genuinely do not know is worth a word.
KNOWN_SCHEMAS = (1, 2)
SCHEMA = 2                       # what a manifest we generate declares
LEGACY_SCHEMA = 1

MANIFEST_NAME = "fixs.json"
LEGACY_MANIFEST_NAME = "apps.json"

HERE = os.path.dirname(os.path.abspath(__file__))

# Identity for a run with no application selected. It is a real key, not a null:
# a generic run still has scenario yamls and a saved profile, and they need a home
# that cannot collide with an app id (hence the leading underscore).
GENERIC = "_generic"


# --------------------------------------------------------------------------- #
# Locations
# --------------------------------------------------------------------------- #
def app_root():
    """The application repo root: the directory containing FIXS/. Anchored on the
    FIXS root rather than counted up from this file, so it survives a move.
    Mirrors import_map._app_root()."""
    return fixs_paths.app_root(HERE)


def catalog_path(root=None):
    """Path to the app manifest, first of these that exists:

        $FIXS_APPS_JSON  ->  <repo>/fixs.json  ->  <repo>/apps/apps.json

    fixs.json is the generic form (schema 2): it sits beside the front door and
    can point at app folders anywhere in the repo, which is what lets FIXS be
    dropped into a repo that is not laid out like FIXS_Applications. apps/apps.json
    is the original (schema 1) and keeps working untouched.

    Returns the apps/apps.json path when neither exists, so the "no manifest"
    message names the conventional location rather than an invented one."""
    override = os.environ.get("FIXS_APPS_JSON")
    if override:
        return override
    base = root or app_root()
    generic = os.path.join(base, MANIFEST_NAME)
    if os.path.isfile(generic):
        return generic
    return os.path.join(base, "apps", LEGACY_MANIFEST_NAME)


def apps_home(app_id=None):
    """Machine-local app state: ~/.fixs/apps[/<app_id>]. Holds the staged copies of
    an app's scenario yamls. Kept beside carla.json (outside any repo) so edits are
    per-machine and survive `initialize`, which wipes FIXS/."""
    d = os.path.join(os.path.dirname(env.CONFIG_PATH), "apps")
    if app_id:
        d = os.path.join(d, app_id)
    return d


def app_dir(app, root=None):
    """Absolute path of the app's folder.

    load_catalog resolved this once, against the manifest that declared it, and
    parked it on the entry - so read that. The fallback is for app dicts built by
    hand (tests, callers that never loaded a manifest) and keeps the original
    schema-1 rule."""
    if app.get("path"):
        return app["path"]
    return os.path.join(root or app_root(), "apps", app.get("dir") or app["id"])


def launch_command(app, root=None):
    """(argv, cwd) for the app's `launch` command, or (None, None) if it declares none.

    The first token is resolved in the app folder and given the platform's script
    extension when it has none - `run_mlk_eco_driving` -> run_mlk_eco_driving.bat on
    Windows, .sh elsewhere - which is the convention run_cosim / import_map /
    place_tls already ship both halves of. Everything after the first token is passed
    through verbatim and never interpreted: the app owns its own arguments."""
    if not app or not app.get("launch"):
        return None, None
    import shlex
    parts = shlex.split(app["launch"], posix=(os.name != "nt"))
    if not parts:
        return None, None
    here = app_dir(app, root)
    exe = parts[0]
    if not os.path.splitext(exe)[1]:
        exe += ".bat" if os.name == "nt" else ".sh"
    path = exe if os.path.isabs(exe) else os.path.join(here, exe)
    if not os.path.isfile(path):
        _warn(f"app '{app['id']}': launch command '{app['launch']}' not found "
              f"at {path}; nothing will be started for it.")
        return None, None
    return [path] + parts[1:], here


def scenario_dir(app_id, map_name=None):
    """Where an app's scenario yamls live: ~/.fixs/apps/<app_id>/, flat.

    One folder per application, mirroring apps/<app_id>/ in the repo, with the
    yamls sitting directly in it - the staged copies of the app's own configs and
    the generated per-map ones side by side. `map_name` is accepted and ignored so
    callers can pass it; the map is in the FILE name, not in a subfolder.

    Scenario yamls are app-bounded, never map-bounded, because they are edited and
    ~/.fixs/maps/ is a cache of downloaded artifacts a user should be able to
    delete wholesale to reclaim gigabytes. GENERIC is the key for a run with no
    application selected."""
    return apps_home(app_id or GENERIC)


def scenario_path(app_id, map_name):
    """The generated scenario yaml: ~/.fixs/apps/<app_id>/<map_name>.yaml.

    Named for the map rather than nested under one, so the app folder stays flat
    and two maps under the same app still get their own file - the CARLA endpoint
    and TL subscriptions inside are specific to (app, map)."""
    return os.path.join(scenario_dir(app_id), f"{map_name}.yaml")


def migrate_scenarios(app_id, map_name, legacy_dir, quiet=False):
    """Move scenario yamls from where older FIXS versions put them, once.

    Two earlier homes, both left behind rather than abandoned - these files are
    hand-edited, so a user who tuned CarlaServerIP in one must not silently get a
    freshly generated default instead:

        ~/.fixs/maps/<map>/*.yaml                 (before yamls were app-bounded)
        ~/.fixs/apps/<app>/maps/<map>/*.yaml      (before the app folder went flat)

    The map's own config.yaml becomes <map>.yaml; anything beside it keeps its
    name, prefixed with the map if that would collide. Never overwrites: a name
    already taken in the destination is left where it is."""
    import shutil
    dest = scenario_dir(app_id)
    moved = []
    sources = [legacy_dir, os.path.join(dest, "maps", map_name)]
    for src in sources:
        try:
            names = sorted(f for f in os.listdir(src)
                           if f.lower().endswith((".yaml", ".yml")))
        except OSError:
            continue
        for name in names:
            stem, ext = os.path.splitext(name)
            # Everything here belonged to ONE map, so everything gets that map's
            # name: config.yaml is the generated one and becomes <map>.yaml, a
            # variant beside it becomes <map>_<variant>.yaml. Without the prefix a
            # variant would sort ahead of the generated yaml in the flat folder and
            # be offered as the default - and would read as if it applied to every
            # map the app runs, which it does not.
            target = f"{map_name}{ext}" if stem == "config" else f"{map_name}_{name}"
            if os.path.exists(os.path.join(dest, target)):
                continue                   # already migrated; leave the original
            try:
                os.makedirs(dest, exist_ok=True)
                shutil.move(os.path.join(src, name), os.path.join(dest, target))
                moved.append(f"{name} -> {target}")
            except OSError as exc:
                _warn(f"could not move {name} ({exc}); leaving it in {src}.")
        # Tidy the emptied nested folder so the old shape does not linger.
        try:
            os.rmdir(src)
            os.rmdir(os.path.dirname(src))
        except OSError:
            pass
    if moved and not quiet:
        print(f"[apps] scenario configs live flat under the app now; moved "
              f"{', '.join(moved)}\n[apps]   -> {dest}")
    return moved


# --------------------------------------------------------------------------- #
# Parsing
# --------------------------------------------------------------------------- #
def _warn(msg):
    print(f"[apps] {msg}")


def _normalize_map(raw, app_id):
    """A declared map -> its name, or None if unusable. An object with a 'name' key
    is accepted too, so a manifest that grows richer map entries later still loads
    on today's FIXS."""
    if isinstance(raw, dict):
        raw = raw.get("name")
    if not isinstance(raw, str) or not raw.strip():
        _warn(f"app '{app_id}': ignoring a map entry that is not a name.")
        return None
    return raw.strip()


def _normalize_config(raw, app_id):
    """A <config> entry -> {path (relative), title, engine}, or None if unusable."""
    if isinstance(raw, str):
        raw = {"path": raw}
    if not isinstance(raw, dict):
        _warn(f"app '{app_id}': ignoring a config entry that is neither a string nor an object.")
        return None
    path = (raw.get("path") or "").strip()
    if not path:
        _warn(f"app '{app_id}': ignoring a config entry with no 'path'.")
        return None
    engine = (raw.get("engine") or "").strip().lower() or None
    if engine not in (None, "py", "cpp"):
        _warn(f"app '{app_id}': config '{path}' declares engine '{engine}'; "
              f"expected 'py' or 'cpp'. Ignoring the declaration.")
        engine = None
    return {"path": path.replace("\\", "/"),
            "title": (raw.get("title") or "").strip(),
            "engine": engine}


def _resolve_dir(declared, schema, base):
    """Absolute path of an app folder, per the schema that declared it.

    schema 1 - `dir` is a NAME under the repo's apps/ folder, and `base` is the
               repo root: <repo>/apps/<dir>. Unchanged from before schema 2, and
               the reason a committed apps/apps.json needs no edits.
    schema 2 - `dir` is a PATH relative to the manifest itself, at any depth
               ("apps/mlk_eco_driving", "src/controllers/eco", "../shared/app").
               That is the whole generalization: an app folder no longer has to
               live under a directory called apps/.

    Keyed on the declared schema, never on which filename the manifest came from,
    so the two forms stay independent of where they are written."""
    if schema == LEGACY_SCHEMA:
        return os.path.join(base, "apps", declared)
    return os.path.normpath(os.path.join(base, *declared.split("/")))


def _normalize_app(raw, schema=LEGACY_SCHEMA, base=None):
    """One manifest entry -> a normalized app dict, or None if unusable.

    `base` is the directory app folders are resolved against - the repo root for
    schema 1, the manifest's own folder for schema 2. Omitted by callers that only
    want the declared fields normalized, which leaves 'path' off the result."""
    if not isinstance(raw, dict):
        _warn("ignoring a manifest entry that is not an object.")
        return None
    app_id = (raw.get("id") or "").strip()
    if not app_id:
        _warn("ignoring an app entry with no 'id'.")
        return None
    defaults = raw.get("defaults")
    if not isinstance(defaults, dict):
        if defaults is not None:
            _warn(f"app '{app_id}': 'defaults' is not an object; ignoring it.")
        defaults = {}
    # This app's deviations from the co-sim SUMO convention. Kept as raw text keyed by
    # the SUMO flag, so a new SUMO option needs no support here; run_cosim decides
    # precedence and prints where each flag came from. A null value drops a convention
    # flag. Keys must look like flags, or a typo would be passed to SUMO verbatim.
    sumo_args = raw.get("sumo_args")
    if not isinstance(sumo_args, dict):
        if sumo_args is not None:
            _warn(f"app '{app_id}': 'sumo_args' is not an object; ignoring it.")
        sumo_args = {}
    clean_args = {}
    for flag, value in sumo_args.items():
        if isinstance(flag, str) and flag.startswith("--"):
            clean_args[flag] = value
        else:
            _warn(f"app '{app_id}': sumo_args key '{flag}' is not a --flag; ignoring it.")
    sumo_args = clean_args
    # No "maps" -> the app id IS the map name to look for. Apps named after their
    # location (roosevelt, atlanta) therefore need no map declaration at all.
    maps = [m for m in (_normalize_map(m, app_id) for m in raw.get("maps") or []) if m]
    if not maps:
        maps = [app_id]
    configs = [c for c in (_normalize_config(c, app_id) for c in raw.get("configs") or []) if c]
    # Extra python packages this app needs on top of the engine's own env, as a path
    # relative to the app folder. The engine owns environment.yml and nothing else;
    # an app's plotting or analysis stack is the app's business, and pushing it
    # upstream would put every FIXS consumer's env at the mercy of one application.
    # Absent -> the app declares none, and nothing is installed for it.
    requirements = (raw.get("requirements") or "").strip() or None
    # A command run alongside the co-sim stack - the app's controller, XIL host, or
    # whatever else attaches to TrafficLayer, and the thing that may report which
    # scenario to run (see FIXS_HANDOFF above). Kept as one opaque string: run_cosim
    # resolves it in the app folder and never reads its arguments, so what an app
    # needs to pass itself costs no change here.
    launch = (raw.get("launch") or "").strip() or None
    app = {"id": app_id,
           "title": (raw.get("title") or "").strip() or app_id,
           "dir": (raw.get("dir") or "").strip() or app_id,
           "note": (raw.get("note") or "").strip() or None,
           "maps": maps,
           "configs": configs,
           "defaults": defaults,
           "sumo_args": sumo_args,
           "requirements": requirements,
           "launch": launch}
    # Where the folder actually is, decided once, here, where the manifest that
    # declared it is still in hand. Everything downstream reads app['path'] and
    # never re-derives a location from a schema rule.
    if base is not None:
        app["path"] = _resolve_dir(app["dir"], schema, base)
    return app


def load_catalog(root=None):
    """The declared applications, newest schema rules applied: a list of normalized
    app dicts (possibly empty). Never raises - a missing, unreadable or malformed
    manifest degrades to [] with a warning, because app awareness is an enhancement
    to run_cosim, not a precondition for it."""
    path = catalog_path(root)
    if not os.path.isfile(path):
        return []
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError) as exc:
        _warn(f"could not read {path} ({exc}); continuing without apps.")
        return []
    if not isinstance(doc, dict):
        _warn(f"{path}: top level must be an object; continuing without apps.")
        return []
    # An absent 'schema' is read from the filename: fixs.json is the generic form,
    # apps.json the original. This is the ONE place the filename carries meaning,
    # and an explicit declaration always wins over it - so either form can be
    # written in either file, and a hand-written fixs.json needs no boilerplate.
    declared = doc.get("schema")
    if declared is None:
        schema = SCHEMA if os.path.basename(path) == MANIFEST_NAME else LEGACY_SCHEMA
    elif declared in KNOWN_SCHEMAS:
        schema = declared
    else:
        # Only an unrecognised schema is worth saying anything about. Warning
        # whenever `declared != SCHEMA` meant every repo with a committed schema-1
        # manifest printed a scary line on every launch the moment SCHEMA moved.
        schema = SCHEMA
        _warn(f"{path}: schema {declared} is not one this FIXS knows "
              f"({', '.join(str(s) for s in KNOWN_SCHEMAS)}). Reading it as "
              f"schema {schema}; update FIXS if entries look wrong.")
    # schema 1 resolves app folders against the repo root, schema 2 against the
    # manifest's own folder - which for <repo>/apps/apps.json and a schema-1
    # 'dir' are the same place, and that is what keeps today's repos identical.
    base = (root or app_root()) if schema == LEGACY_SCHEMA else os.path.dirname(path)
    apps = [a for a in (_normalize_app(a, schema, base) for a in doc.get("apps") or []) if a]
    seen = {}
    for a in apps:
        if a["id"] in seen:
            _warn(f"duplicate app id '{a['id']}' in {path}; the first one wins.")
        seen.setdefault(a["id"], a)
    return list(seen.values())


def find_app(apps, ident):
    """The app known by `ident` - its id, its folder, or its title (case-insensitive)
    - or None. Accepts a folder path too, so `--app apps/roosevelt` works."""
    if not ident:
        return None
    key = os.path.basename(str(ident).replace("\\", "/").rstrip("/")).strip().lower()
    for a in apps or []:
        # basename on 'dir' too: under schema 2 it is a path, so `--app eco` has to
        # match a folder declared as "src/controllers/eco". Under schema 1 it is
        # already a bare name and basename leaves it alone.
        folder = os.path.basename(a["dir"].replace("\\", "/").rstrip("/")).lower()
        if key in (a["id"].lower(), folder, a["title"].lower()):
            return a
    return None


# --------------------------------------------------------------------------- #
# Picking
# --------------------------------------------------------------------------- #
def choose_app(apps, root=None, current=None):
    """Numbered menu of the declared applications; returns the chosen app dict, or
    None for "no app" (the generic, pre-app-awareness run). Auto-selects nothing:
    even a single app is offered, because the None escape has to stay reachable.
    Returns None (silently) in a non-interactive session so callers keep working
    from --app / a saved profile.

    `current` is the app id the setup is running: it is marked, and Enter keeps it.
    Without it Enter always meant item 1, so opening this row on a setup running any
    other app and pressing Enter switched the app - and took the map and the
    scenario yaml with it, since both are invalidated by an app change."""
    if not apps:
        return None
    if not sys.stdin.isatty():
        return None
    ids = [a["id"] for a in apps]
    idx = ids.index(current) + 1 if current in ids else 1
    print("\n[apps] Pick an application to run:")
    for i, a in enumerate(apps, 1):
        missing = "" if os.path.isdir(app_dir(a, root)) else "   (folder missing)"
        maps = ", ".join(a["maps"]) or "-"
        mark = "  (current)" if a["id"] == current else ""
        print(f"   {i:>2}) {a['title']:<34} maps: {maps}{missing}{mark}")
    print("    0) none - just pick a map (generic co-sim)")
    while True:
        try:
            ans = input(f"[apps] Which? [0-{len(apps)}], Enter = {idx}: ").strip()
        except EOFError:
            return None
        if ans == "":
            return apps[idx - 1]
        if ans == "0":
            return None
        if ans.isdigit() and 1 <= int(ans) <= len(apps):
            return apps[int(ans) - 1]
        print("[apps] invalid choice; enter a number from the list.")


# --------------------------------------------------------------------------- #
# Scenario yamls: repo -> ~/.fixs/apps/<id>/
# --------------------------------------------------------------------------- #
def _sha256(path):
    h = hashlib.sha256()
    try:
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
    except OSError:
        return None
    return h.hexdigest()


def _stage_index_path(app_id):
    return os.path.join(apps_home(app_id), ".sources.json")


def _load_stage_index(app_id):
    try:
        with open(_stage_index_path(app_id), encoding="utf-8") as f:
            doc = json.load(f)
        return doc if isinstance(doc, dict) else {}
    except (OSError, ValueError):
        return {}


def _save_stage_index(app_id, index):
    try:
        os.makedirs(apps_home(app_id), exist_ok=True)
        with open(_stage_index_path(app_id), "w", encoding="utf-8") as f:
            json.dump(index, f, indent=2)
    except OSError:
        pass  # the index is an optimisation; a lost one only costs a refresh notice


def stage_configs(app, root=None, quiet=False):
    """Copy the app's declared scenario yamls into ~/.fixs/apps/<id>/ and return
    [{path, title, engine, source}] for the ones that exist (path = the STAGED copy,
    which is what run_cosim reads and the user edits).

    Update policy, so a machine-specific edit is never lost and an upstream fix is
    never silently withheld:
      - copy missing         -> copy it
      - upstream unchanged   -> leave it alone
      - upstream changed, copy untouched -> refresh in place (you had no edits)
      - upstream changed, copy edited    -> keep yours, write <name>.yaml.new, say so
    """
    import shutil
    staged = []
    if not app or not app.get("configs"):
        return staged
    dest_dir = apps_home(app["id"])
    index = _load_stage_index(app["id"])
    changed = False
    for cfg in app["configs"]:
        src = os.path.join(app_dir(app, root), *cfg["path"].split("/"))
        if not os.path.isfile(src):
            _warn(f"app '{app['id']}': declared config not found, skipping: {src}")
            continue
        base = os.path.basename(src)
        dst = os.path.join(dest_dir, base)
        src_hash = _sha256(src)
        recorded = (index.get(base) or {}).get("hash")
        try:
            os.makedirs(dest_dir, exist_ok=True)
            if not os.path.isfile(dst):
                shutil.copy2(src, dst)
                if not quiet:
                    print(f"[apps] staged {base} -> {dst}\n"
                          f"[apps]   edit that copy for machine-specific values "
                          f"(IPs, ports); it is never committed.")
                index[base] = {"source": cfg["path"], "hash": src_hash}
                changed = True
            elif src_hash and src_hash != recorded:
                if recorded and _sha256(dst) == recorded:
                    shutil.copy2(src, dst)          # untouched copy: safe to refresh
                    if not quiet:
                        print(f"[apps] {base} updated upstream and your copy was "
                              f"unedited; refreshed {dst}")
                else:
                    new = dst + ".new"
                    shutil.copy2(src, new)
                    if not quiet:
                        print(f"[apps] {base} changed upstream but your copy has local "
                              f"edits.\n[apps]   keeping yours; new version written to "
                              f"{os.path.basename(new)} - merge if you want it.")
                index[base] = {"source": cfg["path"], "hash": src_hash}
                changed = True
        except OSError as exc:
            _warn(f"app '{app['id']}': could not stage {base} ({exc}); using the repo copy.")
            dst = src
        staged.append({"path": dst, "title": cfg["title"] or base,
                       "engine": cfg["engine"], "source": src})
    if changed:
        _save_stage_index(app["id"], index)
    return staged
