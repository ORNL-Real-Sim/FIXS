"""app_setup.py - declare an application in this repo's fixs.json.

The manifest is small enough to hand-write, and for a while that was the answer.
It is the wrong answer for the thing this actually gates: a new integrator does
not struggle to type four JSON fields, they struggle to know that a launcher is
handed FIXS_PYTHON, FIXS_CONFIG_YAML and FIXS_HANDOFF, and that writing
{"sumocfg": ...} to the last one is how an application takes over the scenario.
None of that is discoverable from an empty "apps": [].

So the useful half of this module is the SCAFFOLD: it writes a launcher pair that
documents the contract in the file where it is needed. Finding an existing script
and appending an entry is the easy part.

Deliberately small. It writes four fields - launch, title, description, maps -
and stops. requirements, sumo_args, defaults and an explicit id are all still
accepted by app_catalog; an app that needs them is past the point where a wizard
helps, and asking six more questions to reach an empty answer is worse than not
asking.
"""
import io
import json
import os
import re

import app_catalog

# Where we refuse to go looking for launchers. FIXS/ is the engine's own bundle
# (it has plenty of .bat files, none of them an application), and the rest are
# large, uninteresting, or not source.
SKIP_DIRS = {"FIXS", ".git", "__pycache__", "node_modules", ".venv", "venv",
             "RealSim_tmp", "build", "dist", ".idea", ".vscode"}
SCAN_DEPTH = 4          # deep enough for projects/<name>/<variant>/, shallow enough to be quick


# --------------------------------------------------------------------------- #
# Finding what is already there
# --------------------------------------------------------------------------- #
def find_launchers(root, declared=()):
    """Candidate launcher scripts under `root`, as manifest-relative extensionless
    paths, sorted. A .bat and .sh of the same name collapse to one entry - that
    pair IS one launcher, which is why the manifest spells it without a suffix.

    `declared` are launch strings already in the manifest; they are excluded so
    the list only offers what is not yet an application."""
    taken = {d.replace("\\", "/").strip() for d in declared}
    found = set()
    root = os.path.abspath(root)
    for dirpath, dirnames, filenames in os.walk(root):
        rel = os.path.relpath(dirpath, root)
        depth = 0 if rel == "." else rel.count(os.sep) + 1
        # prune in place so os.walk does not descend
        dirnames[:] = [] if depth >= SCAN_DEPTH else [
            d for d in dirnames if d not in SKIP_DIRS and not d.startswith(".")]
        for name in filenames:
            stem, ext = os.path.splitext(name)
            if ext.lower() not in (".bat", ".sh"):
                continue
            p = os.path.join(dirpath, name)
            relp = os.path.relpath(os.path.splitext(p)[0], root).replace("\\", "/")
            if relp not in taken:
                found.add(relp)
    return sorted(found)


def _both_halves(root, launch):
    """(has_bat, has_sh) for an extensionless manifest-relative launcher."""
    base = os.path.join(root, *launch.split("/"))
    return os.path.isfile(base + ".bat"), os.path.isfile(base + ".sh")


# --------------------------------------------------------------------------- #
# Naming
# --------------------------------------------------------------------------- #
def slug(text):
    """A title -> a key that is safe as a directory name and as a CLI word."""
    s = re.sub(r"[^0-9a-zA-Z]+", "_", (text or "").strip().lower()).strip("_")
    return re.sub(r"_+", "_", s)


def derive_id(launch, title=None, taken=()):
    """The id for a new entry: the launcher's own basename (minus a leading
    'run_'), else a slug of the title. Suffixed until unique, because the id is a
    directory under ~/.fixs/apps/ and a collision would silently share one app's
    remembered scenario with another."""
    base = app_catalog._derive_id(launch, app_catalog.SCHEMA) or slug(title) or "app"
    base = slug(base) or "app"
    cand, n = base, 2
    while cand in set(taken):
        cand, n = f"{base}_{n}", n + 1
    return cand


# --------------------------------------------------------------------------- #
# Scaffolding a launcher
# --------------------------------------------------------------------------- #
_SH = """#!/usr/bin/env bash
# {title}
#
# Started by run_cosim BEFORE SUMO, because an application may decide which
# scenario the run uses. run_cosim never reads this script's arguments - what it
# needs to tell itself is its own business.
#
# What FIXS puts in the environment for you:
#
#   FIXS_PYTHON        the interpreter run_cosim resolved and installed this app's
#                      requirements into. USE THIS, not whatever python is on PATH -
#                      it is the only env with the carla + SUMO clients.
#   FIXS_CONFIG_YAML   the scenario yaml TrafficLayer is being given. Read it to
#                      learn the ports, the map and the cadence.
#   FIXS_HANDOFF       where to report a scenario you generated yourself:
#                        echo '{{"sumocfg": "/abs/path.sumocfg"}}' > "$FIXS_HANDOFF"
#                      Write {{}} (or nothing) to run the map's own scenario.
#                      Reporting one also switches OFF run_cosim's SUMO convention:
#                      a scenario you built is yours, and it will not be second-
#                      guessed. The --step-length contract still applies.
#
# The stack is already starting. Do NOT launch SUMO or TrafficLayer here - a
# second copy would fight for the TraCI and bridge ports.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${{FIXS_PYTHON:-python}}"

# Nothing to report: run the scenario the map ships with. Delete these two lines
# once this app generates its own.
[[ -n "${{FIXS_HANDOFF:-}}" ]] && printf '{{}}' > "$FIXS_HANDOFF"

echo "[{id}] starting under $PY"
exec "$PY" "$HERE/main.py" "$@"
"""

_BAT = """@echo off
REM {title}
REM
REM Started by run_cosim BEFORE SUMO, because an application may decide which
REM scenario the run uses. run_cosim never reads this script's arguments - what it
REM needs to tell itself is its own business.
REM
REM What FIXS puts in the environment for you:
REM
REM   FIXS_PYTHON        the interpreter run_cosim resolved and installed this
REM                      app's requirements into. USE THIS, not whatever python is
REM                      on PATH - it is the only env with the carla + SUMO clients.
REM   FIXS_CONFIG_YAML   the scenario yaml TrafficLayer is being given. Read it to
REM                      learn the ports, the map and the cadence.
REM   FIXS_HANDOFF       where to report a scenario you generated yourself:
REM                        echo {{"sumocfg": "C:/abs/path.sumocfg"}} > %FIXS_HANDOFF%
REM                      Write {{}} (or nothing) to run the map's own scenario.
REM                      Reporting one also switches OFF run_cosim's SUMO
REM                      convention: a scenario you built is yours. The
REM                      --step-length contract still applies.
REM
REM The stack is already starting. Do NOT launch SUMO or TrafficLayer here - a
REM second copy would fight for the TraCI and bridge ports.
setlocal
set "HERE=%~dp0"
set "PY=%FIXS_PYTHON%"
if not defined PY set "PY=python"

REM Nothing to report: run the scenario the map ships with. Delete these two lines
REM once this app generates its own.
if defined FIXS_HANDOFF echo {{}}> "%FIXS_HANDOFF%"

echo [{id}] starting under %PY%
"%PY%" "%HERE%main.py" %*
exit /b %ERRORLEVEL%
"""


def scaffold(root, launch, title, app_id):
    """Write the .bat + .sh pair for `launch` (manifest-relative, extensionless).
    Returns the paths written. Never overwrites: an existing half is left alone,
    which is also how you add the missing half to a one-platform app."""
    base = os.path.join(root, *launch.split("/"))
    os.makedirs(os.path.dirname(base) or root, exist_ok=True)
    written = []
    for ext, body, newline in ((".sh", _SH, "\n"), (".bat", _BAT, "\r\n")):
        path = base + ext
        if os.path.isfile(path):
            continue
        with io.open(path, "w", encoding="utf-8", newline=newline) as f:
            f.write(body.format(title=title, id=app_id))
        if ext == ".sh":
            try:
                os.chmod(path, os.stat(path).st_mode | 0o111)
            except OSError:
                pass                      # best effort; git records the mode anyway
        written.append(path)
    return written


# --------------------------------------------------------------------------- #
# Writing the manifest
# --------------------------------------------------------------------------- #
def build_entry(launch, title=None, description=None, maps=None, app_id=None):
    """The manifest entry, in the order a human wants to read it. Empty fields are
    omitted rather than written as null - a file full of nulls reads like settings
    someone turned off."""
    entry = {"launch": launch}
    if app_id:
        entry["id"] = app_id
    if title:
        entry["title"] = title
    if description:
        entry["description"] = description
    if maps:
        entry["maps"] = list(maps)
    return entry


def add_entry(manifest_path, entry, repo=None, version=None):
    """Append `entry` to the manifest's "apps", creating the file if needed.

    Rewrites the whole document with json.dump, so any "//" comment keys survive
    (they are ordinary keys) but formatting choices do not. That is the trade for
    not carrying a JSON round-tripper; the file is small and machine-owned.

    Refuses a schema-1 manifest rather than writing a schema-2 entry into it - the
    two disagree about what `launch` and `dir` mean, and a silent mix would resolve
    app folders to places that do not exist."""
    doc = {}
    if os.path.isfile(manifest_path):
        with io.open(manifest_path, encoding="utf-8") as f:
            doc = json.load(f) or {}
        if not isinstance(doc, dict):
            raise ValueError(f"{manifest_path}: top level must be an object.")
    declared = doc.get("schema")
    name = os.path.basename(manifest_path)
    if declared == app_catalog.LEGACY_SCHEMA or (
            declared is None and name == app_catalog.LEGACY_MANIFEST_NAME):
        raise ValueError(
            f"{manifest_path} is a schema-{app_catalog.LEGACY_SCHEMA} manifest. "
            f"Its 'launch' and 'dir' mean something different, so a new-style "
            f"entry cannot be appended to it. Convert it to fixs.json first.")

    doc.setdefault("schema", app_catalog.SCHEMA)
    fixs = doc.setdefault("fixs", {})
    if repo and not fixs.get("repo"):
        fixs["repo"] = repo
    if version and not fixs.get("version"):
        fixs["version"] = version
    doc.setdefault("apps", [])
    if not isinstance(doc["apps"], list):
        raise ValueError(f"{manifest_path}: 'apps' must be a list.")
    doc["apps"].append(entry)

    tmp = manifest_path + ".tmp"
    with io.open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(doc, f, indent=2, ensure_ascii=False)
        f.write("\n")
    os.replace(tmp, manifest_path)       # atomic: never a half-written manifest
    return manifest_path


# --------------------------------------------------------------------------- #
# The walkthrough
# --------------------------------------------------------------------------- #
def _ask(prompt, default=""):
    try:
        ans = input(prompt).strip()
    except EOFError:
        return default
    return ans or default


def _yes(prompt, default=True):
    d = "Y/n" if default else "y/N"
    ans = _ask(f"{prompt} [{d}]: ").lower()
    return default if not ans else ans.startswith("y")


def run_wizard(root, map_name=None, repo=None, version=None):
    """Declare an application, interactively. Returns its id, or None if the user
    backed out. The map comes in already chosen - that is why this runs from the
    app row AFTER a map is picked, so the one question it could not answer well
    ("which map?") is already answered."""
    manifest = app_catalog.catalog_path(root)
    existing = app_catalog.load_catalog(root)
    taken = {a["id"] for a in existing}

    print("\n[apps] Declaring an application in %s" % os.path.basename(manifest))
    print("[apps] An application is a controller, XIL host, or anything else that\n"
          "       runs alongside the co-sim. FIXS starts it and tells it where the\n"
          "       stack is; what it does with that is yours.")

    found = find_launchers(root, declared=[a["launch"] for a in existing if a["launch"]])
    launch = None
    if found:
        print("\n[apps] Scripts in this repo that could be its launcher:")
        for i, f in enumerate(found[:20], 1):
            bat, sh = _both_halves(root, f)
            halves = "".join(x for x, ok in ((".bat", bat), (".sh", sh)) if ok)
            print(f"   {i:>2}) {f:<52} {halves}")
        if len(found) > 20:
            print(f"       ... and {len(found) - 20} more (choose 'n' to type a path)")
        ans = _ask(f"[apps] Which one? [1-{min(len(found), 20)}], "
                   f"or Enter to create a new launcher: ")
        if ans.isdigit() and 1 <= int(ans) <= min(len(found), 20):
            launch = found[int(ans) - 1]
    else:
        print("\n[apps] No launcher scripts found in this repo yet.")

    scaffolding = launch is None
    if scaffolding:
        # The reason this module exists: hand over the FIXS_* contract in the file
        # where it is needed, instead of leaving it to be found in the docs.
        launch = _ask("[apps] Path for the new launcher, relative to the repo and\n"
                      "       WITHOUT an extension (e.g. projects/autolab/run_ctrl): ")
        launch = launch.replace("\\", "/").strip("/")
        if not launch:
            print("[apps] nothing entered; not declaring an application.")
            return None

    default_title = app_catalog._derive_id(launch, app_catalog.SCHEMA) or "app"
    title = _ask(f"[apps] Name it, for the menu [{default_title}]: ", default_title)
    description = _ask("[apps] One line about what it does (optional): ")

    app_id = derive_id(launch, title, taken)
    if app_id != app_catalog._derive_id(launch, app_catalog.SCHEMA):
        print(f"[apps] id '{app_id}'   (that name is taken, or the launcher gave none)")
    else:
        print(f"[apps] id '{app_id}'   (--app {app_id}, and ~/.fixs/apps/{app_id}/)")

    maps = None
    if map_name and _yes(f"[apps] Does it only work on the '{map_name}' network?",
                         default=False):
        maps = [map_name]

    entry = build_entry(launch, title=title, description=description, maps=maps,
                        app_id=app_id if app_id != app_catalog._derive_id(
                            launch, app_catalog.SCHEMA) else None)
    print("\n[apps] adding to %s:" % manifest)
    for line in json.dumps(entry, indent=2, ensure_ascii=False).splitlines():
        print("         " + line)
    if not _yes("[apps] write it?", default=True):
        print("[apps] nothing written.")
        return None

    if scaffolding:
        for p in scaffold(root, launch, title, app_id):
            print(f"[apps] wrote {os.path.relpath(p, root)}")
        print("[apps] Both halves are written so the app runs on either platform.\n"
              "       Edit them - they start ./main.py and report no scenario.")
    else:
        bat, sh = _both_halves(root, launch)
        if not (bat and sh):
            missing = ".sh" if bat else ".bat"
            print(f"[apps] NOTE: only the {'.bat' if bat else '.sh'} half exists. "
                  f"On the other platform this app will not start;\n"
                  f"       add {launch}{missing} when you need it.")

    add_entry(manifest, entry, repo=repo, version=version)
    print(f"[apps] {os.path.basename(manifest)} updated - commit it.")
    return app_id
