"""
run_profile.py - remembered co-sim run setups (~/.fixs/run_profiles.json).

The first interactive run answers a handful of questions (which app, which map,
which scenario yaml, which bridge). Answering all of them again on every later
run is the friction this module removes: the answers are saved as a PROFILE, and
the next run opens with a summary plus a numbered menu, so you re-answer only the
parts you actually want to change.

    [cosim] Saved setup 'roosevelt' (last run 2026-07-29 10:22):
       1) app      roosevelt
       2) map      roosevelt_full
       3) scenario config.yaml            (generated, per-map)
       4) engine   py
       5) CARLA    source  C:/src_ext/Carla  ->  127.0.0.1:2000
       6) SUMO     gui, step 0.05
    [cosim] Enter = run as-is | numbers to change (e.g. "2 4") | A = all | Q = quit:

Storage is profile-SHAPED even though today's UI only ever manages one profile
per application (its id is the app id, or '_generic' with no app). The file
carries {active, profiles{}} from the start, so a future front-end can list,
name, and switch saved test setups without a format migration or a second store.

Cascades are data-driven rather than hardcoded: a record records whether its
scenario yaml is app-scoped or map-scoped, so "I want a different map" can know
whether the yaml choice survives.

    changed 'app'  -> map and scenario are re-asked (they belonged to the old app)
    changed 'map'  -> scenario re-asked only if it was the map's generated yaml

Explicit CLI flags always win over a profile: the caller applies a slot only when
the corresponding flag was not given, so `run_cosim --map atlanta` changes exactly
that one thing with no prompt at all.
"""
import json
import os
import sys
from datetime import datetime

import app_catalog
import carla_env_setup as env

SCHEMA = 1
# Profile id for a run with no application selected. Shared with app_catalog rather
# than spelled twice: it is the same identity that names ~/.fixs/apps/_generic/.
GENERIC = app_catalog.GENERIC

# Slot key -> (menu label, cascade: slots invalidated when this one changes).
# The order here is the order shown in the summary.
SLOTS = [
    ("app", "app"),
    ("map", "map"),
    ("config", "scenario"),
    ("engine", "engine"),
    ("carla", "CARLA"),
    ("sumo", "SUMO"),
]


def profiles_path():
    """~/.fixs/run_profiles.json - beside carla.json, outside any repo."""
    return os.path.join(os.path.dirname(env.CONFIG_PATH), "run_profiles.json")


# --------------------------------------------------------------------------- #
# Store
# --------------------------------------------------------------------------- #
def load_doc():
    """The whole profile document: {"schema", "active", "profiles": {...}}. A
    missing/corrupt file yields an empty document - a lost profile costs one extra
    round of prompts, so it is never worth failing a run over."""
    try:
        with open(profiles_path(), encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError):
        doc = {}
    if not isinstance(doc, dict):
        doc = {}
    doc.setdefault("schema", SCHEMA)
    doc.setdefault("active", None)
    if not isinstance(doc.get("profiles"), dict):
        doc["profiles"] = {}
    return doc


def save_doc(doc):
    path = profiles_path()
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2)
            f.write("\n")
    except OSError as exc:
        print(f"[cosim] could not save the run profile ({exc}); this run is not remembered.")


def resolve(doc, name=None, app_id=None):
    """(profile_id, record) to open with.

    Precedence: an explicit --profile name, else the profile for `app_id` (so
    switching app recalls that app's own last setup), else whatever ran last.
    `record` is None when nothing is remembered yet."""
    profiles = doc.get("profiles") or {}
    if name:
        return name, profiles.get(name)
    if app_id:
        return app_id, profiles.get(app_id)
    active = doc.get("active")
    if active and active in profiles:
        return active, profiles[active]
    return None, None


def remember(profile_id, record):
    """Store `record` under `profile_id` and make it the active profile."""
    doc = load_doc()
    record = dict(record)
    record["updated"] = datetime.now().isoformat(timespec="seconds")
    doc["profiles"][profile_id] = record
    doc["active"] = profile_id
    save_doc(doc)
    return record


# --------------------------------------------------------------------------- #
# Summary + change menu
# --------------------------------------------------------------------------- #
def _fmt(slot, rec, carla_cfg):
    """One summary line's value text for `slot`."""
    if slot == "app":
        return rec.get("app") or "(none - generic co-sim)"
    if slot == "map":
        origin = rec.get("map_origin")
        return (rec.get("map") or "?") + (f"   ({origin})" if origin else "")
    if slot == "config":
        path = rec.get("config")
        if not path:
            return "(auto-generated per map)"
        scope = rec.get("config_scope") or "map"
        where = "app-owned, editable in ~/.fixs/apps" if scope == "app" \
            else "generated, per-map"
        return f"{os.path.basename(path)}   ({where})"
    if slot == "engine":
        eng = rec.get("engine") or "py"
        how = "run_synchronization.py" if eng == "py" else "TrafficLayer + VirCarlaEnv"
        return f"{eng}   ({how})"
    if slot == "carla":
        mode = (carla_cfg or {}).get("mode") or "?"
        root = (carla_cfg or {}).get("carla_root") or "?"
        host = rec.get("carla_host") or "127.0.0.1"
        port = rec.get("carla_port") or 2000
        return f"{mode}  {root}  ->  {host}:{port}"
    if slot == "sumo":
        gui = "gui" if rec.get("sumo_gui", True) else "headless"
        return f"{gui}, step {rec.get('step_length', 0.05)}"
    return "?"


def show(profile_id, rec, carla_cfg=None):
    """Print the saved setup as a numbered list."""
    when = (rec.get("updated") or "").replace("T", " ")
    stamp = f" (last run {when})" if when else ""
    print(f"\n[cosim] Saved setup '{profile_id}'{stamp}:")
    for i, (slot, label) in enumerate(SLOTS, 1):
        print(f"   {i}) {label:<9}{_fmt(slot, rec, carla_cfg)}")


def cascade(rec, changed):
    """Widen `changed` (a set of slot keys) with the slots it invalidates.

    Changing the app invalidates the map and the scenario yaml - both belonged to
    the app you are leaving. Changing the map invalidates the scenario yaml only
    when that yaml was the map's own generated config.yaml; an app-owned yaml is
    written against the app, not the map, so it survives."""
    out = set(changed)
    if "app" in out:
        out.update({"map", "config"})
    if "map" in out and (rec.get("config_scope") or "map") == "map":
        out.add("config")
    return out


def review(profile_id, rec, carla_cfg=None):
    """Show the saved setup and ask what to change.

    Returns a set of slot keys to re-ask (empty = run it as-is), or None if the
    user chose to quit. A non-interactive session prints the summary and returns
    an empty set, so scheduled/scripted runs reuse the profile verbatim - explicit
    CLI flags are how they deviate from it."""
    show(profile_id, rec, carla_cfg)
    if not sys.stdin.isatty():
        print("[cosim] non-interactive: running the saved setup as-is "
              "(pass flags to override, --fresh to ignore it).")
        return set()
    keys = [s for s, _ in SLOTS]
    while True:
        try:
            ans = input('\n[cosim] Enter = run as-is | numbers to change (e.g. "2 4") '
                        '| A = all | Q = quit: ').strip().lower()
        except EOFError:
            return set()
        if ans == "":
            return set()
        if ans in ("q", "quit"):
            return None
        if ans in ("a", "all"):
            return set(keys)
        picks = [t for t in ans.replace(",", " ").split() if t]
        if picks and all(t.isdigit() and 1 <= int(t) <= len(keys) for t in picks):
            chosen = {keys[int(t) - 1] for t in picks}
            widened = cascade(rec, chosen)
            extra = widened - chosen
            if extra:
                labels = ", ".join(label for slot, label in SLOTS if slot in extra)
                print(f"[cosim] also re-asking: {labels} (it depended on what you changed)")
            return widened
        print("[cosim] enter numbers from the list, or Enter / A / Q.")
