"""
run_profile.py - named run setups (~/.fixs/run_profiles.json) and the review loop.

A co-sim run is defined by six choices: which application, which map, which
scenario yaml, which bridge, which CARLA, and how SUMO runs. Answering all six on
every run is the friction this removes. They are saved as a NAMED SETUP, and every
later run starts from the list of setups rather than from a blank prompt:

    [cosim] Run setups:
       1) roosevelt_default   roosevelt | roosevelt_full | config.yaml | py | gui
       2) roosevelt_fast      roosevelt | roosevelt_full | config_fast.yaml | cpp | gui
       3) mlk_dspace          mlk_eco_driving | mlk_full | ..._dSPACE.yaml | cpp | gui
       N) new setup
    [cosim] Which? [1-3 / N], Enter = 1 (last run):

Pick one and you get its settings, which you edit until they are right and then
confirm:

    [cosim] Run setup 'roosevelt_default'  (last run 2026-07-29 13:21)
       1) app       roosevelt
       2) map       roosevelt_full            (Digital-Twin-Library)
       3) scenario  config.yaml               (generated, this app on this map)
       4) engine    py                        (run_synchronization.py)
       5) CARLA     source  C:/src_ext/Carla  ->  127.0.0.1:2000
       6) SUMO      gui, step 0.05

    [cosim] Enter = run it | 1-6 = change | S = switch setup | N = new | Q = quit:

Editing loops: after each change the summary is redrawn, so you see the result and
can keep going. Nothing heavy happens until you confirm - all six are pure
decisions, so no map is downloaded, cooked or loaded while you are still deciding.

Running a setup saves it back under the same name; `N` is how you keep the old one
and start another. The store is one file so the whole list can be read, shown and
switched in one place - and a future front-end can drive exactly the same file:

    {"schema": 1, "last": "roosevelt_default", "setups": {"<name>": {...}}}

Explicit CLI flags outrank a saved setup: the caller applies a slot only when the
corresponding flag was not given, so `run_cosim --map atlanta` changes exactly that
one thing and prompts for nothing.
"""
import json
import os
from datetime import datetime

import app_catalog
import carla_env_setup as env

SCHEMA = 1
# Identity used in a setup's `app` field when no application is selected.
GENERIC = app_catalog.GENERIC

# slot key -> (menu label, slots invalidated when it changes)
SLOTS = [
    ("app", "app"),
    ("map", "map"),
    ("config", "scenario"),
    ("engine", "engine"),
    ("carla", "CARLA"),
    ("sumo", "SUMO"),
]
SLOT_KEYS = [k for k, _ in SLOTS]


def profiles_path():
    """~/.fixs/run_profiles.json - beside carla.json, outside any repo."""
    return os.path.join(os.path.dirname(env.CONFIG_PATH), "run_profiles.json")


# --------------------------------------------------------------------------- #
# Store
# --------------------------------------------------------------------------- #
def load_doc():
    """The whole store: {"schema", "last", "setups": {name: rec}}.

    A missing or corrupt file reads as empty - a lost setup costs one extra round
    of prompts, never a failed run. An older {active, profiles:{<app_id>: rec}}
    file is upgraded in memory: each app's remembered run becomes a named setup
    '<app_id>_default', which is exactly what it was."""
    try:
        with open(profiles_path(), encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError):
        doc = {}
    if not isinstance(doc, dict):
        doc = {}
    if not isinstance(doc.get("setups"), dict):
        legacy = doc.get("profiles")
        if isinstance(legacy, dict):
            doc["setups"] = {f"{k}_default": v for k, v in legacy.items()
                             if isinstance(v, dict)}
            active = doc.get("active")
            doc["last"] = f"{active}_default" if active else None
        else:
            doc["setups"] = {}
    doc.setdefault("schema", SCHEMA)
    doc.setdefault("last", None)
    doc.pop("profiles", None)
    doc.pop("active", None)
    return doc


def save_doc(doc):
    path = profiles_path()
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2)
            f.write("\n")
    except OSError as exc:
        print(f"[cosim] could not save the run setup ({exc}); this run is not remembered.")


def save(name, rec):
    """Store `rec` under `name` and mark it as the one that ran last."""
    doc = load_doc()
    rec = dict(rec)
    rec["updated"] = datetime.now().isoformat(timespec="seconds")
    doc["setups"][name] = rec
    doc["last"] = name
    save_doc(doc)
    return rec


def delete(name):
    doc = load_doc()
    if doc["setups"].pop(name, None) is not None:
        if doc.get("last") == name:
            doc["last"] = None
        save_doc(doc)
        return True
    return False


def order(doc):
    """Setup names for display: the one that ran last first, then the rest by recency.

    `last` leads explicitly rather than by sorting on its timestamp, because the
    timestamps are second-resolution and two setups saved in the same second tie -
    which left the entry Enter selects sitting somewhere down the list. It is also
    just the right reading order: the default is item 1."""
    setups = doc.get("setups") or {}
    rest = sorted(setups, key=lambda n: (setups[n].get("updated") or ""), reverse=True)
    last = doc.get("last")
    if last in setups:
        rest.remove(last)
        return [last] + rest
    return rest


def suggest_name(app_id, doc):
    """A free name for a new setup: '<app>_default', then '<app>_2', '<app>_3'..."""
    base = app_id or "generic"
    setups = doc.get("setups") or {}
    name = f"{base}_default"
    n = 2
    while name in setups:
        name = f"{base}_{n}"
        n += 1
    return name


# --------------------------------------------------------------------------- #
# Rendering
# --------------------------------------------------------------------------- #
def summarize(rec):
    """One-line digest of a setup, for the list.

    Deliberately no engine or CARLA endpoint: those live in the scenario yaml, and
    printing a copy of them here would mean reading every setup's yaml to draw a
    list - or, worse, showing a remembered value that the yaml no longer says. The
    yaml is named, which is the part that identifies the setup anyway."""
    bits = [rec.get("app") or "no app",
            rec.get("map") or "no map",
            os.path.basename(rec.get("config") or "") or "auto config",
            "gui" if rec.get("sumo_gui", True) else "headless",
            f"step {rec.get('step_length', 0.05)}"]
    return " | ".join(bits)


def _fmt(slot, rec, carla_cfg, derived=None):
    """The value text for one summary line.

    `derived` carries the settings the SCENARIO YAML owns - the bridge and the
    CARLA endpoint - read fresh by the caller each time this is drawn. They are
    shown but not stored, so a yaml edited by hand is reflected here immediately
    instead of the summary repeating a stale copy."""
    derived = derived or {}
    if slot == "app":
        return rec.get("app") or "(none - generic co-sim)"
    if slot == "map":
        origin = rec.get("map_origin")
        return f"{rec.get('map') or '(not chosen)':<26}" + (f"({origin})" if origin else "")
    if slot == "config":
        path = rec.get("config")
        if not path:
            return "(auto-generated for this map)"
        scope = rec.get("config_scope") or "map"
        where = "app-owned, edit in ~/.fixs/apps" if scope == "app" \
            else "generated, this app on this map"
        return f"{os.path.basename(path):<26}({where})"
    if slot == "engine":
        eng = derived.get("engine") or "py"
        how = "run_synchronization.py" if eng == "py" else "TrafficLayer + VirCarlaEnv"
        return f"{eng:<26}({how}, from the yaml)"
    if slot == "carla":
        mode = (carla_cfg or {}).get("mode") or "?"
        root = (carla_cfg or {}).get("carla_root") or "?"
        return f"{mode}  {root}  ->  {derived.get('carla_host') or '127.0.0.1'}:" \
               f"{derived.get('carla_port') or 2000}"
    if slot == "sumo":
        gui = "gui" if rec.get("sumo_gui", True) else "headless"
        return f"{gui}, step {rec.get('step_length', 0.05)}"
    return "?"


def show(name, rec, carla_cfg=None, derived=None):
    """Print one setup as a numbered list of its settings."""
    when = (rec.get("updated") or "").replace("T", " ")
    stamp = f"  (last run {when})" if when else "  (new)"
    print(f"\n[cosim] Run setup '{name}'{stamp}:")
    for i, (slot, label) in enumerate(SLOTS, 1):
        print(f"   {i}) {label:<10}{_fmt(slot, rec, carla_cfg, derived)}")


def _input(prompt, default=""):
    try:
        return input(prompt).strip()
    except EOFError:
        return default


def choose_setup(doc, interactive=True):
    """List the saved setups and ask which to open. Returns a name, None to build a
    new one, or QUIT. Enter takes the one that ran last; a non-interactive session
    takes it silently."""
    names = order(doc)
    if not names:
        return None
    last = doc.get("last") if doc.get("last") in names else names[0]
    if not interactive:
        return last
    width = min(max(len(n) for n in names), 24)
    while True:
        print("\n[cosim] Saved run setups:")
        for i, n in enumerate(names, 1):
            mark = "   <- last run" if n == last else ""
            print(f"   {i:>2}) {n:<{width}}  {summarize(doc['setups'][n])}{mark}")
        print("    N) new setup")
        print("    D) delete a setup")
        ans = _input(f"[cosim] Which? [1-{len(names)} / N / D], "
                     f"Enter = {names.index(last) + 1} ({last}): ").lower()
        if ans == "":
            return last
        if ans == "n":
            return None
        if ans in ("q", "quit"):
            return QUIT
        if ans == "d":
            victim = _input(f"[cosim] Delete which? [1-{len(names)}], Enter = cancel: ")
            if victim.isdigit() and 1 <= int(victim) <= len(names):
                gone = names[int(victim) - 1]
                delete(gone)
                print(f"[cosim] deleted '{gone}'.")
                doc = load_doc()
                names = order(doc)
                if not names:
                    return None
                last = doc.get("last") if doc.get("last") in names else names[0]
                width = min(max(len(n) for n in names), 24)
            continue
        if ans.isdigit() and 1 <= int(ans) <= len(names):
            return names[int(ans) - 1]
        print("[cosim] enter a number from the list, N for a new setup, or D to delete.")


def name_setup(doc, current, app_id, dirty, interactive=True):
    """The name to save this run under, asked only when it can matter.

    Untouched setup -> keep its name silently; running something you did not edit
    should not interrogate you. Edited, or brand new -> ask, defaulting to the
    obvious answer. That single prompt is what makes "change one thing" a choice
    between amending this setup and forking a new one, instead of a silent
    overwrite of a setup you meant to keep."""
    if current and not dirty:
        return current
    default = current or suggest_name(app_id, doc)
    if not interactive:
        return default
    if current:
        prompt = (f"[cosim] Settings changed. Save as [Enter = '{current}' "
                  f"(overwrite), or type a new name]: ")
    else:
        prompt = f"[cosim] Name this setup [Enter = '{default}']: "
    while True:
        ans = _input(prompt)
        if ans == "":
            return default
        clean = "".join(c for c in ans if c.isalnum() or c in "-_.")
        if not clean:
            print("[cosim] use letters, digits, '-', '_' or '.'.")
            continue
        if clean in (doc.get("setups") or {}) and clean != current:
            if _input(f"[cosim] '{clean}' exists. Overwrite it? [y/N]: ").lower() \
                    not in ("y", "yes"):
                continue
        return clean


def cascade(rec, changed):
    """Widen `changed` with the slots it invalidates.

    Changing the app invalidates the map and the scenario yaml - both belonged to
    the app you are leaving. Changing the map invalidates the scenario yaml only
    when that yaml was the map's own generated config; an app-owned yaml is written
    against the app, not the map, so it survives."""
    out = set(changed)
    if "app" in out:
        out.update({"map", "config"})
    if "map" in out and (rec.get("config_scope") or "map") == "map":
        out.add("config")
    return out


#: what ask() returns besides a set of slots to edit
RUN, QUIT, SWITCH, NEW = "run", "quit", "switch", "new"


def ask(name, rec, carla_cfg=None, interactive=True, can_switch=True, derived=None):
    """Show a setup and ask what to do with it.

    Returns RUN / QUIT / SWITCH / NEW, or a set of slot keys to edit. A
    non-interactive session prints the summary and runs it - scheduled and
    scripted runs reuse a setup verbatim, and CLI flags are how they deviate."""
    if not interactive:
        show(name, rec, carla_cfg, derived)
        print("[cosim] non-interactive: running this setup as-is "
              "(pass flags to override, --fresh to start over).")
        return RUN
    while True:
        show(name, rec, carla_cfg, derived)
        extra = " | S = switch setup | N = new" if can_switch else ""
        ans = _input(f"[cosim] Enter = run it | 1-{len(SLOTS)} = change "
                     f"(e.g. \"2 4\"){extra} | Q = quit: ").lower()
        if ans == "":
            return RUN
        if ans in ("q", "quit"):
            return QUIT
        if can_switch and ans == "s":
            return SWITCH
        if can_switch and ans == "n":
            return NEW
        if ans in ("a", "all"):
            return set(SLOT_KEYS)
        picks = [t for t in ans.replace(",", " ").split() if t]
        if picks and all(t.isdigit() and 1 <= int(t) <= len(SLOTS) for t in picks):
            chosen = {SLOT_KEYS[int(t) - 1] for t in picks}
            widened = cascade(rec, chosen)
            extra_slots = widened - chosen
            if extra_slots:
                labels = ", ".join(l for s, l in SLOTS if s in extra_slots)
                print(f"[cosim] also asking for: {labels} (it depended on what you changed)")
            return widened
        print("[cosim] enter numbers to change, Enter to run"
              + (", S / N to switch" if can_switch else "") + ", or Q.")
