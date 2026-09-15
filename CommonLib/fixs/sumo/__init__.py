"""fixs.sumo -- the SUMO side of a scenario, as functions rather than scripts.

An application that runs against a Digital-Twin-Library map needs a handful of
answers about its .sumocfg before anything starts: where is the network, does
this scenario already carry my ego, which file defines the vehicle types I want
to substitute, and -- the big one -- build me a scenario with my ego in it.

Every one of those was being re-implemented per application. mlk_eco_driving
carried 145 lines of it, including a fourth copy of "read net-file out of a
sumocfg" and a subprocess call that scraped sumo_ego's stdout for its result.
This is that surface, once.

    import fixs

    if not fixs.sumo.has_ego(bundle_cfg):
        types = fixs.sumo.vtypes_file(bundle_cfg, ids=("CAV", "HDV"))
        cfg = fixs.sumo.build_ego_scenario(bundle_cfg, run_dir,
                                           route_from="route1", depart=29100,
                                           replace=[f"{types.name}={my_types}"])

UNLIKE the rest of `fixs`, nothing here needs a session. `fixs.vehicle` and
friends raise until connect() has run, because they answer about a tick that has
to have arrived. These answer about FILES, and the whole point is that they run
BEFORE the stack exists -- an app builds its scenario, reports it, and only then
connects. So there is no connection guard here, and adding one later to make the
namespaces look alike would break every caller.

The command line is still there. Carla/utils/sumo_ego.py drives ego.py exactly
as before, prints what it always printed, and is what a shell script or an
application that shells out should keep using.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from . import ego

__all__ = ["build_ego_scenario", "has_ego", "vtypes_file", "net_file",
           "route_files", "set_run_settings"]


def _flags_by_dest():
    """{dest: flag} taken from ego's own parser.

    Derived rather than listed so a kwarg cannot drift from the option it sets,
    and so a typo names the dests that exist instead of failing inside argparse.
    It also catches the one place the two disagree: --id sets `ego_id`.
    """
    return {a.dest: a.option_strings[0]
            for a in ego.build_parser()._actions if a.option_strings}


def build_ego_scenario(sumocfg, out_dir, **options):
    """Build a run scenario with this application's ego in it. Returns its path.

    `options` are ego.py's own, as python names: route_from, route_edges, repeat,
    depart, depart_lane, depart_pos, depart_speed, type_from, speed_factor,
    accel, decel, inject, replace, end, step_length, time_to_teleport, seed.
    A list value repeats its flag, which is what `replace` and `route_file` want.

    The map bundle is read and never written; everything generated lands under
    `out_dir`.
    """
    flags = _flags_by_dest()
    argv = ["--sumocfg", str(sumocfg), "--out-dir", str(out_dir)]
    for dest, value in options.items():
        if dest not in flags:
            raise TypeError(
                f"build_ego_scenario() got an unexpected option {dest!r}; "
                f"ego.py declares {', '.join(sorted(flags))}")
        if value is None or value is False:
            continue
        flag = flags[dest]
        if value is True:
            argv.append(flag)
        elif isinstance(value, (list, tuple)):
            for item in value:
                argv += [flag, str(item)]
        else:
            argv += [flag, str(value)]
    out_cfg, _ego_rou, _n_edges = ego.generate(ego.build_parser().parse_args(argv))
    return out_cfg


def route_files(sumocfg):
    """The route files a .sumocfg names, as absolute paths, in order."""
    cfg = Path(sumocfg).resolve()
    node = ET.parse(cfg).getroot().find("./input/route-files")
    out = []
    for name in (node.get("value") or "").split(",") if node is not None else []:
        name = name.strip()
        if not name:
            continue
        path = Path(name)
        out.append(path if path.is_absolute() else cfg.parent / path)
    return out


def net_file(sumocfg):
    """The network a .sumocfg opens, as an absolute path, or None.

    Ask the config rather than keeping a second answer beside it: the two
    disagree the moment either moves.
    """
    cfg = Path(sumocfg).resolve()
    node = ET.parse(cfg).getroot().find("./input/net-file")
    if node is None or not node.get("value"):
        return None
    path = Path(node.get("value"))
    return path if path.is_absolute() else cfg.parent / path


def has_ego(sumocfg, ego_id=ego.DEFAULT_EGO_ID):
    """Whether this scenario already defines a vehicle with that id.

    A map bundle's does not -- the ego is the application's. A scenario prepared
    for one application does. An app handed either, with no way to tell them
    apart from the outside, can ask here and skip building over an ego that is
    already in place.

    iterparse and clear(), not a full parse: a demand file is routinely 1.3 MB
    and a few thousand vehicles, and this runs before anything has started.
    """
    for path in route_files(sumocfg):
        if not path.is_file():
            continue
        for _event, el in ET.iterparse(path, events=("end",)):
            if el.tag == "vehicle" and el.get("id") == ego_id:
                return True
            if el.tag in ("vehicle", "flow", "route"):
                el.clear()
    return False


def vtypes_file(sumocfg, ids):
    """The route file defining `ids`, found by looking for the TYPES.

    By content, not by filename, so a bundle that calls it something other than
    vtypes.rou.xml still works. `ids` is what the caller means by "the types I
    am going to substitute" -- CAV/HDV for a market-penetration study, whatever
    a different experiment calls its own.

    Raises if that file also carries demand. This file is meant to be SUBSTITUTED
    (see --replace); substituting one that also holds the vehicles replaces the
    demand with a handful of type definitions, and an ego injected into that
    demand goes with them. That failed silently once: a full run directory,
    exit 0, and a controller log with no rows in it.
    """
    ids = set(ids)
    for path in route_files(sumocfg):
        if not path.is_file():
            continue
        root = ET.parse(path).getroot()
        if not ids <= {v.get("id") for v in root.findall("vType")}:
            continue
        # .find, not .iter: iter returns a generator, which is always truthy.
        carries_demand = any(root.find(tag) is not None
                             for tag in ("vehicle", "flow", "route"))
        if carries_demand:
            raise SystemExit(
                f"{path.name} defines {', '.join(sorted(ids))} but also carries "
                f"demand. These types are meant to be substituted, so they must "
                f"live in a file of their own -- substituting this one would "
                f"replace the demand. Split them with Carla/utils/split_vtypes.py, "
                f"or use a bundle that already ships a vTypes file.")
        return path
    raise SystemExit(
        f"{Path(sumocfg).resolve()} names no route file defining "
        f"{', '.join(sorted(ids))}.")


def set_run_settings(sumocfg, out_path=None, *, end=None, step_length=None,
                     time_to_teleport=None, seed=None):
    """Write a run's own SUMO settings into a .sumocfg. Returns where it wrote.

    The same four build_ego_scenario applies, for a scenario that did not come
    through it -- one prepared by hand, say. `out_path` defaults to writing back
    to `sumocfg`; pass it to write a copy instead.

    These belong in the config rather than on the runner's command line. A value
    declared per-app there overrides whatever the scenario says, silently, so the
    two drift and the flag wins (FIXS_Applications#45: a restated
    --time-to-teleport 30 overrode a generated config's 150 and walked an app
    12.75 m/s off its reference results).

    Nor is the seed optional for anyone comparing runs. SUMO's default is 23423,
    and a different random stream draws different per-vehicle speedFactors --
    which is the free-flow speed a controller plans against.

    Creates each element if the config does not have it. The obvious loop,

        for node in root.iter("end"):
            node.set("value", ...)

    silently does nothing when the element is absent, which is the shape this
    replaces: it worked only because the scenarios in hand happened to carry one.
    """
    tree = ET.parse(Path(sumocfg))
    root = tree.getroot()
    if end is not None:
        ego._set_value(root, "time", "end", end)
    if step_length is not None:
        ego._set_value(root, "time", "step-length", step_length)
    if time_to_teleport is not None:
        ego._set_value(root, "processing", "time-to-teleport", time_to_teleport)
    if seed is not None:
        ego._set_value(root, "random_number", "seed", seed)
    target = Path(out_path) if out_path is not None else Path(sumocfg)
    tree.write(target, encoding="UTF-8", xml_declaration=True)
    return target
