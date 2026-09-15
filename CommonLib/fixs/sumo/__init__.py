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

import shutil
import typing
import xml.etree.ElementTree as ET
from pathlib import Path

from .. import simulationEndTime
from . import ego as ego_module

__all__ = ["scenario", "Scenario", "build_ego_scenario", "has_ego",
           "vtypes_file", "net_file", "route_files", "set_run_settings"]


def _flags_by_dest():
    """{dest: flag} taken from ego's own parser.

    Derived rather than listed so a kwarg cannot drift from the option it sets,
    and so a typo names the dests that exist instead of failing inside argparse.
    It also catches the one place the two disagree: --id sets `ego_id`.
    """
    return {a.dest: a.option_strings[0]
            for a in ego_module.build_parser()._actions if a.option_strings}


def build_ego_scenario(sumocfg, out_dir, **options):
    """Build a run scenario with this application's ego in it. Returns its path.

    `options` are ego.py's own, as python names: route_from, route_edges, repeat,
    depart, depart_lane, depart_pos, depart_speed, type_from, speed_factor,
    accel, decel, inject, replace, end, step_length, time_to_teleport, seed.
    A list value repeats its flag, which is what `replace` and `route_file` want.

    The map bundle is read and never written; everything generated lands under
    `out_dir`.
    """
    # The RUN's end time, defaulted rather than asked for. It has one owner --
    # SimulationSetup.SimulationEndTime, which TrafficLayer reads -- and an
    # application passing it here would be copying a value out of a file it
    # should not have to open. Pass end= explicitly only to override it.
    options.setdefault("end", simulationEndTime())

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
    out_cfg, _ego_rou, _n_edges = ego_module.generate(ego_module.build_parser().parse_args(argv))
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


def has_ego(sumocfg, ego_id=ego_module.DEFAULT_EGO_ID):
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


_UNSET = object()


def set_run_settings(sumocfg, out_path=None, *, end=_UNSET, step_length=None,
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
    # Same as build_ego_scenario: the run's end time is the scenario yaml's, and
    # a caller should not have to fetch it. end=None still means "leave it alone".
    if end is _UNSET:
        end = simulationEndTime()

    tree = ET.parse(Path(sumocfg))
    root = tree.getroot()
    if end is not None:
        ego_module._set_value(root, "time", "end", end)
    if step_length is not None:
        ego_module._set_value(root, "time", "step-length", step_length)
    if time_to_teleport is not None:
        ego_module._set_value(root, "processing", "time-to-teleport", time_to_teleport)
    if seed is not None:
        ego_module._set_value(root, "random_number", "seed", seed)
    target = Path(out_path) if out_path is not None else Path(sumocfg)
    tree.write(target, encoding="UTF-8", xml_declaration=True)
    return target


class Scenario(typing.NamedTuple):
    """What a run needs to know about the scenario it is about to run."""
    path: str          # what to start SUMO on, and what to report to FIXS_HANDOFF
    built: bool        # True if the ego was inserted; False if it was already there
    types_file: object # the route file this run's vType attributes were written into
    ego_depart: object # the ego's depart time, read back from what was built
    begin: object      # SUMO's begin time, for a controller that idles until entry


def _apply_vtypes(sumocfg, out_dir, vtypes):
    """A copy of the route file defining these types, with the attributes set.

    "This run disagrees with the map's calibration about these types" -- a market
    penetration sweep is the usual reason and `probability` the usual attribute,
    but nothing here is specific to either.

    A COPY is modified, never the source, and it keeps the original's basename so
    a config naming it relatively picks it up. The copy carries whatever else the
    file had: if the types live in their own file that is a few hundred bytes; if
    they live with the demand, the demand comes along, which is correct rather
    than an error. Substituting a types-ONLY file for one carrying demand is what
    once replaced 8977 vehicles with four definitions and dropped the injected
    ego with them.
    """
    wanted = set(vtypes)
    for path in route_files(sumocfg):
        if not path.is_file():
            continue
        tree = ET.parse(path)
        root = tree.getroot()
        if not wanted <= {v.get("id") for v in root.findall("vType")}:
            continue
        for vtype in root.findall("vType"):
            for name, value in (vtypes.get(vtype.get("id")) or {}).items():
                vtype.set(name, str(value))
        copy = Path(out_dir) / path.name
        tree.write(copy, encoding="UTF-8", xml_declaration=True)
        return path, copy
    raise SystemExit(
        f"{Path(sumocfg).resolve()} names no route file defining "
        f"{', '.join(sorted(wanted))}, so this run's vType settings have nowhere "
        f"to go.")


def _copy_inputs(sumocfg, out_dir, skip):
    """Put the config's inputs beside the copy of it we are writing.

    A generated config lives in the run directory but names its inputs the way
    the original did -- relatively -- so those names resolve only if the files are
    there. It also keeps a run replayable on its own, and it is what keeps a
    relative `timedEvent dest` (signal_result.xml) writing into the run directory
    rather than back into the scenario folder.
    """
    cfg = Path(sumocfg).resolve()
    root = ET.parse(cfg).getroot()
    for tag in ("net-file", "additional-files"):
        node = root.find("./input/" + tag)
        names = (node.get("value") or "").split(",") if node is not None else []
        for name in names:
            name = name.strip()
            if not name or name in skip:
                continue
            src = Path(name)
            src = src if src.is_absolute() else cfg.parent / src
            if src.is_file():
                shutil.copy(src, Path(out_dir) / src.name)


def scenario(sumocfg, out_dir, *, ego=None, vtypes=None, **run_settings):
    """(path, path) -> Scenario -- this run's SUMO inputs, ready to start.

    The one call an application needs here. It is handed the scenario run_cosim
    gave it and returns what to start SUMO on, which is what goes to
    FIXS_HANDOFF::

        cfg = fixs.sumo.scenario(
            source_cfg, run_dir,
            ego=dict(route_from="route1", depart=29100, depart_lane="1",
                     type_from="EGO_TYPE", speed_factor=1.1273,
                     accel=2.0, decel=2.0, repeat=20, inject=demand_copy),
            vtypes={"CAV": {"probability": mpr},
                    "HDV": {"probability": 1 - mpr}})

    `ego` is build_ego_scenario's options, and is what the EXPERIMENT says. If
    the scenario already contains that ego it is used as prepared rather than
    built over: run_cosim passes whatever the user chose and cannot tell a map
    bundle from a scenario prepared for this app, the scenario itself can, and an
    application should not have to ask.

    `vtypes` is {type id: {attribute: value}}, applied to a copy.

    `run_settings` are end / step_length / time_to_teleport / seed. `end`
    defaults to SimulationSetup.SimulationEndTime, so a run's clock has one owner.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ego = dict(ego or {})
    ego_id = ego.get("ego_id", ego_module.DEFAULT_EGO_ID)

    types_src = types_copy = None
    if vtypes:
        types_src, types_copy = _apply_vtypes(sumocfg, out_dir, vtypes)

    if ego and not has_ego(sumocfg, ego_id):
        if types_copy is not None:
            ego.setdefault("replace", []).append(
                types_src.name + "=" + str(types_copy))
        path = build_ego_scenario(sumocfg, out_dir, **ego, **run_settings)
        built = True
        if not has_ego(path, ego_id):
            raise SystemExit(
                "the generated scenario has no vehicle " + repr(ego_id) +
                " (" + str(path) + "). Refusing to start: a controller would run "
                "with nothing to control and write an empty log.")
    else:
        # Already carries the ego, so nothing is inserted. Its inputs are named
        # relatively and the config is moving, so they come along.
        path = out_dir / Path(sumocfg).name
        _copy_inputs(sumocfg, out_dir, skip={types_src.name} if types_src else set())
        set_run_settings(sumocfg, out_path=path, **run_settings)
        built = False

    return Scenario(path=str(path), built=built, types_file=types_copy,
                    ego_depart=_ego_depart(path, ego_id), begin=_begin(path))


def _ego_depart(sumocfg, ego_id):
    """The ego's depart time out of the scenario, or None."""
    for path in route_files(sumocfg):
        if not path.is_file():
            continue
        for _event, el in ET.iterparse(path, events=("end",)):
            if el.tag == "vehicle" and el.get("id") == ego_id:
                depart = el.get("depart")
                return None if depart is None else float(depart)
            if el.tag in ("vehicle", "flow", "route"):
                el.clear()
    return None


def _begin(sumocfg):
    """SUMO's configured begin time, or None."""
    node = ET.parse(Path(sumocfg)).getroot().find("./time/begin")
    if node is None or node.get("value") is None:
        return None
    return float(node.get("value"))
