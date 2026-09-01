"""
sumo_ego.py - put an application's ego vehicle into a map's SUMO scenario,
without the application owning a copy of that scenario.

A co-sim application needs one thing the map bundle cannot ship: its own ego, on
its own route, departing when the experiment says. The obvious way to get it is
to copy the bundle's route file, edit a route inside it, and run the copy - and
that is the wrong way round. The copy is map data the application now owns, so it
goes stale the moment the bundle is updated, and every change to the map has to
be mirrored by hand. MLK reached 1.8 MB of duplicated network and demand, and a
71-edge route list pasted into a Python file beside the route file it came from.

SUMO reads a LIST of route files. So the ego does not need to live in the map's:

    <route-files value="<the bundle's, untouched>,ego.rou.xml"/>

This writes that second file - vType, route, vehicle, nothing else, typically
under a kilobyte - plus a generated .sumocfg that points at the bundle in place
and sends the run's outputs to a run directory. The bundle is never copied and
never edited.

The generated .sumocfg path is printed on the last line, which is what an app
reports to run_cosim through FIXS_HANDOFF.

Stdlib only.

Usage
-----
  python sumo_ego.py --sumocfg ~/.fixs/maps/<map>/sumo/<map>.sumocfg \\
                     --out-dir MPR/run_0828 \\
                     --route-edges "51066109#1 E1 51066109#3" \\
                     --depart 29100 --repeat 20 \\
                     --type-from EGO_TYPE --speed-factor 1.1273 --accel 2.0

  # or take the route from one already in the bundle, by id
  python sumo_ego.py --sumocfg ... --out-dir ... --route-from route1 --depart 29100
"""
from __future__ import annotations

import argparse
import copy
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_EGO_ID = "ego"
DEFAULT_TYPE_ID = "EGO_TYPE_EXTERNAL"

# Outputs a run wants in its own directory rather than on top of the last run's.
# Every one is a <sumocfg><output> child except the timed event, which hides in
# <report>; both are rewritten to absolute paths under --out-dir.
OUTPUT_ELEMENTS = ("summary-output", "tripinfo-output", "fcd-output",
                   "netstate-dump", "emission-output", "full-output",
                   "queue-output", "statistic-output", "log")


def _split_list(value):
    return [p.strip() for p in (value or "").split(",") if p.strip()]


def _abs_from(base_dir, name):
    """A sumocfg's file reference resolved against the config that named it."""
    p = Path(name)
    return str(p if p.is_absolute() else (base_dir / p).resolve())


def route_from_bundle(route_files, route_id):
    """The edge list of an existing route, by id, from the bundle's demand.

    Lets an app say "the ego drives the corridor route the map already defines"
    instead of pasting that route into its own source, where it becomes a second
    copy that nothing keeps in step.
    """
    for rf in route_files:
        for _, el in ET.iterparse(rf, events=("end",)):
            if el.tag == "route" and el.get("id") == route_id:
                return (el.get("edges") or "").split()
            if el.tag in ("route", "vehicle", "flow"):
                el.clear()
    raise SystemExit(f"no route id '{route_id}' in: {', '.join(route_files)}")


def vtype_from_bundle(route_files, type_id):
    """A copy of a vType the bundle defines, so the ego inherits the calibrated
    car-following parameters instead of re-stating them."""
    for rf in route_files:
        for _, el in ET.iterparse(rf, events=("end",)):
            if el.tag == "vType" and el.get("id") == type_id:
                return copy.deepcopy(el)
            if el.tag in ("vType", "vehicle", "flow", "route"):
                el.clear()
    raise SystemExit(f"no vType id '{type_id}' in: {', '.join(route_files)}")


def build_ego_routes(opt, route_files):
    """The app's own route file: a vType, a route, and one vehicle."""
    root = ET.Element("routes")

    if opt.type_from:
        vtype = vtype_from_bundle(route_files, opt.type_from)
        vtype.set("id", opt.type_id)
    else:
        vtype = ET.Element("vType", {"id": opt.type_id, "vClass": "passenger"})
    for attr, value in (("accel", opt.accel), ("decel", opt.decel),
                        ("speedFactor", opt.speed_factor), ("speedDev", opt.speed_dev)):
        if value is not None:
            vtype.set(attr, str(value))
        else:
            # An inherited value must be droppable, or "use SUMO's default" is
            # unreachable for any app whose bundle vType happens to set it.
            vtype.attrib.pop(attr, None)
    root.append(vtype)

    edges = (opt.route_edges.split() if opt.route_edges
             else route_from_bundle(route_files, opt.route_from))
    if not edges:
        raise SystemExit("the ego route is empty")
    if opt.repeat > 0 and len(edges) > 1 and edges[0] == edges[-1]:
        # `repeat` concatenates whole edge lists, so a lap that names its start
        # edge again at the end would make an A -> A transition between laps.
        edges = edges[:-1]
    route = ET.SubElement(root, "route", {"id": opt.route_id, "edges": " ".join(edges)})
    if opt.repeat > 0:
        route.set("repeat", str(opt.repeat))

    veh = {"id": opt.ego_id, "type": opt.type_id, "route": opt.route_id,
           "depart": str(opt.depart), "departLane": opt.depart_lane,
           "departPos": opt.depart_pos, "departSpeed": str(opt.depart_speed)}
    ET.SubElement(root, "vehicle", veh)
    return ET.ElementTree(root), len(edges)


def build_sumocfg(opt, base_cfg, ego_rou):
    """The bundle's config, with the ego file appended and outputs redirected.

    Everything the bundle names is rewritten to an ABSOLUTE path so the generated
    config can live in the run directory while the network and demand stay in the
    map cache. That is the whole point: nothing is copied.
    """
    base_dir = base_cfg.parent
    tree = ET.parse(base_cfg)
    root = tree.getroot()
    inp = root.find("input")
    if inp is None:
        raise SystemExit(f"{base_cfg} has no <input> section")

    for tag in ("net-file", "additional-files"):
        node = inp.find(tag)
        if node is not None and node.get("value"):
            node.set("value", ",".join(_abs_from(base_dir, f)
                                       for f in _split_list(node.get("value"))))

    routes = inp.find("route-files")
    if routes is None or not routes.get("value"):
        raise SystemExit(f"{base_cfg} names no route-files")
    bundle_routes = [_abs_from(base_dir, f) for f in _split_list(routes.get("value"))]
    routes.set("value", ",".join(bundle_routes + [str(ego_rou)]))

    if opt.end is not None:
        # `x = root.find(t) or ET.SubElement(...)` is wrong here: an Element with
        # no children is FALSY, so an existing childless <end/> would be replaced
        # by a second one and SUMO refuses the config as "defined twice".
        time = root.find("time")
        if time is None:
            time = ET.SubElement(root, "time")
        node = time.find("end")
        if node is None:
            node = ET.SubElement(time, "end")
        node.set("value", str(opt.end))

    for section in ("output", "report"):
        sec = root.find(section)
        if sec is None:
            continue
        for node in list(sec):
            if node.tag in OUTPUT_ELEMENTS and node.get("value"):
                node.set("value", str(opt.out_dir / Path(node.get("value")).name))
            for ev in node.iter():
                if ev.get("dest"):
                    ev.set("dest", str(opt.out_dir / Path(ev.get("dest")).name))
    return tree, bundle_routes


def build_parser():
    p = argparse.ArgumentParser(
        description="Add an app's ego to a map's SUMO scenario without copying it.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Usage\n-----\n")[-1])
    p.add_argument("--sumocfg", required=True, help="the map bundle's .sumocfg (read only)")
    p.add_argument("--out-dir", required=True, help="run directory for the generated files and outputs")

    r = p.add_argument_group("the ego's route (one of these)")
    r.add_argument("--route-edges", help="space-separated edge ids")
    r.add_argument("--route-from", metavar="ROUTE_ID",
                   help="take the edges from a route already defined - in the bundle's "
                        "demand, or in any --route-file the app ships")
    r.add_argument("--route-file", action="append", default=[], metavar="ROU.XML",
                   help="an app's own route file to search for --route-from / --type-from "
                        "(repeatable). Lets the ego's route be shipped as data next to the "
                        "app instead of pasted into its code")
    r.add_argument("--repeat", type=int, default=0,
                   help="drive the route this many extra times (0 = once)")

    e = p.add_argument_group("the ego")
    e.add_argument("--id", dest="ego_id", default=DEFAULT_EGO_ID)
    e.add_argument("--route-id", default="ego_route")
    e.add_argument("--type-id", default=DEFAULT_TYPE_ID)
    e.add_argument("--type-from", metavar="VTYPE_ID",
                   help="inherit the bundle's calibrated vType of this id")
    e.add_argument("--depart", type=float, required=True)
    e.add_argument("--depart-lane", default="first")
    e.add_argument("--depart-pos", default="free")
    e.add_argument("--depart-speed", default="0.1")
    e.add_argument("--accel", type=float, default=None)
    e.add_argument("--decel", type=float, default=None)
    e.add_argument("--speed-factor", type=float, default=None,
                   help="omit to keep the inherited/default distribution")
    e.add_argument("--speed-dev", type=float, default=None)

    o = p.add_argument_group("run")
    o.add_argument("--end", type=float, default=None, help="override the scenario end time")
    o.add_argument("--name", default=None,
                   help="basename for the generated files (default: the bundle cfg's stem + _ego)")
    return p


def main(argv=None):
    opt = build_parser().parse_args(argv)
    if bool(opt.route_edges) == bool(opt.route_from):
        raise SystemExit("give exactly one of --route-edges or --route-from")

    base_cfg = Path(opt.sumocfg).resolve()
    if not base_cfg.is_file():
        raise SystemExit(f"no such sumocfg: {base_cfg}")
    opt.out_dir = Path(opt.out_dir).resolve()
    opt.out_dir.mkdir(parents=True, exist_ok=True)

    stem = opt.name or (base_cfg.stem + "_ego")
    ego_rou = opt.out_dir / f"{stem}.rou.xml"
    out_cfg = opt.out_dir / f"{stem}.sumocfg"

    # route-files are needed before the ego file exists, to read a vType or route
    # out of the bundle
    base_dir = base_cfg.parent
    rf_node = ET.parse(base_cfg).getroot().find("input/route-files")
    if rf_node is None or not rf_node.get("value"):
        raise SystemExit(f"{base_cfg} names no route-files")
    bundle_routes = [_abs_from(base_dir, f) for f in _split_list(rf_node.get("value"))]

    # the app's own route files are searched FIRST: an ego route the app ships is
    # its own statement of intent and must win over a same-named route in the map
    search = [str(Path(f).resolve()) for f in opt.route_file] + bundle_routes
    for f in search[:len(opt.route_file)]:
        if not Path(f).is_file():
            raise SystemExit(f"no such --route-file: {f}")
    routes_tree, n_edges = build_ego_routes(opt, search)
    routes_tree.write(ego_rou, encoding="UTF-8", xml_declaration=True)

    cfg_tree, _ = build_sumocfg(opt, base_cfg, ego_rou)
    cfg_tree.write(out_cfg, encoding="UTF-8", xml_declaration=True)

    print(f"[sumo_ego] bundle   {base_cfg}   (not copied, not modified)")
    print(f"[sumo_ego] ego      {opt.ego_id} on {n_edges} edges, depart {opt.depart:g}"
          + (f", repeat {opt.repeat}" if opt.repeat else "")
          + f"  -> {ego_rou.name} ({ego_rou.stat().st_size} bytes)")
    print(out_cfg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
