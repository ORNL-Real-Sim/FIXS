"""
Phase 1c (demand): add SUMO-matching background traffic to the signalized VISSIM
corridor, then consolidate into the canonical simple_traffic_light.inpx.

Background traffic flows through the corridor + cross-streets and exits (SUMO
style); the ego loops via the U-turns under CarMaker. Volumes match SUMO's
.rou.xml flows:
    corridor W->E : 400 veh/h     corridor E->W : 400 veh/h
    each of 6 cross-street feeders: 80 veh/h
Routing sends corridor traffic straight through; each cross feeder turns onto the
corridor toward an exit (matching the SUMO route direction).

Approach: SUMO edges <-> xodr roads matched by geometry (coords align, no offset);
road -> VISSIM link by name ("<road>-0-Right"); link No != road id.

Reads simple_traffic_light_signals.inpx, writes simple_traffic_light.inpx (final).
"""
from __future__ import annotations
import math, sys, pathlib, re
import xml.etree.ElementTree as ET
import pythoncom, win32com.client

PROGID = "VISSIM.Vissim.2200"
HERE = pathlib.Path(__file__).resolve().parent
NET = HERE.parent.parent.parent / "Python" / "SimpleTrafficLight" / "simple_traffic_light.net.xml"
XODR = HERE / "simple_traffic_light.xodr"
INPX_IN = HERE / "simple_traffic_light_signals.inpx"
INPX_OUT = HERE / "simple_traffic_light.inpx"

# flow -> (entry edge, exit edge, veh/h), exit per SUMO route direction
FLOWS = [
    ("WE",       "corridor_w_in", "corridor_e_out", 400),
    ("EW",       "corridor_e_in", "corridor_w_out", 400),
    ("west_N",   "west_N_in",     "corridor_e_out", 80),
    ("west_S",   "west_S_in",     "corridor_w_out", 80),
    ("center_N", "center_N_in",   "corridor_e_out", 80),
    ("center_S", "center_S_in",   "corridor_w_out", 80),
    ("east_N",   "east_N_in",     "corridor_e_out", 80),
    ("east_S",   "east_S_in",     "corridor_w_out", 80),
]


def edge_geom(net_path):
    """SUMO edge id -> (start(x,y), heading) from its first lane shape."""
    root = ET.parse(net_path).getroot()
    out = {}
    for e in root.findall("edge"):
        if e.get("function") == "internal":
            continue
        ln = e.find("lane")
        if ln is None:
            continue
        pts = [tuple(map(float, p.split(","))) for p in ln.get("shape").split()]
        (x0, y0), (x1, y1) = pts[0], pts[1]
        out[e.get("id")] = ((x0, y0), math.atan2(y1 - y0, x1 - x0))
    return out


def road_geom(xodr_path):
    """xodr road id (junction=-1) -> (start(x,y), heading)."""
    xml = xodr_path.read_text(encoding="utf-8")
    out = {}
    for m in re.finditer(r'<road[^>]*id="(\d+)" junction="-1">.*?<geometry s="0[^"]*" '
                         r'x="([-\d.]+)" y="([-\d.]+)" hdg="([-\d.]+)"', xml, re.S):
        rid, x, y, hdg = m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4))
        out[rid] = ((x, y), hdg)
    return out


def match_edge_to_road(edges, roads, want):
    """nearest road by start point with aligned heading."""
    res = {}
    for e in want:
        (ex, ey), eh = edges[e]
        best, bestd = None, 1e9
        for rid, ((rx, ry), rh) in roads.items():
            d = math.hypot(ex - rx, ey - ry)
            if d < bestd and math.cos(eh - rh) > 0.7:   # same direction
                best, bestd = rid, d
        res[e] = (best, bestd)
    return res


def main():
    edges, roads = edge_geom(NET), road_geom(XODR)
    want = sorted({e for _, a, b, _ in FLOWS for e in (a, b)})
    e2r = match_edge_to_road(edges, roads, want)
    print("=== edge -> xodr road (dist m) ===")
    for e, (r, d) in e2r.items():
        print(f"  {e:16s} -> road {r}  (d={d:.1f} m)")
    if any(r is None or d > 15 for r, d in e2r.values()):
        print("WARNING: some edges unmatched / far", file=sys.stderr)

    pythoncom.CoInitializeEx(pythoncom.COINIT_APARTMENTTHREADED)
    v = win32com.client.Dispatch(PROGID, clsctx=pythoncom.CLSCTX_LOCAL_SERVER)
    print("dispatch OK", file=sys.stderr)
    v.LoadNet(str(INPX_IN), False)
    net = v.Net

    r2l = {}
    for lk in net.Links:
        nm = lk.AttValue("Name") or ""
        tok = nm.split("-")[0]
        if tok.isdigit():
            r2l.setdefault(tok, int(lk.AttValue("No")))

    def link_for(edge):
        rid = e2r[edge][0]
        return r2l.get(rid)

    in_key = rt_key = dec_key = 0
    for name, entry, exit_, vol in FLOWS:
        eln, xln = link_for(entry), link_for(exit_)
        if eln is None or xln is None:
            print(f"  SKIP {name}: entry/exit link missing ({entry}->{eln}, {exit_}->{xln})", file=sys.stderr)
            continue
        # vehicle input on entry link
        in_key += 1
        vi = net.VehicleInputs.AddVehicleInput(in_key, net.Links.ItemByKey(eln))
        for attr in ("Volume(1)", "Volume"):
            try:
                vi.SetAttValue(attr, float(vol)); break
            except Exception:
                continue
        # routing decision on entry -> route to exit link end
        dec_key += 1
        dec = net.VehicleRoutingDecisionsStatic.AddVehicleRoutingDecisionStatic(
            dec_key, net.Links.ItemByKey(eln), 5.0)
        rt_key += 1
        xlen = float(net.Links.ItemByKey(xln).AttValue("Length2D"))
        dec.VehRoutSta.AddVehicleRouteStatic(rt_key, net.Links.ItemByKey(xln), max(1.0, xlen - 3.0))
        print(f"  {name:10s} input {vol} veh/h on link {eln} -> route to link {xln}")

    v.SaveNetAs(str(INPX_OUT), False)
    print(f"saved {INPX_OUT.name}")

    # validate: run + count vehicles in network
    v.LoadNet(str(INPX_OUT), False)
    v.Simulation.SetAttValue("SimPeriod", 120)
    v.Simulation.SetAttValue("SimRes", 1)
    counts = []
    for s in range(120):
        v.Simulation.RunSingleStep()
        if s % 20 == 19:
            counts.append((s + 1, v.Net.Vehicles.Count))
    v.Simulation.Stop()
    print("=== vehicles in network (sec, count) ===", counts)
    v = None
    return 0


if __name__ == "__main__":
    sys.exit(main())
