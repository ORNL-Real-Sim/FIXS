"""
FIXS #172: map the SUMO-recorded lane sequence (sumo_lane_seq.txt, the clean
no-lane-change loop) to CarMaker lane-paths, to build the ego/traffic Route.

Chain:  SUMO lane (edge/internal, lane 0)  --geometry-->  xodr road id
        xodr road id  --rd5 Tag odrRoadId + LaneR.0-->  CM lane-path
SUMO lane 0 = rightmost = xodr lane -1 = CM LaneR.0, so the whole route stays in
one lane. Output -> route_drvpath.txt (deduped lane-path chain).
"""
from __future__ import annotations
import math, re, pathlib, xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
NET = HERE / "simple_traffic_light.net.xml"
XODR = HERE / "simple_traffic_light.xodr"
RD = REPO / "ProprietaryFiles" / "CM13_proj" / "Data" / "Road" / "simple_traffic_light.rd5"


def sumo_lane_geo():
    """SUMO lane id -> (midpoint x,y, heading) from net.xml lane shapes (incl internal)."""
    root = ET.parse(NET).getroot()
    out = {}
    for edge in root.findall("edge"):
        for ln in edge.findall("lane"):
            pts = [tuple(map(float, p.split(","))) for p in ln.get("shape").split()]
            if len(pts) < 2:
                continue
            i = len(pts)//2
            (x0, y0), (x1, y1) = pts[max(0, i-1)], pts[min(len(pts)-1, i)]
            out[ln.get("id")] = ((pts[i-1][0]+pts[i][0])/2 if i else pts[0][0],
                                 (pts[i-1][1]+pts[i][1])/2 if i else pts[0][1],
                                 math.atan2(y1-y0, x1-x0))
    return out


def xodr_road_samples():
    """xodr road id -> list of (x,y,hdg) sampled along the reference line."""
    root = ET.parse(XODR).getroot()
    out = {}
    for road in root.findall("road"):
        pts = []
        for g in road.findall("./planView/geometry"):
            x, y, hdg, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
            n = max(2, int(L/5))   # dense sampling (~every 5 m) for accurate nearest match
            fs = [i/n for i in range(n+1)]
            if g.find("line") is not None:
                for f in fs:
                    pts.append((x+f*L*math.cos(hdg), y+f*L*math.sin(hdg), hdg))
            else:
                p = g.find("paramPoly3"); a = {k: float(p.get(k)) for k in ("aU","bU","cU","dU","aV","bV","cV","dV")}
                pr = L if p.get("pRange") == "arcLength" else 1.0
                for f in fs:
                    t = f*pr
                    u = a["aU"]+a["bU"]*t+a["cU"]*t**2+a["dU"]*t**3; v = a["aV"]+a["bV"]*t+a["cV"]*t**2+a["dV"]*t**3
                    du = a["bU"]+2*a["cU"]*t+3*a["dU"]*t**2; dv = a["bV"]+2*a["cV"]*t+3*a["dV"]*t**2
                    pts.append((x+u*math.cos(hdg)-v*math.sin(hdg), y+u*math.sin(hdg)+v*math.cos(hdg), hdg+math.atan2(dv, du)))
        out[road.get("id")] = pts
    return out


def ang(a, b): return abs((a-b+math.pi) % (2*math.pi) - math.pi)


def road_to_lanepath(t):
    """road -> lane-path of its RIGHTMOST lane (= SUMO lane 0). The rightmost lane
    is the highest LaneR.<n> index (LaneR.0=innermost). Calibrated against osc2cm:
    corridor_w_in (3 lanes) lane0 -> LaneR.2 -> lane 367 -> lane-path 374."""
    lane_lp = {int(m.group(2)): int(m.group(1)) for m in re.finditer(r"LanePath\.\d+ = (\d+) (\d+)", t)}
    out = {}

    def rightmost(lanes):
        # rightmost DRIVING lane = highest LaneR index whose lane has a lane-path
        # (shoulders/non-driving lanes have no LanePath -> excluded)
        cand = sorted((int(i) for i, lid in lanes if int(lid) in lane_lp), reverse=True)
        if not cand:
            return None
        top = cand[0]
        lid = next(int(lid) for i, lid in lanes if int(i) == top)
        return lane_lp[lid]

    # normal links: lines that START with "Link." (anchored so we don't also hit Junction.X.Link.Y)
    for m in re.finditer(r"(?m)^Link\.(\d+)\.Tag = odrRoadId:(\S+)", t):
        lk, road = m.group(1), m.group(2)
        lanes = re.findall(rf"(?m)^Link\.{lk}\.LaneSection\.0\.LaneR\.(\d+)\.ID = (\d+)", t)
        lp = rightmost(lanes)
        if lp is not None:
            out[road] = lp
    for m in re.finditer(r"Junction\.(\d+)\.Link\.(\d+)\.Tag = odrRoadId:(\S+)", t):
        j, lk, road = m.group(1), m.group(2), m.group(3)
        lanes = re.findall(rf"Junction\.{j}\.Link\.{lk}\.LaneSection\.0\.LaneR\.(\d+)\.ID = (\d+)", t)
        lp = rightmost(lanes)
        if lp is not None:
            out.setdefault(road, lp)
    return out


def main():
    seq = (HERE / "sumo_lane_seq.txt").read_text().split()
    lgeo = sumo_lane_geo()
    rsamp = xodr_road_samples()
    r2lp = road_to_lanepath(RD.read_text(encoding="utf-8", errors="ignore"))

    drv, seen, unmatched = [], set(), []
    for lane in seq:
        if lane not in lgeo:
            unmatched.append(lane); continue
        mx, my, mh = lgeo[lane]
        best, bd = None, 1e9
        for rid, pts in rsamp.items():
            for (px, py, ph) in pts:
                d = math.hypot(px-mx, py-my)
                if d < bd and ang(ph, mh) < 0.6:
                    best, bd = rid, d
        if best is None or bd > 9.5:   # >= max lane offset from road reference line
            unmatched.append(f"{lane}(d={bd:.1f})"); continue
        lp = r2lp.get(best)
        if lp and (not drv or drv[-1] != lp):
            drv.append(lp)
    print(f"matched {len(seq)-len(unmatched)}/{len(seq)} lanes; unmatched: {unmatched}")
    print(f"DrvPath ({len(drv)} lane-paths) = {' '.join(map(str, drv))}")
    (HERE / "route_drvpath.txt").write_text(" ".join(map(str, drv)) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
