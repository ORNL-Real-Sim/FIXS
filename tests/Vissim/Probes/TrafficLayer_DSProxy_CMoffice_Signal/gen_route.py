"""
FIXS #172: derive the full closed-loop DrvPath as a LANE-SPECIFIC chain, so the
ego stays in one lane (no lane changes) except the roundabout U-turn.

Lane graph (from the xodr): every connector road C (junction!=-1) has
pred=(roadP,..) succ=(roadS,..) and per-lane <link> pred/succ ids, giving the
exact movement  (roadP, laneP) --C--> (roadS, laneS).  Main-road lanes only
continue through connectors.

Traverse from a start (road,lane): among the movements leaving the current
(road,lane), prefer the one that KEEPS THE SAME LANE id and continues the
heading (= straight through an intersection / around the roundabout); fall back
to a lane change only if forced. Stop when we return to the start (road,lane).
Then map each road -> its lane-specific lane-path (rd5 LaneR.<|lane|-1>) and dedup.
"""
from __future__ import annotations
import math, re, pathlib, xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
XODR = HERE / "simple_traffic_light.xodr"
RD = REPO / "ProprietaryFiles" / "CM13_proj" / "Data" / "Road" / "simple_traffic_light.rd5"
START_XY = (100.0, 294.0)


def end_pose(g):
    x, y, hdg, L = float(g.get("x")), float(g.get("y")), float(g.get("hdg")), float(g.get("length"))
    if g.find("line") is not None:
        return (x+L*math.cos(hdg), y+L*math.sin(hdg), hdg)
    p = g.find("paramPoly3"); a = {k: float(p.get(k)) for k in ("aU","bU","cU","dU","aV","bV","cV","dV")}
    pr = L if p.get("pRange") == "arcLength" else 1.0
    u = a["aU"]+a["bU"]*pr+a["cU"]*pr**2+a["dU"]*pr**3; v = a["aV"]+a["bV"]*pr+a["cV"]*pr**2+a["dV"]*pr**3
    du = a["bU"]+2*a["cU"]*pr+3*a["dU"]*pr**2; dv = a["bV"]+2*a["cV"]*pr+3*a["dV"]*pr**2
    return (x+u*math.cos(hdg)-v*math.sin(hdg), y+u*math.sin(hdg)+v*math.cos(hdg), hdg+math.atan2(dv, du))


def parse():
    root = ET.parse(XODR).getroot()
    R = {}
    moves = {}   # (roadP, laneP) -> list of (connector, laneC, roadS, laneS, exit_hdg)
    for road in root.findall("road"):
        rid = road.get("id"); jn = road.get("junction")
        g = road.findall("./planView/geometry")
        R[rid] = {"j": jn, "start": (float(g[0].get("x")), float(g[0].get("y")), float(g[0].get("hdg"))),
                  "end": end_pose(g[-1])}
        if jn == "-1":
            continue
        lk = road.find("link"); pr = lk.find("predecessor"); su = lk.find("successor")
        if pr is None or su is None or pr.get("elementType") != "road" or su.get("elementType") != "road":
            continue
        roadP, roadS = pr.get("elementId"), su.get("elementId")
        eh = end_pose(g[-1])[2]
        for ln in road.findall("./lanes/laneSection/right/lane"):
            l = ln.find("link")
            if l is None or l.find("predecessor") is None or l.find("successor") is None:
                continue
            lp, ls = l.find("predecessor").get("id"), l.find("successor").get("id")
            moves.setdefault((roadP, lp), []).append((rid, ln.get("id"), roadS, ls, eh))
    return R, moves


def ang(a, b): return abs((a-b+math.pi) % (2*math.pi) - math.pi)


def traverse(R, moves, start_lane):
    start = min((r for r in R if R[r]["j"] == "-1"),
                key=lambda r: math.hypot(R[r]["start"][0]-START_XY[0], R[r]["start"][1]-START_XY[1])
                + 5*ang(R[r]["start"][2], 0.0))
    cur = (start, start_lane); path = [start]; visited = {(start, start_lane)}; changes = 0
    for _ in range(400):
        cands = moves.get(cur, [])
        if not cands:
            return path, False, f"dead lane {cur}", changes
        ch = R[cur[0]]["end"][2]
        # prefer same-lane + straight; penalize lane change + heading change
        cands = sorted(cands, key=lambda c: (c[1] != cur[1] or c[3] != cur[1], ang(c[4], ch)))
        conn, laneC, roadS, laneS, _ = cands[0]
        if laneS != cur[1]:
            changes += 1
        nxt = (roadS, laneS)
        path.append(conn)
        if roadS == start and len(path) > 8:
            path.append(roadS); return path, True, "closed", changes
        if nxt in visited:
            return path, False, f"revisit {nxt}", changes
        path.append(roadS); visited.add(nxt); cur = nxt
    return path, False, "maxiter", changes


def road_lane_to_lanepath(t):
    lane_lp = {int(m.group(2)): int(m.group(1)) for m in re.finditer(r"LanePath\.\d+ = (\d+) (\d+)", t)}
    # (road, laneidx) -> lanepath, laneidx = |lane|-1 -> LaneR.<laneidx>
    out = {}
    for kind in ("Link", "Junction.+?Link"):
        for m in re.finditer(rf"({kind})\.(\d+)\.Tag = odrRoadId:(\S+)", t):
            pass
    # simpler: scan Link and Junction.Link blocks
    for lk, road in re.findall(r"\bLink\.(\d+)\.Tag = odrRoadId:(\S+)", t):
        for li in range(3):
            r = re.search(rf"\bLink\.{lk}\.LaneSection\.0\.LaneR\.{li}\.ID = (\d+)", t)
            if r and int(r.group(1)) in lane_lp:
                out[(road, li)] = lane_lp[int(r.group(1))]
    for j, lk, road in re.findall(r"Junction\.(\d+)\.Link\.(\d+)\.Tag = odrRoadId:(\S+)", t):
        for li in range(3):
            r = re.search(rf"Junction\.{j}\.Link\.{lk}\.LaneSection\.0\.LaneR\.{li}\.ID = (\d+)", t)
            if r and int(r.group(1)) in lane_lp:
                out.setdefault((road, li), lane_lp[int(r.group(1))])
    return out


def main():
    R, moves = parse()
    best = None
    for sl in ("-1", "-2", "-3"):
        path, closed, why, changes = traverse(R, moves, sl)
        print(f"start lane {sl}: {len(path)} roads, closed={closed} ({why}), lane-changes={changes}")
        if closed and (best is None or changes < best[3]):
            best = (path, closed, why, changes, sl)
    if not best:
        print("no closed loop found"); return
    path, closed, why, changes, sl = best
    print(f"\nBEST start lane {sl}: closed loop, {len(path)} roads, lane-changes={changes}")
    r2lp = road_lane_to_lanepath(RD.read_text(encoding="utf-8", errors="ignore"))
    # we tracked roads; for lane-paths use lane index 0 fallback then try exact
    lps, seen = [], set()
    for rd in path:
        lp = r2lp.get((rd, 0)) or r2lp.get((rd, 1)) or r2lp.get((rd, 2))
        if lp and lp not in seen:
            lps.append(str(lp)); seen.add(lp)
    print(f"DrvPath ({len(lps)} lane-paths) = {' '.join(lps)}")
    (HERE / "route_drvpath.txt").write_text(" ".join(lps) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
