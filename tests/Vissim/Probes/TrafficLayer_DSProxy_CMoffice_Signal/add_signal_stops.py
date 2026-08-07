"""
FIXS #172 -- SignalStop: make the CarMaker ego physically STOP at red lights.

The base demo (simple_traffic_light.rd5 from osc2cm) renders the VISSIM-driven signal heads
but the ego drives straight through reds, because:
  (a) osc2cm imports signal HEADS but no DrvStop stop markers, and IPGDriver only brakes for
      a traffic light via a DrvStop marker on the route (heads alone are ignored), and
  (b) the osc2cm TestRun uses DriverTemplate.FName = Car_Normal_osc, a trajectory-replay tune
      that doesn't execute stops; the stock Car_Normal tune does.

This script post-processes the committed rd5 (route + heads + controllers already present; it
does NOT re-run osc2cm, which would wipe the hand-made Route) into
`simple_traffic_light.rd5`:

  1. RELOCATE each approach's signal-head mount to the ACROSS edge (the edge after the
     junction, in the ego's travel direction) so the approaching ego sees the heads ahead,
     beyond the stop line. The stop still fires: IPGDriver brakes whenever a DrvStop marker
     AND a head referencing that controller both exist -- the head LOCATION is free (approach
     or far-side). (An earlier "far-side breaks the stop" note was a corridor ROUTE bug,
     since fixed -- see the cm-signal-stop-mechanism memory.)
  2. Re-place each head laterally over its SUMO lane. CM mounts the heads on a gantry and
     renders INCREASING hOff from the RIGHT lane to the LEFT. The SUMO lane lateral `t` is
     negative (right of the road reference line) and MORE negative for the rightmost lane, so
         hOff = t + gantry_len        (gantry_len = the mount's beam length, field 9)
     maps the rightmost lane (most negative t) to the SMALLEST hOff (rendered on the right)
     and the leftmost to the largest -- matching SUMO's lane order. THE BUG WE CHASED: the
     old `hOff = -t + 1.0` had the wrong sign, so it MIRRORED the lanes (left-turn heads
     landed on the right lane, etc.). That looked like "flipped arrows" but was a flipped
     LANE mapping. The arrow TYPE is left exactly as osc2cm set it (faithful to the xodr
     signal subtype 10/20/30 = left/right/straight = the SUMO movement dir -- do NOT
     fake-swap it); `facing` is also kept -- flipping facing pushes heads OFF-ROAD because
     CM measures hOff in the facing frame.
  3. Add ONE straight-movement DrvStop per approach the ego's Route crosses, on the ego's
     lane-path, anchored to its downstream end (lonR=1, s=STOP_BACK ~ at the stop line). One
     marker per route->movement, never one per head (a green movement-head would otherwise
     cancel a red one -- the Q1 finding).

It also writes the TestRun `SimpleTrafficLight_ego` = the working SimpleTrafficLight_import run with
Road.FName -> the new rd5 and DriverTemplate.FName -> Car_Normal (McLaren + Route kept).

Run:  python add_signal_stops.py        (then run_signal_demo_gui.bat for the full co-sim)
"""
from __future__ import annotations
import re, pathlib

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
SUMO_NET = REPO / "tests" / "Sumo" / "networks" / "simple_traffic_light"   # shared geometry (xodr)
BASE_RD = CMPROJ / "Data" / "Road" / "simple_traffic_light_import.rd5"     # raw osc2cm import (intermediate)
OUT_RD = CMPROJ / "Data" / "Road" / "simple_traffic_light.rd5"             # demo road (heads + DrvStops)
BASE_TR = CMPROJ / "Data" / "TestRun" / "SimpleTrafficLight_import"        # raw osc2cm TestRun (intermediate)
OUT_TR = CMPROJ / "Data" / "TestRun" / "SimpleTrafficLight_ego"          # ego-only base (build_cosim_testrun extends it)

STOP_BACK = 2.0       # DrvStop this many metres before the approach lane-path's downstream end
                      # (the junction) -> at the stop line, near the end of the approach edge
HEAD_S = 1.0          # mount the relocated head this many metres into the across edge
HEAD_SPREAD = 0.5     # lateral spacing between heads on one mount (m) -> cluster them close together


def parse_route(t):
    m = re.search(r"(?m)^Route\.0\.DrvPath:\n((?:\t.*\n)+)", t)
    return [int(x) for x in m.group(1).split()]


def map_lanepath_to_rl(t):
    lp2lane = {int(a): int(b) for a, b in re.findall(r"(?m)^LanePath\.\d+ = (\d+) (\d+)", t)}
    lane2rl = {}
    for blk in re.split(r"(?m)^(?=Link\.\d+\.ID = )", t):
        rlm = re.search(r"RL\.ID = (\d+)", blk)
        if not rlm:
            continue
        for lid in re.findall(r"Lane[RL]\.\d+\.ID = (\d+)", blk):
            lane2rl[int(lid)] = int(rlm.group(1))
    return {lp: lane2rl.get(lane) for lp, lane in lp2lane.items()}


def rl_nodes(t):
    out = {}
    for blk in re.split(r"(?m)^(?=Link\.\d+\.ID = )", t):
        rlm = re.search(r"RL\.ID = (\d+)", blk)
        n0 = re.search(r"Node0 = ([\d.-]+) ([\d.-]+)", blk)
        n1 = re.search(r"Node1 = ([\d.-]+) ([\d.-]+)", blk)
        if rlm and n0 and n1:
            out[int(rlm.group(1))] = ((float(n0.group(1)), float(n0.group(2))),
                                      (float(n1.group(1)), float(n1.group(2))))
    return out


def main():
    t = BASE_RD.read_text(encoding="utf-8", errors="ignore")
    maxid = int(re.search(r"MaxUsedObjId = (\d+)", t).group(1))
    nid = maxid

    def newid():
        nonlocal nid; nid += 1; return nid

    import math
    route = parse_route(t)
    lp2rl = map_lanepath_to_rl(t)
    nodes = rl_nodes(t)
    lines = t.split("\n")

    # Each head's correct lateral position comes from the xodr signal's t, matched EXACTLY by the
    # odrSignalId tag (= the xodr signal id). CM hOff = t + gantry_len puts the head over its lane
    # in SUMO's right->left order (see hoff_of). IMPORTANT: join on the xodr signal id, NOT
    # signal_plan.json's linkIndex -- signal_plan uses its own numbering that differs from
    # netconvert's signal ids in the rd5 tags. osc2cm itself mis-placed ~half the heads laterally
    # (piling them on one lane); this recomputes hOff from the xodr t so each sits over its lane.
    id2t = {}
    try:
        xtext = (SUMO_NET / "simple_traffic_light.xodr").read_text(errors="ignore")
        id2t = {m.group(1): float(m.group(2))
                for m in re.finditer(r'<signal id="([^"]+)"[^>]*\bt="([\d.-]+)"', xtext)}
    except Exception:
        pass

    def hoff_of(tval, glen):
        # Heads sit on a gantry of length `glen` spanning the road; CM renders
        # increasing hOff from the RIGHT lane to the LEFT. SUMO lateral t is
        # negative (right of the reference line) and MORE negative for the
        # rightmost lane, so hOff = t + glen maps the rightmost lane (most negative
        # t) to the SMALLEST hOff (rendered on the right) and the leftmost lane to
        # the largest hOff -- matching SUMO's lane order. (Old -t+1.0 mirrored it.)
        return (tval + glen) if tval is not None else glen / 2.0

    # straight controller per RL (read BEFORE relocating the heads)
    straight_ctrl = {}
    for ln in lines:
        hm = re.match(r"RL\.(\d+)\.Mount\.\d+\.\d+ = 1 (\d+) [\d.]+ [\d.-]+ .* (\d+)$", ln)
        if hm and int(hm.group(3)) == 1:
            straight_ctrl.setdefault(int(hm.group(1)), int(hm.group(2)))

    # ---- 1. relocate EVERY head-bearing mount (main corridor AND minor cross streets) to the
    #         BEGINNING of its across edge -- the edge after the junction. The across edge is the
    #         collinear edge whose Node0 (its forward start) sits just past this edge's downstream
    #         end (a1), going the same direction. Matching only on the departure edge's Node0
    #         (forward traversal) avoids the undivided-2-way mismatch -- it reproduces the ego
    #         ROUTE's far-side targets on the corridor and also resolves the cross streets.
    #         osc2cm's head facing is kept; the heads on each mount are CLUSTERED close together. ----
    GAP_MIN, GAP_MAX, DIR_DOT = 0.5, 45.0, 0.95

    def unit(a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]; d = math.hypot(dx, dy) or 1.0
        return (dx / d, dy / d)

    def across_edge(rl_a):
        a0, a1 = nodes[rl_a]; da = unit(a0, a1)          # travel dir N0->N1; downstream end = a1
        best = None
        for rl_d, (d0, d1) in nodes.items():
            if rl_d == rl_a:
                continue
            gap = math.dist(a1, d0)                       # across edge is entered at its Node0
            if not (GAP_MIN <= gap <= GAP_MAX):
                continue
            dd = unit(d0, d1)
            if da[0] * dd[0] + da[1] * dd[1] < DIR_DOT:   # must continue the same direction
                continue
            if best is None or gap < best[0]:
                best = (gap, rl_d)
        return None if best is None else best[1]

    mblk = {}                                # (rl, mi) -> [start_line, end_line]
    for i, ln in enumerate(lines):
        m = re.match(r"RL\.(\d+)\.Mount\.(\d+)(?:\.|\s|$)", ln)
        if m:
            b = mblk.setdefault((int(m.group(1)), int(m.group(2))), [i, i]); b[1] = i
    head_mounts = {k: v for k, v in mblk.items()
                   if any(re.match(rf"RL\.{k[0]}\.Mount\.{k[1]}\.\d+ = 1 \d+ ", lines[j])
                          for j in range(v[0], v[1] + 1))}
    next_mi = {rl: (max([int(x) for x in re.findall(rf"(?m)^RL\.{rl}\.Mount\.(\d+)\.ID = ", t)] or [-1]) + 1)
               for rl in nodes}

    drop, moved_blocks, moved_log = set(), [], []
    for (r, mi), (s_i, e_i) in head_mounts.items():
        rl_d = across_edge(r) if r in nodes else None
        if rl_d is None:                                 # no across edge (ramp/stub) -> leave it
            continue
        new_mi = next_mi[rl_d]; next_mi[rl_d] += 1
        block = lines[s_i:e_i + 1]
        # head part-index -> SUMO lateral t (via its odrSignalId tag's linkIndex)
        part_t = {}
        for ln in block:
            tg = re.match(rf"RL\.{r}\.Mount\.{mi}\.(\d+)\.Tag = odrSignalId:(\S+)$", ln)
            if tg:
                part_t[int(tg.group(1))] = id2t.get(tg.group(2))
        # gantry length of THIS mount (field 9 of the mount value line) sizes hOff
        glen = 10.6
        for ln in block:
            mlm = re.match(rf"RL\.{r}\.Mount\.{mi} = (.+)$", ln)
            if mlm:
                mt = mlm.group(1).split()
                if len(mt) >= 10:
                    try:
                        glen = float(mt[9])
                    except ValueError:
                        pass
                break
        # group heads by lane (t); place each lane at hOff=t+glen, heads SHARING a lane cluster
        lanes = {}
        for p, tv in part_t.items():
            lanes.setdefault(tv, []).append(p)
        part_hoff = {}
        for tv, ps in lanes.items():
            base = hoff_of(tv, glen)
            for j2, p in enumerate(sorted(ps)):
                part_hoff[p] = base + (j2 - (len(ps) - 1) / 2) * HEAD_SPREAD
        out = []
        for ln in block:
            ln2 = ln.replace(f"RL.{r}.Mount.{mi}", f"RL.{rl_d}.Mount.{new_mi}")
            mm = re.match(rf"RL\.{rl_d}\.Mount\.{new_mi}\.ID = (\d+) (\d+)$", ln2)
            ml = re.match(rf"RL\.{rl_d}\.Mount\.{new_mi} = (.+)$", ln2)
            hd = re.match(rf"RL\.{rl_d}\.Mount\.{new_mi}\.(\d+) = 1 (.+)$", ln2)
            if mm:                                       # mount .ID: fix dependsOnObjId -> rl_d
                ln2 = f"RL.{rl_d}.Mount.{new_mi}.ID = {mm.group(1)} {rl_d}"
            elif ml:                                     # mount line: move to the across edge's start
                pp = ml.group(1).split(); pp[0] = f"{HEAD_S}"; pp[1] = "0"
                ln2 = f"RL.{rl_d}.Mount.{new_mi} = " + " ".join(pp)
            elif hd:                                     # set hOff to the head's own SUMO lane
                part = int(hd.group(1)); pp = hd.group(2).split()
                if part in part_hoff:
                    pp[2] = f"{part_hoff[part]:.2f}"
                # Keep osc2cm's facing AND arrow type. The arrow type comes faithfully
                # from the xodr signal subtype (= the SUMO movement dir), and osc2cm's
                # facing keeps the head over its correct lane (CM measures hOff in the
                # facing frame, so flipping facing pushes the head off-road). The head
                # faces away from the ego so arrows currently render mirrored -- the
                # orientation fix is tracked separately, not by faking the type.
                ln2 = f"RL.{rl_d}.Mount.{new_mi}.{part} = 1 " + " ".join(pp)
            out.append(ln2)
        moved_blocks.append("\n".join(out))
        for j in range(s_i, e_i + 1):
            drop.add(j)
        moved_log.append((r, rl_d))

    lines = [ln for j, ln in enumerate(lines) if j not in drop]
    t = "\n".join(lines)

    # ---- 2. straight-movement DrvStop on each approach the ego's Route crosses ----
    stops, blocks = [], []
    blocks.append("\n".join(moved_blocks) + ("\n" if moved_blocks else ""))
    for lp in route:
        rl = lp2rl.get(lp)
        if rl in straight_ctrl:
            stops.append((rl, lp, straight_ctrl[rl]))
    for rl, lp, ctrl in stops:
        idxs = [int(x) for x in re.findall(rf"(?m)^RL\.{rl}\.Marker\.(\d+)\.ID = ", t)]
        mk_i = (max(idxs) + 1) if idxs else 0
        # DrvStop Param = s lonR val refObjId type dur ; lonR=1 -> s from lane-path END (junction)
        blocks.append(
            f"RL.{rl}.Marker.{mk_i}.ID = {newid()} {lp}\n"
            f"RL.{rl}.Marker.{mk_i}.Type = DrvStop\n"
            f"RL.{rl}.Marker.{mk_i}.Param = {STOP_BACK} 1 1 {ctrl} 2 0\n")
    t = t.replace(f"MaxUsedObjId = {maxid}", "".join(blocks) + f"MaxUsedObjId = {nid}")

    # CM requires each RL's Mount indices to start at 0 and be contiguous. Relocating a mount
    # away from an RL (and adding another at a higher index) can leave a gap (e.g. Mount.1 with
    # no Mount.0) -> CM won't render it. Renumber every RL's mounts to contiguous 0-based.
    rl_mi = {}
    for m in re.finditer(r"(?m)^RL\.(\d+)\.Mount\.(\d+)\.ID = ", t):
        rl_mi.setdefault(int(m.group(1)), set()).add(int(m.group(2)))
    for rl, idxs in rl_mi.items():
        remap = {old: new for new, old in enumerate(sorted(idxs))}
        if all(o == n for o, n in remap.items()):
            continue
        for old in idxs:                                    # phase 1: shift to a temp range
            t = re.sub(rf"(?m)^(RL\.{rl}\.Mount\.){old}\b", rf"\g<1>{old + 1000}", t)
        for old, new in remap.items():                      # phase 2: temp -> contiguous
            t = re.sub(rf"(?m)^(RL\.{rl}\.Mount\.){old + 1000}\b", rf"\g<1>{new}", t)

    # Make every CM traffic-light controller PURELY EXTERNALLY DRIVEN (listen only to
    # VISSIM): set initialPhase = 0 (off) and all 5 phase durations = 0. The
    # Control.TrfLight format is `objId name "<startCond>" initialPhase d0 d1 d2 d3 d4`
    # where d0..d4 are the seconds spent in off/green/yellow/red/red+yellow (per the
    # CarMaker IPGRoad struct tRoadCtrlTrfLight + doc/CarMakerDoc.md). osc2cm emits a
    # non-zero fixed-time cycle (e.g. "0 0 30 3 25 3"), which makes CM run its OWN
    # countdown on top of the sync -- with all durations 0 the controller never
    # advances a phase, so the only thing that changes its state is the VISSIM sync.
    n_ext = 0
    t, n_ext = re.subn(r'(?m)^(Control\.TrfLight\.\d+ = \d+ \S+ "[^"]*") .*$',
                       r"\g<1> 0 0 0 0 0 0", t)

    OUT_RD.write_text(t, encoding="utf-8")
    print(f"[rd5] {OUT_RD.name}: relocated {len(moved_log)} mount(s) to the across edge (after the "
          f"junction), {len(stops)} straight DrvStop(s) (stop {STOP_BACK} m before the junction), "
          f"{n_ext} controller(s) set to external (timing 0, initial off)")
    for rl_a, rl_d in sorted(moved_log):
        print(f"      head mount RL.{rl_a} -> RL.{rl_d} (across-edge start)")

    # ---- 3. TestRun: SimpleTrafficLight_import with new road + Car_Normal driver ----
    tr = BASE_TR.read_text(encoding="utf-8", errors="ignore")
    tr = re.sub(r"(?m)^Road\.FName = .*$", f"Road.FName = {OUT_RD.name}", tr, count=1)
    tr = re.sub(r"(?m)^Vehicle\.DriverTemplate\.FName = .*$",
                "Vehicle.DriverTemplate.FName = Car_Normal", tr, count=1)
    OUT_TR.write_text(tr, encoding="utf-8")
    print(f"[testrun] {OUT_TR.name}: Road -> {OUT_RD.name}, DriverTemplate -> Car_Normal")


if __name__ == "__main__":
    main()
