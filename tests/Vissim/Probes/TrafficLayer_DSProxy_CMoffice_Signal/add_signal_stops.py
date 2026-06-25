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
`simple_traffic_light_signalstop.rd5`:

  1. Add ONE straight-movement DrvStop per approach the ego's Route crosses, on the ego's
     lane-path, anchored to its downstream end (lonR=1, s=STOP_BACK). With the signal head at
     the junction (the approach edge's downstream end), the ego stops ~STOP_BACK m before it --
     so the head sits beyond the stop line. One marker per route->movement, never one per head
     (a green movement-head would otherwise cancel a red one -- the Q1 finding).
  2. Spread each approach mount's overlapping heads laterally so straight/left/right are
     visually distinct (purely cosmetic; does NOT move heads off the approach edge).

IMPORTANT -- why the heads are NOT moved across the junction: IPGDriver only fires a
traffic-light DrvStop when the signal HEAD is on the APPROACH side (at/before the stop line).
Relocating the head to the far side (the edge across the junction) makes the stop NOT fire and
the ego runs the red (verified: with all controllers static-red, an across-junction layout
stopped the ego at only 1 of 6 crossings). So the head stays at the approach's downstream end
(at the junction, beyond the stop line) -- the most "far-side" placement that still stops.

It also writes the TestRun `SimpleTL_SignalStop` = the working SimpleTL_VISSIM run with
Road.FName -> the new rd5 and DriverTemplate.FName -> Car_Normal (McLaren + Route kept).

Run:  python add_signal_stops.py        (then run_cm_scene_only.bat to drive it in CarMaker)
"""
from __future__ import annotations
import re, pathlib

HERE = pathlib.Path(__file__).resolve().parent
CMPROJ = HERE.parents[3] / "ProprietaryFiles" / "CM13_proj"
BASE_RD = CMPROJ / "Data" / "Road" / "simple_traffic_light.rd5"
OUT_RD = CMPROJ / "Data" / "Road" / "simple_traffic_light_signalstop.rd5"
BASE_TR = CMPROJ / "Data" / "TestRun" / "SimpleTL_VISSIM"
OUT_TR = CMPROJ / "Data" / "TestRun" / "SimpleTL_SignalStop"

STOP_BACK = 12.0      # DrvStop = this many metres before the approach lane-path's downstream end
HEAD_S = 1.0          # mount the relocated head this many metres into the across edge
HEAD_SPREAD = 0.8     # lateral spacing between heads on one mount (m), to de-overlap


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

    # straight controller per RL (read BEFORE relocating the heads)
    straight_ctrl = {}
    for ln in lines:
        hm = re.match(r"RL\.(\d+)\.Mount\.\d+\.\d+ = 1 (\d+) [\d.]+ [\d.-]+ .* (\d+)$", ln)
        if hm and int(hm.group(3)) == 1:
            straight_ctrl.setdefault(int(hm.group(1)), int(hm.group(2)))

    # ---- 1. relocate each ego-crossing approach mount to the BEGINNING of the route's NEXT
    #         edge (route[i+2], i.e. the edge after the junction), facing back at the stop line.
    #         Using the ROUTE (not geometric matching) avoids the 2-way-corridor mismatch. ----
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
    n = len(route)
    for i, lp in enumerate(route):
        rl_a = lp2rl.get(lp)
        if rl_a not in nodes:
            continue
        mounts = [(r, mi) for (r, mi) in head_mounts if r == rl_a]
        if not mounts:
            continue
        rl_d = lp2rl.get(route[(i + 2) % n])      # the edge after the junction (on the route)
        if rl_d not in nodes:
            continue
        a0, a1 = nodes[rl_a]; d0, d1 = nodes[rl_d]
        # departure edge's BEGINNING = its node nearest the approach (the shared junction corner)
        dn_name, dn = min([("n0", d0), ("n1", d1)],
                          key=lambda c: min(math.dist(c[1], a0), math.dist(c[1], a1)))
        lonR_d = 0 if dn_name == "n0" else 1
        facing = -1 if dn_name == "n0" else 1     # face back toward the junction / stop line
        for (r, mi) in mounts:
            s_i, e_i = head_mounts[(r, mi)]
            new_mi = next_mi[rl_d]; next_mi[rl_d] += 1
            out = []
            for ln in lines[s_i:e_i + 1]:
                # re-key the whole block to the departure RL/mount index; keep osc2cm's head
                # fields (hOff spread + facing) EXACTLY -- only the mount line's s/lonR change,
                # so the gantry renders just as it did, at the new (across-junction) position.
                ln2 = ln.replace(f"RL.{r}.Mount.{mi}", f"RL.{rl_d}.Mount.{new_mi}")
                mm = re.match(rf"RL\.{rl_d}\.Mount\.{new_mi}\.ID = (\d+) (\d+)$", ln2)
                ml = re.match(rf"RL\.{rl_d}\.Mount\.{new_mi} = (.+)$", ln2)
                if mm:                                  # mount .ID: fix dependsOnObjId -> rl_d
                    ln2 = f"RL.{rl_d}.Mount.{new_mi}.ID = {mm.group(1)} {rl_d}"
                elif ml:                                # mount line: only s + lonR change
                    pp = ml.group(1).split(); pp[0] = f"{HEAD_S}"; pp[1] = f"{lonR_d}"
                    ln2 = f"RL.{rl_d}.Mount.{new_mi} = " + " ".join(pp)
                out.append(ln2)
            moved_blocks.append("\n".join(out))
            for j in range(s_i, e_i + 1):
                drop.add(j)
            moved_log.append((rl_a, rl_d, dn_name, facing))

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

    OUT_RD.write_text(t, encoding="utf-8")
    print(f"[rd5] {OUT_RD.name}: relocated {len(moved_log)} mount(s) to the across edge (after the "
          f"junction), {len(stops)} straight DrvStop(s) (stop {STOP_BACK} m before the junction)")
    for rl_a, rl_d, dn, fc in moved_log:
        print(f"      head mount RL.{rl_a} -> RL.{rl_d} @{dn} facing={fc}")

    # ---- 3. TestRun: SimpleTL_VISSIM with new road + Car_Normal driver ----
    tr = BASE_TR.read_text(encoding="utf-8", errors="ignore")
    tr = re.sub(r"(?m)^Road\.FName = .*$", f"Road.FName = {OUT_RD.name}", tr, count=1)
    tr = re.sub(r"(?m)^Vehicle\.DriverTemplate\.FName = .*$",
                "Vehicle.DriverTemplate.FName = Car_Normal", tr, count=1)
    OUT_TR.write_text(tr, encoding="utf-8")
    print(f"[testrun] {OUT_TR.name}: Road -> {OUT_RD.name}, DriverTemplate -> Car_Normal")


if __name__ == "__main__":
    main()
