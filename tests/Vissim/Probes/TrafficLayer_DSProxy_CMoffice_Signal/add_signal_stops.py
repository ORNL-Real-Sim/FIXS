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

STOP_BACK = 12.0      # DrvStop = this many metres before the approach lane-path's downstream
                      # (junction) end -> the ego stops ~12 m before the signal head at the junction.
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


def main():
    t = BASE_RD.read_text(encoding="utf-8", errors="ignore")
    maxid = int(re.search(r"MaxUsedObjId = (\d+)", t).group(1))
    nid = maxid

    def newid():
        nonlocal nid; nid += 1; return nid

    lines = t.split("\n")

    # ---- 1. spread the overlapping heads on each approach mount (lateral hOff only) ----
    #     collect heads per (rl, mount)
    groups = {}
    for i, ln in enumerate(lines):
        m = re.match(r"RL\.(\d+)\.Mount\.(\d+)\.\d+ = 1 \d+ ", ln)
        if m:
            groups.setdefault((int(m.group(1)), int(m.group(2))), []).append(i)
    for idxs in groups.values():
        n = len(idxs)
        if n < 2:
            continue
        for k, i in enumerate(idxs):
            p = lines[i].split()
            p[5] = f"{2.6 + (k - (n - 1) / 2) * HEAD_SPREAD:.2f}"   # hOff field
            lines[i] = " ".join(p)
    t = "\n".join(lines)

    # ---- 2. straight-movement DrvStop on each approach the ego's Route crosses ----
    route = parse_route(t)
    lp2rl = map_lanepath_to_rl(t)
    straight_ctrl = {}                       # RL -> first straight (type-1) controller
    for ln in t.split("\n"):
        hm = re.match(r"RL\.(\d+)\.Mount\.\d+\.\d+ = 1 (\d+) [\d.]+ [\d.-]+ .* (\d+)$", ln)
        if hm and int(hm.group(3)) == 1:
            straight_ctrl.setdefault(int(hm.group(1)), int(hm.group(2)))

    stops, blocks = [], []
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
    OUT_RD.write_text(t, encoding="utf-8")
    print(f"[rd5] {OUT_RD.name}: {len(stops)} straight DrvStop(s) (stop {STOP_BACK} m before the "
          f"junction; head at the junction is beyond the stop line), heads spread")

    # ---- 3. TestRun: SimpleTL_VISSIM with new road + Car_Normal driver ----
    tr = BASE_TR.read_text(encoding="utf-8", errors="ignore")
    tr = re.sub(r"(?m)^Road\.FName = .*$", f"Road.FName = {OUT_RD.name}", tr, count=1)
    tr = re.sub(r"(?m)^Vehicle\.DriverTemplate\.FName = .*$",
                "Vehicle.DriverTemplate.FName = Car_Normal", tr, count=1)
    OUT_TR.write_text(tr, encoding="utf-8")
    print(f"[testrun] {OUT_TR.name}: Road -> {OUT_RD.name}, DriverTemplate -> Car_Normal")


if __name__ == "__main__":
    main()
