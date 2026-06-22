"""
Phase 1c builder: inject the faithful SUMO signal plan into the VISSIM corridor.

Reads signal_plan.json (from parse_signals.py), then:
  1. writes one VISSIG .sig per controller (fixed-time, redAmber=0, amber=3 s,
     cycle + green/red begins transcribed from SUMO),
  2. COM-loads simple_traffic_light.inpx, creates the 3 signal controllers wired
     to their .sig (SupplyFile1/2), adds the signal groups, and places signal
     heads at each movement's (approach road -> VISSIM link, lane = SUMO fromLane+1,
     pos = xodr s) assigned to its group,
  3. saves simple_traffic_light_signals.inpx,
  4. reloads + runs the sim and prints each SG's SigState sequence to confirm the
     programs cycle faithfully (must show RED and GREEN per group).

SC numbering (also the toTlsData "<ctrl>_<grp>" id): int_west=1, int_center=2, int_east=3.
"""
from __future__ import annotations
import json, sys, pathlib
import pythoncom, win32com.client

PROGID = "VISSIM.Vissim.2200"
HERE = pathlib.Path(__file__).resolve().parent
PLAN = json.loads((HERE / "signal_plan.json").read_text())
INPX_IN = HERE / "simple_traffic_light.inpx"
INPX_OUT = HERE / "simple_traffic_light_signals.inpx"

SC_NO = {"int_west": 1, "int_center": 2, "int_east": 3}
AMBER_MS = 3000
REDAMBER_MS = 0


def begin_ms(transitions, state):
    for sec, st in transitions:
        if st == state:
            return sec * 1000
    return None


def write_sig(tl, sc_no, p):
    cyc = p["cycle"] * 1000
    sg_defs, prog_sgs = [], []
    for g in p["groups"]:
        sg_defs.append(
            f'    <sg id="{g["sg"]}" name="" defaultSignalSequence="3">\n'
            f'      <defaultDurations>\n'
            f'        <defaultDuration display="2" duration="{REDAMBER_MS}" />\n'
            f'        <defaultDuration display="4" duration="{AMBER_MS}" />\n'
            f'        <defaultDuration display="1" duration="0" />\n'
            f'        <defaultDuration display="3" duration="0" />\n'
            f'      </defaultDurations>\n    </sg>')
        gb = begin_ms(g["transitions"], "GREEN")
        rb = begin_ms(g["transitions"], "RED")
        prog_sgs.append(
            f'        <sg sg_id="{g["sg"]}" signal_sequence="3">\n'
            f'          <cmds>\n'
            f'            <cmd display="3" begin="{rb}" />\n'
            f'            <cmd display="1" begin="{gb}" />\n'
            f'          </cmds>\n'
            f'          <fixedstates>\n'
            f'            <fixedstate display="2" duration="{REDAMBER_MS}" />\n'
            f'            <fixedstate display="4" duration="{AMBER_MS}" />\n'
            f'          </fixedstates>\n        </sg>')
    xml = (f'<?xml version="1.0" encoding="UTF-8"?>\n'
           f'<sc id="{sc_no}" name="" frequency="1" defaultIntergreenMatrix="0">\n'
           f'  <sgs>\n' + "\n".join(sg_defs) + "\n  </sgs>\n"
           f'  <intergreenmatrices />\n  <progs>\n'
           f'    <prog id="1" cycletime="{cyc}" switchpoint="0" offset="{p["offset"]*1000}" '
           f'intergreens="0" name="Signal program">\n      <sgs>\n'
           + "\n".join(prog_sgs) +
           f'\n      </sgs>\n    </prog>\n  </progs>\n'
           f'  <stages />\n  <interstageProgs />\n  <stageProgs />\n  <dailyProgLists />\n</sc>\n')
    path = HERE / f"{tl}.sig"
    path.write_text(xml, encoding="utf-8")
    return path.name


def main():
    # 1. write .sig files
    for tl, p in PLAN.items():
        name = write_sig(tl, SC_NO[tl], p)
        print(f"[sig] wrote {name} (cycle {p['cycle']}s, {len(p['groups'])} groups)")

    # 2. COM build
    pythoncom.CoInitializeEx(pythoncom.COINIT_APARTMENTTHREADED)
    v = win32com.client.Dispatch(PROGID, clsctx=pythoncom.CLSCTX_LOCAL_SERVER)
    print("dispatch OK", file=sys.stderr)
    v.LoadNet(str(INPX_IN), False)
    net = v.Net

    # road id -> (link no, lane count, length); names look like "116-0-Right"
    road2link = {}
    for lk in net.Links:
        nm = lk.AttValue("Name") or ""
        no = int(lk.AttValue("No"))
        tok = nm.split("-")[0]
        if tok.isdigit():
            road2link.setdefault(tok, (no, int(lk.AttValue("NumLanes")), float(lk.AttValue("Length2D"))))
    print(f"mapped {len(road2link)} numeric approach roads -> links", file=sys.stderr)

    head_key = 0
    placed, skipped = 0, 0
    for tl, p in PLAN.items():
        sc_no = SC_NO[tl]
        sc = net.SignalControllers.AddSignalController(sc_no)
        sc.SetAttValue("SupplyFile1", "#exe#vissig.config")
        sc.SetAttValue("SupplyFile2", f"{tl}.sig")
        for g in p["groups"]:
            sg = sc.SGs.AddSignalGroup(g["sg"])
            try:
                sg.SetAttValue("Amber", AMBER_MS / 1000.0)
                sg.SetAttValue("RedAmber", REDAMBER_MS / 1000.0)
            except Exception:
                pass
            seen_lane = set()
            for m in g["members"]:
                road, s, fl = m.get("road"), m.get("s"), m.get("fromLane")
                if road is None or road not in road2link or fl is None:
                    skipped += 1; continue
                link_no, nlanes, length = road2link[road]
                lane_no = min(fl + 1, nlanes)              # SUMO lane0 (right) -> VISSIM lane1
                if (link_no, lane_no) in seen_lane:        # one head per lane per approach
                    continue
                seen_lane.add((link_no, lane_no))
                pos = min(float(s), length - 1.0)
                try:
                    lane = net.Links.ItemByKey(link_no).Lanes.ItemByKey(lane_no)
                    head_key += 1
                    head = net.SignalHeads.AddSignalHead(head_key, lane, pos)
                    head.SetAttValue("SG", f"{sc_no}-{g['sg']}")
                    placed += 1
                except Exception as e:
                    print(f"  head FAIL {tl} sg{g['sg']} road{road} lane{lane_no}: {type(e).__name__}", file=sys.stderr)
                    skipped += 1
    print(f"placed {placed} heads, skipped {skipped}", file=sys.stderr)
    v.SaveNetAs(str(INPX_OUT), False)
    print(f"saved {INPX_OUT.name}")

    # 3. validate: reload + run, sample SigState per SG
    v.LoadNet(str(INPX_OUT), False)
    net = v.Net
    v.Simulation.SetAttValue("SimPeriod", 95)
    v.Simulation.SetAttValue("SimRes", 1)
    sgs = []
    for tl, p in PLAN.items():
        sc = net.SignalControllers.ItemByKey(SC_NO[tl])
        for g in p["groups"]:
            sgs.append((f"{tl}.SG{g['sg']}", sc.SGs.ItemByKey(g["sg"])))
    samples = {name: [] for name, _ in sgs}
    for _ in range(95):
        v.Simulation.RunSingleStep()
        for name, sg in sgs:
            samples[name].append(str(sg.AttValue("SigState"))[:1])  # R/G/A/...
    v.Simulation.Stop()
    print("\n=== SigState over 95 s (first letter; faithful = each shows G and R) ===")
    for name, _ in sgs:
        seq = "".join(samples[name])
        print(f"  {name:18s} {sorted(set(samples[name]))}  {seq}")
    v = None
    return 0


if __name__ == "__main__":
    sys.exit(main())
