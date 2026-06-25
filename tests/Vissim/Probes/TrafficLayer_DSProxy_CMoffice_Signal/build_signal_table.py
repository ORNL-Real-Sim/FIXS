"""
Phase 2 (Q2): build the RSsignalTable.csv that wires VISSIM signal-group states
to the CarMaker traffic-light objects, and rename the CM controllers to the
VISSIM convention so the rd5 is self-documenting.

WHY this mapping is what it is (verified against the runtime, not guessed):

  DSProxyMode.cpp::toTlsData() sends ONE TlsData per (controller, signal group):
      name  = "<ControllerID>_<SignalGroupID>"   e.g. "2_1"
      state = a SINGLE char (that group's state)
  SocketHelper keys TlsDataRecv_um by that name; VirEnvHelper::runStep then does
      TrfLight.Objs[cmTrafficLightIndex].State = tlsChar2CmState(state.at(headId))
  Because each name's state string has length 1, the head index is always 0.

  So in RSsignalTable.csv (cols: SignalControllerId, SignalGroupId, SignalHeadId,
  CmTrafficLightIndex, CmControllerId):
    SignalControllerId  = "<SCno>_<sg>"   <- the runtime match key (tlsId)
    SignalGroupId       = -1              <- unused by the reader
    SignalHeadId        = 0               <- single-char state per name
    CmTrafficLightIndex = the Control.TrfLight.<i> array index in the rd5
    CmControllerId      = "<SCno>_<sg>_<linkIndex>"  (unique CM label, also the
                          name we write into Control.TrfLight so the rd5 shows
                          which VISSIM group drives each head)

The CM-head -> VISSIM-group join is the odrSignalId tag osc2cm left on every head
part ("<intersection>_<linkIndex>"), resolved to a signal group via signal_plan.json.

SC numbering matches build_signals.py / the .sig files: int_west=1, int_center=2,
int_east=3.

Run AFTER add_signal_stops.py (it reads/renames simple_traffic_light_signalstop.rd5).
Idempotent: the join is keyed on the stable odrSignalId tag, never on the current
controller name, so re-running after add_signal_stops re-derives cleanly.
"""
from __future__ import annotations
import argparse, csv, json, pathlib, re, sys

HERE = pathlib.Path(__file__).resolve().parent
CM_ROAD = pathlib.Path(
    r"C:/src_git/RS_FIXS/172_cm_signal_demo/ProprietaryFiles/CM13_proj/Data/Road"
)
RD5 = CM_ROAD / "simple_traffic_light_signalstop.rd5"
PLAN = HERE / "signal_plan.json"

SC_NO = {"int_west": 1, "int_center": 2, "int_east": 3}


def load_link2sg(plan_path):
    """(intersection, linkIndex) -> signal group id, from signal_plan.json."""
    plan = json.loads(plan_path.read_text())
    out = {}
    for inter, d in plan.items():
        for g in d["groups"]:
            for m in g["members"]:
                out[(inter, m["linkIndex"])] = g["sg"]
    return out


def parse_rd5(text):
    """Return (ctrl_by_idx, objid2tag).

    ctrl_by_idx : cmIdx -> (objId, current_name)  from Control.TrfLight.<i> lines
    objid2tag   : ctrlObjId -> 'int_center_0'     from head parts + their .Tag
    """
    ctrl_by_idx = {}
    for m in re.finditer(r"(?m)^Control\.TrfLight\.(\d+)\s*=\s*(\d+)\s+(\S+)", text):
        ctrl_by_idx[int(m.group(1))] = (int(m.group(2)), m.group(3))

    # head part line:  RL.<rl>.Mount.<m>.<p> = 1 <ctrlObjId> ...
    # its tag line:    RL.<rl>.Mount.<m>.<p>.Tag = odrSignalId:<inter>_<link>
    part2obj = {}
    for m in re.finditer(r"(?m)^(RL\.\d+\.Mount\.\d+\.\d+)\s*=\s*\d+\s+(\d+)\b", text):
        part2obj[m.group(1)] = int(m.group(2))
    objid2tag = {}
    for m in re.finditer(
        r"(?m)^(RL\.\d+\.Mount\.\d+\.\d+)\.Tag\s*=\s*odrSignalId:(\S+)", text
    ):
        obj = part2obj.get(m.group(1))
        if obj is not None:
            objid2tag[obj] = m.group(2)
    return ctrl_by_idx, objid2tag


def split_tag(tag):
    """'int_center_0' -> ('int_center', 0).  Intersection name may contain '_'."""
    inter, _, link = tag.rpartition("_")
    return inter, int(link)


def build(rd5_path, plan_path, apply_rename):
    text = rd5_path.read_text()
    ctrl_by_idx, objid2tag = parse_rd5(text)
    link2sg = load_link2sg(plan_path)

    rows = []           # (vissimKey, -1, 0, cmIdx, cmName)
    renames = []        # (old_name, new_name) for the rd5 rewrite
    problems = []
    for cmIdx in sorted(ctrl_by_idx):
        objId, cur_name = ctrl_by_idx[cmIdx]
        tag = objid2tag.get(objId)
        if tag is None:
            problems.append(f"cmIdx {cmIdx} (obj {objId}, {cur_name}): no head/odrSignalId references it")
            continue
        inter, link = split_tag(tag)
        sc = SC_NO.get(inter)
        sg = link2sg.get((inter, link))
        if sc is None or sg is None:
            problems.append(f"cmIdx {cmIdx}: tag {tag} -> sc={sc} sg={sg} (unmapped)")
            continue
        vissim_key = f"{sc}_{sg}"
        cm_name = f"{sc}_{sg}_{link}"
        rows.append((vissim_key, -1, 0, cmIdx, cm_name))
        if cur_name != cm_name:
            renames.append((cmIdx, objId, cur_name, cm_name))

    if problems:
        print("WARNING: unmapped controllers:")
        for p in problems:
            print("  " + p)

    # rewrite the Control.TrfLight name fields in place
    if apply_rename and renames:
        for cmIdx, objId, old, new in renames:
            pat = re.compile(rf"(?m)^(Control\.TrfLight\.{cmIdx}\s*=\s*{objId}\s+)\S+")
            text, n = pat.subn(rf"\g<1>{new}", text)
            if n != 1:
                raise RuntimeError(f"rename failed for cmIdx {cmIdx} ({old}->{new}), matched {n}")
        rd5_path.write_text(text)

    return rows, renames, problems


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rd5", type=pathlib.Path, default=RD5)
    ap.add_argument("--plan", type=pathlib.Path, default=PLAN)
    ap.add_argument("--no-rename", action="store_true",
                    help="dry run: derive + print the table, do not touch the rd5")
    args = ap.parse_args()

    rows, renames, problems = build(args.rd5, args.plan, apply_rename=not args.no_rename)

    out_csv = args.rd5.with_name(args.rd5.stem + "_RSsignalTable.csv")
    if not args.no_rename:
        with out_csv.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["SignalControllerId", "SignalGroupId", "SignalHeadId",
                        "CmTrafficLightIndex", "CmControllerId"])
            w.writerows(rows)

    # summary: how many CM heads each VISSIM group drives
    by_key = {}
    for vissim_key, _, _, cmIdx, _ in rows:
        by_key.setdefault(vissim_key, []).append(cmIdx)
    print(f"\n{len(rows)} CM traffic-light(s) mapped across {len(by_key)} VISSIM signal group(s):")
    for k in sorted(by_key, key=lambda s: tuple(int(x) for x in s.split("_"))):
        print(f"  {k:>5}  drives {len(by_key[k]):>2} CM head(s)  -> CmTrafficLightIndex {sorted(by_key[k])}")
    if args.no_rename:
        print("\n[dry run] no files written.")
    else:
        print(f"\nrenamed {len(renames)} controller(s); wrote {out_csv.name}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
