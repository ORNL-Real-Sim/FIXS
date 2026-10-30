"""#356 probe, phase A: tap SUMO's own traci at Connection._sendCmd.

Records, for a set of representative calls, the four values the seam produces
(cmdID, varID, objID, packed payload) plus the exact response bytes traci sees
after the status message. Writes capture.json (ground truth) and requests.txt
(input for the C++ probe).

usage: capture.py <port> <steps> <outdir>
"""
import json
import os
import sys

import traci
from traci.connection import Connection

import probe_calls

PORT, STEPS, OUTDIR = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]

records = []
shapes = []
label = [None]
shape = [None]
_orig = Connection._sendCmd


def _patched(self, cmdID, varID, objID, format="", *values):
    rec = None
    if label[0] is not None:
        rec = {"name": label[0], "cmdID": cmdID, "varID": varID, "objID": str(objID),
               "format": format, "values": [repr(v) for v in values],
               "payload": self._pack(format, *values).hex(), "py_tail": None}
        records.append(rec)
    if shape[0] is not None:
        shapes.append({"name": shape[0], "cmdID": cmdID, "varID": repr(varID), "objID": str(objID),
                       "format": format, "payload": self._pack(format, *values).hex()})
    r = _orig(self, cmdID, varID, objID, format, *values)
    if rec is not None and r is not None:
        rec["py_tail"] = bytes(r._content[r._pos:]).hex()
    return r


Connection._sendCmd = _patched

traci.init(PORT)
for _ in range(STEPS):
    traci.simulationStep()

CALLS = probe_calls.calls()

results = {}
for name, fn in CALLS:
    label[0] = name
    try:
        results[name] = {"value": repr(fn()), "error": None}
    except traci.TraCIException as e:
        results[name] = {"value": None, "error": str(e)}
    finally:
        label[0] = None
    print("%-20s %s" % (name, results[name]))

# did the relayed setters actually land? compare the state one step later
traci.simulationStep()
after = {"speed": traci.vehicle.getSpeed("ego"), "lane": traci.vehicle.getLaneIndex("ego")}
print("after one more step: %s" % after)

# what other shapes reach the same seam? these cannot be expressed as
# (cmdID, varID, objID, payload) and so must be branched or refused by the shim
import traci.constants as tc  # noqa: E402
for nm, fn in [("simulationStep", lambda: traci.simulationStep()),
               ("subscribe", lambda: traci.vehicle.subscribe("ego", [tc.VAR_SPEED, tc.VAR_POSITION])),
               ("subscribeContext", lambda: traci.vehicle.subscribeContext(
                   "ego", tc.CMD_GET_VEHICLE_VARIABLE, 50.0, [tc.VAR_SPEED]))]:
    shape[0] = nm
    fn()
    shape[0] = None
for sh in shapes:
    print("shape %-16s cmdID=0x%02x varID=%s objID=%r payload=%s"
          % (sh["name"], sh["cmdID"], sh["varID"], sh["objID"], sh["payload"]))

traci.close()

os.makedirs(OUTDIR, exist_ok=True)
with open(os.path.join(OUTDIR, "capture.json"), "w") as fp:
    json.dump({"steps": STEPS, "records": records, "results": results, "after": after, "shapes": shapes}, fp, indent=2)
with open(os.path.join(OUTDIR, "requests.txt"), "w") as fp:
    for r in records:
        fp.write("%s|%d|%d|%s|%s\n" % (r["name"], r["cmdID"], r["varID"], r["objID"], r["payload"]))
print("captured %d _sendCmd calls -> %s" % (len(records), OUTDIR))
