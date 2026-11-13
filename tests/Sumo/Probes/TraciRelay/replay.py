"""#356 probe, phase C: the shim, minus the socket.

Binds SUMO's own traci domains to a connection object whose _sendCmd does not
talk to SUMO at all - it returns the bytes the C++ probe got back from
libtraci::Connection::doCommand. If the stock traci parsers turn those bytes into
the same values phase A got from a real connection, the relay design holds on the
Python side.

usage: replay.py <dir with capture.json and responses.txt>
"""
import json
import os
import sys

import traci
from traci.connection import Connection
from traci.domain import DOMAINS
from traci.storage import Storage
from traci.exceptions import TraCIException

import probe_calls

D = sys.argv[1]
cap = json.load(open(os.path.join(D, "capture.json")))
resp = []
for line in open(os.path.join(D, "responses.txt")):
    line = line.rstrip("\n")
    if line:
        name, status, body = line.split("|", 2)
        resp.append((name, status, body))

after_cpp = None
if resp and resp[-1][0] == "__after__":
    after_cpp = resp.pop()[2]
assert len(resp) == len(cap["records"]), "%d responses for %d requests" % (len(resp), len(cap["records"]))

mismatch = []


class RelayConnection(object):
    """What fixs.sumo.traci would install in place of traci's socket connection."""

    _pack = Connection._pack  # reused verbatim: the shim serializes nothing itself

    def __init__(self):
        self.i = 0

    def _sendCmd(self, cmdID, varID, objID, format="", *values):
        payload = self._pack(format, *values)
        rec = cap["records"][self.i]
        name, status, body = resp[self.i]
        self.i += 1
        # the shim must produce byte-identical requests to what traci itself sent
        got = (cmdID, varID, str(objID), payload.hex())
        want = (rec["cmdID"], rec["varID"], rec["objID"], rec["payload"])
        if got != want:
            mismatch.append(("request", name, got, want))
        if status == "EXC":
            raise TraCIException(body, cmdID, None)
        return Storage(bytes.fromhex(body))


relay = RelayConnection()
for d in DOMAINS:
    d._setConnection(relay)

ok = True
for name, fn in probe_calls.calls():
    want = cap["results"][name]
    try:
        got = {"value": repr(fn()), "error": None}
    except TraCIException as e:
        got = {"value": None, "error": str(e)}
    same = got["value"] == want["value"] and (got["error"] is None) == (want["error"] is None)
    ok &= same
    print("%-20s %-6s replayed=%s  live=%s" % (name, "OK" if same else "DIFF",
                                               got["value"] or "raise: " + str(got["error"]),
                                               want["value"] or "raise: " + str(want["error"])))

for m in mismatch:
    ok = False
    print("REQUEST MISMATCH", m)

# independent check: the bytes C++ returned vs the bytes traci saw on the wire
for rec, (name, status, body) in zip(cap["records"], resp):
    if status == "OK":
        if rec["py_tail"] != body:
            ok = False
            print("TAIL MISMATCH %s\n  py  %s\n  cpp %s" % (name, rec["py_tail"], body))
        else:
            print("%-20s tail identical (%d bytes)" % (name, len(body) // 2))

# the relayed setters must have moved SUMO the same way the real traci calls did
if after_cpp is not None:
    after_py = "%.9g,%d" % (cap["after"]["speed"], cap["after"]["lane"])
    same = after_cpp == after_py
    ok &= same
    print("\nstate one step after the setters: relayed=%s  live=%s  %s"
          % (after_cpp, after_py, "OK" if same else "DIFF"))

print("\nPHASE C:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
