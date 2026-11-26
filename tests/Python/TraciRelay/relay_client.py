"""A TraCI script, run through FIXS by `import fixs.sumo.traci as traci`. Exits non-zero on any failure."""
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path[:0] = [ROOT, os.path.join(ROOT, "CommonLib")]
os.environ.setdefault("FIXS_CONFIG_YAML", os.path.join(HERE, "config.yaml"))

import fixs                     # noqa: E402  (only to read the feed for the cross-check)
import fixs.sumo.traci as traci  # noqa: E402

failures = []


def check(name, ok, detail=""):
    print("  %-34s %s %s" % (name, "OK  " if ok else "FAIL", detail))
    if not ok:
        failures.append(name)


def raises(name, exc, fn, needle=""):
    try:
        fn()
    except exc as e:
        check(name, needle in str(e), "%s: %s" % (type(e).__name__, str(e)[:60]))
        return
    except Exception as e:      # noqa: BLE001
        check(name, False, "raised %s" % type(e).__name__)
        return
    check(name, False, "did not raise")


def f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]


traci.start(["sumo", "-c", os.path.join(ROOT, "tests", "Sumo", "Probes", "TraciRelay", "net", "probe.sumocfg")])
seen = 0
while seen < 32:
    traci.simulationStep()
    if "ego" not in traci.vehicle.getIDList():
        continue
    seen += 1
    if seen == 30:
        t = traci.simulation.getTime()
        print("[relay-client] t=%.1f" % t)
        relayed = traci.vehicle.getSpeed("ego")
        feed = fixs.vehicle.get("ego").speed
        check("getSpeed == feed (float32)", f32(relayed) == feed, "relayed %r, feed %r" % (relayed, feed))
        check("getTime == feed time (float32)", f32(t) == fixs.sim.time, "relayed %r, feed %r" % (t, fixs.sim.time))
        check("getVersion", traci.getVersion()[0] > 0, str(traci.getVersion()))
        check("lane.getIDList", len(traci.lane.getIDList()) >= 4)
        check("vehicle.getRoute", tuple(traci.vehicle.getRoute("ego")) == ("e0", "e1"))
        traci.vehicle.setParameter("ego", "fixs356", "hello")
        check("set/getParameter", traci.vehicle.getParameter("ego", "fixs356") == "hello")
        big = "x" * 20000
        traci.vehicle.setParameter("ego", "big", big)
        check("20 KB both ways", traci.vehicle.getParameter("ego", "big") == big)
        raises("unknown vehicle", traci.TraCIException, lambda: traci.vehicle.getSpeed("nosuchveh"), "nosuchveh")
        raises("subscribe refused", traci.FatalTraCIError, lambda: traci.vehicle.subscribe("ego", [0x40]), "fixs.vehicle")
        raises("load refused", traci.FatalTraCIError, lambda: traci.load(["-c", "x"]), "reload")
    if seen == 31:
        check("parameter survives a step", traci.vehicle.getParameter("ego", "fixs356") == "hello")
traci.close()

print("[relay-client] %s" % ("PASS" if not failures else "FAIL: " + ", ".join(failures)))
sys.exit(1 if failures else 0)
