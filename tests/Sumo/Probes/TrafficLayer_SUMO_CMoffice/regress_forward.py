#!/usr/bin/env python3
# =============================================================================
# FIXS #177 -- behavior-preserving regression harness for the SUMO forward path.
#
# Drives an EXISTING echo-client test (SimpleEchoClient / SimpleTrafficLight)
# fully headless + deterministic, and captures EVERY forwarded VehData field per
# (simTime, vehId) into a normalized golden file. The A/B/C/D optimizations to
# CommonLib/TrafficHelper.cpp must leave this capture BYTE-IDENTICAL.
#
# Pipeline (no CarMaker):
#   sumo (headless, fixed seed) --TraCI--> TrafficLayer.exe (the C++ under test)
#   TrafficLayer --FIXS socket--> THIS script (acts as the app client, records
#                                 every field via CommonLib MsgHelper, echoes back)
#
# Determinism: SUMO seeded; the co-sim is lockstep (TL waits for our echo each
# step), so per-step content is timing-independent. Run to a fixed sim end.
#
# Run with the realsim_dev python (needs CommonLib + yaml):
#   python regress_forward.py --test-dir <dir> --out <file.txt> --end 60
# =============================================================================
import os, sys, socket, subprocess, time, argparse, signal, copy
import pathlib

REPO = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO))
import yaml
from CommonLib.SocketHelper import SocketHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.ConfigHelper import ConfigHelper

TL_EXE = REPO / "TrafficLayer" / "x64" / "Release" / "TrafficLayer.exe"
SUMO = pathlib.Path(os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo")) / "bin" / "sumo.exe"


def derive_config(test_dir, end):
    """Copy the test's config.yaml, force headless + external SUMO + fixed end.
    Returns (derived_path, traci_port, client_ip, client_port, fields)."""
    src = test_dir / "config.yaml"
    with open(src) as f:
        cfg = yaml.safe_load(f)
    ss = cfg.setdefault("SimulationSetup", {})
    ss["EnableVerboseLog"] = False
    ss["SimulationEndTime"] = end
    # Only force auto-launch off when a SumoSetup exists (SimpleEchoClient sets it
    # true). When absent (SimpleTrafficLight), the default is connect-to-external.
    if "SumoSetup" in cfg:
        cfg["SumoSetup"]["EnableAutoLaunch"] = False
    traci_port = ss.get("TrafficSimulatorPort", 1337)
    sub = cfg["ApplicationSetup"]["VehicleSubscription"][0]
    client_ip, client_port = sub["ip"][0], sub["port"][0]
    fields = ss.get("VehicleMessageField", ["id", "speed"])
    derived = test_dir / "_regress_config.yaml"
    with open(derived, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return derived, traci_port, client_ip, client_port, fields


def fmt(v):
    if isinstance(v, float):
        return f"{v:.6f}"
    if isinstance(v, bytes):
        return v.decode(errors="replace").strip()
    return str(v).strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--test-dir", required=True)
    ap.add_argument("--sumocfg", required=True, help="sumocfg filename relative to test-dir")
    ap.add_argument("--out", required=True)
    ap.add_argument("--end", type=float, default=60.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--step", type=float, default=0.1)
    a = ap.parse_args()
    test_dir = pathlib.Path(a.test_dir).resolve()
    sumocfg = (test_dir / a.sumocfg).resolve()

    derived, traci_port, client_ip, client_port, fields = derive_config(test_dir, a.end)
    print(f"[regress] test={test_dir.name} fields={fields}", file=sys.stderr)
    print(f"[regress] sumo={sumocfg.name} traci:{traci_port} client:{client_ip}:{client_port}", file=sys.stderr)

    sumo_proc = tl_proc = None
    records = []
    try:
        # 1) SUMO headless, external TraCI server, fixed seed/end
        sumo_proc = subprocess.Popen(
            [str(SUMO), "-c", str(sumocfg), "--remote-port", str(traci_port),
             "--step-length", str(a.step), "--seed", str(a.seed),
             "--end", str(a.end), "--no-step-log", "true", "--no-warnings", "true",
             "--quit-on-end", "true"],
            cwd=str(test_dir))
        time.sleep(2.0)  # let SUMO open the TraCI port

        # 2) TrafficLayer (connects to SUMO, serves us on client_port).
        # libtracicpp.dll lives in CommonLib/libsumo/bin -- put it on PATH.
        tl_env = dict(os.environ)
        libdir = str(REPO / "CommonLib" / "libsumo" / "bin")
        sumobin = str(SUMO.parent)
        tl_env["PATH"] = libdir + os.pathsep + sumobin + os.pathsep + tl_env.get("PATH", "")
        tl_proc = subprocess.Popen([str(TL_EXE), "-f", str(derived)], cwd=str(test_dir), env=tl_env)
        time.sleep(3.0)  # let TL connect to SUMO and open the client port

        # 3) act as the FIXS application client: load config via CommonLib
        ch = ConfigHelper(); ch.getConfig(str(derived))
        mh = MsgHelper(); mh.set_vehicle_message_field(fields)
        sh = SocketHelper(ch, mh)
        cs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        for _ in range(30):
            try:
                cs.connect((client_ip, client_port)); break
            except OSError:
                time.sleep(0.5)
        else:
            raise RuntimeError("could not connect to TrafficLayer client port")

        steps = 0
        while True:
            sh.clear_data()
            sim_state, sim_time = sh.recv_data(cs)
            if sim_state == 0:
                break
            for veh in sh.vehicle_data_receive_list:
                rec = " ".join(f"{fl}={fmt(getattr(veh, fl, ''))}" for fl in fields)
                records.append((round(float(sim_time), 3), getattr(veh, "id", b"").decode(errors="replace").strip()
                                if isinstance(getattr(veh, "id", ""), bytes) else str(getattr(veh, "id", "")).strip(),
                                rec))
                sh.vehicle_data_send_list.append(veh)     # echo back (lockstep)
            sh.sendData(sim_state, sim_time, cs)
            steps += 1
        cs.close()
        print(f"[regress] captured {steps} steps, {len(records)} veh-records", file=sys.stderr)
    finally:
        for p in (tl_proc, sumo_proc):
            if p and p.poll() is None:
                try: p.terminate(); p.wait(timeout=5)
                except Exception:
                    try: p.kill()
                    except Exception: pass

    # normalized golden: sorted by (time, vehId), fixed precision
    records.sort(key=lambda r: (r[0], r[1]))
    with open(a.out, "w") as f:
        for t, vid, rec in records:
            f.write(f"t={t:.3f} {rec}\n")
    print(f"[regress] wrote {a.out} ({len(records)} lines)", file=sys.stderr)


if __name__ == "__main__":
    main()
