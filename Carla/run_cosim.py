"""
run_cosim.py - cross-platform SUMO <-> CARLA co-simulation launcher.

Resolves the CARLA server from CARLA_ROOT (OS-aware), launches it, waits for the
RPC port, loads the target map, and runs the SUMO <-> CARLA synchronization.
Works on Windows and Linux. The same helpers back the gated smoke test in
verify_demo.py.

Examples:
  # let it launch CARLA (CARLA_ROOT points at a CARLA install):
  CARLA_ROOT=/opt/carla-0.9.15 python run_cosim.py \
      --sumocfg fixtures/grid_tls.sumocfg --map Town01

  # CARLA already running (e.g. an editor in Play, or a RoadRunner map):
  python run_cosim.py --no-launch --no-net-offset \
      --map RP_Ver0529 --sumocfg <cfg> --tl-table <csv> --sumo-gui
"""
import argparse
import os
import platform
import signal
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SYNC = os.path.join(HERE, "sumo", "run_synchronization", "run_synchronization.py")


def resolve_carla_exe(carla_root):
    """Return the CARLA server executable for this OS, or None if not found."""
    if platform.system() == "Windows":
        cands = [os.path.join(carla_root, "CarlaUE4.exe"),
                 os.path.join(carla_root, "WindowsNoEditor", "CarlaUE4.exe")]
    else:
        cands = [os.path.join(carla_root, "CarlaUE4.sh"),
                 os.path.join(carla_root, "LinuxNoEditor", "CarlaUE4.sh")]
    return next((c for c in cands if os.path.isfile(c)), None)


def wait_for_port(host, port, timeout=180):
    deadline = time.time() + timeout
    while time.time() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)
            if s.connect_ex((host, port)) == 0:
                return True
        time.sleep(2)
    return False


def launch_carla(carla_root, port=2000, render_offscreen=False):
    exe = resolve_carla_exe(carla_root)
    if exe is None:
        raise FileNotFoundError(
            f"No CARLA launcher under CARLA_ROOT={carla_root} (CarlaUE4.exe / CarlaUE4.sh)")
    cmd = [exe, f"-carla-rpc-port={port}"]
    if render_offscreen:
        cmd.append("-RenderOffScreen")  # headless, no display (Linux/CI)
    print(f"[CARLA] launching: {' '.join(cmd)}")
    if platform.system() == "Windows":
        return subprocess.Popen(cmd)
    return subprocess.Popen(cmd, preexec_fn=os.setsid)


def kill_carla(proc):
    try:
        if platform.system() == "Windows":
            proc.terminate()
        else:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        proc.kill()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sumocfg", required=True, help="SUMO .sumocfg")
    ap.add_argument("--map", default="Town01", help="CARLA map to load_world()")
    ap.add_argument("--tl-table", default=None, help="traffic_light_table.csv (for --tls-manager sumo)")
    ap.add_argument("--tls-manager", default="sumo", choices=["sumo", "carla", "none"])
    ap.add_argument("--step-length", type=float, default=0.05)
    ap.add_argument("--no-net-offset", action="store_true",
                    help="zero the SUMO net offset (RoadRunner-local maps)")
    ap.add_argument("--carla-host", default="localhost")
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--no-launch", action="store_true", help="CARLA is already running")
    ap.add_argument("--render-offscreen", action="store_true", help="headless CARLA")
    ap.add_argument("--sumo-gui", action="store_true")
    args = ap.parse_args()

    carla_proc = None
    try:
        if not args.no_launch:
            carla_root = os.environ.get("CARLA_ROOT")
            if not carla_root:
                sys.exit("CARLA_ROOT not set. Set it to your CARLA install, or start CARLA "
                         "yourself and pass --no-launch.")
            carla_proc = launch_carla(carla_root, args.carla_port, args.render_offscreen)
            if not wait_for_port(args.carla_host, args.carla_port):
                sys.exit("CARLA RPC port did not open in time.")

        import carla
        client = carla.Client(args.carla_host, args.carla_port)
        client.set_timeout(60.0)
        print(f"[CARLA] loading world: {args.map}")
        client.load_world(args.map)

        cmd = [sys.executable, SYNC, args.sumocfg,
               "--tls-manager", args.tls_manager,
               "--step-length", str(args.step_length),
               "--carla-host", args.carla_host, "--carla-port", str(args.carla_port)]
        if args.tl_table:
            cmd += ["--tl-table", args.tl_table]
        if args.no_net_offset:
            cmd.append("--no-net-offset")
        if args.sumo_gui:
            cmd.append("--sumo-gui")
        print(f"[SYNC] {' '.join(cmd)}")
        return subprocess.call(cmd)
    finally:
        if carla_proc is not None:
            print("[CARLA] terminating server")
            kill_carla(carla_proc)


if __name__ == "__main__":
    sys.exit(main())
