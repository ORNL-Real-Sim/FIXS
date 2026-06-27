"""
run_cosim.py - cross-platform SUMO <-> CARLA co-simulation launcher.

Reads the per-machine CARLA env saved by carla_env_setup.py (~/.fixs/carla.json),
launches that CARLA (packaged build or source editor, OS-aware), waits for the RPC
port, loads the target map, and runs the SUMO <-> CARLA synchronization. Works on
Windows and Linux.

The very first time this runs on a fresh clone there is no saved config, so it
auto-invokes carla_env_setup.run_setup() to ask which CARLA to use and remember
it; every run afterwards is seamless. To switch CARLA later, run carla_env_setup.py
(or setup_carla.bat / setup_carla.sh), or pass --reconfigure here.

Examples:
  # first run prompts for CARLA, then launches and runs; later runs are seamless:
  python run_cosim.py --sumocfg fixtures/grid_tls.sumocfg --map Town01

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

import carla_env_setup as env

HERE = os.path.dirname(os.path.abspath(__file__))
SYNC = os.path.join(HERE, "sumo", "run_synchronization", "run_synchronization.py")


def resolve_carla_env(reconfigure=False):
    """Return the saved CARLA env, running the setup prompt if missing/forced."""
    cfg = None if reconfigure else env.load_config()
    if cfg is None:
        if reconfigure:
            print("[cosim] reconfiguring CARLA env ...")
        else:
            print(f"[cosim] no CARLA env configured ({env.CONFIG_PATH}); "
                  "running first-time setup ...")
        cfg = env.run_setup()
    return cfg


def wait_for_port(host, port, timeout=180):
    deadline = time.time() + timeout
    while time.time() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)
            if s.connect_ex((host, port)) == 0:
                return True
        time.sleep(2)
    return False


def _carla_command(cfg, port, render_offscreen):
    """Build the server launch command for a packaged build or source editor."""
    if cfg["mode"] == "packaged":
        exe = env.packaged_exe(cfg["carla_root"])
        if exe is None:
            raise FileNotFoundError(
                f"No CarlaUE4 launcher under {cfg['carla_root']}. "
                "Re-run carla_env_setup.py (or --reconfigure) to fix the path.")
        cmd = [exe, f"-carla-rpc-port={port}"]
    else:  # source build, launched through the Unreal editor in -game mode
        uproject, editor = env.source_paths(cfg["carla_root"], cfg["ue4_root"])
        if not os.path.isfile(editor) or not os.path.isfile(uproject):
            raise FileNotFoundError(
                f"Source CARLA not found (editor={editor}, uproject={uproject}). "
                "Re-run carla_env_setup.py (or --reconfigure) to fix the paths.")
        cmd = [editor, uproject, "-game", f"-carla-rpc-port={port}"]
    if render_offscreen:
        cmd.append("-RenderOffScreen")  # headless, no display (Linux/CI)
    return cmd


def launch_carla(cfg, port=2000, render_offscreen=False):
    cmd = _carla_command(cfg, port, render_offscreen)
    print(f"[CARLA] launching ({cfg['mode']}): {' '.join(cmd)}")
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
    ap.add_argument("--reconfigure", action="store_true",
                    help="re-run CARLA env setup before launching (pick a different CARLA)")
    ap.add_argument("--render-offscreen", action="store_true", help="headless CARLA")
    ap.add_argument("--sumo-gui", action="store_true")
    args = ap.parse_args()

    carla_proc = None
    try:
        if not args.no_launch:
            cfg = resolve_carla_env(reconfigure=args.reconfigure)
            carla_proc = launch_carla(cfg, args.carla_port, args.render_offscreen)
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
