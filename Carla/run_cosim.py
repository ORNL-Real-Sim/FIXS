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


def ensure_runtime(cfg):
    """Repair a config that lacks a usable python env (e.g. written by an older
    setup before env resolution existed): resolve the interpreter + matching
    carla and save it, WITHOUT re-asking the CARLA / UE4 paths."""
    py = cfg.get("python")
    if py and os.path.isfile(py) and env._python_can_import(py, ("carla",)):
        return cfg
    print("[cosim] saved config has no usable python env (carla not importable); "
          "resolving it now (CARLA paths kept) ...")
    cfg["python"] = env.resolve_python()
    wheel = env.ensure_carla(cfg["python"], cfg["mode"], cfg["carla_root"])
    if wheel:
        cfg["carla_wheel"] = wheel
    env.save_config(cfg)
    return cfg


def maybe_reexec(cfg):
    """Re-run this script under the configured python (the env that actually has
    the carla + SUMO clients) if we are not already running under it. Lets the
    .bat/.sh stay trivial and work on any machine, whatever the env is named."""
    target = cfg.get("python")
    if not target or not os.path.isfile(target):
        return
    same = os.path.normcase(os.path.normpath(target)) == \
        os.path.normcase(os.path.normpath(sys.executable))
    if same or os.environ.get("FIXS_REEXEC") == "1":
        return
    child_args = [a for a in sys.argv[1:] if a != "--reconfigure"]
    cmd = [target, os.path.abspath(__file__), *child_args]
    print(f"[cosim] switching to configured python env:\n        {target}")
    osenv = dict(os.environ, FIXS_REEXEC="1")
    sys.exit(subprocess.call(cmd, env=osenv))


def confirm_world_ready(client, expected_map, timeout):
    """After load_world, block until the loaded world IS the expected map AND it
    is producing frames (ticking). On a heavy source build, load_world can return
    while the map is still switching/streaming - this makes sure we don't start
    SUMO against the wrong or half-loaded world. Returns the map name, or None."""
    deadline = time.time() + timeout
    last = ""
    while time.time() < deadline:
        try:
            world = client.get_world()
            last = world.get_map().name
            if expected_map.lower() in last.lower():
                world.wait_for_tick(10.0)  # the server is actually producing frames
                return last
        except RuntimeError:
            pass  # server busy (e.g. compiling shaders) - retry
        time.sleep(1.0)
    return None


def wait_for_port(host, port, timeout=180):
    deadline = time.time() + timeout
    while time.time() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)
            if s.connect_ex((host, port)) == 0:
                return True
        time.sleep(2)
    return False


def _carla_command(cfg, port, render_offscreen, quality_level=None):
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
    if quality_level:
        cmd.append(f"-quality-level={quality_level}")  # Low is much cheaper to render
    if render_offscreen:
        cmd.append("-RenderOffScreen")  # headless, no display (Linux/CI)
    return cmd


def launch_carla(cfg, port=2000, render_offscreen=False, quality_level=None):
    cmd = _carla_command(cfg, port, render_offscreen, quality_level)
    print(f"[CARLA] launching ({cfg['mode']}): {' '.join(cmd)}")
    if platform.system() == "Windows":
        return subprocess.Popen(cmd)
    return subprocess.Popen(cmd, preexec_fn=os.setsid)


def _frame_from_actors(world):
    """Centroid + span (m) of the placed traffic-light actors, in CARLA coords."""
    tls = world.get_actors().filter("traffic.traffic_light*")
    locs = [t.get_transform().location for t in tls]
    if not locs:
        return None
    xs = [l.x for l in locs]; ys = [l.y for l in locs]; zs = [l.z for l in locs]
    return _frame_stats(xs, ys, zs, f"{len(locs)} traffic lights")


def _table_junctions(tl_table, no_net_offset):
    """junction_id -> list of (x, y, z) CARLA-coord points from the TL table.
    With --no-net-offset (RoadRunner-local maps) SUMO (x, y) -> CARLA (x, -y)."""
    import csv
    juncs = {}
    try:
        with open(tl_table, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                x = float(row["x"])
                y = -float(row["y"]) if no_net_offset else float(row["y"])
                z = float(row.get("z") or 0.0)
                juncs.setdefault(row["junction_id"], []).append((x, y, z))
    except (OSError, KeyError, ValueError):
        return {}
    return juncs


def _frame_from_table(tl_table, no_net_offset, junction=None, whole=False):
    """Frame from the TL table. By default zooms to ONE intersection (the busiest
    by signal-head count) so the signal sync is legible; whole=True frames the
    entire network; junction=<id> targets a specific intersection."""
    juncs = _table_junctions(tl_table, no_net_offset)
    if not juncs:
        return None
    if whole:
        pts = [p for pts in juncs.values() for p in pts]
        anchor = f"{len(juncs)} junctions (network)"
    elif junction is not None:
        if junction not in juncs:
            print(f"[VIEW] junction '{junction}' not in table; using the busiest one")
            junction = None
        pts = juncs.get(junction) or max(juncs.values(), key=len)
        anchor = f"junction {junction}" if junction else "busiest junction"
    else:
        jid = max(juncs, key=lambda j: len(juncs[j]))
        pts = juncs[jid]
        anchor = f"junction {jid} ({len(pts)} heads)"
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
    return _frame_stats(xs, ys, zs, anchor)


def _frame_stats(xs, ys, zs, anchor):
    cx, cy, cz = sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs)
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    return cx, cy, cz, span, anchor


def position_spectator(world, frame, pitch=-55.0):
    """Point the spectator at the anchor from an angled overhead view. Height is
    scaled to the anchor's span (so a single intersection is seen up close), and
    the camera is offset back along -X so the anchor sits in the view centre."""
    import math
    import carla
    cx, cy, cz, span, anchor = frame
    height = min(max(span * 1.2, 35.0), 600.0)
    back = height / math.tan(math.radians(-pitch))  # so the look ray hits (cx, cy)
    world.get_spectator().set_transform(carla.Transform(
        carla.Location(x=cx - back, y=cy, z=cz + height),
        carla.Rotation(pitch=pitch, yaw=0.0, roll=0.0)))
    print(f"[VIEW] spectator -> ({cx - back:.0f}, {cy:.0f}, {cz + height:.0f}) "
          f"looking at ({cx:.0f}, {cy:.0f}), span~{span:.0f}m, anchored on {anchor}")


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
    ap.add_argument("--auto-import", action="store_true",
                    help="source build: if the map isn't imported, cook it before launching")
    ap.add_argument("--map-package", default=None,
                    help="import package name if it differs from --map")
    ap.add_argument("--map-package-url", default=None,
                    help="URL of the map package zip for --auto-import (e.g. a release asset)")
    ap.add_argument("--map-config", default=None,
                    help="text file declaring the import package (package= and url= lines)")
    ap.add_argument("--reimport", action="store_true",
                    help="re-import the map even if already cooked (re-download + re-cook)")
    ap.add_argument("--tl-table", default=None, help="traffic_light_table.csv (for --tls-manager sumo)")
    ap.add_argument("--tls-manager", default="sumo", choices=["sumo", "carla", "none"])
    ap.add_argument("--step-length", type=float, default=0.05,
                    help="sim timestep in seconds / CARLA fixed_delta_seconds (default 0.05; "
                         "0.1 halves the ticks-per-second and is fine for traffic)")
    ap.add_argument("--quality-level", choices=["Low", "Medium", "High", "Epic"], default=None,
                    help="CARLA render quality; Low is much faster on heavy maps")
    ap.add_argument("--fast", action="store_true",
                    help="do not pace the co-sim to real time (run as fast as possible)")
    ap.add_argument("--no-net-offset", action="store_true",
                    help="zero the SUMO net offset (RoadRunner-local maps)")
    ap.add_argument("--carla-host", default="localhost")
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--carla-timeout", type=float, default=10.0,
                    help="CARLA client connect timeout in seconds (default: 10; "
                         "raise for heavy source-build maps)")
    ap.add_argument("--load-timeout", type=float, default=300.0,
                    help="client timeout (s) for load_world; the first load of a freshly "
                         "imported map compiles shaders and can take minutes (default 300)")
    ap.add_argument("--no-launch", action="store_true", help="CARLA is already running")
    ap.add_argument("--reconfigure", action="store_true",
                    help="re-run CARLA env setup before launching (pick a different CARLA)")
    ap.add_argument("--render-offscreen", action="store_true", help="headless CARLA")
    ap.add_argument("--no-spectator", action="store_true",
                    help="do not auto-frame the CARLA spectator on the scene")
    ap.add_argument("--spectator-all", action="store_true",
                    help="frame the whole network instead of one intersection")
    ap.add_argument("--spectator-junction", default=None,
                    help="frame this junction id (default: the busiest intersection)")
    ap.add_argument("--sumo-gui", action="store_true")
    args = ap.parse_args()

    # Resolve the saved CARLA env (running first-time setup if needed) and make
    # sure we are on its python before importing carla. --no-launch still needs
    # the env for the python/carla client, but won't force CARLA-path setup.
    cfg = env.load_config()
    if not args.no_launch and (cfg is None or args.reconfigure):
        cfg = resolve_carla_env(reconfigure=args.reconfigure)
    if cfg is not None:
        cfg = ensure_runtime(cfg)  # repair stale configs that lack a python env
        maybe_reexec(cfg)          # may not return (re-execs under the env python)
    elif args.no_launch:
        print("[cosim] no CARLA env configured; running under the current python. "
              "If 'import carla' fails, run setup_carla first.")

    # Source-build preflight: a custom map must be cooked into the build before
    # CARLA can load it. Detect a missing map on disk and either import it
    # (--auto-import) or fail with a clear instruction - far better than an
    # opaque load_world() error after launch.
    if not args.no_launch and cfg is not None and cfg.get("mode") == "source":
        import import_map
        pkg, url = args.map_package, args.map_package_url
        if args.map_config:
            mc = import_map.read_map_config(args.map_config)
            pkg = pkg or mc.get("package")
            url = url or mc.get("url")
        pkg = pkg or args.map
        imported = import_map.map_is_imported(cfg["carla_root"], pkg)
        if not imported or args.reimport:
            if args.auto_import or args.reimport:
                verb = "re-importing" if args.reimport else "importing"
                print(f"[cosim] {verb} map '{pkg}' before launch ...")
                import_map.ensure_map(pkg, carla_root=cfg["carla_root"],
                                      ue4_root=cfg.get("ue4_root"),
                                      package_url=url, force=args.reimport)
            else:
                sys.exit(
                    f"[cosim] map '{pkg}' is not imported into {cfg['carla_root']}.\n"
                    f"        Import it once (e.g. import_<app>_map.bat), or pass "
                    f"--auto-import [--map-package-url <release zip>].")

        # TL preflight: a map whose OpenDRIVE has no dynamic signals needs the
        # traffic lights placed from the table (else the TL sync has no actors).
        # Idempotent via a marker; re-done after a (re-)import that wiped it.
        if args.tl_table:
            import place_tls
            if (not place_tls.tls_placed(cfg["carla_root"], pkg)) or args.reimport:
                if args.auto_import or args.reimport:
                    print(f"[cosim] placing traffic lights for '{pkg}' before launch ...")
                    place_tls.place_tls(pkg, args.tl_table, carla_root=cfg["carla_root"],
                                        ue4_root=cfg.get("ue4_root"), force=args.reimport)
                else:
                    print(f"[cosim] note: traffic lights not placed for '{pkg}'. Run "
                          f"place_tls (or pass --auto-import) to add them, else no TL sync.")

    carla_proc = None
    try:
        if not args.no_launch:
            carla_proc = launch_carla(cfg, args.carla_port, args.render_offscreen,
                                      args.quality_level)
            if not wait_for_port(args.carla_host, args.carla_port):
                sys.exit("CARLA RPC port did not open in time.")

        import carla
        client = carla.Client(args.carla_host, args.carla_port)
        client.set_timeout(args.load_timeout)
        print(f"[CARLA] loading world: {args.map}")
        print("[CARLA] (the first load of a freshly imported map compiles shaders - "
              "this can take a few minutes; later loads are fast)")
        client.load_world(args.map)
        # Don't trust load_world's return alone on a heavy source map: confirm the
        # world IS this map and is ticking before we start SUMO.
        print(f"[CARLA] confirming '{args.map}' is loaded and ready ...")
        loaded = confirm_world_ready(client, args.map, args.load_timeout)
        if loaded is None:
            current = ""
            try:
                current = client.get_world().get_map().name
            except Exception:
                pass
            sys.exit(f"[cosim] map '{args.map}' not confirmed loaded "
                     f"(current world: '{current}'); aborting before SUMO.")
        print(f"[CARLA] map ready: {loaded}")
        world = client.get_world()

        if not args.no_spectator:
            # Default: zoom to one intersection from the TL table so the signal
            # sync is legible. --spectator-all frames the whole network (placed TL
            # actors, else the full table).
            frame = None
            if args.tl_table and not args.spectator_all:
                frame = _frame_from_table(args.tl_table, args.no_net_offset,
                                          junction=args.spectator_junction)
            if frame is None:
                frame = _frame_from_actors(world)
            if frame is None and args.tl_table:
                frame = _frame_from_table(args.tl_table, args.no_net_offset, whole=True)
            if frame is not None:
                position_spectator(world, frame)
            else:
                print("[VIEW] no TL actors/table to anchor on; leaving spectator as is")

        cmd = [sys.executable, SYNC, args.sumocfg,
               "--tls-manager", args.tls_manager,
               "--step-length", str(args.step_length),
               "--carla-host", args.carla_host, "--carla-port", str(args.carla_port),
               "--carla-timeout", str(args.carla_timeout)]
        if args.tl_table:
            cmd += ["--tl-table", args.tl_table]
        if args.no_net_offset:
            cmd.append("--no-net-offset")
        if args.fast:
            cmd.append("--no-realtime")
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
