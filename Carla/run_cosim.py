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
import json
import os
import platform
import signal
import socket
import subprocess
import sys
import time
import urllib.request

import carla_env_setup as env

HERE = os.path.dirname(os.path.abspath(__file__))
SYNC = os.path.join(HERE, "sumo", "run_synchronization", "run_synchronization.py")
FIXS_ROOT = os.path.dirname(HERE)          # FIXS/Carla -> FIXS (the fetched bundle root)
APP_ROOT = os.path.dirname(FIXS_ROOT)      # FIXS -> the app dir (holds initialize.{sh,ps1})


# --------------------------------------------------------------------------- #
# FIXS bundle freshness. FIXS/ is a gitignored, fetched artifact; the rolling
# v0.9.0-alpha release republishes in place, so its TAG NAME never changes -
# compare the BUILD COMMIT (BUILD_INFO.txt) against the tag's current commit.
# Best-effort: any hiccup (offline, rate-limited, parse) is swallowed; a stale
# bundle only ever prompts, never blocks.
# --------------------------------------------------------------------------- #
def _first_group(path, pattern):
    import re
    try:
        with open(path, encoding="utf-8", errors="ignore") as f:
            m = re.search(pattern, f.read())
        return m.group(1) if m else None
    except OSError:
        return None


def _local_fixs_commit():
    return (_first_group(os.path.join(FIXS_ROOT, "BUILD_INFO.txt"),
                         r"Git Commit:\s*([0-9a-fA-F]{7,40})") or "").lower() or None


def _fixs_tag_repo():
    """(tag, repo) from FIXS_VERSION.txt: line 1 is the tag; a 'Source:' line the repo."""
    tag, repo = None, "ORNL-Real-Sim/FIXS"
    try:
        with open(os.path.join(FIXS_ROOT, "FIXS_VERSION.txt"), encoding="utf-8") as f:
            lines = [ln.strip() for ln in f if ln.strip()]
        if lines:
            tag = lines[0]
        for ln in lines:
            if ln.lower().startswith("source:"):
                repo = ln.split(":", 1)[1].strip() or repo
    except OSError:
        pass
    return tag, repo


def _remote_fixs_commit(repo, tag, timeout=4.0):
    """The commit the remote <tag> release points at now (target_commitish), or None
    if it cannot be told (offline, or the field is a branch name, not a sha)."""
    url = "https://api.github.com/repos/%s/releases/tags/%s" % (repo, tag)
    req = urllib.request.Request(url, headers={"User-Agent": "fixs-fetch",
                                               "Accept": "application/vnd.github+json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        sha = (json.load(r).get("target_commitish") or "").lower()
    is_sha = len(sha) >= 7 and all(c in "0123456789abcdef" for c in sha)
    return sha if is_sha else None


def _run_initialize(tag):
    """Re-fetch the FIXS bundle for <tag> via the app's initialize script (the tag is
    passed as its argument, so it runs non-interactively). True on success."""
    if platform.system() == "Windows":
        script = os.path.join(APP_ROOT, "initialize.ps1")
        cmd = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", script, tag]
    else:
        script = os.path.join(APP_ROOT, "initialize.sh")
        cmd = ["bash", script, tag]
    if not os.path.isfile(script):
        print(f"[cosim] cannot self-update: {script} not found.")
        return False
    print(f"[cosim] updating FIXS -> {tag} via {os.path.basename(script)} ...")
    return subprocess.call(cmd) == 0


def maybe_update_fixs(no_check=False):
    """Detect a local FIXS bundle that diverges from the published rolling release
    and, when interactive, offer to update + relaunch. Advisory only: every failure
    is swallowed so a run is never blocked by the check."""
    if no_check or os.environ.get("FIXS_NO_FRESHNESS"):
        return
    try:
        local = _local_fixs_commit()
        tag, repo = _fixs_tag_repo()
        if not local or not tag:
            return
        remote = _remote_fixs_commit(repo, tag)
        if not remote:
            return
        n = min(len(local), len(remote))
        if local[:n] == remote[:n]:
            return  # up to date
        print(f"[cosim] local FIXS bundle {local[:8]} diverges from the published "
              f"{tag} ({remote[:8]}).")
        if not sys.stdin.isatty():
            print("[cosim] non-interactive; run initialize to update. Continuing.")
            return
        ans = input("[cosim] update the local FIXS bundle now? [y/N]: ").strip().lower()
        if ans not in ("y", "yes"):
            print("[cosim] keeping the current bundle.")
            return
        if _run_initialize(tag):
            print("[cosim] FIXS updated; relaunching run_cosim ...")
            os.environ["FIXS_NO_FRESHNESS"] = "1"   # the relaunch is already current
            os.execv(sys.executable,
                     [sys.executable, os.path.join(HERE, "run_cosim.py")] + sys.argv[1:])
        print("[cosim] update did not complete; continuing with the current bundle.")
    except Exception as e:
        if os.environ.get("FIXS_DEBUG"):
            print(f"[cosim] freshness check skipped: {e}")


# --------------------------------------------------------------------------- #
# Co-sim engine selection. 'py' (default) = the standalone run_synchronization.py
# bridge; 'cpp' = the FIXS-native stack (SUMO + TrafficLayer + VirCarlaEnv), driven
# by a scenario yaml. The bridge is chosen by CarlaSetup.Backend in that yaml, read
# through the shipped CommonLib/ConfigHelper.py; --engine overrides per run.
# --------------------------------------------------------------------------- #
def _native_binaries():
    exe = ".exe" if platform.system() == "Windows" else ""
    return (os.path.join(FIXS_ROOT, "TrafficLayer" + exe),
            os.path.join(FIXS_ROOT, "VirCarlaEnv" + exe))


def read_backend(config_yaml):
    """CarlaSetup.Backend from `config_yaml` via CommonLib/ConfigHelper.py (the
    canonical parser the C++ engine mirrors). 'py' on absence or any parse problem."""
    if not config_yaml or not os.path.isfile(config_yaml):
        return "py"
    try:
        sys.path.insert(0, os.path.join(FIXS_ROOT, "CommonLib"))
        import ConfigHelper
        ch = ConfigHelper.ConfigHelper()
        ch.getConfig(config_yaml)
        return str(ch.Carla_setup["Backend"] or "py").strip().lower()
    except Exception as e:
        print(f"[cosim] could not read Backend from {config_yaml} ({e}); using 'py'.")
        return "py"


def _tl_junction_ids(tl_table):
    """Unique junction ids in the TL table, for the VirCarlaEnv SignalSubscription."""
    import csv
    ids = []
    try:
        with open(tl_table, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                j = (row.get("junction_id") or "").strip()
                if j and j not in ids:
                    ids.append(j)
    except (OSError, KeyError):
        pass
    return ids


def generate_config_yaml(path, tl_table, carla_host, carla_port, realtime=True,
                         refresh=0.05, backend="py"):
    """Write a probe-shaped VirCarlaEnv scenario config: mirror EVERY SUMO vehicle
    (#176 all:['true']) into CARLA and sync the tl_table's junctions. Visualization-
    only (#77): no XIL, no ego. Machine bits (CARLA host/port) come from the resolved
    env; Backend=cpp records the engine choice. Modelled on
    tests/Sumo/Probes/TrafficLayer_SUMO_Carla/config.yaml. Regenerated on --reimport."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    junctions = _tl_junction_ids(tl_table) if tl_table else []
    rt = "true" if realtime else "false"
    half = round(refresh / 2.0, 6)
    # The C++ side takes a literal address, not a resolvable hostname: normalise the
    # python client's 'localhost' default so both readers agree on 127.0.0.1.
    if carla_host in ("localhost", ""):
        carla_host = "127.0.0.1"
    sig = ""
    if junctions:
        names = ", ".join(f"'{j}'" for j in junctions)
        sig = ("  SignalSubscription:\n"
               "  - type: intersection\n"
               f"    attribute: {{ name: [{names}] }}\n"
               "    ip: ['127.0.0.1']\n"
               "    port: [440]\n")
    text = f"""\
# Auto-generated by run_cosim: the per-map co-sim scenario config.
# Visualization-only: CARLA mirrors every SUMO vehicle; SUMO owns the traffic.
#
# CarlaSetup.Backend selects the bridge on the next run:
#   py  -> the standalone run_synchronization.py bridge (default; ignores this file)
#   cpp -> the FIXS-native stack (TrafficLayer + VirCarlaEnv), which reads it fully
# Edit it here, or override per run with: run_cosim --engine cpp
# Regenerate from scratch with --reimport.
SimulationSetup:
  EnableRealSim: true
  EnableVerboseLog: false
  SelectedTrafficSimulator: 'SUMO'
  VehicleMessageField: [id, type, vehicleClass, speed, speedDesired, positionX, positionY, positionZ, heading, grade, length, width, height, color, linkId, laneId]
  TrafficSimulatorIP: '127.0.0.1'
  TrafficSimulatorPort: 1337
SumoSetup:
  SpeedMode: 32
ApplicationSetup:
  EnableApplicationLayer: true
  VehicleSubscription:
  - type: ego
    attribute: {{ all: ['true'] }}     # #176: mirror ALL vehicles (no ego anchor)
    ip: ['127.0.0.1']
    port: [440]
{sig}XilSetup:
  EnableXil: false
CarlaSetup:
  Backend: {backend}
  EnableVerboseLog: true
  EnableCosimulation: true
  EnableExternalControl: false          # #77 visualization-only
  UseVehicleTypeAsBlueprint: false
  EnableSpectatorFollow: false          # no ego to follow; run_cosim frames the view
  RealtimePacing: {rt}                   # standalone viz -> pace to real time (--fast makes this false)
  TrafficRefreshRate: {refresh}          # data cadence (s); matches the SUMO --step-length feed
  CarlaTimeStep: 0.0                     # render sub-step: 0 = tick 1:1 with the feed; a divisor (e.g. {half}) interpolates for smoother motion
  CarlaServerIP: {carla_host}
  CarlaServerPort: {carla_port}
  CarlaClientIP: 127.0.0.1
  CarlaClientPort: 440
"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"[cosim] generated VirCarlaEnv config -> {path}"
          + (f" ({len(junctions)} TL junctions)" if junctions else " (no TL sync)"))
    return path


def run_native_stack(config_yaml, sumocfg, tl_table, cfg, args):
    """FIXS-native bridge: launch SUMO (TraCI 1337) + TrafficLayer (-f config) +
    VirCarlaEnv (-f config -t tl_table). CARLA is already up and the map loaded by
    run_cosim's preflight. Ports mirror tests/Sumo/Probes/TrafficLayer_SUMO_Carla:
    SUMO 1337, TrafficLayer<->VirCarlaEnv 440. TrafficLayer is the sole TraCI client,
    so it steps SUMO (no external controller needed). Blocks on VirCarlaEnv - the
    co-sim front-end - and tears the others down on exit."""
    import shutil
    tl_exe, vce_exe = _native_binaries()
    missing = [p for p in (tl_exe, vce_exe) if not os.path.isfile(p)]
    if missing:
        sys.exit("[cosim] engine 'cpp' needs the FIXS-native binaries, not found:\n  "
                 + "\n  ".join(missing) + "\nRe-run initialize to fetch the bundle "
                 "(VirCarlaEnv is built only when the libcarla dep is available).")
    sumo_bin = "sumo-gui" if args.sumo_gui else "sumo"
    if not shutil.which(sumo_bin):
        sys.exit(f"[cosim] {sumo_bin} not on PATH (install SUMO / add %SUMO_HOME%/bin).")

    procs = []          # [(label, Popen)] in start order

    def _alive(p):
        return p.poll() is None

    def _check(label, p, hint=""):
        """Report whether a just-started component survived; die loudly if not - a
        silent early exit is the failure mode that reads as 'nothing is happening'."""
        if _alive(p):
            print(f"[cosim]   OK   {label} running (pid {p.pid})")
            return True
        print(f"[cosim]   DEAD {label} exited immediately (code {p.returncode}). {hint}")
        return False

    try:
        sumo_cmd = [sumo_bin, "-c", sumocfg, "--remote-port", "1337",
                    "--step-length", str(args.step_length)]
        if args.sumo_gui:
            sumo_cmd.append("--start")
        print(f"[SUMO] {' '.join(sumo_cmd)}")
        sumo = subprocess.Popen(sumo_cmd)
        procs.append(("SUMO", sumo))
        # SUMO must be listening before TrafficLayer connects as its TraCI client.
        for _ in range(30):
            if _port_in_use("127.0.0.1", 1337) or not _alive(sumo):
                break
            time.sleep(0.5)
        if not _check("SUMO", sumo, "check the sumocfg path / SUMO install."):
            return 1
        print(f"[cosim]   OK   SUMO TraCI listening on 1337"
              if _port_in_use("127.0.0.1", 1337) else
              "[cosim]   WARN SUMO is up but TraCI 1337 is not listening yet")

        print(f"[TL]   {os.path.basename(tl_exe)} -f {config_yaml}")
        tl = subprocess.Popen([tl_exe, "-f", config_yaml])
        procs.append(("TrafficLayer", tl))
        # TrafficLayer serves the bridge on 440; wait for that port before starting
        # VirCarlaEnv, which connects to it.
        for _ in range(30):
            if _port_in_use("127.0.0.1", 440) or not _alive(tl):
                break
            time.sleep(0.5)
        if not _check("TrafficLayer", tl,
                      "check the config yaml (a bad key or an in-use port 440/1337)."):
            return 1
        if not _port_in_use("127.0.0.1", 440):
            print("[cosim]   WARN TrafficLayer is running but port 440 is not open yet; "
                  "VirCarlaEnv may fail to subscribe.")
        else:
            print("[cosim]   OK   TrafficLayer serving the bridge on 440")

        vce_cmd = [vce_exe, "-f", config_yaml] + (["-t", tl_table] if tl_table else [])
        print(f"[VCE]  {os.path.basename(vce_exe)} -f {config_yaml}"
              + (f" -t {tl_table}" if tl_table else ""))
        vce = subprocess.Popen(vce_cmd)
        procs.append(("VirCarlaEnv", vce))
        time.sleep(3)   # long enough for a config/CARLA-connection failure to surface
        if not _check("VirCarlaEnv", vce,
                      f"check CARLA is reachable at {args.carla_host}:{args.carla_port} "
                      f"and see CarlaClient.log in {os.getcwd()}."):
            return 1

        print("\n[cosim] native stack up: SUMO + TrafficLayer + VirCarlaEnv.\n"
              "[cosim] vehicles should now appear in the CARLA window. Ctrl+C here, or "
              "close VirCarlaEnv, to stop.\n")

        # Supervise: block on VirCarlaEnv (the front-end) but surface it if any other
        # component dies first - otherwise a dead TrafficLayer just looks like a freeze.
        while _alive(vce):
            dead = [n for n, p in procs if n != "VirCarlaEnv" and not _alive(p)]
            if dead:
                print(f"[cosim] {', '.join(dead)} exited while the co-sim was running; "
                      f"the feed has stopped. Shutting the stack down.")
                break
            time.sleep(1.0)
        rc = vce.poll()
        if rc is None:
            rc = 0
        else:
            print(f"[cosim] VirCarlaEnv exited ({rc}); stopping the native stack.")
        return rc
    finally:
        for name, p in reversed(procs):
            if p.poll() is None:
                print(f"[cosim] stopping {name} ...")
                try:
                    p.terminate()
                except Exception:
                    pass
        for _name, p in reversed(procs):
            try:
                p.wait(timeout=5)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass


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


def _carla_command(cfg, port, render_offscreen, quality_level=None, level=None):
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
        cmd = [editor, uproject]
        # Boot straight into the map we are about to drive. Without this the engine
        # browses to the project's GameDefaultMap, which is a per-build setting we
        # do not own - if it names a map that has since been removed, startup dies
        # with "Failed to enter <map>: Can't find file" before the RPC port opens.
        if level:
            cmd.append(level)
        cmd += ["-game", f"-carla-rpc-port={port}"]
    if quality_level:
        cmd.append(f"-quality-level={quality_level}")  # Low is much cheaper to render
    if render_offscreen:
        cmd.append("-RenderOffScreen")  # headless, no display (Linux/CI)
    return cmd


def launch_carla(cfg, port=2000, render_offscreen=False, quality_level=None, level=None):
    cmd = _carla_command(cfg, port, render_offscreen, quality_level, level)
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


def _port_in_use(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(1)
        return s.connect_ex((host, port)) == 0


def _pid_on_port(port):
    """PID LISTENING on `port` (any host), or None. Windows netstat / Linux lsof."""
    try:
        if platform.system() == "Windows":
            out = subprocess.check_output(["netstat", "-ano", "-p", "tcp"],
                                          text=True, stderr=subprocess.DEVNULL)
            for line in out.splitlines():
                p = line.split()
                if len(p) >= 5 and p[3].upper() == "LISTENING" and p[1].endswith(f":{port}"):
                    return int(p[-1])
        else:
            out = subprocess.check_output(["lsof", "-ti", f"tcp:{port}", "-sTCP:LISTEN"],
                                          text=True, stderr=subprocess.DEVNULL)
            return int(out.split()[0]) if out.strip() else None
    except Exception:
        return None
    return None


def _process_name(pid):
    try:
        if platform.system() == "Windows":
            out = subprocess.check_output(["tasklist", "/FI", f"PID eq {pid}", "/FO", "CSV", "/NH"],
                                          text=True, stderr=subprocess.DEVNULL).strip()
            return out.splitlines()[0].split(",")[0].strip('"') if out else ""
        return subprocess.check_output(["ps", "-p", str(pid), "-o", "comm="],
                                       text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return ""


def _is_carla_process(name):
    n = (name or "").lower()
    return "ue4editor" in n or "carlaue4" in n


def _kill_pid_tree(pid):
    """Kill a PID and its whole process tree (CARLA spawns shader workers / a game
    child / CrashReportClient that a plain kill leaves holding the RPC port)."""
    try:
        if platform.system() == "Windows":
            subprocess.call(["taskkill", "/F", "/T", "/PID", str(pid)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            try:
                os.killpg(os.getpgid(pid), signal.SIGTERM)
            except Exception:
                os.kill(pid, signal.SIGTERM)
    except Exception:
        pass


def kill_carla(proc):
    _kill_pid_tree(proc.pid)


def _net_from_sumocfg(sumocfg):
    """Absolute path to the <net-file> referenced by a .sumocfg, or None."""
    import xml.etree.ElementTree as ET
    try:
        nf = ET.parse(sumocfg).getroot().find(".//net-file")
        if nf is not None and nf.get("value"):
            return os.path.join(os.path.dirname(os.path.abspath(sumocfg)), nf.get("value"))
    except Exception:
        pass
    return None


def resolve_tl_table(sumocfg, force=False, cache_name=None):
    """Find this scenario's traffic-light table: a traffic_light_table.csv committed
    next to the sumocfg, else one generated from the SUMO net. It is cached in the
    map's per-map folder ~/.fixs/maps/<cache_name>/tl_table.csv (cache_name = the
    cooked map name), else the shared ~/.fixs/tables/<net>_tls.csv. `force`
    regenerates even if a cache exists (used on --reimport). Returns a path, or None
    if neither is possible. Generation needs pandas/shapely but not SUMO installed."""
    scen = os.path.dirname(os.path.abspath(sumocfg))
    committed = os.path.join(scen, "traffic_light_table.csv")
    if os.path.isfile(committed):
        print(f"[cosim] TL table: committed {committed}")
        return committed
    net = _net_from_sumocfg(sumocfg)
    if not net or not os.path.isfile(net):
        print("[cosim] no TL table and no net to generate one from; TL sync off.")
        return None
    maps_root = os.path.join(os.path.dirname(env.CONFIG_PATH), "maps")
    if cache_name:
        cache = os.path.join(maps_root, cache_name)
        out = os.path.join(cache, "tl_table.csv")
    else:
        cache = os.path.join(os.path.dirname(env.CONFIG_PATH), "tables")
        out = os.path.join(cache, os.path.splitext(os.path.basename(net))[0] + "_tls.csv")
    if os.path.isfile(out) and not force:
        print(f"[cosim] TL table: cached generated {out}")
        return out
    try:
        sys.path.insert(0, os.path.join(HERE, "utils"))
        import extract_sumo_tls_as_table as gen
        os.makedirs(cache, exist_ok=True)
        gen.generate_table(net, out)
        print(f"[cosim] TL table: generated from {os.path.basename(net)} -> {out}")
        return out
    except Exception as exc:
        print(f"[cosim] could not generate TL table ({exc}); TL sync off.")
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sumocfg", default=None,
                    help="SUMO .sumocfg. Optional: if omitted it comes from the chosen "
                         "map bundle's sumo/ (a DT-Library map ships its scenario). Pass "
                         "it to override with your own demand on a shared map.")
    ap.add_argument("--map", default=None,
                    help="Map to run: a Digital-Twin-Library location (e.g. 'roosevelt'), "
                         "or an already-cooked map name. If omitted, pick from the catalog "
                         "/ local ~/.fixs/maps (lists options, you choose, it auto-downloads).")
    ap.add_argument("--repo", default=None,
                    help="owner/repo whose map-* releases to offer when --map is omitted "
                         "(or a repo= line in --map-config)")
    ap.add_argument("--tag-prefix", default=None,
                    help="release tag prefix for the picker (e.g. 'map-'); the map name is "
                         "the tag minus this prefix")
    ap.add_argument("--auto-import", action="store_true",
                    help="source build: if the map isn't imported, cook it before launching "
                         "(implied when you pick a version via --repo)")
    ap.add_argument("--map-package", default=None,
                    help="import package name if it differs from --map")
    ap.add_argument("--map-package-url", default=None,
                    help="URL of the map package zip for --auto-import (e.g. a release asset)")
    ap.add_argument("--map-config", default=None,
                    help="text file declaring the import package (package= and url= lines)")
    ap.add_argument("--reimport", action="store_true",
                    help="re-import the map even if already cooked (re-download + re-cook)")
    ap.add_argument("--tl-table", default=None, help="traffic_light_table.csv (for --tls-manager sumo)")
    ap.add_argument("--tls-manager", default=None, choices=["sumo", "carla", "none"],
                    help="who drives the lights (default: the map's catalog setting, else 'sumo')")
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
    ap.add_argument("--prep-only", action="store_true",
                    help="import the map + place traffic lights and signs, then stop "
                         "(do not launch CARLA or run the co-sim)")
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
    ap.add_argument("--engine", choices=["py", "cpp"], default=None,
                    help="co-sim bridge: py=run_synchronization.py (default), "
                         "cpp=TrafficLayer+VirCarlaEnv (FIXS-native). Overrides the "
                         "scenario yaml's CarlaSetup.Backend.")
    ap.add_argument("--config", default=None,
                    help="[cpp] scenario yaml for the native bridge (default: "
                         "~/.fixs/maps/<cooked>/config.yaml, generated if missing).")
    ap.add_argument("--no-update-check", action="store_true",
                    help="skip the FIXS-bundle freshness check against the release")
    args = ap.parse_args()

    # Advisory: nudge to update a stale local FIXS bundle before doing real work.
    maybe_update_fixs(no_check=args.no_update_check)

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

    # Decide which map to run. An explicit --map wins (seamless / scriptable).
    # Otherwise list the published map releases of --repo and let the user pick a
    # version; the chosen version is what we both import AND load, so the imported
    # map and the loaded map can never drift apart.
    import import_map
    repo, tag_prefix = import_map.resolve_map_source(args.repo, args.tag_prefix)
    catalog = import_map.fetch_catalog(repo)

    # Two slots to fill: a CARLA map (to cook + load) and a SUMO scenario. A
    # Digital-Twin-Library bundle fills both; the catalog gives the real cooked
    # name + per-map settings WITHOUT a download; --map / --sumocfg override; a
    # local pick fills either slot.
    settings = {}
    picked_tag = None            # DT release to (lazily) download for import / sumo
    picked_local = None          # local .zip / folder to import from
    target_map = args.map        # provisional; a catalog location resolves to the real name

    ent = import_map.catalog_entry(catalog, args.map) if args.map else None
    if ent:                                      # --map is a DT-Library location
        target_map = ent["map_name"]
        settings = ent.get("settings", {})
        picked_tag = ent["release"]
        print(f"[cosim] map '{args.map}' -> '{target_map}'  (DT release {picked_tag})")
    elif args.map:                               # legacy: --map is a cooked-map name
        target_map = args.map
    else:                                        # pick from catalog / local
        target_map, picked_tag, picked_local = import_map.choose_map(
            repo, tag_prefix, cfg.get("carla_root") if cfg else None)
        picked = import_map.catalog_entry(catalog, target_map)
        if picked:                               # a catalog pick: use its real name + settings
            settings = picked.get("settings", {})
            target_map = picked["map_name"]
        print(f"[cosim] selected map: {target_map}")

    # Per-map settings are defaults; explicit CLI flags override.
    no_net_offset = args.no_net_offset or settings.get("net_offset") == "zero"
    tls_manager = args.tls_manager or settings.get("tls_manager") or "sumo"

    sumo_dir = None              # dir holding the chosen bundle's .sumocfg (set on open)
    # /Game/... path to boot CARLA into (set by the source-build preflight). None
    # (packaged build / --no-launch) = let the engine pick.
    target_level = None

    # Source-build preflight: a custom map must be cooked into the build before
    # CARLA can load it. Import it if missing - from a DT-Library bundle (downloaded
    # + cached, split into carla/ + sumo/) or a local pick - else fail clearly.
    if not args.no_launch and cfg is not None and cfg.get("mode") == "source":
        resolved = None if args.reimport else \
            import_map.resolve_cooked_map(cfg["carla_root"], target_map)

        # Already imported? If this was a FRESH source pick (an online release or a
        # local .zip/folder), offer to reimport - re-cook + re-place TLs/signs +
        # regenerate the TL table. A pick of an already-imported map (both picked_*
        # None) is run as-is, no prompt.
        if resolved is not None and (picked_tag or picked_local) \
                and not args.reimport and sys.stdin.isatty():
            ans = input(f"[cosim] '{resolved[0]}' is already imported. Reimport "
                        f"(re-cook + re-place TLs/signs + regen TL table)? [y/N]: ").strip().lower()
            if ans.startswith("y"):
                args.reimport = True
                resolved = None

        # Materialize the bundle if we must import, or need its sumo/ (no --sumocfg).
        # download_release_zip caches under ~/.fixs/maps; open_bundle splits it.
        carla_src = None
        if (resolved is None or args.sumocfg is None) and (picked_local or picked_tag):
            zip_path = picked_local or import_map.download_release_zip(
                repo, picked_tag, force_redownload=args.reimport, cache_name=target_map)
            carla_src, sumo_dir = import_map.open_bundle(zip_path, cache_name=target_map)

        if resolved is None:
            if carla_src is not None:                    # a DT/local bundle or raw export
                real = import_map.map_name_in(carla_src) or target_map
                verb = "re-importing" if args.reimport else "importing"
                print(f"[cosim] {verb} '{real}' before launch ...")
                import_map.ensure_map(real, carla_root=cfg["carla_root"],
                                      ue4_root=cfg.get("ue4_root"),
                                      package_dir=carla_src, force=args.reimport)
                target_map = real
            elif args.auto_import or args.reimport:      # legacy explicit --map + url/config
                url = args.map_package_url
                if args.map_config:
                    url = url or import_map.read_map_config(args.map_config).get("url")
                verb = "re-importing" if args.reimport else "importing"
                print(f"[cosim] {verb} map '{target_map}' before launch ...")
                import_map.ensure_map(target_map, carla_root=cfg["carla_root"],
                                      ue4_root=cfg.get("ue4_root"),
                                      package_url=url, force=args.reimport)
            else:
                sys.exit(
                    f"[cosim] map '{target_map}' is not imported into {cfg['carla_root']}.\n"
                    f"        Pick a DT-Library map (--map <location>), a local bundle "
                    f"(--package-dir <zip/folder>), or --auto-import [--map-package-url <zip>].")
            resolved = import_map.resolve_cooked_map(cfg["carla_root"], target_map)

        if resolved is None:
            print(f"[cosim] could not tell which cooked map '{target_map}' provides; "
                  f"pick the one to load:")
            target_map = import_map.choose_imported_map(cfg["carla_root"])
            target_level = import_map.choose_level_path(cfg["carla_root"], target_map)
        else:
            if resolved[0] != target_map:
                print(f"[cosim] package '{target_map}' provides map '{resolved[0]}'")
            target_map, target_level = resolved

        note = import_map.duplicate_level_note(cfg["carla_root"], target_map, target_level)
        if note:
            print(note)

    # SUMO slot: --sumocfg wins; else the chosen bundle's sumo/. A catalog map that
    # was already cooked (so no bundle opened above) fetches its bundle now, just
    # for sumo/ (cached).
    sumocfg = args.sumocfg
    if sumocfg is None:
        if sumo_dir is None and (picked_local or picked_tag):
            zip_path = picked_local or import_map.download_release_zip(
                repo, picked_tag, cache_name=target_map)
            _carla, sumo_dir = import_map.open_bundle(zip_path, cache_name=target_map)
        sumocfg = import_map.bundle_sumocfg(sumo_dir)
        # A Local (already-imported) pick opens no bundle, and a carla-only pick's
        # bundle has no sumo/ - but the map's sumo/ may already be cached at
        # ~/.fixs/maps/<name>/sumo (from a prior import or a separate sumo pick).
        # Reuse it before prompting.
        if sumocfg is None:
            cached_sumo = import_map.map_sumo_dir(target_map)
            if cached_sumo:
                sumo_dir = cached_sumo
                sumocfg = import_map.bundle_sumocfg(sumo_dir)
                print(f"[cosim] using cached SUMO scenario for '{target_map}': {sumocfg}")
        # Still nothing (first time, no cache): pick one now. choose_sumo_source
        # caches it under ~/.fixs/maps/<name>/sumo so the next run reuses it.
        if sumocfg is None:
            sumo_dir = import_map.choose_sumo_source(cache_name=target_map)
            sumocfg = import_map.bundle_sumocfg(sumo_dir)
    if sumocfg is None:
        sys.exit("[cosim] no SUMO scenario to run: pass --sumocfg, or choose a map "
                 "bundle / sumo source that provides one.")

    # TL table for signal sync + placement: --tl-table wins, else committed next to
    # the sumocfg, else generated from the SUMO net (cached ~/.fixs/tables).
    tl_table = args.tl_table
    if tls_manager == "sumo" and not tl_table:
        tl_table = resolve_tl_table(sumocfg, force=args.reimport, cache_name=target_map)

    # TL + sign placement (source build only; idempotent via markers). Runs after
    # the import + sumocfg/TL-table resolution so the table exists to place from.
    if not args.no_launch and cfg is not None and cfg.get("mode") == "source":
        imported_now = (picked_tag is not None or picked_local is not None
                        or args.auto_import or args.reimport)
        if tl_table:
            import place_tls
            if (not place_tls.tls_placed(cfg["carla_root"], target_map)) or args.reimport:
                if imported_now:
                    print(f"[cosim] placing traffic lights for '{target_map}' before launch ...")
                    place_tls.place_tls(target_map, tl_table, carla_root=cfg["carla_root"],
                                        ue4_root=cfg.get("ue4_root"), force=args.reimport)
                else:
                    print(f"[cosim] note: traffic lights not placed for '{target_map}'. Run "
                          f"place_tls (or --reimport) to add them, else no TL sync.")
        import place_signs
        if (not place_signs.signs_placed(cfg["carla_root"], target_map)) or args.reimport:
            if imported_now:
                print(f"[cosim] placing road signs for '{target_map}' before launch ...")
                place_signs.place_signs(target_map, carla_root=cfg["carla_root"],
                                        ue4_root=cfg.get("ue4_root"), force=args.reimport)

    if args.prep_only:
        print(f"[cosim] --prep-only: '{target_map}' imported + prepped (TLs/signs); not launching.")
        return 0

    carla_proc = None
    try:
        if not args.no_launch:
            # If the RPC port is already taken, a CARLA from a previous run is
            # probably still alive. Launching anyway gives a split brain (the new
            # server can't bind the port, so it sits on the default map while we
            # drive the OLD one). Auto-kill a *stale CARLA* holding the port; only
            # abort if a non-CARLA process owns it (don't kill something unrelated).
            if _port_in_use(args.carla_host, args.carla_port):
                pid = _pid_on_port(args.carla_port)
                name = _process_name(pid) if pid else ""
                if pid and _is_carla_process(name):
                    print(f"[cosim] a stale CARLA is holding port {args.carla_port} "
                          f"(PID {pid}, {name}); terminating it ...")
                    _kill_pid_tree(pid)
                    for _ in range(40):
                        if not _port_in_use(args.carla_host, args.carla_port):
                            break
                        time.sleep(0.5)
                    if _port_in_use(args.carla_host, args.carla_port):
                        sys.exit(f"[cosim] could not free port {args.carla_port}; "
                                 f"kill CARLA manually and retry.")
                    print(f"[cosim] port {args.carla_port} freed; launching a fresh CARLA.")
                else:
                    sys.exit(
                        f"[cosim] port {args.carla_port} is in use by a non-CARLA process "
                        f"({name or 'unknown'}); free it or pass --no-launch to use what's running.")
            carla_proc = launch_carla(cfg, args.carla_port, args.render_offscreen,
                                      args.quality_level, target_level)
            if not wait_for_port(args.carla_host, args.carla_port):
                sys.exit("CARLA RPC port did not open in time.")

        import carla
        client = carla.Client(args.carla_host, args.carla_port)
        client.set_timeout(args.load_timeout)
        # Load by full /Game/... path when we have one: a bare name makes CARLA
        # pick the first .umap of that name it happens to find, which is the wrong
        # copy as soon as two packages ship the same map name.
        load_arg = target_level or target_map
        print(f"[CARLA] loading world: {load_arg}")
        print("[CARLA] (the first load of a freshly imported map compiles shaders - "
              "this can take a few minutes; later loads are fast)")
        client.load_world(load_arg)
        # Don't trust load_world's return alone on a heavy source map: confirm the
        # world IS this map and is ticking before we start SUMO.
        print(f"[CARLA] confirming '{target_map}' is loaded and ready ...")
        loaded = confirm_world_ready(client, target_map, args.load_timeout)
        if loaded is None:
            current = ""
            try:
                current = client.get_world().get_map().name
            except Exception:
                pass
            sys.exit(f"[cosim] map '{target_map}' not confirmed loaded "
                     f"(current world: '{current}'); aborting before SUMO.")
        print(f"[CARLA] map ready: {loaded}")
        world = client.get_world()

        if not args.no_spectator:
            # Default: zoom to one intersection from the TL table so the signal
            # sync is legible. --spectator-all frames the whole network (placed TL
            # actors, else the full table).
            frame = None
            if tl_table and not args.spectator_all:
                frame = _frame_from_table(tl_table, no_net_offset,
                                          junction=args.spectator_junction)
            if frame is None:
                frame = _frame_from_actors(world)
            if frame is None and tl_table:
                frame = _frame_from_table(tl_table, no_net_offset, whole=True)
            if frame is not None:
                position_spectator(world, frame)
            else:
                print("[VIEW] no TL actors/table to anchor on; leaving spectator as is")

        # The scenario yaml is a per-map artifact like tl_table.csv: always written
        # on first use (and refreshed by --reimport), whichever bridge runs. That
        # keeps it inspectable/editable BEFORE a cpp run - and makes its
        # CarlaSetup.Backend a real switch, since a missing file would otherwise
        # always read as 'py' and could never generate itself.
        config_yaml = args.config or import_map.map_config_path(target_map)
        if args.reimport or not os.path.isfile(config_yaml):
            # A regenerate must not silently undo hand edits: carry the existing
            # Backend choice over, and keep the old file as .bak to fall back on.
            prior = read_backend(config_yaml) if os.path.isfile(config_yaml) else None
            if prior:
                import shutil as _sh
                _sh.copy2(config_yaml, config_yaml + ".bak")
                print(f"[cosim] regenerating {os.path.basename(config_yaml)} "
                      f"(previous kept as .bak; Backend={prior} preserved)")
            generate_config_yaml(config_yaml, tl_table, args.carla_host,
                                 args.carla_port, realtime=not args.fast,
                                 refresh=args.step_length,
                                 backend=args.engine or prior or "py")

        # Engine dispatch: CarlaSetup.Backend in that yaml picks the bridge
        # (--engine overrides). cpp = FIXS-native (TrafficLayer + VirCarlaEnv); py
        # (default) = the standalone run_synchronization.py bridge below.
        backend = args.engine or read_backend(config_yaml)
        if backend == "cpp":
            print(f"[cosim] engine=cpp (FIXS-native); config {config_yaml}")
            return run_native_stack(config_yaml, sumocfg, tl_table, cfg, args)

        print("[cosim] engine=py (run_synchronization.py)")
        cmd = [sys.executable, SYNC, sumocfg,
               "--tls-manager", tls_manager,
               "--step-length", str(args.step_length),
               "--carla-host", args.carla_host, "--carla-port", str(args.carla_port),
               "--carla-timeout", str(args.carla_timeout)]
        if tl_table:
            cmd += ["--tl-table", tl_table]
        if no_net_offset:
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
