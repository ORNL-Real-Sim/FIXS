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

import app_catalog
import carla_env_setup as env
import run_profile

HERE = os.path.dirname(os.path.abspath(__file__))
SYNC = os.path.join(HERE, "sumo", "run_synchronization", "run_synchronization.py")
FIXS_ROOT = os.path.dirname(HERE)          # FIXS/Carla -> FIXS (the fetched bundle root)
APP_ROOT = os.path.dirname(FIXS_ROOT)      # FIXS -> the app dir (holds initialize.{sh,ps1})

# Defaults for the native stack's two ports, used ONLY to seed a freshly generated
# scenario yaml and as the fallback when one cannot be read. The yaml is the single
# source of truth at run time - read_stack_ports() takes them from
# SimulationSetup.TrafficSimulatorPort and CarlaSetup.CarlaClientPort, so editing
# the yaml moves the ports for every component instead of just some of them.
DEFAULT_TRACI_PORT = 1337    # SUMO TraCI server (TrafficLayer connects as its client)
DEFAULT_BRIDGE_PORT = 440    # TrafficLayer serves VirCarlaEnv here
DEFAULT_CARLA_HOST = "localhost"   # CARLA RPC; CarlaSetup.CarlaServerIP overrides
DEFAULT_CARLA_PORT = 2000


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


def _read_scenario_config(config_yaml):
    """The parsed scenario yaml via CommonLib/ConfigHelper.py (the canonical parser
    the C++ engine mirrors), or None if it cannot be read."""
    if not config_yaml or not os.path.isfile(config_yaml):
        return None
    try:
        sys.path.insert(0, os.path.join(FIXS_ROOT, "CommonLib"))
        import ConfigHelper
        ch = ConfigHelper.ConfigHelper()
        ch.getConfig(config_yaml)
        return ch
    except Exception as e:
        print(f"[cosim] could not read {config_yaml} ({e}); using defaults.")
        return None


def read_backend(config_yaml):
    """'py' or 'cpp' from CarlaSetup.EnablePythonBackend. A bool rather than a
    free-text engine name, so a typo cannot silently pick the wrong bridge.
    'py' on absence or any parse problem."""
    ch = _read_scenario_config(config_yaml)
    if ch is None:
        return "py"
    return "py" if ch.Carla_setup["EnablePythonBackend"] else "cpp"


def read_stack_ports(config_yaml):
    """(traci_port, bridge_port) from the scenario yaml - SUMO's TraCI server
    (SimulationSetup.TrafficSimulatorPort) and the TrafficLayer<->VirCarlaEnv bridge
    (CarlaSetup.CarlaClientPort). Read rather than hard-coded so the yaml stays the
    single source of truth: the exes get these values from the same file."""
    ch = _read_scenario_config(config_yaml)
    if ch is None:
        return DEFAULT_TRACI_PORT, DEFAULT_BRIDGE_PORT
    try:
        traci_port = int(ch.simulation_setup["TrafficSimulatorPort"] or DEFAULT_TRACI_PORT)
        bridge_port = int(ch.Carla_setup["CarlaClientPort"] or DEFAULT_BRIDGE_PORT)
        return traci_port, bridge_port
    except (TypeError, ValueError):
        return DEFAULT_TRACI_PORT, DEFAULT_BRIDGE_PORT


def read_carla_endpoint(config_yaml):
    """(host, port) of the CARLA RPC server from CarlaSetup.CarlaServerIP/Port,
    or (None, None) if unreadable.

    VirCarlaEnv takes its CARLA endpoint from this yaml, so run_cosim has to
    launch/probe/connect to the SAME one or the two disagree silently: run_cosim
    reports a healthy CARLA on localhost:2000 while VirCarlaEnv dials whatever
    the yaml says and times out. Note this is a DIFFERENT link from
    CarlaSetup.CarlaClientIP/Port, which despite the name is not CARLA at all -
    it is TrafficLayer's bridge endpoint (mainVirCarla: trafficLayerIP_)."""
    ch = _read_scenario_config(config_yaml)
    if ch is None:
        return None, None
    try:
        host = (ch.Carla_setup["CarlaServerIP"] or "").strip() or None
        port = int(ch.Carla_setup["CarlaServerPort"] or 0) or None
        return host, port
    except (TypeError, ValueError, AttributeError):
        return None, None


def read_sumo_num_clients(config_yaml):
    """SumoSetup.NumClients - how many TraCI clients SUMO must wait for before it
    starts stepping. 1 (the SUMO default) unless the yaml says otherwise."""
    ch = _read_scenario_config(config_yaml)
    if ch is None:
        return 1
    try:
        return max(1, int(ch.Sumo_setup["NumClients"] or 1))
    except (TypeError, ValueError, KeyError):
        return 1


def read_sumo_autostart(config_yaml):
    """SumoSetup.AutoStart: whether sumo-gui gets --start. True (default) = it steps
    as soon as it loads; False = it opens loaded but PAUSED so you press Play.
    run_cosim launches SUMO either way. --sumo-no-start overrides."""
    ch = _read_scenario_config(config_yaml)
    if ch is None:
        return True
    return bool(ch.Sumo_setup["AutoStart"])


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
                         refresh=0.05, backend="py",
                         traci_port=DEFAULT_TRACI_PORT, bridge_port=DEFAULT_BRIDGE_PORT):
    """Write a probe-shaped VirCarlaEnv scenario config: mirror EVERY SUMO vehicle
    (#176 all:['true']) into CARLA and sync the tl_table's junctions. Visualization-
    only (#77): no XIL, no ego. Machine bits (CARLA host/port) come from the resolved
    env; Backend=cpp records the engine choice. Modelled on
    tests/Sumo/Probes/TrafficLayer_SUMO_Carla/config.yaml. Regenerated on --reimport."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    junctions = _tl_junction_ids(tl_table) if tl_table else []
    rt = "true" if realtime else "false"
    use_py = "true" if backend == "py" else "false"
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
               f"    port: [{bridge_port}]\n")
    text = f"""\
# Auto-generated by run_cosim: the per-map co-sim scenario config.
# Visualization-only: CARLA mirrors every SUMO vehicle; SUMO owns the traffic.
#
# CarlaSetup.EnablePythonBackend selects the bridge on the next run:
#   true  -> the standalone run_synchronization.py bridge (default; ignores this file)
#   false -> the FIXS-native stack (TrafficLayer + VirCarlaEnv), which reads it fully
# Edit it here, or override per run with: run_cosim --engine cpp
# Regenerate from scratch with --reimport.
SimulationSetup:
  EnableRealSim: true
  EnableVerboseLog: false
  SelectedTrafficSimulator: 'SUMO'
  VehicleMessageField: [id, type, vehicleClass, speed, speedDesired, positionX, positionY, positionZ, heading, grade, length, width, height, color, linkId, laneId]
  TrafficSimulatorIP: '127.0.0.1'
  TrafficSimulatorPort: {traci_port}     # SUMO TraCI; run_cosim launches SUMO on this
SumoSetup:
  SpeedMode: 32
  # sumo-gui's --start. true -> the simulation runs as soon as it loads.
  # false -> the window opens loaded but PAUSED and you press Play, so you control
  #          when stepping begins (--sumo-no-start overrides for one run).
  AutoStart: true
ApplicationSetup:
  EnableApplicationLayer: true
  VehicleSubscription:
  - type: ego
    attribute: {{ all: ['true'] }}     # #176: mirror ALL vehicles (no ego anchor)
    ip: ['127.0.0.1']
    port: [{bridge_port}]
{sig}XilSetup:
  EnableXil: false
CarlaSetup:
  # true  -> the standalone Python bridge (run_synchronization.py); ignores the rest
  # false -> the FIXS-native stack (TrafficLayer + VirCarlaEnv), which reads it fully
  EnablePythonBackend: {use_py}
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
  CarlaClientPort: {bridge_port}
"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"[cosim] generated VirCarlaEnv config -> {path}"
          + (f" ({len(junctions)} TL junctions)" if junctions else " (no TL sync)"))
    return path


def run_native_stack(config_yaml, sumocfg, tl_table, cfg, args):
    """FIXS-native bridge: launch SUMO (TraCI server) + TrafficLayer (-f config) +
    VirCarlaEnv (-f config -t tl_table). CARLA is already up and the map loaded by
    run_cosim's preflight. Ports mirror tests/Sumo/Probes/TrafficLayer_SUMO_Carla:
    both ports come from the yaml (SimulationSetup.TrafficSimulatorPort and
    CarlaSetup.CarlaClientPort). TrafficLayer is the sole TraCI client,
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

    # Clear stale FIXS-side processes before launching (what the reference probe
    # .bat does with taskkill). A previous run closed by its window X - or killed
    # mid-tick - leaves SUMO holding the TraCI port or TrafficLayer holding the bridge
    # port, and the
    # new SUMO then cannot bind: the stack comes up but every component loops on
    # "Could not connect to TraCI server". CARLA is deliberately left alone (it is
    # slow to start and run_cosim's own preflight owns it).
    # Ports come from the scenario yaml, so moving one there moves it for every
    # component. AutoStart only decides whether sumo-gui gets --start.
    traci_port, bridge_port = read_stack_ports(config_yaml)
    sumo_autostart = read_sumo_autostart(config_yaml) and not args.sumo_no_start
    num_clients = read_sumo_num_clients(config_yaml)

    for label, port in (("SUMO (TraCI)", traci_port),
                        ("TrafficLayer (bridge)", bridge_port)):
        pid = _pid_on_port(port)
        if not pid:
            continue
        name = _process_name(pid) or "?"
        print(f"[cosim] port {port} still held by {name} (pid {pid}) from an earlier "
              f"run; stopping it so {label} can bind.")
        _kill_pid_tree(pid)
        for _ in range(10):
            if not _port_listening(port):
                break
            time.sleep(0.5)
        if _port_listening(port):
            sys.exit(f"[cosim] port {port} is still in use; close the leftover "
                     f"{name} window and re-run.")

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
        sumo_cmd = [sumo_bin, "-c", sumocfg, "--remote-port", str(traci_port),
                    "--step-length", str(args.step_length),
                    "--num-clients", str(num_clients)]
        # SUMO blocks until ALL num_clients TraCI clients have connected and called
        # setOrder. TrafficLayer is one of them; anything above 1 means you are
        # attaching your own second client, so say so rather than looking hung.
        if num_clients > 1:
            print(f"[cosim]   ->   SumoSetup.NumClients={num_clients}: SUMO will not "
                  f"step until {num_clients} TraCI clients have connected "
                  f"(TrafficLayer is one).")
        # --start makes sumo-gui begin stepping as soon as it loads. Omit it and the
        # window opens loaded but paused, so you press Play (and can watch the TraCI
        # handshake happen first). Headless sumo has no Play button, so the flag is
        # only meaningful for the GUI.
        if args.sumo_gui and sumo_autostart:
            sumo_cmd.append("--start")
        print(f"[SUMO] {' '.join(sumo_cmd)}")
        sumo = subprocess.Popen(sumo_cmd)
        procs.append(("SUMO", sumo))
        # SUMO must be listening before TrafficLayer connects as its TraCI client.
        for _ in range(30):
            if _port_listening(traci_port) or not _alive(sumo):
                break
            time.sleep(0.5)
        if not _check("SUMO", sumo, "check the sumocfg path / SUMO install."):
            return 1
        if _port_listening(traci_port):
            print(f"[cosim]   OK   SUMO TraCI listening on {traci_port}")
        else:
            # SUMO opens the TraCI socket before it loads the net, so 15s of silence
            # is a real problem (bad sumocfg, port taken), not a slow GUI.
            print(f"[cosim]   WARN SUMO is up but never opened TraCI {traci_port}; "
                  f"TrafficLayer will not be able to connect.")
        if args.sumo_gui and not sumo_autostart:
            print("[cosim]   ->   sumo-gui started PAUSED (AutoStart off): press Play "
                  "in its window when you want the simulation to run.")

        print(f"[TL]   {os.path.basename(tl_exe)} -f {config_yaml}")
        tl = subprocess.Popen([tl_exe, "-f", config_yaml])
        procs.append(("TrafficLayer", tl))
        # TrafficLayer serves the bridge port; wait for it before starting
        # VirCarlaEnv, which connects to it.
        for _ in range(30):
            if _port_listening(bridge_port) or not _alive(tl):
                break
            time.sleep(0.5)
        if not _check("TrafficLayer", tl,
                      f"check the config yaml (a bad key, or ports {bridge_port}/"
                      f"{traci_port} already in use)."):
            return 1
        if not _port_listening(bridge_port):
            print(f"[cosim]   WARN TrafficLayer is running but port {bridge_port} is not "
                  f"open yet; VirCarlaEnv may fail to subscribe.")
        else:
            print(f"[cosim]   OK   TrafficLayer serving the bridge on {bridge_port}")

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
    """True if `host:port` accepts a TCP connection.

    This OPENS AND DROPS A REAL CONNECTION, so it is only safe against servers
    that accept many clients (CARLA's RPC port). For the single-client servers in
    the native stack use _port_listening() instead - see the note there."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(1)
        return s.connect_ex((host, port)) == 0


def _port_listening(port):
    """True if something is LISTENING on `port`, observed WITHOUT connecting.

    Both servers in the native stack accept exactly ONE client and stop listening
    the moment they get it: SUMO's TraCI server defaults to --num-clients 1, and
    TrafficLayer counts the first accept on its bridge port as VirCarlaEnv. A
    connect() readiness probe against those is not a passive observation - it
    *consumes* the one client slot and then hangs up, which left SUMO holding a
    CLOSE_WAIT socket with no listener and TrafficLayer looping forever on
    "Could not connect to TraCI server at 127.0.0.1:1337" while TrafficLayer had
    already logged a phantom "Handling client #1 / All Clients Connected!" before
    VirCarlaEnv was even launched. Reading the OS socket table has no such side
    effect, so readiness polling is free to be as chatty as it likes."""
    return _pid_on_port(port) is not None


def _is_local_host(host):
    """True if `host` names this machine, so CARLA there is ours to launch/kill."""
    h = (host or "").strip().lower()
    if h in ("", "localhost", "127.0.0.1", "::1", "0.0.0.0"):
        return True
    try:
        return socket.gethostbyname(h) in ("127.0.0.1", "::1") \
            or h == socket.gethostname().lower()
    except OSError:
        return False


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


# --------------------------------------------------------------------------- #
# Application + saved run profile.
#
# Two features that only make sense together: apps/apps.json says WHAT can be run
# (and with which maps / scenario yamls), the run profile remembers WHAT YOU RAN
# so the next run is one keypress. Both are optional - no manifest and no profile
# gives exactly the pre-app-awareness behaviour.
#
# Precedence for every setting, strongest first:
#   explicit CLI flag  >  saved profile  >  app defaults  >  built-in default
# The CLI wins outright so `run_cosim --map atlanta` changes that one thing and
# prompts for nothing.
# --------------------------------------------------------------------------- #
DEFAULT_STEP_LENGTH = 0.05

# Slot -> the args attributes it owns. Kept as data so the summary, the change
# menu and the apply step cannot drift apart.
PROFILE_SLOTS = {
    "app": ("app",),
    "map": ("map",),
    "config": ("config",),
    "engine": ("engine",),
    "carla": ("carla_host", "carla_port"),
    "sumo": ("sumo_gui", "step_length"),
}
# The three with no sensible default: they must be answered before a first run.
# engine / carla / sumo always have one (the yaml, carla.json, the built-ins), so
# they start filled in and are edited only if you ask for them.
MUST_ANSWER = ("app", "map", "config")


def _ask(prompt, default=None):
    """input() that treats EOF as 'take the default' rather than exploding."""
    try:
        return input(prompt).strip()
    except EOFError:
        return default or ""


def _menu(title, options, current=None):
    """Numbered single-choice menu. `options` is [(value, label)]. Enter keeps
    `current` when it is one of the values, else takes the first. Returns a value."""
    values = [v for v, _ in options]
    idx = values.index(current) + 1 if current in values else 1
    print(f"\n[cosim] {title}")
    for i, (value, label) in enumerate(options, 1):
        mark = "  (current)" if value == current else ""
        print(f"   {i}) {label}{mark}")
    if not sys.stdin.isatty():
        return values[idx - 1]
    while True:
        ans = _ask(f"[cosim] Which? [1-{len(options)}], Enter = {idx}: ")
        if ans == "":
            return values[idx - 1]
        if ans.isdigit() and 1 <= int(ans) <= len(options):
            return values[int(ans) - 1]
        print("[cosim] invalid choice; enter a number from the list.")


# --------------------------------------------------------------------------- #
# Slot editors. One per line of the summary, so every line the menu offers is a
# line the menu can actually change - and each returns the new value(s) rather
# than reaching into the run, which is what lets the loop redraw and ask again.
# --------------------------------------------------------------------------- #
def edit_engine(current):
    """Which bridge runs the co-sim."""
    return _menu("Co-sim engine:",
                 [("py", "py    standalone run_synchronization.py bridge"),
                  ("cpp", "cpp   FIXS-native stack (TrafficLayer + VirCarlaEnv)")],
                 current)


def edit_sumo(gui, step):
    """How SUMO runs: window or headless, and the shared timestep."""
    gui = _menu("SUMO:", [(True, "gui        sumo-gui window"),
                          (False, "headless   no window")], bool(gui))
    if sys.stdin.isatty():
        while True:
            ans = _ask(f"[cosim] Timestep in seconds [Enter = {step}]: ")
            if ans == "":
                break
            try:
                val = float(ans)
            except ValueError:
                print("[cosim] enter a number, e.g. 0.05.")
                continue
            # CARLA's default physics substepping cannot integrate a step above
            # 0.1s; SUMO would happily take it and the two would disagree.
            if not 0 < val <= 0.1:
                print("[cosim] must be > 0 and <= 0.1 (CARLA's max with default "
                      "physics substepping).")
                continue
            step = val
            break
    return gui, step


def edit_carla(cfg, host, port):
    """The CARLA to talk to: which install (carla.json) and which RPC endpoint."""
    what = _menu(
        "CARLA:",
        [("endpoint", f"set the RPC endpoint  (now {host or '127.0.0.1'}:{port or 2000})"),
         ("install", f"switch the CARLA install  (now {(cfg or {}).get('mode', '?')} "
                     f"{(cfg or {}).get('carla_root', '?')})")],
        "endpoint")
    if what == "install":
        cfg = env.run_setup()
        return cfg, host, port
    if sys.stdin.isatty():
        ans = _ask(f"[cosim] CARLA host [Enter = {host or DEFAULT_CARLA_HOST}]: ")
        host = ans or host or DEFAULT_CARLA_HOST
        while True:
            ans = _ask(f"[cosim] CARLA RPC port [Enter = {port or DEFAULT_CARLA_PORT}]: ")
            if ans == "":
                port = port or DEFAULT_CARLA_PORT
                break
            if ans.isdigit() and 0 < int(ans) < 65536:
                port = int(ans)
                break
            print("[cosim] enter a port number.")
    return cfg, host, port


def _bind_app(rec, apps, ctx):
    """Resolve the setup's app id to its manifest entry and stage its yamls.
    Called whenever the record's app could have changed - on load and on edit -
    so `ctx` always describes the app the record currently names."""
    app = app_catalog.find_app(apps, rec.get("app")) if rec.get("app") else None
    if rec.get("app") and apps and app is None:
        print(f"[cosim] setup names app '{rec['app']}', which is not in "
              f"{app_catalog.catalog_path()}; continuing without it.")
    ctx["app"] = app
    ctx["staged"] = app_catalog.stage_configs(app) if app else []
    return app


def _apply_cli(rec, args):
    """Overlay explicitly passed flags onto the setup. This is what makes
    `run_cosim --map atlanta` change exactly one thing: the flag wins over what was
    saved, everything else in the setup is untouched, and nothing is prompted."""
    if args.no_app:
        rec["app"] = None
    elif args.app is not None:
        rec["app"] = args.app
    if args.map is not None:
        rec["map"], rec["map_origin"], rec["map_local"] = args.map, None, None
    if args.config is not None:
        rec["config"], rec["config_scope"] = args.config, "map"
    if args.engine is not None:
        rec["engine"] = args.engine
    if args.carla_host is not None:
        rec["carla_host"] = args.carla_host
    if args.carla_port is not None:
        rec["carla_port"] = args.carla_port
    if args.sumo_gui is not None:
        rec["sumo_gui"] = args.sumo_gui
    if args.step_length is not None:
        rec["step_length"] = args.step_length


def _edit_slots(slots, rec, ctx, cfg, apps, catalog, repo, tag_prefix):
    """Run the editor for each requested slot. Always in SLOT_KEYS order, so a
    dependency is settled before the thing that depends on it (pick the app, then
    its map, then the scenario for that pairing). Returns the CARLA config, which
    the CARLA editor may have replaced."""
    import import_map
    for slot in run_profile.SLOT_KEYS:
        if slot not in slots:
            continue
        if slot == "app":
            app = app_catalog.choose_app(apps) if apps else None
            rec["app"] = app["id"] if app else None
            _bind_app(rec, apps, ctx)
            if app:
                print(f"[cosim] app: {app['title']}")
                if app.get("note"):
                    print(f"[cosim]   {app['note']}")
                # App defaults sit under the CLI and the saved setup, above the
                # built-ins - so switching app brings its preferred engine with it.
                for key in ("engine", "sumo_gui", "step_length"):
                    if app["defaults"].get(key) is not None:
                        rec[key] = app["defaults"][key]
        elif slot == "map":
            app = ctx.get("app")
            target, tag, local = import_map.choose_map(
                repo, tag_prefix, cfg.get("carla_root") if cfg else None,
                catalog=catalog, preferred=(app or {}).get("maps"),
                app_label=(app or {}).get("title"))
            ent = import_map.catalog_entry(catalog, target)
            rec["map"] = ent["map_name"] if ent else target
            rec["map_origin"] = ("Digital-Twin-Library" if tag else
                                 "local file" if local else "cooked")
            rec["map_local"] = local
            ctx["picked_local"] = local
            # A map chosen from the menu right now is the only case where "this is
            # already cooked - reimport it?" is worth asking; a replayed one is not.
            ctx["picked_now"] = True
            print(f"[cosim] selected map: {rec['map']}")
        elif slot == "config":
            if not rec.get("map"):
                continue                      # nothing to scope a scenario to yet
            rec["config"], rec["config_scope"] = edit_config(
                ctx.get("staged"), (ctx.get("app") or {}).get("title"),
                rec["map"], rec.get("app") or run_profile.GENERIC)
            # Pin the bridge the first time a config is chosen, so the summary
            # states what will actually run instead of leaving it to be derived.
            if rec.get("engine") is None:
                rec["engine"] = (declared_engine(ctx.get("staged"), rec["config"])
                                 or read_backend(rec["config"]))
        elif slot == "engine":
            rec["engine"] = edit_engine(rec.get("engine"))
        elif slot == "carla":
            cfg, host, port = edit_carla(cfg, rec.get("carla_host"), rec.get("carla_port"))
            rec["carla_host"], rec["carla_port"] = host, port
        elif slot == "sumo":
            rec["sumo_gui"], rec["step_length"] = edit_sumo(
                rec.get("sumo_gui", True), rec.get("step_length", DEFAULT_STEP_LENGTH))
    return cfg


def configure_run(args, cfg, repo, tag_prefix, catalog):
    """Decide everything this run needs, looping until the user confirms.

    Opens on the list of saved setups (or straight on one, when --profile / --app
    names it), then on that setup's settings, which are edited and redrawn until
    the user runs it. Every one of the six is a pure decision - no map is
    downloaded, cooked or loaded while you are still deciding what to run - so the
    loop is cheap and nothing is half-done if you quit out of it.

    Returns (app, setup_name, staged_configs, rec, cfg, ctx)."""

    apps = [] if args.no_app else app_catalog.load_catalog()
    interactive = sys.stdin.isatty()
    doc = run_profile.load_doc()
    ctx = {"app": None, "staged": [], "picked_local": None, "picked_now": False}

    # 1. Which saved setup do we open on?
    name, rec = None, None
    if not args.fresh:
        if args.profile:
            name = args.profile
            stored = (doc["setups"] or {}).get(name)
            rec = dict(stored) if stored else None
            if rec is None:
                print(f"[cosim] no saved setup '{name}'; starting a new one.")
                name = None
        elif args.app:
            # A pinned app skips the list: reuse that app's most recent setup.
            for n in run_profile.order(doc):
                if (doc["setups"][n].get("app") or "") == args.app:
                    name, rec = n, dict(doc["setups"][n])
                    break
        else:
            picked = run_profile.choose_setup(doc, interactive)
            if picked == run_profile.QUIT:
                sys.exit("[cosim] cancelled.")
            if picked:
                name, rec = picked, dict(doc["setups"][picked])
    elif doc["setups"]:
        print("[cosim] --fresh: ignoring the saved setups and starting a new one.")

    # 2. Open it, edit it, confirm it. `dirty` decides whether running asks for a
    #    name: an untouched setup keeps its own, an edited one offers to fork.
    dirty = rec is None
    while True:
        if rec is None:
            rec = {"app": None, "sumo_gui": True, "step_length": DEFAULT_STEP_LENGTH}
            _apply_cli(rec, args)
            _bind_app(rec, apps, ctx)
            # A new setup has no answer for these three, and no default worth
            # guessing; the other three come from the yaml / carla.json / built-ins.
            pending = {s for s in MUST_ANSWER if not rec.get(s)}
            dirty = True
        else:
            _apply_cli(rec, args)
            _bind_app(rec, apps, ctx)
            pending = set()

        while True:
            if pending:
                cfg = _edit_slots(pending, rec, ctx, cfg, apps, catalog, repo, tag_prefix)
                dirty = True
                pending = set()
            action = run_profile.ask(name or "(unsaved)", rec, cfg, interactive,
                                     can_switch=bool(doc["setups"]))
            if action == run_profile.QUIT:
                sys.exit("[cosim] cancelled.")
            if action == run_profile.RUN:
                name = run_profile.name_setup(doc, name, rec.get("app"), dirty,
                                              interactive)
                _rec_to_args(rec, args)
                return ctx.get("app"), name, ctx.get("staged"), rec, cfg, ctx
            if action == run_profile.NEW:
                name, rec = None, None
                break
            if action == run_profile.SWITCH:
                picked = run_profile.choose_setup(doc, interactive)
                if picked == run_profile.QUIT:
                    sys.exit("[cosim] cancelled.")
                name = picked
                rec = dict(doc["setups"][picked]) if picked else None
                dirty = rec is None
                break
            pending = action          # a set of slot keys to edit, then redraw


def _rec_to_args(rec, args):
    """Publish the settled setup onto args, which the rest of the run reads."""
    args.app = rec.get("app")
    args.map = rec.get("map")
    args.config = rec.get("config")
    args.engine = rec.get("engine")
    args.carla_host = rec.get("carla_host")
    args.carla_port = rec.get("carla_port")
    args.sumo_gui = bool(rec.get("sumo_gui", True))
    args.step_length = rec.get("step_length") or DEFAULT_STEP_LENGTH


def edit_config(staged, app_title, map_name, setup_app_id):
    """Pick the scenario yaml, from every candidate that exists for this pairing.

    Two kinds are legitimate and both are listed together, because from where the
    user sits they are simply "which config do I run":
      app-owned  the app's own yamls, staged in ~/.fixs/apps/<id>/ - they carry
                 that application's subscriptions and endpoints and are valid on
                 whichever map it runs against
      per-map    ~/.fixs/apps/<app>/maps/<map>/*.yaml - generated for this app on
                 this map, plus any variant the user dropped beside it
    Returns (path, scope). The scope matters downstream twice: an app-owned yaml is
    AUTHORED, so the generator must never overwrite it, and only a per-map one is
    invalidated when the map changes."""
    import import_map
    # One-shot: lift pre-app yamls out of ~/.fixs/maps/<map>/ into the app tree.
    app_catalog.migrate_scenarios(setup_app_id, map_name,
                                  import_map._map_cache_dir(map_name))
    per_map_dir = app_catalog.scenario_dir(setup_app_id, map_name)
    generated = app_catalog.scenario_path(setup_app_id, map_name)
    try:
        found = sorted(os.path.join(per_map_dir, f) for f in os.listdir(per_map_dir)
                       if f.lower().endswith((".yaml", ".yml")))
    except OSError:
        found = []

    options = [(c["path"], f"{os.path.basename(c['path']):<42} {c['title']}"
                           + (f"  [engine {c['engine']}]" if c["engine"] else ""))
               for c in staged]
    for p in found:
        options.append((p, f"{os.path.basename(p):<42} for this app on this map"))
    if generated not in found:
        options.append((generated, f"{os.path.basename(generated):<42} "
                                   f"auto-generate for this map"))

    who = f"{app_title} on '{map_name}'" if app_title else f"'{map_name}'"
    app_paths = {c["path"] for c in staged}
    if len(options) == 1:
        return options[0][0], ("app" if options[0][0] in app_paths else "map")
    picked = _menu(f"Scenario config for {who}:", options)
    print(f"[cosim] scenario config: {picked}")
    return picked, ("app" if picked in app_paths else "map")


def declared_engine(staged, config_yaml):
    """The bridge an app declares its yaml is written for, or None.

    Needed because a hand-written app yaml usually omits
    CarlaSetup.EnablePythonBackend, and ConfigHelper defaults that to true - so a
    yaml built for TrafficLayer + VirCarlaEnv would silently run the python bridge
    instead. The app says which stack it means; --engine still overrides."""
    for c in staged or []:
        if c["engine"] and os.path.normcase(c["path"]) == os.path.normcase(config_yaml or ""):
            return c["engine"]
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--app", default=None,
                    help="application to run, by id, from the app repo's apps/apps.json. "
                         "Omit it and you pick from the declared apps; the choice "
                         "pre-selects that app's map(s) and scenario yamls.")
    ap.add_argument("--no-app", action="store_true",
                    help="ignore apps/apps.json entirely (generic, map-only co-sim)")
    ap.add_argument("--profile", default=None,
                    help="run setup to reuse from ~/.fixs/run_profiles.json (default: the "
                         "one saved for the chosen app, else whatever ran last)")
    ap.add_argument("--fresh", action="store_true",
                    help="ignore the saved run profile and ask every question again")
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
    # default=None (not the literal default) so "was this flag given?" stays
    # answerable: an unset flag may be filled from the saved run profile, an
    # explicit one must win over it. Resolved to DEFAULT_STEP_LENGTH below.
    ap.add_argument("--step-length", type=float, default=None,
                    help="sim timestep in seconds / CARLA fixed_delta_seconds (default 0.05; "
                         "0.1 halves the ticks-per-second and is fine for traffic)")
    ap.add_argument("--quality-level", choices=["Low", "Medium", "High", "Epic"], default=None,
                    help="CARLA render quality; Low is much faster on heavy maps")
    ap.add_argument("--fast", action="store_true",
                    help="do not pace the co-sim to real time (run as fast as possible)")
    ap.add_argument("--no-net-offset", action="store_true",
                    help="zero the SUMO net offset (RoadRunner-local maps)")
    ap.add_argument("--carla-host", default=None,
                    help="CARLA RPC host (default: CarlaSetup.CarlaServerIP from "
                         "the scenario yaml, else localhost). A non-local host "
                         "implies --no-launch.")
    ap.add_argument("--carla-port", type=int, default=None,
                    help="CARLA RPC port (default: CarlaSetup.CarlaServerPort from "
                         "the scenario yaml, else 2000)")
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
    # Tri-state for the same reason as --step-length: the one-click launchers always
    # pass --sumo-gui, so a plain store_true would make "headless" unsaveable in a
    # run profile. None = not specified, fill from the profile.
    ap.add_argument("--sumo-gui", dest="sumo_gui", action="store_true", default=None,
                    help="run SUMO with its GUI (default)")
    ap.add_argument("--no-sumo-gui", dest="sumo_gui", action="store_false",
                    help="run SUMO headless")
    ap.add_argument("--engine", choices=["py", "cpp"], default=None,
                    help="co-sim bridge: py=run_synchronization.py (default), "
                         "cpp=TrafficLayer+VirCarlaEnv (FIXS-native). Overrides the "
                         "scenario yaml's CarlaSetup.Backend.")
    ap.add_argument("--config", default=None,
                    help="[cpp] scenario yaml for the native bridge (default: "
                         "~/.fixs/maps/<cooked>/config.yaml, generated if missing).")
    ap.add_argument("--no-update-check", action="store_true",
                    help="skip the FIXS-bundle freshness check against the release")
    ap.add_argument("--sumo-no-start", action="store_true",
                    help="[cpp] launch sumo-gui but omit --start, so it opens loaded "
                         "and waits for you to press Play (overrides SumoSetup.AutoStart)")
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

    import import_map
    repo, tag_prefix = import_map.resolve_map_source(args.repo, args.tag_prefix)
    catalog = import_map.fetch_catalog(repo)

    # Everything the run needs - application, map, scenario yaml, bridge, CARLA,
    # SUMO - is decided here, from a saved setup you pick and edit until you
    # confirm. Deliberately AFTER maybe_reexec: it prompts, and re-execing under
    # the env python afterwards would ask the same questions twice. And
    # deliberately BEFORE anything heavy: no download, cook or load happens while
    # the user is still deciding what to run, so quitting out of it leaves nothing
    # half-done.
    app, setup_name, staged_configs, setup, cfg, ctx = configure_run(
        args, cfg, repo, tag_prefix, catalog)

    def cached_sumo_dir(name):
        """An already-extracted ~/.fixs/maps/<name>/sumo, or None.

        Consulted at EVERY site that would otherwise reach for the map bundle,
        because opening the bundle is not free: download_release_zip prompts
        "[U]se it / [R]e-download" over a ~380MB archive that a map with its
        sumo/ already extracted would immediately throw away. There is more than
        one such site - the source-build preflight, and the SUMO slot below that
        also runs for --no-launch / packaged builds - and fixing only one of them
        just moves the prompt. --sumocfg and --reimport deliberately bypass it:
        one supplies the scenario outright, the other means "refresh from the
        bundle"."""
        if args.sumocfg is not None or args.reimport:
            return None
        found = import_map.map_sumo_dir(name)
        if found:
            print(f"[cosim] using cached SUMO scenario for '{name}': "
                  f"{import_map.bundle_sumocfg(found)}")
        return found

    # Two slots to fill: a CARLA map (to cook + load) and a SUMO scenario. A
    # Digital-Twin-Library bundle fills both. The map itself was settled above; what
    # is derived here is how to GET it - the release to download, the per-map
    # settings - which comes from the catalog by name and so never has to be stored
    # in the setup or re-asked.
    target_map = args.map
    if not target_map:
        sys.exit("[cosim] no map chosen; pass --map or pick one when prompted.")
    ent = import_map.catalog_entry(catalog, target_map)
    settings = ent.get("settings", {}) if ent else {}
    picked_tag = ent["release"] if ent else None      # DT release to (lazily) download
    if ent and ent["map_name"] != target_map:
        print(f"[cosim] map '{target_map}' -> '{ent['map_name']}'  "
              f"(DT release {picked_tag})")
        target_map = ent["map_name"]
    map_origin = setup.get("map_origin") or ("Digital-Twin-Library" if ent else "cooked")
    # A local .zip/folder pick cannot be re-derived from a name, so it rides along
    # in the setup; it is only reused while the file is still there.
    picked_local = ctx.get("picked_local") or setup.get("map_local")
    if picked_local and not os.path.exists(picked_local):
        picked_local = None
    # Was the map chosen from the menu on THIS run? Only then is "you just picked a
    # map that is already cooked - reimport it?" worth asking. A replayed setup is a
    # deliberate re-run, and prompting about re-cooking it every time is noise.
    picked_now = bool(ctx.get("picked_now"))

    # Per-map settings are defaults; explicit CLI flags override.
    no_net_offset = args.no_net_offset or settings.get("net_offset") == "zero"
    tls_manager = args.tls_manager or settings.get("tls_manager") or "sumo"

    sumo_dir = None              # dir holding the chosen bundle's .sumocfg (set on open)
    # /Game/... path to boot CARLA into (set by the source-build preflight). None
    # (packaged build / --no-launch) = let the engine pick.
    target_level = None

    # Is CARLA on this machine? Decided BEFORE the source-build preflight below,
    # because everything that preflight does - cooking the package into carla_root
    # via Util/BuildTools/Import.py, placing TLs/signs through the local UE4 editor
    # - writes to a LOCAL CARLA install and is wasted (minutes of cooking) when the
    # server we will actually talk to is on another host. Best-effort by design: a
    # map with no yaml yet cannot name a remote CARLA, and the yaml we are about to
    # generate for it will name a local one. The authoritative host/port resolution
    # still happens after config_yaml is final.
    if args.carla_host is None:
        peek = args.config or app_catalog.scenario_path(
            setup.get("app") or app_catalog.GENERIC, target_map)
        if os.path.isfile(peek):
            peek_host, _peek_port = read_carla_endpoint(peek)
            if peek_host and not _is_local_host(peek_host):
                args.carla_host = peek_host
    if args.carla_host and not _is_local_host(args.carla_host) and not args.no_launch:
        print(f"[cosim] CARLA host is {args.carla_host} (not this machine); "
              f"implying --no-launch. NOTE: importing/cooking the map and placing "
              f"TLs/signs are local-only operations - that CARLA must already have "
              f"'{target_map}' cooked with TLs/signs placed.")
        args.no_launch = True

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
        if resolved is not None and picked_now and (picked_tag or picked_local) \
                and not args.reimport and sys.stdin.isatty():
            ans = input(f"[cosim] '{resolved[0]}' is already imported. Reimport "
                        f"(re-cook + re-place TLs/signs + regen TL table)? [y/N]: ").strip().lower()
            if ans.startswith("y"):
                args.reimport = True
                resolved = None

        # The bundle fills two slots - the CARLA package to cook, and the SUMO
        # scenario - so check what is actually still missing before touching it.
        if sumo_dir is None:
            sumo_dir = cached_sumo_dir(target_map)

        # download_release_zip caches under ~/.fixs/maps; open_bundle splits it.
        carla_src = None
        need_bundle = (resolved is None                          # must cook the map
                       or (args.sumocfg is None and sumo_dir is None))   # need its sumo/
        if need_bundle and (picked_local or picked_tag):
            zip_path = picked_local or import_map.download_release_zip(
                repo, picked_tag, force_redownload=args.reimport, cache_name=target_map)
            carla_src, bundle_sumo = import_map.open_bundle(zip_path, cache_name=target_map)
            if bundle_sumo:            # keep a cached sumo/ if this bundle has none
                sumo_dir = bundle_sumo

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

    # SUMO slot: --sumocfg wins; else an already-extracted sumo/, else the chosen
    # bundle's. This also runs for the paths that skip the source-build preflight
    # above (--no-launch, packaged builds), so the cache is checked here too - the
    # bundle is the LAST resort, not the first.
    sumocfg = args.sumocfg
    if sumocfg is None:
        if sumo_dir is None:
            sumo_dir = cached_sumo_dir(target_map)
        if sumo_dir is None and (picked_local or picked_tag):
            zip_path = picked_local or import_map.download_release_zip(
                repo, picked_tag, cache_name=target_map)
            _carla, sumo_dir = import_map.open_bundle(zip_path, cache_name=target_map)
        sumocfg = import_map.bundle_sumocfg(sumo_dir)
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

    # The scenario yaml, written like tl_table.csv: always, on first use (refreshed
    # by --reimport). Deliberately BEFORE the CARLA launch and the --prep-only
    # return - it is a config artifact, so it must not depend on a running CARLA.
    #
    # Every scenario yaml is APP-BOUNDED, under ~/.fixs/apps/<app>/, because yamls
    # are edited and ~/.fixs/maps/ is a deletable cache of downloaded artifacts.
    # Two kinds live there, and config_scope records which one won:
    #   'app'  the app's own yaml, staged from the repo - map-independent, AUTHORED,
    #          so the generator below must never touch it
    #   'map'  apps/<app>/maps/<map>/config.yaml - generated for this app on this map
    # The scope also tells the run profile whether changing the map invalidates the
    # yaml choice.
    config_yaml = args.config or app_catalog.scenario_path(
        setup.get("app") or app_catalog.GENERIC, target_map)
    config_scope = setup.get("config_scope") or "map"

    if config_scope == "map" and (args.reimport or not os.path.isfile(config_yaml)):
        # A regenerate must not silently undo hand edits: carry the existing
        # Backend choice over, and keep the old file as .bak to fall back on.
        prior = read_backend(config_yaml) if os.path.isfile(config_yaml) else None
        if prior:
            import shutil as _sh
            _sh.copy2(config_yaml, config_yaml + ".bak")
            print(f"[cosim] regenerating {os.path.basename(config_yaml)} "
                  f"(previous kept as .bak; Backend={prior} preserved)")
        generate_config_yaml(config_yaml, tl_table,
                             args.carla_host or DEFAULT_CARLA_HOST,
                             args.carla_port or DEFAULT_CARLA_PORT,
                             realtime=not args.fast,
                             refresh=args.step_length,
                             backend=args.engine or prior or "py")

    # Which bridge runs. Resolved once, here, rather than at the dispatch below, so
    # the saved profile records the bridge that ACTUALLY ran instead of the raw
    # (usually empty) --engine flag. An app may declare which stack its yaml is
    # written for; --engine still wins over everything.
    backend = args.engine or declared_engine(staged_configs, config_yaml) \
        or read_backend(config_yaml)
    args.engine = backend

    # CARLA RPC endpoint: the scenario yaml is the source of truth, because that
    # is what VirCarlaEnv dials. An explicit --carla-host/--carla-port still wins
    # (and seeds the yaml above when it is first generated), but otherwise the two
    # must not be allowed to drift - that is how you get run_cosim reporting a
    # healthy CARLA while VirCarlaEnv times out against a different address.
    yaml_host, yaml_port = read_carla_endpoint(config_yaml)
    if args.carla_host is None and yaml_host:
        args.carla_host = yaml_host
    if args.carla_port is None and yaml_port:
        args.carla_port = yaml_port
    args.carla_host = args.carla_host or DEFAULT_CARLA_HOST
    args.carla_port = args.carla_port or DEFAULT_CARLA_PORT

    # Catches the case the early peek could not see: --config picked a different
    # yaml, or the file only existed after generation. Same rule as above - a CARLA
    # we cannot reach the filesystem of is one we can only connect to.
    if not _is_local_host(args.carla_host) and not args.no_launch:
        print(f"[cosim] CARLA host is {args.carla_host} (not this machine); "
              f"implying --no-launch.")
        args.no_launch = True

    # Everything is decided: save the setup under its name, so the next run opens on
    # it. Saved BEFORE the launch on purpose - a setup that failed to start is
    # exactly the one you want to come back to and tweak one setting of. The values
    # written are the RESOLVED ones (the real cooked map name, the bridge that will
    # actually run), not the raw flags, so replaying it reproduces this run.
    setup.update({
        "app": app["id"] if app else None,
        "map": target_map,
        "map_origin": map_origin,
        "map_local": picked_local,
        "config": config_yaml,
        "config_scope": config_scope,
        "engine": backend,
        "sumo_gui": bool(args.sumo_gui),
        "step_length": args.step_length,
        "carla_host": args.carla_host,
        "carla_port": args.carla_port,
    })
    run_profile.save(setup_name, setup)
    args.map = target_map

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
        # Engine dispatch: CarlaSetup.Backend in that yaml picks the bridge
        # (--engine overrides). cpp = FIXS-native (TrafficLayer + VirCarlaEnv); py
        # (default) = the standalone run_synchronization.py bridge below.
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
