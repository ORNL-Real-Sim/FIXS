"""
Subscription comparison harness for issue #176.

For each subscription variant (ego_only / radius / all) this script:
  1. launches TrafficLayer.exe with the matching config (which auto-launches SUMO
     in-process via libsumo on the multi-vehicle loop),
  2. runs simple_echo_client.py as a pure observer for a fixed number of steps,
  3. records the peak vehicle count the client received,
  4. tears TrafficLayer down before the next variant.

It then prints a comparison table. Use it to:
  - BEFORE the fix: confirm the bug (all == ego_only; only radius differs).
  - AFTER the fix:  confirm all == vehicles-in-network > radius > ego_only.

Run from this directory:
    <realsim_python> run_subscription_compare.py
"""

import argparse
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
TRAFFICLAYER_EXE = os.path.join(REPO_ROOT, 'TrafficLayer', 'x64', 'Release', 'TrafficLayer.exe')
LIBSUMO_BIN = os.path.join(REPO_ROOT, 'CommonLib', 'libsumo', 'bin')
ECHO_CLIENT = os.path.join(HERE, 'simple_echo_client.py')
SUMO_CFG = os.path.join(HERE, 'simple_loop_traffic.sumocfg')
SUMO_HOME = os.environ.get('SUMO_HOME', r'C:\Program Files (x86)\Eclipse\Sumo')
SUMO_BIN = os.path.join(SUMO_HOME, 'bin', 'sumo.exe')   # headless sumo (no GUI window)

VARIANTS = [
    ('ego_only', 'config_ego_only.yaml'),
    ('radius',   'config_radius.yaml'),
    ('all',      'config_all.yaml'),
]

SUMO_PORT = 1337
SERVER_PORT = 2444


def kill_sim_processes():
    """Tear down any external SUMO / TrafficLayer processes between variants.
    TrafficLayer launches sumo-gui as a separate process, so terminating
    TrafficLayer alone leaves the GUI (and port 1337) behind. A stale sumo-gui
    holding port 1337 causes the next variant's TrafficLayer to deadlock on the
    TraCI handshake, so we kill both images here and then wait for the ports."""
    for image in ('sumo-gui.exe', 'sumo.exe', 'TrafficLayer.exe'):
        subprocess.run(['taskkill', '/F', '/IM', image],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def port_free(port):
    """True if nothing is listening on 127.0.0.1:port (connect refused)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.5)
    try:
        s.connect(('127.0.0.1', port))
        return False   # something accepted -> still in use
    except OSError:
        return True
    finally:
        s.close()


def wait_ports_free(ports, timeout):
    """Block until all given ports are free, or timeout. Returns True if freed."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if all(port_free(p) for p in ports):
            return True
        time.sleep(0.5)
    return False


def run_variant(name, config, steps, warmup, startup_wait):
    print(f'\n=== Variant: {name}  ({config}) ===', flush=True)

    env = dict(os.environ)
    env['PATH'] = LIBSUMO_BIN + os.pathsep + env.get('PATH', '')

    # 1) Launch headless SUMO ourselves and own its lifecycle (configs set
    #    EnableAutoLaunch: false). --start --num-clients 1 makes sumo open the TraCI
    #    port and wait for TrafficLayer to connect before stepping. No GUI window.
    sumo_log = open(os.path.join(HERE, f'_sumo_{name}.log'), 'w')
    sumo = subprocess.Popen(
        [SUMO_BIN, '-c', SUMO_CFG, '--remote-port', str(SUMO_PORT),
         '--num-clients', '1', '--step-length', '0.1', '--start'],
        cwd=HERE, stdout=sumo_log, stderr=subprocess.STDOUT,
    )
    # Give sumo a moment to open the TraCI listen socket. Do NOT probe port 1337 — a
    # probe connection would be consumed as the single TraCI client.
    time.sleep(2)

    # 2) Launch TrafficLayer (connects to the echo client on 2444, then to sumo on 1337).
    tl_log = open(os.path.join(HERE, f'_trafficlayer_{name}.log'), 'w')
    tl = subprocess.Popen(
        [TRAFFICLAYER_EXE, '-f', config],
        cwd=HERE, env=env, stdout=tl_log, stderr=subprocess.STDOUT,
    )

    result = {'name': name, 'config': config, 'max_vehicles': None, 'distinct_total': None, 'ids': ''}
    try:
        # TrafficLayer waits for the echo client to connect before it connects to sumo, so
        # the echo client must be the first/only connection on 2444. Do NOT probe 2444
        # (that would be consumed as client #1). Give TrafficLayer time to open its socket;
        # the client itself retries the connection.
        time.sleep(startup_wait)

        client = subprocess.run(
            [sys.executable, ECHO_CLIENT,
             '--config', config, '--steps', str(steps), '--warmup', str(warmup),
             '--report'],
            cwd=HERE, capture_output=True, text=True,
        )
        for line in client.stdout.splitlines():
            print('  ' + line, flush=True)
            if line.startswith('RESULT'):
                for tok in line.split():
                    if tok.startswith('max_vehicles='):
                        result['max_vehicles'] = int(tok.split('=', 1)[1])
                    elif tok.startswith('distinct_total='):
                        result['distinct_total'] = int(tok.split('=', 1)[1])
                    elif tok.startswith('ids='):
                        result['ids'] = tok.split('=', 1)[1]
        if client.returncode != 0 and client.stderr:
            print('  [client stderr]\n  ' + client.stderr.replace('\n', '\n  '), flush=True)
    finally:
        for proc in (tl, sumo):
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
        tl_log.close()
        sumo_log.close()
        kill_sim_processes()
        # Wait until SUMO (1337) and the client port (2444) are actually free before the
        # next variant, so a lingering listener can't deadlock the next TraCI handshake.
        if not wait_ports_free([SUMO_PORT, SERVER_PORT], timeout=20):
            print('  [WARN] ports 1337/2444 still busy after teardown; next variant may stall', flush=True)
        time.sleep(2)

    return result


def main():
    p = argparse.ArgumentParser(description='Compare vehicle visibility across subscription variants (#176)')
    p.add_argument('--steps', type=int, default=300, help='steps per variant (0.1s each; 300 = 30 s)')
    p.add_argument('--warmup', type=int, default=250, help='steps to skip before counting (let traffic insert)')
    p.add_argument('--startup-wait', type=int, default=5, help='seconds to let TrafficLayer open its listen socket before the client connects')
    p.add_argument('--only', default=None, help='comma-separated subset of variants to run (ego_only,radius,all)')
    args = p.parse_args()

    variants = VARIANTS
    if args.only:
        wanted = set(args.only.split(','))
        variants = [v for v in VARIANTS if v[0] in wanted]

    if not os.path.exists(TRAFFICLAYER_EXE):
        print(f'ERROR: TrafficLayer.exe not found at {TRAFFICLAYER_EXE}', file=sys.stderr)
        print('Build it first: scripts\\dispatch\\2_core_components.bat', file=sys.stderr)
        return 2

    # Pre-clean any leftover processes from a prior (possibly interrupted) run
    kill_sim_processes()
    wait_ports_free([SUMO_PORT, SERVER_PORT], timeout=20)

    results = [run_variant(name, cfg, args.steps, args.warmup, args.startup_wait) for name, cfg in variants]

    print('\n================ SUBSCRIPTION COMPARISON (#176) ================')
    print(f'{"variant":<12}{"max_vehicles":>14}{"distinct_total":>16}   ids')
    for r in results:
        print(f'{r["name"]:<12}{str(r["max_vehicles"]):>14}{str(r["distinct_total"]):>16}   {r["ids"]}')
    print('================================================================')

    counts = {r['name']: r['max_vehicles'] for r in results}
    if None in counts.values():
        print('\nINCONCLUSIVE: a variant produced no RESULT (see _trafficlayer_*.log).')
        return 1

    if not {'all', 'radius', 'ego_only'} <= set(counts):
        print('\n(Partial run via --only; skipping the comparison verdict.)')
        return 0

    if counts['all'] > counts['radius'] > counts['ego_only']:
        print("\nFIX CONFIRMED: all > radius > ego_only (the all flag now subscribes every vehicle).")
    elif counts['all'] <= counts['radius']:
        print("\nBUG REPRODUCED: 'all' is not the largest set — the all flag is a no-op on the SUMO path "
              f"(all={counts['all']}, radius={counts['radius']}, ego_only={counts['ego_only']}).")
    else:
        print('\nUNEXPECTED ordering — inspect the per-variant logs.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
