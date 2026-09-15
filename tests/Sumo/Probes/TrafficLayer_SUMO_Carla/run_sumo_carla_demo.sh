#!/usr/bin/env bash
# =============================================================================
# run_sumo_carla_demo.sh -- Linux counterpart of run_sumo_carla_demo.bat (#65)
#
#   SUMO --[TraCI]--> TrafficLayer --[VehFullData]--> VirCarlaEnv --> CARLA
#
# Click-to-run: no arguments brings CARLA up, loads the map, runs the co-sim and
# takes it all down. CARLA bring-up/teardown lives in Carla/launch_carla.sh, the
# counterpart of Carla\launch_carla.bat, so this file stays about the demo.
#
# Differences from the .bat, all forced by the platform:
#   * CARLA gets its OWN port (2100), not the conventional 2000, and only a
#     server this script started is stopped. A box that develops Carla usually
#     has a session on 2000; spawning 40 vehicles into it -- or killing it --
#     would be a nasty surprise. --carla-port at something already listening
#     attaches instead: no map load, no shutdown.
#   * windows when there is a $DISPLAY, headless when there is not, so the same
#     script works over ssh and on a runner. --headless forces that.
#   * the config is staged, never edited: auto-launch off (this script owns the
#     SUMO process), our CARLA port, and any privileged port moved (below).
#
# Usage:
#   ./run_sumo_carla_demo.sh                    # runs to SimulationEndTime
#   ./run_sumo_carla_demo.sh --duration 30      # wall-clock cap, for a smoke run
#   ./run_sumo_carla_demo.sh --headless         # no windows (ssh / CI)
#   ./run_sumo_carla_demo.sh --carla-port 2000  # attach to a running server
# =============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../../.." && pwd)"
CONFIG="config.yaml"
# Empty = run until the co-simulation ENDS ON ITS OWN, i.e. when SimulationEndTime
# in the config is reached. --duration is a wall-clock CAP for smoke runs; it was
# the default once, which meant this script -- not the config -- decided when a
# demo stopped, and 20 s of a 1000 s scenario looked like the run had been cut off.
DURATION=""
CARLA_PORT=2100
HEADLESS=0; [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] || HEADLESS=1
CARLA_ROOT_ARG=()

while [ $# -gt 0 ]; do
    case "$1" in
        --config)     CONFIG="$2";     shift 2 ;;
        --duration)   DURATION="$2";   shift 2 ;;
        --carla-port) CARLA_PORT="$2"; shift 2 ;;
        --carla-root) CARLA_ROOT_ARG=(--carla-root "$2"); shift 2 ;;
        --headless)   HEADLESS=1;      shift ;;
        -h|--help)    sed -n '2,26p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
    esac
done

cd "$TEST_DIR" || exit 1
die() { echo "ERROR: $*" >&2; exit 1; }

# Empty if coreutils' stdbuf is missing: the run still works, the live view
# just lags. Never a reason to refuse to run.
STDBUF=""; command -v stdbuf >/dev/null 2>&1 && STDBUF="stdbuf -oL -eL"

TL="$REPO_ROOT/build/TrafficLayer"
VCE="$REPO_ROOT/build/VirCarlaEnv"
SUMOCFG="$REPO_ROOT/tests/Sumo/networks/simple_loop/simple_loop_ego.sumocfg"
XODR="$REPO_ROOT/tests/Vissim/SimpleEcho/simple_loop.xodr"
LAUNCH="$REPO_ROOT/Carla/launch_carla.sh"
for f in "$TL" "$VCE"; do
    [ -x "$f" ] || die "$(basename "$f") not built at $f
       Build it:  $REPO_ROOT/scripts/dispatch/dispatch.sh --with-carla"
done
[ -f "$SUMOCFG" ] || die "SUMO scenario missing: $SUMOCFG"

# sumo-gui when we have a display, headless sumo otherwise. Prefer the PINNED
# server over PATH: a distro or PPA sumo is a different version from the
# libtraci client we link.
if [ "$HEADLESS" -eq 0 ] && command -v sumo-gui >/dev/null 2>&1; then
    SUMO_BIN="$(command -v sumo-gui)"
elif [ -x "$REPO_ROOT/CommonLib/libsumo/bin/sumo" ]; then
    SUMO_BIN="$REPO_ROOT/CommonLib/libsumo/bin/sumo"
else
    SUMO_BIN="$(command -v sumo)" || die "no SUMO server found. Build the pinned one:
       $REPO_ROOT/scripts/fetch_native_deps.sh --with-server"
    echo "WARNING: using $SUMO_BIN from PATH; it may not match the pinned client version."
fi

TRAFFIC_PORT="$(grep -E '^[[:space:]]*TrafficSimulatorPort:' "$CONFIG" | head -1 | tr -d " '\r" | cut -d: -f2)"
APP_PORT="$(grep -E '^[[:space:]]*CarlaClientPort:' "$CONFIG" | head -1 | tr -d " '\r" | cut -d: -f2)"
: "${TRAFFIC_PORT:=1337}" "${APP_PORT:=440}"

# Ports below 1024 need root on Linux, so the committed 440 makes TrafficLayer
# die with "bind() failed: Permission denied". Move it in BOTH places --
# ApplicationSetup port and CarlaSetup.CarlaClientPort are the two ends of one
# socket, and moving only one produces a bridge that connects to nothing.
NEW_APP_PORT="$APP_PORT"
if [ "$APP_PORT" -lt 1024 ]; then
    NEW_APP_PORT=$((APP_PORT + 4000))
    echo "note: app port $APP_PORT is privileged on Linux; staging config on $NEW_APP_PORT"
fi

# --- CARLA --------------------------------------------------------------------
OWN_CARLA=0
if timeout 3 bash -c "</dev/tcp/127.0.0.1/$CARLA_PORT" 2>/dev/null; then
    echo "note: attaching to the CARLA on $CARLA_PORT -- not loading a map into it,"
    echo "      not stopping it, and vehicles land at SimpleLoop coordinates"
    echo "      whatever world it has loaded"
else
    HL=(); [ "$HEADLESS" -eq 1 ] && HL=(--headless)
    "$LAUNCH" --port "$CARLA_PORT" "${HL[@]}" "${CARLA_ROOT_ARG[@]}" || exit 1
    OWN_CARLA=1
fi

STAGED="$(mktemp --suffix=.yaml)"
sed -e 's/^\([[:space:]]*\)EnableAutoLaunch:.*/\1EnableAutoLaunch: false/' \
    -e "s/^\([[:space:]]*\)CarlaServerPort:.*/\1CarlaServerPort: $CARLA_PORT/" \
    -e "s/^\([[:space:]]*\)CarlaClientPort:.*/\1CarlaClientPort: $NEW_APP_PORT/" \
    -e "s/^\([[:space:]]*\)port:[[:space:]]*\[$APP_PORT\]/\1port: [$NEW_APP_PORT]/" \
    "$CONFIG" > "$STAGED"

SUMO_PID=""; TL_PID=""; VCE_PID=""
# SIGTERM then SIGKILL: TrafficLayer parked in accept() waiting for a client
# does not act on SIGTERM (the handler raises a flag the blocked loop never
# reads), so a plain kill + wait hangs this script forever.
stop() {
    local pid="$1" label="$2" i
    [ -n "$pid" ] || return 0
    kill -TERM "$pid" 2>/dev/null || return 0
    for i in 1 2 3 4 5; do kill -0 "$pid" 2>/dev/null || return 0; sleep 1; done
    echo "  $label ignored SIGTERM, sending SIGKILL"
    kill -KILL "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
}
cleanup() {
    kill "${TAIL_TL:-}" "${TAIL_VCE:-}" 2>/dev/null
    echo; echo "--- shutting down (bridge first, so it despawns its actors) ---"
    stop "$VCE_PID" VirCarlaEnv; stop "$TL_PID" TrafficLayer; stop "$SUMO_PID" SUMO
    [ "$OWN_CARLA" -eq 1 ] && "$LAUNCH" --port "$CARLA_PORT" --stop
    rm -f "$STAGED"
}
trap cleanup EXIT INT TERM

# The map only goes into a server we own -- replacing the world of a server
# someone else is watching would be rude. simple_loop.xodr is meshed
# procedurally (no map package, no cook) and is the geometry the SUMO net uses,
# so the vehicles land on the road.
if [ "$OWN_CARLA" -eq 1 ]; then
    PY="$(sed -n 's/.*"python"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$HOME/.fixs/carla.json" 2>/dev/null | head -1)"
    [ -x "$PY" ] || PY="$(command -v python3)"
    if "$PY" -c 'import carla' 2>/dev/null; then
        echo "loading simple_loop.xodr as the world ..."
        "$PY" "$REPO_ROOT/Carla/load_opendrive_world.py" "$XODR" --port "$CARLA_PORT" \
            > loadworld.log 2>&1 || { tail -10 loadworld.log; die "loading the world failed (see loadworld.log)"; }
    else
        echo "WARNING: no python with the carla module (set \"python\" in ~/.fixs/carla.json)."
        echo "         Skipping the map load -- vehicles will not line up with the road."
    fi
fi

ENDTIME="$(grep -E '^[[:space:]]*SimulationEndTime:' "$CONFIG" | head -1 | tr -d " '\r" | cut -d: -f2)"
echo "SUMO $SUMO_BIN | TraCI $TRAFFIC_PORT | CARLA $CARLA_PORT"
if [ -n "$DURATION" ]; then echo "stops: after ${DURATION}s wall clock"
else echo "stops: at SimulationEndTime=${ENDTIME:-?} sim s"; fi
echo "[1/3] starting SUMO ..."
"$SUMO_BIN" -c "$SUMOCFG" --remote-port "$TRAFFIC_PORT" --num-clients 1 --step-length 0.1 --start > sumo.log 2>&1 &
SUMO_PID=$!
sleep 2; kill -0 "$SUMO_PID" 2>/dev/null || { tail -20 sumo.log; die "SUMO exited immediately"; }

# stdbuf -oL: a console gives cout/printf LINE buffering, a pipe or file gives
# 4 KB BLOCK buffering -- which is the whole difference between "Windows shows
# TrafficLayer's output as it happens" and a log that lags minutes behind. A
# 1000 s run showed SimTime 700 live while the process was already at 1000.
echo "[2/3] starting TrafficLayer ..."
$STDBUF "$TL" -f "$STAGED" > trafficlayer.log 2>&1 &
TL_PID=$!
sleep 3; kill -0 "$TL_PID" 2>/dev/null || { tail -25 trafficlayer.log; die "TrafficLayer exited immediately"; }

echo "[3/3] starting VirCarlaEnv ..."
$STDBUF "$VCE" -f "$STAGED" -t "$TEST_DIR/traffic_light_table.csv" > vircarlaenv.log 2>&1 &
VCE_PID=$!

# The .bat gets a console per process from `start cmd /k`. The equivalent here
# is to follow the logs in this one, prefixed so two streams in one terminal
# stay readable -- and it still works over ssh, where extra windows would not.
# sed -u for the same reason stdbuf is used above, one layer up: when this
# script's own stdout is a pipe (a tee, a grep, a CI log), an unbuffered
# producer feeding a BUFFERED sed is still invisible. A short run wrote 52
# lines and showed none of them.
tail -n +1 -f trafficlayer.log 2>/dev/null | sed -u 's/^/[TL ] /' &
TAIL_TL=$!
tail -n +1 -f vircarlaenv.log 2>/dev/null | sed -u 's/^/[VCE] /' &
TAIL_VCE=$!

echo
if [ -n "$DURATION" ]; then
    echo "--- running (wall-clock cap ${DURATION}s; Ctrl-C to stop early) ---"
    sleep "$DURATION"
else
    # Wait on the BRIDGE, not on TrafficLayer: VirCarlaEnv is the process that
    # stops when the scenario ends, and TrafficLayer then follows.
    echo "--- running until SimulationEndTime (Ctrl-C to stop early) ---"
    wait "$VCE_PID"
    VCE_RC=$?
fi

# Two ways to pass, and they are different claims: capped means "still healthy
# when we stopped it", uncapped means "ran the scenario to its end and exited".
# Count the two OUTCOMES separately. 'grep -ci spawn' also matches
# "[Warning] Failed to spawn actor", so a run with one refused spawn point
# reported one MORE vehicle than it placed -- which read as cross-distro
# non-determinism when three identical runs were compared.
SPAWNS=$(grep -c 'Spawned Carla actor' vircarlaenv.log 2>/dev/null || echo 0)
FAILED=$(grep -c 'Failed to spawn' vircarlaenv.log 2>/dev/null || echo 0)
sleep 1; kill "$TAIL_TL" "$TAIL_VCE" 2>/dev/null
echo
if [ -n "$DURATION" ]; then
    if kill -0 "$VCE_PID" 2>/dev/null && [ "$SPAWNS" -gt 0 ]; then
        echo "RESULT: PASS -- $SPAWNS vehicles spawned ($FAILED refused), still running at the ${DURATION}s cap"
        exit 0
    fi
elif [ "${VCE_RC:-1}" -eq 0 ] && [ "$SPAWNS" -gt 0 ]; then
    echo "RESULT: PASS -- $SPAWNS vehicles spawned ($FAILED refused), ran to SimulationEndTime and exited cleanly"
    exit 0
fi
echo "RESULT: FAIL -- bridge exit ${VCE_RC:-still running}, $SPAWNS spawned / $FAILED refused"
echo "        see vircarlaenv.log / trafficlayer.log / sumo.log in $TEST_DIR"
exit 1
