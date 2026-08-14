#!/usr/bin/env bash
# =============================================================================
# run_sumo_carla_demo.sh -- Linux counterpart of run_sumo_carla_demo.bat (#65)
#
#   SUMO --[TraCI]--> TrafficLayer --[VehFullData @ 440]--> VirCarlaEnv --> CARLA
#
# CLICK-TO-RUN: with no arguments it brings up its own CARLA, loads the map,
# runs the co-simulation and takes everything down again. Same property as the
# .bat, reached differently in two places -- both because this has to work
# unattended on a headless box, and next to a CARLA someone else is using:
#
#   * it starts HEADLESS sumo, not sumo-gui (which needs fox/OpenGL and a
#     display), and stages a config with EnableAutoLaunch:false so TrafficLayer
#     does not try to spawn a GUI itself. The committed config.yaml is never
#     modified. --gui opts back in.
#
#   * it starts CARLA on its OWN port (2100), not the conventional 2000, and
#     stops only the server it started. A box that develops Carla usually has a
#     session on 2000 already; spawning 40 vehicles into it -- or killing it at
#     teardown -- would be a nasty surprise. Point --carla-port at a server that
#     is already listening to attach to it instead: then this script neither
#     loads a map (that would replace the world you are looking at) nor stops it.
#
# Usage:
#   ./run_sumo_carla_demo.sh                    # everything, 20 s, own server
#   ./run_sumo_carla_demo.sh --duration 60      # watch it longer
#   ./run_sumo_carla_demo.sh --render           # windowed, not RenderOffScreen
#   ./run_sumo_carla_demo.sh --carla-port 2000  # attach to a running server
#   ./run_sumo_carla_demo.sh --config config_l2.yaml --gui
# =============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../../.." && pwd)"
CONFIG="config.yaml"
DURATION=20
USE_GUI=0
CARLA_PORT=2100          # ours, deliberately not the conventional 2000
CARLA_ROOT="${CARLA_ROOT:-}"
XODR="$REPO_ROOT/tests/Vissim/SimpleEcho/simple_loop.xodr"
RENDER=0
STEP_LENGTH=0.1

while [ $# -gt 0 ]; do
    case "$1" in
        --config)     CONFIG="$2";     shift 2 ;;
        --duration)   DURATION="$2";   shift 2 ;;
        --carla-port) CARLA_PORT="$2"; shift 2 ;;
        --carla-root) CARLA_ROOT="$2"; shift 2 ;;
        --render)     RENDER=1;        shift ;;
        --gui)        USE_GUI=1;       shift ;;
        -h|--help)    sed -n '2,33p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
    esac
done

cd "$TEST_DIR" || exit 1
die() { echo "ERROR: $*" >&2; exit 1; }

# --- locate the pieces --------------------------------------------------------
TL="$REPO_ROOT/build/TrafficLayer"
VCE="$REPO_ROOT/build/VirCarlaEnv"
[ -x "$TL" ]  || die "TrafficLayer not built at $TL
       Build it:  $REPO_ROOT/scripts/dispatch/dispatch.sh --with-carla"
[ -x "$VCE" ] || die "VirCarlaEnv not built at $VCE
       It is skipped unless the CARLA client SDK is present:
         $REPO_ROOT/scripts/dispatch/dispatch.sh --with-carla"

SUMOCFG="$REPO_ROOT/tests/Sumo/networks/simple_loop/simple_loop_ego.sumocfg"
[ -f "$SUMOCFG" ] || die "SUMO scenario missing: $SUMOCFG"
TLS="$TEST_DIR/traffic_light_table.csv"

if [ "$USE_GUI" -eq 1 ]; then
    SUMO_BIN="$(command -v sumo-gui || true)"
    [ -n "$SUMO_BIN" ] || die "--gui needs sumo-gui on PATH (the pinned build is headless only)"
elif [ -x "$REPO_ROOT/CommonLib/libsumo/bin/sumo" ]; then
    # Prefer the PINNED server next to libtracicpp: a distro or PPA sumo is a
    # different version from the client we link, and TraCI refuses a mismatch.
    SUMO_BIN="$REPO_ROOT/CommonLib/libsumo/bin/sumo"
else
    SUMO_BIN="$(command -v sumo || true)"
    [ -n "$SUMO_BIN" ] || die "no SUMO server found. Build the pinned one:
       $REPO_ROOT/scripts/fetch_native_deps.sh --with-server"
    echo "WARNING: using $SUMO_BIN from PATH; it may not match the pinned client version."
fi

TRAFFIC_PORT="$(grep -E '^[[:space:]]*TrafficSimulatorPort:' "$CONFIG" | head -1 | tr -d " '\r" | cut -d: -f2)"
[ -n "$TRAFFIC_PORT" ] || TRAFFIC_PORT=1337

listening() { timeout 3 bash -c "</dev/tcp/127.0.0.1/$1" 2>/dev/null; }

# The CarlaUE4 processes serving OUR port, never this script or its parents.
# pkill -f would be shorter and is a trap: the pattern occurs in the command
# line of any shell that typed it, so a bare `pkill -f -- -carla-rpc-port=N`
# happily kills the very shell running the teardown (observed).
carla_server_pids() {
    pgrep -f -- "-carla-rpc-port=$CARLA_PORT" 2>/dev/null \
        | grep -vx -e "$$" -e "$PPID" || true
}

# --- CARLA: attach to a running server, or bring up our own -------------------
# OWN_CARLA decides teardown as well as startup: we stop only what we started,
# so attaching to someone's session can never end it.
OWN_CARLA=0
CARLA_PID=""
if listening "$CARLA_PORT"; then
    echo "note: attaching to the CARLA already listening on $CARLA_PORT"
    echo "      (not loading a map into it, and not stopping it at the end --"
    echo "       vehicles will be placed at SimpleLoop coordinates regardless of"
    echo "       which world it has loaded)"
else
    # Candidate order: --carla-root, then ~/.fixs/carla.json, then $CARLA_ROOT.
    # The FIXS config outranks the ambient variable deliberately: a box that has
    # built Carla more than once often exports CARLA_ROOT for OTHER tooling, and
    # on this one it pointed at a 0.9.14 tree while carla.json named the pinned
    # 0.9.15 -- silently launching the wrong server against a 0.9.15 client.
    #
    # Hence the version gate: every candidate must carry a VERSION matching the
    # pin in dependencies.yaml. A wrong server is worth refusing loudly, since
    # the alternative is a protocol mismatch reported as a connect timeout.
    PINNED="$(awk '/^  carla:/{i=1; next} /^  [a-z]/{i=0} i' "$REPO_ROOT/dependencies.yaml" \
              | sed -n 's/^[[:space:]]*version:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' | head -1 | tr -d '\r')"
    JSON_ROOT=""
    [ -f "$HOME/.fixs/carla.json" ] && JSON_ROOT="$(sed -n 's/.*"carla_root"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$HOME/.fixs/carla.json" | head -1)"

    PICKED=""; REJECTED=""
    for cand in "$CARLA_ROOT" "$JSON_ROOT" "${CARLA_ROOT:-}"; do
        [ -n "$cand" ] && [ -x "$cand/CarlaUE4.sh" ] || continue
        cver="$(cat "$cand/VERSION" 2>/dev/null | tr -d ' \r\n')"
        if [ -z "$PINNED" ] || [ -z "$cver" ] || [ "$cver" = "$PINNED" ]; then
            PICKED="$cand"; break
        fi
        REJECTED="$REJECTED
         $cand (CARLA $cver)"
    done

    [ -n "$PICKED" ] || die "no CARLA $PINNED server on port $CARLA_PORT, and no usable tree to start one.${REJECTED:+
       Rejected for the wrong version (the client SDK is $PINNED):$REJECTED}
       Pass --carla-root <dir with CarlaUE4.sh>, put carla_root in
       ~/.fixs/carla.json, or set CARLA_ROOT. To use a server you started
       yourself, pass --carla-port <its port>."
    CARLA_ROOT="$PICKED"

    # RenderOffScreen by default: this must work over ssh and on a headless box.
    # --render gives a window when you actually want to watch it.
    RENDER_FLAG="-RenderOffScreen"; [ "$RENDER" -eq 1 ] && RENDER_FLAG=""
    echo "starting CARLA from $CARLA_ROOT on port $CARLA_PORT ..."
    ( cd "$CARLA_ROOT" && ./CarlaUE4.sh $RENDER_FLAG -carla-rpc-port="$CARLA_PORT" \
        -quality-level=Low ) > carla.log 2>&1 &
    CARLA_PID=$!
    OWN_CARLA=1

    # UE4 takes tens of seconds to open the RPC port; poll rather than sleep a
    # guessed constant, and give up loudly instead of leaving the rest of the
    # pipeline waiting on a server that never came up.
    #
    # Liveness is "is a CarlaUE4 process serving OUR port", not "is $CARLA_PID
    # alive": CarlaUE4.sh hands off to CarlaUE4-Linux-Shipping and the launcher
    # can exit first, which read as a crash and aborted a server that was in
    # fact starting normally.
    carla_alive() { pgrep -f "carla-rpc-port=$CARLA_PORT" >/dev/null 2>&1 || kill -0 "$CARLA_PID" 2>/dev/null; }
    for _ in $(seq 1 60); do
        listening "$CARLA_PORT" && break
        carla_alive || { tail -15 carla.log; die "CARLA exited during startup (see carla.log)"; }
        sleep 2
    done
    listening "$CARLA_PORT" || { tail -15 carla.log; die "CARLA did not open port $CARLA_PORT within 120 s (see carla.log)"; }
    echo "  CARLA is up."
fi

# --- the map ------------------------------------------------------------------
# Only into a server we own. simple_loop.xodr is meshed procedurally, no map
# package and no Unreal cook, and is the same geometry the SUMO net uses -- so
# the vehicles land on the road rather than at arbitrary offsets.
if [ "$OWN_CARLA" -eq 1 ]; then
    PY="$(sed -n 's/.*"python"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$HOME/.fixs/carla.json" 2>/dev/null | head -1)"
    [ -n "$PY" ] && [ -x "$PY" ] || PY="$(command -v python3 || true)"
    if [ -n "$PY" ] && "$PY" -c 'import carla' 2>/dev/null; then
        echo "loading simple_loop.xodr as the world ..."
        "$PY" "$REPO_ROOT/Carla/load_opendrive_world.py" "$XODR" --port "$CARLA_PORT" \
            > loadworld.log 2>&1 || { tail -10 loadworld.log; die "loading the world failed (see loadworld.log)"; }
    else
        # Not fatal: the bridge still runs, the vehicles just drive on whatever
        # world the server booted with. Say so rather than let it look wrong.
        echo "WARNING: no python with the carla module (set \"python\" in ~/.fixs/carla.json)."
        echo "         Skipping the map load -- vehicles will not line up with the road."
    fi
fi

echo "=============================="
echo "SUMO <-> TrafficLayer <-> CARLA (Linux)"
echo "=============================="
echo "  TrafficLayer: $TL"
echo "  VirCarlaEnv:  $VCE"
echo "  SUMO server:  $SUMO_BIN"
echo "  config:       $CONFIG   (TraCI $TRAFFIC_PORT, CARLA $CARLA_PORT)"
echo "  duration:     ${DURATION}s"
echo

# --- stage a config: no auto-launch, our CARLA port, unprivileged app port ---
# The committed config publishes on port 440, which Windows binds happily and
# Linux does not: ports below 1024 need root or CAP_NET_BIND_SERVICE, so
# TrafficLayer dies with "bind() failed: Permission denied" as a normal user.
# Rather than ask anyone to run a co-simulation as root, the staged config is
# shifted into the unprivileged range -- in BOTH places, since ApplicationSetup
# port and CarlaSetup.CarlaClientPort are the two ends of the same socket.
APP_PORT_CFG="$(grep -E '^[[:space:]]*CarlaClientPort:' "$CONFIG" | head -1 | tr -d " '\r" | cut -d: -f2)"
APP_PORT="${APP_PORT_CFG:-440}"
if [ "$APP_PORT" -lt 1024 ]; then
    NEW_APP_PORT=$((APP_PORT + 4000))
    echo "note: app port $APP_PORT is privileged on Linux; staging config on $NEW_APP_PORT"
else
    NEW_APP_PORT="$APP_PORT"
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
stop() {  # $1 = pid, $2 = label
    local pid="$1" label="$2" i
    [ -n "$pid" ] || return 0
    kill -TERM "$pid" 2>/dev/null || return 0
    for i in 1 2 3 4 5; do kill -0 "$pid" 2>/dev/null || return 0; sleep 1; done
    echo "  $label ignored SIGTERM, sending SIGKILL"
    kill -KILL "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
}
cleanup() {
    echo
    echo "--- shutting down (bridge first, so it despawns its actors) ---"
    stop "$VCE_PID"  VirCarlaEnv
    stop "$TL_PID"   TrafficLayer
    stop "$SUMO_PID" SUMO
    # Only ours. A server we attached to belongs to someone else and is left
    # exactly as we found it.
    #
    # Matched on the port, not on $CARLA_PID: CarlaUE4.sh hands off to
    # CarlaUE4-Linux-Shipping, and killing the launcher leaves the server
    # running. UE4 also acknowledges SIGTERM (RequestExit appears in carla.log)
    # and can then take longer than the shell does to exit, so this escalates
    # rather than assuming a clean exit -- a survivor holds the RPC port and
    # the GPU, and the next run would silently ATTACH to it.
    if [ "$OWN_CARLA" -eq 1 ]; then
        stop "$CARLA_PID" "CARLA launcher"
        local i
        for i in $(seq 1 10); do
            [ -z "$(carla_server_pids)" ] && break
            [ "$i" -eq 1 ] && kill -TERM $(carla_server_pids) 2>/dev/null
            sleep 1
        done
        if [ -n "$(carla_server_pids)" ]; then
            echo "  CARLA ignored SIGTERM, sending SIGKILL"
            kill -KILL $(carla_server_pids) 2>/dev/null
            sleep 1
        fi
        [ -n "$(carla_server_pids)" ] \
            && echo "  WARNING: a CARLA on port $CARLA_PORT is still running" \
            || echo "  CARLA stopped."
    fi
    rm -f "$STAGED"
}
trap cleanup EXIT INT TERM

# --- 1. SUMO ------------------------------------------------------------------
echo "[1/3] starting SUMO on TraCI port $TRAFFIC_PORT ..."
"$SUMO_BIN" -c "$SUMOCFG" --remote-port "$TRAFFIC_PORT" --num-clients 1 \
            --step-length "$STEP_LENGTH" --start > sumo.log 2>&1 &
SUMO_PID=$!
sleep 2
kill -0 "$SUMO_PID" 2>/dev/null || { echo "--- sumo.log ---"; tail -20 sumo.log; die "SUMO exited immediately"; }

# --- 2. TrafficLayer ----------------------------------------------------------
echo "[2/3] starting TrafficLayer (SUMO path) ..."
"$TL" -f "$STAGED" > trafficlayer.log 2>&1 &
TL_PID=$!
sleep 3
kill -0 "$TL_PID" 2>/dev/null || { echo "--- trafficlayer.log ---"; tail -25 trafficlayer.log; die "TrafficLayer exited immediately"; }

# --- 3. the Carla bridge ------------------------------------------------------
echo "[3/3] running VirCarlaEnv for ${DURATION}s ..."
"$VCE" -f "$STAGED" -t "$TLS" > vircarlaenv.log 2>&1 &
VCE_PID=$!
sleep "$DURATION"

# --- verdict ------------------------------------------------------------------
# Read the bridge's own verbose output rather than asserting on CARLA state:
# the run is over by the time we look, and the actors are gone with it.
echo
echo "--- VirCarlaEnv (tail) ---"
tail -12 vircarlaenv.log

SPAWNS=$(grep -ciE 'spawn' vircarlaenv.log 2>/dev/null || echo 0)
CONNECTED=$(grep -ciE 'connected|carla server|client version' vircarlaenv.log 2>/dev/null || echo 0)
ALIVE=0; kill -0 "$VCE_PID" 2>/dev/null && ALIVE=1

echo
if [ "$ALIVE" -eq 1 ] && [ "$SPAWNS" -gt 0 ]; then
    echo "RESULT: PASS -- bridge connected, spawned vehicles ($SPAWNS spawn lines), still running at ${DURATION}s"
    exit 0
fi
echo "RESULT: FAIL -- alive=$ALIVE connect_lines=$CONNECTED spawn_lines=$SPAWNS"
echo "        see vircarlaenv.log / trafficlayer.log / sumo.log in $TEST_DIR"
exit 1
