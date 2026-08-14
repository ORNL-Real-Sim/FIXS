#!/usr/bin/env bash
# =============================================================================
# run_sumo_carla_demo.sh -- Linux counterpart of run_sumo_carla_demo.bat (#65)
#
#   SUMO --[TraCI]--> TrafficLayer --[VehFullData @ 440]--> VirCarlaEnv --> CARLA
#
# Two deliberate differences from the .bat, both for the same reason -- this
# script has to work unattended on a headless box:
#
#   * it starts HEADLESS sumo, not sumo-gui (which needs fox/OpenGL and a
#     display), and stages a config with EnableAutoLaunch:false so TrafficLayer
#     does not try to spawn a GUI itself. The committed config.yaml is never
#     modified. --gui opts back in.
#
#   * it does NOT start or stop the CARLA server. Bringing CARLA up is
#     machine-specific (package vs source build, GPU, RenderOffScreen), and a
#     script that killed a server someone else was using would be worse than
#     one that asks. Start it yourself and pass --carla-port:
#
#       ~/CARLA_0.9.15_Package/CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2100
#       python Carla/load_opendrive_world.py \
#              tests/Vissim/SimpleEcho/simple_loop.xodr --port 2100
#
# Usage:
#   ./run_sumo_carla_demo.sh                       # 20 s, server on 2000
#   ./run_sumo_carla_demo.sh --carla-port 2100
#   ./run_sumo_carla_demo.sh --duration 30 --gui
#   ./run_sumo_carla_demo.sh --config config_l2.yaml
# =============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../../.." && pwd)"
CONFIG="config.yaml"
DURATION=20
USE_GUI=0
CARLA_PORT=""
STEP_LENGTH=0.1

while [ $# -gt 0 ]; do
    case "$1" in
        --config)     CONFIG="$2";     shift 2 ;;
        --duration)   DURATION="$2";   shift 2 ;;
        --carla-port) CARLA_PORT="$2"; shift 2 ;;
        --gui)        USE_GUI=1;       shift ;;
        -h|--help)    sed -n '2,30p' "${BASH_SOURCE[0]}"; exit 0 ;;
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
CFG_CARLA_PORT="$(grep -E '^[[:space:]]*CarlaServerPort:' "$CONFIG" | head -1 | tr -d " '\r" | cut -d: -f2)"
[ -n "$CARLA_PORT" ] || CARLA_PORT="${CFG_CARLA_PORT:-2000}"

# --- CARLA must already be up -------------------------------------------------
# Checked before anything is started: VirCarlaEnv would otherwise sit in its
# connect retry loop while SUMO and TrafficLayer run pointlessly.
if ! timeout 5 bash -c "</dev/tcp/127.0.0.1/$CARLA_PORT" 2>/dev/null; then
    die "nothing is listening on 127.0.0.1:$CARLA_PORT -- start the CARLA server first.
       See the header of this script for the two commands."
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
