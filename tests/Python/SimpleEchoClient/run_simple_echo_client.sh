#!/usr/bin/env bash
# =============================================================================
# run_simple_echo_client.sh -- Linux counterpart of run_simple_echo_client.bat
#
# Runs the full three-process loop: SUMO <-> TrafficLayer <-> Python echo client.
#
# One deliberate difference from the .bat. The .bat leaves EnableAutoLaunch:true
# and lets TrafficLayer start SUMO itself -- but that code path launches
# **sumo-gui**, which on Linux means pulling in fox/OpenGL and needing a display.
# So this script takes the .bat's OTHER path: it starts a HEADLESS sumo itself
# and hands TrafficLayer a staged config with EnableAutoLaunch:false. The
# committed config.yaml is never modified (same approach the VISSIM probes use
# for seeds). --gui opts back into sumo-gui when you have a display (WSLg does).
#
# Usage:
#   ./run_simple_echo_client.sh              # headless, self-cleaning
#   ./run_simple_echo_client.sh --gui        # use sumo-gui instead
#   ./run_simple_echo_client.sh --config config_ego_only.yaml
#   ./run_simple_echo_client.sh --duration 20
# =============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
CONFIG="config.yaml"
DURATION=15
USE_GUI=0
STEP_LENGTH=0.1

while [ $# -gt 0 ]; do
    case "$1" in
        --config)   CONFIG="$2";   shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --gui)      USE_GUI=1;     shift ;;
        -h|--help)  sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
    esac
done

cd "$TEST_DIR" || exit 1
die() { echo "ERROR: $*" >&2; exit 1; }

# --- locate the pieces --------------------------------------------------------
TL="$REPO_ROOT/build/TrafficLayer"
[ -x "$TL" ] || die "TrafficLayer not built at $TL
       Build it first:  $REPO_ROOT/scripts/dispatch/dispatch.sh"

SUMO_BIN=""
if [ "$USE_GUI" -eq 1 ]; then
    SUMO_BIN="$(command -v sumo-gui || true)"
    [ -n "$SUMO_BIN" ] || die "--gui needs sumo-gui on PATH (the pinned build is headless only)"
else
    # Prefer the PINNED server next to libtracicpp over whatever is on PATH:
    # a distro/PPA sumo is a different version from the client we link.
    if [ -x "$REPO_ROOT/CommonLib/libsumo/bin/sumo" ]; then
        SUMO_BIN="$REPO_ROOT/CommonLib/libsumo/bin/sumo"
    else
        SUMO_BIN="$(command -v sumo || true)"
        [ -n "$SUMO_BIN" ] && echo "WARNING: using $SUMO_BIN from PATH; it may not match the pinned client version."
    fi
fi
[ -n "$SUMO_BIN" ] || die "no SUMO server found. Build the pinned one:
       $REPO_ROOT/scripts/fetch_native_deps.sh --with-server"

command -v python3 >/dev/null || die "python3 not found"
# CommonLib's Python side imports both; report them together rather than one per run.
PYMISS=""
python3 -c 'import yaml'  2>/dev/null || PYMISS="$PYMISS pyyaml"
python3 -c 'import numpy' 2>/dev/null || PYMISS="$PYMISS numpy"
[ -z "$PYMISS" ] || die "python3 is missing:$PYMISS
       pip3 install$PYMISS      (or: sudo apt-get install -y python3-yaml python3-numpy)"

PORT="$(grep -E '^[[:space:]]*TrafficSimulatorPort:' "$CONFIG" | head -1 | tr -d ' \r' | cut -d: -f2)"
[ -n "$PORT" ] || PORT=1337

echo "=============================="
echo "SimpleEchoClient (Linux)"
echo "=============================="
echo "  TrafficLayer: $TL"
echo "  SUMO server:  $SUMO_BIN"
echo "  config:       $CONFIG   (port $PORT)"
echo "  duration:     ${DURATION}s"
echo

# --- stage a config with auto-launch OFF -------------------------------------
# We own the SUMO process here, so TrafficLayer must not try to spawn sumo-gui.
STAGED="$(mktemp --suffix=.yaml)"
sed 's/^\([[:space:]]*\)EnableAutoLaunch:.*/\1EnableAutoLaunch: false/' "$CONFIG" > "$STAGED"

SUMO_PID=""; TL_PID=""
# SIGTERM first, then SIGKILL after a grace period. The escalation is not
# belt-and-braces: TrafficLayer parked in accept() waiting for a client does not
# act on SIGTERM (the handler raises a flag that the blocked loop never reads),
# so a plain `kill` + `wait` hangs this script forever.
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
    echo "--- shutting down ---"
    stop "$TL_PID"   TrafficLayer
    stop "$SUMO_PID" SUMO
    rm -f "$STAGED"
}
trap cleanup EXIT INT TERM

# --- 1. SUMO ------------------------------------------------------------------
echo "[1/3] starting SUMO on port $PORT ..."
"$SUMO_BIN" -c ./simple_loop.sumocfg --remote-port "$PORT" --num-clients 1 \
            --step-length "$STEP_LENGTH" --start > sumo.log 2>&1 &
SUMO_PID=$!
sleep 2
kill -0 "$SUMO_PID" 2>/dev/null || { echo "--- sumo.log ---"; tail -20 sumo.log; die "SUMO exited immediately"; }

# --- 2. TrafficLayer ----------------------------------------------------------
echo "[2/3] starting TrafficLayer ..."
"$TL" -f "$STAGED" > trafficlayer.log 2>&1 &
TL_PID=$!
sleep 3
kill -0 "$TL_PID" 2>/dev/null || { echo "--- trafficlayer.log ---"; tail -25 trafficlayer.log; die "TrafficLayer exited immediately"; }

# --- 3. the echo client -------------------------------------------------------
echo "[3/3] running the Python echo client for ${DURATION}s ..."
echo
# ABSOLUTE path on purpose: simple_echo_client.py locates CommonLib via
# Path(__file__).parents[3], and Python only guarantees an absolute __file__
# from 3.9 on. Focal ships 3.8, where "./simple_echo_client.py" leaves __file__
# relative and parents[3] raises IndexError.
timeout "$DURATION" python3 "$TEST_DIR/simple_echo_client.py" --config "$CONFIG"
rc=$?
echo
# 124 is timeout(1) stopping a client that was still happily running, which is
# the expected end of a fixed-duration smoke run.
if [ $rc -eq 124 ] || [ $rc -eq 0 ]; then
    echo "RESULT: echo client ran to the ${DURATION}s cap (exit $rc)"
else
    echo "RESULT: echo client exited $rc -- see trafficlayer.log / sumo.log"
fi
exit $rc
