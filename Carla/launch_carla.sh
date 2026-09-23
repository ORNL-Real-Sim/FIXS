#!/usr/bin/env bash
# =============================================================================
# launch_carla.sh -- Linux counterpart of launch_carla.bat (+ wait_for_rpc.ps1)
#
# Starts a CARLA server and returns once its RPC port answers, so a caller can
# go straight to connecting. --stop takes down the server on a port again,
# which the .bat has no counterpart for because the .bat leaves its windows for
# the user to close.
#
#   launch_carla.sh --port 2100                # start, wait for RPC, return
#   launch_carla.sh --port 2100 --headless     # -RenderOffScreen (ssh / CI)
#   launch_carla.sh --port 2100 --stop         # stop the server on that port
#   launch_carla.sh --port 2100 --carla-root ~/CARLA_0.9.15_Package
#
# The tree is picked in this order, and must carry a VERSION matching the carla
# pin in dependencies.yaml: --carla-root, ~/.fixs/carla.json, $CARLA_ROOT. The
# version gate is not pedantry -- this box exports CARLA_ROOT pointing at a
# 0.9.14 tree while carla.json names the pinned 0.9.15, and an 0.9.14 server
# answers a 0.9.15 client with a protocol error that reads like a bridge bug.
# =============================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/.." && pwd)"
PORT=2000
HEADLESS=0
STOP=0
CLI_ROOT=""
ENV_ROOT="${CARLA_ROOT:-}"
LOG="carla.log"

die() { echo "ERROR: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
    case "$1" in
        --port)       PORT="$2";      shift 2 ;;
        --carla-root) CLI_ROOT="$2";  shift 2 ;;
        --log)        LOG="$2";       shift 2 ;;
        --headless)   HEADLESS=1;     shift ;;
        --stop)       STOP=1;         shift ;;
        -h|--help)    sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) die "unknown argument: $1 (try --help)" ;;
    esac
done

listening() { timeout 3 bash -c "</dev/tcp/127.0.0.1/$PORT" 2>/dev/null; }

# The CarlaUE4 processes serving this port, never the caller or this script.
# pkill -f would be shorter and is a trap: the pattern occurs in the command
# line of any shell that typed it, so a bare pkill kills the very shell running
# the teardown (observed).
server_pids() {
    pgrep -f -- "-carla-rpc-port=$PORT" 2>/dev/null | grep -vx -e "$$" -e "$PPID" || true
}

# --- stop ---------------------------------------------------------------------
if [ "$STOP" -eq 1 ]; then
    [ -n "$(server_pids)" ] || { echo "no CARLA on port $PORT"; exit 0; }
    kill -TERM $(server_pids) 2>/dev/null
    # UE4 acknowledges SIGTERM (RequestExit lands in the log) and can still
    # outlive the shell; a survivor holds the port and the GPU, and the next
    # run would silently attach to it.
    for _ in $(seq 1 10); do [ -z "$(server_pids)" ] && break; sleep 1; done
    if [ -n "$(server_pids)" ]; then
        echo "  CARLA ignored SIGTERM, sending SIGKILL"
        kill -KILL $(server_pids) 2>/dev/null; sleep 1
    fi
    [ -n "$(server_pids)" ] && die "a CARLA on port $PORT is still running"
    echo "  CARLA stopped."
    exit 0
fi

# --- start --------------------------------------------------------------------
listening && { echo "CARLA already listening on $PORT -- leaving it alone"; exit 0; }

PINNED="$(awk '/^  carla:/{i=1; next} /^  [a-z]/{i=0} i' "$REPO_ROOT/dependencies.yaml" \
          | sed -n 's/^[[:space:]]*version:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' | head -1 | tr -d '\r')"
JSON_ROOT=""
[ -f "$HOME/.fixs/carla.json" ] && JSON_ROOT="$(sed -n 's/.*"carla_root"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$HOME/.fixs/carla.json" | head -1)"

ROOT=""; REJECTED=""
for cand in "$CLI_ROOT" "$JSON_ROOT" "$ENV_ROOT"; do
    [ -n "$cand" ] && [ -x "$cand/CarlaUE4.sh" ] || continue
    cver="$(tr -d ' \r\n' < "$cand/VERSION" 2>/dev/null)"
    if [ -z "$PINNED" ] || [ -z "$cver" ] || [ "$cver" = "$PINNED" ]; then ROOT="$cand"; break; fi
    REJECTED="$REJECTED
         $cand (CARLA $cver)"
done
[ -n "$ROOT" ] || die "no CARLA $PINNED tree to launch.${REJECTED:+
       Rejected for the wrong version:$REJECTED}
       Pass --carla-root <dir with CarlaUE4.sh>, or set carla_root in ~/.fixs/carla.json."

RENDER_FLAG=""; [ "$HEADLESS" -eq 1 ] && RENDER_FLAG="-RenderOffScreen"
echo "starting CARLA $PINNED from $ROOT on port $PORT ..."
( cd "$ROOT" && ./CarlaUE4.sh $RENDER_FLAG -carla-rpc-port="$PORT" -quality-level=Low ) > "$LOG" 2>&1 &
LAUNCHER=$!

# UE4 takes tens of seconds to open the port. Liveness is "is a CarlaUE4
# serving our port", not "is $LAUNCHER alive": CarlaUE4.sh hands off to
# CarlaUE4-Linux-Shipping and the launcher can exit first, which read as a
# crash and aborted a server that was starting normally.
for _ in $(seq 1 60); do
    listening && break
    [ -n "$(server_pids)" ] || kill -0 "$LAUNCHER" 2>/dev/null || { tail -15 "$LOG"; die "CARLA exited during startup (see $LOG)"; }
    sleep 2
done
listening || { tail -15 "$LOG"; die "CARLA did not open port $PORT within 120 s (see $LOG)"; }
echo "  CARLA is up."
