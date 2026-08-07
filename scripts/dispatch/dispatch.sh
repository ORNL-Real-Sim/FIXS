#!/usr/bin/env bash
# =============================================================================
# dispatch.sh -- one-command Linux build, the counterpart of dispatch.bat.
#
# Scope is deliberately narrower than the .bat, because most of what that
# builds cannot exist on Linux:
#
#   dispatch.bat step            Linux
#   -------------------------    ---------------------------------------------
#   1 external libraries         yes - yaml-cpp, built as part of the CMake tree
#   2 core components            yes - TrafficLayer
#   3 VISSIM DriverModel DLLs    no  - VISSIM is Windows-only (rejected at
#                                      config-parse time by a Linux build)
#   4 CarMaker / 5 dSPACE / 6 MEX no - licensed Windows toolchains
#
# So this is: check the toolchain -> fetch native deps -> configure -> build ->
# smoke-test -> summary. Same shape as the .bat, minus what does not apply.
#
# Usage:
#   scripts/dispatch/dispatch.sh                 # full build + smoke checks
#   scripts/dispatch/dispatch.sh --clean         # wipe build/ first
#   scripts/dispatch/dispatch.sh --debug         # Debug instead of Release
#   scripts/dispatch/dispatch.sh --no-deps       # skip fetch_native_deps.sh
#   scripts/dispatch/dispatch.sh --no-smoke      # build only
#   scripts/dispatch/dispatch.sh --jobs 8
# =============================================================================
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="$REPO_ROOT/build"
BUILD_TYPE=Release
JOBS="$(nproc 2>/dev/null || echo 4)"
DO_DEPS=1
DO_SMOKE=1
DO_CLEAN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --clean)     DO_CLEAN=1;  shift ;;
        --debug)     BUILD_TYPE=Debug; shift ;;
        --no-deps)   DO_DEPS=0;   shift ;;
        --no-smoke)  DO_SMOKE=0;  shift ;;
        --jobs|-j)   JOBS="$2";   shift 2 ;;
        -h|--help)   sed -n '2,27p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
    esac
done

STEPS_OK=()
STEPS_FAIL=()
step_ok()   { STEPS_OK+=("$1");   echo "===> $1: OK"; }
step_fail() { STEPS_FAIL+=("$1"); echo "===> $1: FAILED"; }

echo "=============================="
echo "FIXS Linux Builder"
echo "=============================="
echo "repo:       $REPO_ROOT"
echo "build dir:  $BUILD_DIR ($BUILD_TYPE)"
echo "jobs:       $JOBS"
echo

# --- step 0: toolchain -------------------------------------------------------
# Report everything missing at once rather than failing on the first one.
MISSING_CMD=""
for tool in cmake c++ git; do
    command -v "$tool" >/dev/null 2>&1 || MISSING_CMD="$MISSING_CMD $tool"
done
if [ -n "$MISSING_CMD" ]; then
    echo "ERROR: missing tools:$MISSING_CMD"
    echo "  sudo apt-get install -y build-essential cmake ninja-build git"
    exit 1
fi
echo "toolchain:  $(c++ --version | head -1) / $(cmake --version | head -1)"
GENERATOR=""
command -v ninja >/dev/null 2>&1 && GENERATOR="-G Ninja"
step_ok "Toolchain check"

# --- step 1: native deps (libsumo/libtraci) ----------------------------------
if [ "$DO_DEPS" -eq 1 ]; then
    if "$REPO_ROOT/scripts/fetch_native_deps.sh" --jobs "$JOBS"; then
        step_ok "Native deps (libsumo)"
    else
        step_fail "Native deps (libsumo)"
        echo
        echo "Cannot continue without libtracicpp. See the message above."
        exit 1
    fi
else
    echo "===> Native deps: SKIPPED (--no-deps)"
fi

# --- step 2: configure + build ----------------------------------------------
# Built INSIDE the repo on purpose: TrafficLayer locates the SUMO runtime by
# walking up from its own executable directory, so a binary parked outside the
# tree cannot find CommonLib/libsumo/bin at run time.
[ "$DO_CLEAN" -eq 1 ] && { echo "Cleaning $BUILD_DIR ..."; rm -rf "$BUILD_DIR"; }
# Logs live inside the build dir, not the repo root, so a build leaves
# nothing behind outside build/ (which is gitignored via **/build/**).
mkdir -p "$BUILD_DIR"
CFG_LOG="$BUILD_DIR/dispatch-configure.log"
BLD_LOG="$BUILD_DIR/dispatch-build.log"

if cmake -S "$REPO_ROOT" -B "$BUILD_DIR" $GENERATOR \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" > "$CFG_LOG" 2>&1; then
    step_ok "CMake configure"
else
    step_fail "CMake configure"
    tail -25 "$CFG_LOG"
    exit 1
fi

if cmake --build "$BUILD_DIR" -j "$JOBS" > "$BLD_LOG" 2>&1; then
    step_ok "TrafficLayer ($BUILD_TYPE)"
else
    step_fail "TrafficLayer ($BUILD_TYPE)"
    grep -E 'error:|FAILED' "$BLD_LOG" | head -25
    echo "(full log: $BLD_LOG)"
    exit 1
fi

WARN_COUNT=$(grep -E 'warning:' "$BLD_LOG" 2>/dev/null | grep -vc 'yaml-cpp')
BIN="$BUILD_DIR/TrafficLayer"

# --- step 3: smoke checks ----------------------------------------------------
if [ "$DO_SMOKE" -eq 1 ]; then
    export LD_LIBRARY_PATH="$REPO_ROOT/CommonLib/libsumo/bin:${LD_LIBRARY_PATH:-}"

    if "$BIN" --help >/dev/null 2>&1; then
        step_ok "Smoke: --help"
    else
        step_fail "Smoke: --help"
    fi

    # A Linux build must REFUSE a VISSIM config rather than wait on a connect()
    # that can never succeed.
    VY="$(mktemp)"
    printf 'SimulationSetup:\n  EnableFIXS: true\n  SelectedTrafficSimulator: VISSIM\n  TrafficSimulatorPort: 1337\n' > "$VY"
    if "$BIN" -f "$VY" >/dev/null 2>&1; then
        step_fail "Smoke: VISSIM config rejected"
    else
        step_ok "Smoke: VISSIM config rejected"
    fi
    rm -f "$VY"
fi

# --- summary -----------------------------------------------------------------
echo
echo "=============================="
if [ ${#STEPS_FAIL[@]} -eq 0 ]; then
    echo "All Linux components built successfully"
else
    echo "Build completed with failures"
    printf 'Failed: %s\n' "${STEPS_FAIL[@]}"
fi
echo "=============================="
echo "binary:            $BIN"
echo "warnings (FIXS):   $WARN_COUNT"
echo "logs:              $CFG_LOG"
echo "                   $BLD_LOG"
echo
echo "Run a config with:"
echo "  export LD_LIBRARY_PATH=$REPO_ROOT/CommonLib/libsumo/bin:\$LD_LIBRARY_PATH"
echo "  $BIN -f <config.yaml>"

[ ${#STEPS_FAIL[@]} -eq 0 ] || exit 1
