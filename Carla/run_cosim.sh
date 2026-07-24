#!/usr/bin/env bash
# Linux/macOS wrapper for the cross-platform run_cosim.py orchestrator.
# Pass any run_cosim.py args, e.g.:
#   ./run_cosim.sh --sumocfg fixtures/grid_tls.sumocfg --map Town01 --render-offscreen
set -euo pipefail
exec python "$(dirname "$0")/run_cosim.py" "$@"
