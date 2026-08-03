#!/usr/bin/env bash
# Linux/macOS wrapper for carla_env_setup.py.
# Run this any time to choose / change which CARLA run_cosim uses
# (packaged build vs source build, or a different install). The choice is
# saved to ~/.fixs/carla.json and reused on every run.
set -euo pipefail
exec python "$(dirname "$0")/carla_env_setup.py" "$@"
