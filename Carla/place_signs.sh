#!/usr/bin/env bash
# Linux/macOS wrapper for the generic road-sign placer place_signs.py.
# Places the RoadRunner sign meshes CARLA's import culled (+ fixes their
# see-through materials) into a cooked map, e.g.:
#   ./place_signs.sh --map <name>     (or --map-config, or pick from cooked maps)
set -euo pipefail
exec python "$(dirname "$0")/place_signs.py" "$@"
