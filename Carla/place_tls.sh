#!/usr/bin/env bash
# Linux/macOS wrapper for the generic traffic-light placer place_tls.py.
# An app passes its map + table, e.g.:
#   ./place_tls.sh --map-config path/to/map.txt --tl-table path/to/traffic_light_table.csv
set -euo pipefail
exec python "$(dirname "$0")/place_tls.py" "$@"
