#!/usr/bin/env bash
# Linux/macOS wrapper for the generic map importer import_map.py.
# An app passes its map via --map-config <file> (package= and url=) or directly
# with --package/--package-url, e.g.:
#   ./import_map.sh --package-pick --map-config path/to/map.txt
set -euo pipefail
exec python "$(dirname "$0")/import_map.py" "$@"
