#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: copy_carla_dependencies.sh /path/to/CARLA_ROOT

Copies CARLA's PythonAPI dependency bundle into FIXS CommonLib as "libcarla".
- Source:  <CARLA_ROOT>/PythonAPI/carla/dependencies
- Dest:    <FIXS_ROOT>/CommonLib/libcarla

If the destination already exists, the script will abort to avoid clobbering it.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -ne 1 ]]; then
  usage
  exit 1
fi

CARLA_ROOT="$(cd "$1" && pwd)"
SRC="${CARLA_ROOT}/PythonAPI/carla/dependencies"
# Script lives in the repo root; use its directory as FIXS root
FIXS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="${FIXS_ROOT}/CommonLib/libcarla"

if [[ ! -d "$SRC" ]]; then
  echo "Error: dependencies directory not found at: $SRC" >&2
  exit 2
fi

if [[ -e "$DST" ]]; then
  echo "Error: destination already exists: $DST" >&2
  echo "Remove it first if you want to replace it." >&2
  exit 3
fi

echo "Copying CARLA dependencies..."
echo "  from: $SRC"
echo "  to  : $DST"
cp -a "$SRC" "$DST"

echo "Done. CARLA dependencies copied to $DST"
