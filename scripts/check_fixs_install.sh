#!/usr/bin/env bash
# ============================================================================
#  Is <dir> a complete, runnable FIXS install? (Linux)  #273
#
#  Run by release.yml against what update_fixs.sh produced - once from the zip it
#  just packed (before publishing) and once from the published release (after).
#  scripts/check_fixs_install.ps1 is the Windows counterpart; change them together.
#
#    check_fixs_install.sh /tmp/t/FIXS v0.9.1-alpha
#
#  Exit 0 = complete. Exit 1 = says what is missing.
# ============================================================================
set -uo pipefail
dir="$1" tag="$2"
bad=0
fail() { echo "[FAIL] $*"; bad=1; }

if [[ ! -x "$dir/TrafficLayer" ]]; then
    fail "TrafficLayer is missing or not executable"
else
    # A load check, not a presence check. TrafficLayer finds libtracicpp.so
    # through its $ORIGIN-relative RPATH (CMakeLists.txt), so with no
    # LD_LIBRARY_PATH set this is exactly what a consumer's first run resolves.
    missing="$(env -u LD_LIBRARY_PATH ldd "$dir/TrafficLayer" | grep 'not found' || true)"
    [[ -z "$missing" ]] || fail "TrafficLayer cannot resolve: $(echo "$missing" | awk '{print $1}' | tr '\n' ' ')"
fi
[[ -f "$dir/CommonLib/libsumo/bin/libtracicpp.so" ]] || fail "CommonLib/libsumo/bin/libtracicpp.so is missing"

# Line 1 is '<tag>' for a pinned release and '<tag> (<published>)' for a rolling
# one. run_cosim.py reads it byte-for-byte, where a BOM becomes part of the tag.
vf="$dir/FIXS_VERSION.txt"
if [[ ! -f "$vf" ]]; then
    fail "FIXS_VERSION.txt is missing"
else
    [[ "$(head -c3 "$vf" | od -An -tx1 | tr -d ' \n')" != "efbbbf" ]] || fail "FIXS_VERSION.txt starts with a UTF-8 BOM"
    line1="$(head -n1 "$vf")"
    [[ "$line1" == "$tag" || "$line1" == "$tag ("* ]] || fail "FIXS_VERSION.txt line 1 is '$line1', expected '$tag'"
fi

[[ "$bad" == 0 ]] || exit 1
echo "Install in $dir is complete ($tag)."
