#!/usr/bin/env bash
# =============================================================================
# pack_native_deps.sh -- Linux counterpart of scripts/dispatch/pack_native_deps.ps1
#
# Produces (and optionally publishes) the Linux native-deps asset for the public
# rolling 'fixs-native-deps' release, so a Linux clone can fetch a prebuilt
# libtraci instead of building SUMO -- which is what the Windows side already
# does. Upstream publishes no Linux binary for a pinned SUMO version (the only
# prebuilt Linux distribution is ppa:sumo/stable, which tracks a DIFFERENT
# version), so we build the pinned one and publish it ourselves.
#
#   libsumo-<sumo_ver>-linux-x86_64.zip   (+ .sha256 sidecar, same format as
#                                          the .ps1: "<sha>  <name>")
#
# Contents mirror the Windows asset exactly, so one fetch script and one CMake
# search work on both platforms:
#
#   libsumo/*.h          headers  (build time)
#   libsumo/bin/libtracicpp.so    library (build + run time)
#
# BUILD ON THE OLDEST SUPPORTED DISTRO. glibc is forward-compatible only, so an
# asset built on 20.04 (glibc 2.31) runs on 22.04 and 24.04, but not the
# reverse. One asset therefore covers the whole matrix; this script refuses to
# publish from a newer distro unless --allow-any-distro is given.
#
# Usage:
#   scripts/pack_native_deps.sh                    # build + pack into dist/
#   scripts/pack_native_deps.sh --publish          # ... and upload to the release
#   scripts/pack_native_deps.sh --out-dir /tmp/x
# =============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG='fixs-native-deps'          # rolling, version-LESS on purpose (as in the .ps1)
GH_REPO='ORNL-Real-Sim/FIXS'
OUT_DIR="$REPO_ROOT/dist"
PUBLISH=0
ALLOW_ANY_DISTRO=0
BUILD_ON="20.04"

die()  { echo "ERROR: $*" >&2; exit 1; }
note() { echo "  $*"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --publish)           PUBLISH=1; shift ;;
        --out-dir)           OUT_DIR="$2"; shift 2 ;;
        --repo)              GH_REPO="$2"; shift 2 ;;
        --allow-any-distro)  ALLOW_ANY_DISTRO=1; shift ;;
        -h|--help)           sed -n '2,33p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) die "unknown argument: $1 (try --help)" ;;
    esac
done

# --- version comes from dependencies.yaml, never a literal -------------------
yaml_sumo_key() {
    awk '/^  sumo:/{inblk=1; next} /^  [a-z]/{inblk=0} inblk' "$REPO_ROOT/dependencies.yaml" \
        | sed -n "s/^[[:space:]]*$1:[[:space:]]*\"\{0,1\}\([^\"]*\)\"\{0,1\}[[:space:]]*$/\1/p" \
        | head -1 | tr -d '\r'
}
SUMO_VERSION="$(yaml_sumo_key version)"
SUMO_LOCATION="$(yaml_sumo_key location)"
[ -n "$SUMO_VERSION" ]  || die "could not read sumo.version from dependencies.yaml"
[ -n "$SUMO_LOCATION" ] || die "could not read sumo.location from dependencies.yaml"

ZIP_NAME="libsumo-${SUMO_VERSION}-linux-x86_64.zip"

. /etc/os-release
echo "Packing $ZIP_NAME"
note "building on: $PRETTY_NAME (glibc $(ldd --version | head -1 | grep -oE '[0-9]+\.[0-9]+$'))"

if [ "${VERSION_ID:-}" != "$BUILD_ON" ] && [ "$ALLOW_ANY_DISTRO" -eq 0 ]; then
    die "refusing to pack on Ubuntu ${VERSION_ID:-unknown}.
       glibc is forward-compatible only, so the published asset must be built on
       the OLDEST supported distro ($BUILD_ON) or it will not run there.
       Override with --allow-any-distro if you know what you are doing."
fi

command -v zip >/dev/null 2>&1 || die "'zip' not found: sudo apt-get install -y zip"

# --- produce the payload ------------------------------------------------------
# Reuse the fetch script rather than duplicating the SUMO build, so the packed
# artifact is byte-for-byte what a developer's source build produces.
note "building libtracicpp via fetch_native_deps.sh"
"$REPO_ROOT/scripts/fetch_native_deps.sh" --force >/dev/null || die "fetch_native_deps.sh failed"

SRC_DIR="$REPO_ROOT/$SUMO_LOCATION"
[ -f "$SRC_DIR/bin/libtracicpp.so" ] || die "libtracicpp.so missing from $SRC_DIR/bin"
[ -f "$SRC_DIR/libtraci.h" ]         || die "headers missing from $SRC_DIR"

# --- stage, exactly mirroring the Windows layout ------------------------------
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT
mkdir -p "$STAGE/libsumo/bin"
cp "$SRC_DIR"/*.h              "$STAGE/libsumo/"
cp "$SRC_DIR/bin/libtracicpp.so" "$STAGE/libsumo/bin/"

# --- verify BEFORE publishing --------------------------------------------------
# The .ps1 load-tests its DLLs because a vendored copy once silently lacked
# geos_c.dll for months (#70). Same intent: refuse to ship a library whose
# NEEDED entries cannot resolve.
if ldd "$STAGE/libsumo/bin/libtracicpp.so" | grep -q 'not found'; then
    ldd "$STAGE/libsumo/bin/libtracicpp.so" | grep 'not found' >&2
    die "libtracicpp.so has unresolved dependencies; refusing to pack"
fi
note "loadability check OK ($(ls "$STAGE/libsumo"/*.h | wc -l) headers)"

# --- zip + sha256 sidecar (same sidecar format as the .ps1) --------------------
mkdir -p "$OUT_DIR"
ZIP_PATH="$OUT_DIR/$ZIP_NAME"
rm -f "$ZIP_PATH" "$ZIP_PATH.sha256"
( cd "$STAGE" && zip -qr "$ZIP_PATH" . )
SHA="$(sha256sum "$ZIP_PATH" | cut -d' ' -f1)"
printf '%s  %s\n' "$SHA" "$ZIP_NAME" > "$ZIP_PATH.sha256"

echo "Packed $ZIP_NAME ($(du -h "$ZIP_PATH" | cut -f1))  sha256=$SHA"

# --- publish -------------------------------------------------------------------
if [ "$PUBLISH" -eq 1 ]; then
    command -v gh >/dev/null 2>&1 || die "'gh' not found; cannot publish"
    echo "Publishing to rolling release '$TAG' on $GH_REPO ..."
    # --clobber replaces only these two asset names; the Windows assets on the
    # same rolling release are untouched.
    gh release upload "$TAG" "$ZIP_PATH" "$ZIP_PATH.sha256" \
        --repo "$GH_REPO" --clobber || die "upload failed"
    echo "Published $ZIP_NAME to $TAG."
else
    echo "(not published -- pass --publish to upload)"
fi
