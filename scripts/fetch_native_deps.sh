#!/usr/bin/env bash
# =============================================================================
# fetch_native_deps.sh -- Linux counterpart of scripts/dispatch/fetch_native_deps.ps1
#
# #238 dropped the vendored CommonLib/libsumo from git. On Windows the .ps1
# restores it by downloading a version-named, SHA-256-verified asset from the
# public rolling 'fixs-native-deps' release. That release currently carries
# WINDOWS binaries only (.lib/.dll), which are useless on Linux -- so the modes
# here are deliberately not symmetric with the .ps1:
#
#   --mode prebuilt  (default) download the published, SHA-256-verified Linux
#                    asset from the rolling release -- the same path the Windows
#                    .ps1 takes. Seconds instead of minutes, and no SUMO build
#                    dependencies needed.
#
#   --mode source    build libtracicpp from the SUMO source tree at
#                    the version pinned in dependencies.yaml, then install the
#                    headers + shared library into CommonLib/libsumo. Building
#                    only the libtracicpp target avoids the gdal/fox/osg chain
#                    that libsumocpp would drag in.
#
#   --mode prebuilt  download a Linux asset from the rolling release. Fails with
#                    an explicit message until such assets are published -- see
#                    pack_native_deps.ps1 for how the Windows ones are made.
#
# The install layout matches what the .ps1 produces, so CMake finds the result
# with no extra flags in either case:
#
#   CommonLib/libsumo/*.h          <- #include <libsumo/libtraci.h> via -ICommonLib
#   CommonLib/libsumo/bin/libtracicpp.so
#
# CommonLib/libsumo is gitignored, so nothing here dirties the working tree.
#
# Usage:
#   scripts/fetch_native_deps.sh                     # sumo, source mode
#   scripts/fetch_native_deps.sh --force             # rebuild even if present
#   scripts/fetch_native_deps.sh --jobs 8
#   scripts/fetch_native_deps.sh --sumo-src ~/src/sumo   # reuse a checkout
# =============================================================================
set -euo pipefail

COMPONENT=sumo
MODE=prebuilt
FORCE=0
JOBS="$(nproc 2>/dev/null || echo 4)"
SUMO_SRC=""
WITH_SERVER=0

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_ON_HINT="20.04"   # distro pack_native_deps.sh publishes from

die()  { echo "ERROR: $*" >&2; exit 1; }
note() { echo "  $*"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --component) COMPONENT="$2"; shift 2 ;;
        --mode)      MODE="$2";      shift 2 ;;
        --jobs|-j)   JOBS="$2";      shift 2 ;;
        --sumo-src)  SUMO_SRC="$2";  shift 2 ;;
        --force)       FORCE=1;      shift ;;
        --with-server) WITH_SERVER=1; shift ;;
        -h|--help)   sed -n '2,40p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *)           die "unknown argument: $1 (try --help)" ;;
    esac
done

[ "$COMPONENT" = "sumo" ] || die "only --component sumo is supported on Linux today.
       libcarla is not fetched here because VirCarlaEnv is not part of the Linux
       build yet (its client ABI is unresolved -- see issue #65, Q1)."

# --- everything about SUMO comes from dependencies.yaml ----------------------
# version, upstream URL and install location are all declared there, so nothing
# about SUMO is hardcoded here: moving the vendor directory or switching the
# upstream fork is a one-line edit in the yaml, not a code change.
#
# tr strips \r as well: dependencies.yaml is CRLF in the repo, and a trailing
# carriage return would silently poison the git tag and the paths below.
yaml_sumo_key() {
    awk '/^  sumo:/{inblk=1; next} /^  [a-z]/{inblk=0} inblk' "$REPO_ROOT/dependencies.yaml" \
        | sed -n "s/^[[:space:]]*$1:[[:space:]]*\"\{0,1\}\([^\"]*\)\"\{0,1\}[[:space:]]*$/\1/p" \
        | head -1 | tr -d '\r'
}

SUMO_VERSION="$(yaml_sumo_key version)"
SUMO_SOURCE_URL="$(yaml_sumo_key source)"
SUMO_LOCATION="$(yaml_sumo_key location)"

[ -n "$SUMO_VERSION" ]    || die "could not read sumo.version from dependencies.yaml"
[ -n "$SUMO_SOURCE_URL" ] || die "could not read sumo.source from dependencies.yaml"
[ -n "$SUMO_LOCATION" ]   || die "could not read sumo.location from dependencies.yaml"
SUMO_TAG="v$(echo "$SUMO_VERSION" | tr . _)"

LIBSUMO_DIR="$REPO_ROOT/$SUMO_LOCATION"
SENTINEL="$LIBSUMO_DIR/bin/libtracicpp.so"

echo "libsumo/libtraci $SUMO_VERSION ($MODE mode)"
echo "  install location: $SUMO_LOCATION (from dependencies.yaml)"

# --- already there? -----------------------------------------------------------
LIB_READY=0
if [ -f "$SENTINEL" ] && [ "$FORCE" -eq 0 ]; then
    note "already present at $SENTINEL (use --force to rebuild)"
    LIB_READY=1
fi

# --- prebuilt: download the published Linux asset -----------------------------
# Same rolling, PUBLIC release the Windows .ps1 uses, same .sha256 sidecar
# convention. The asset is built on the oldest supported distro, so one file
# serves 20.04/22.04/24.04 (glibc is forward-compatible only).
if [ "$MODE" = "prebuilt" ] && [ "$LIB_READY" -eq 0 ]; then
    ASSET="libsumo-${SUMO_VERSION}-linux-x86_64.zip"
    BASE="https://github.com/${FIXS_DEPS_REPO:-ORNL-Real-Sim/FIXS}/releases/download/${FIXS_DEPS_TAG:-fixs-native-deps}"
    TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT

    note "downloading $ASSET"
    curl -fsSL --retry 3 -o "$TMP/$ASSET" "$BASE/$ASSET" || die "could not download $ASSET from $BASE.
       If the SUMO version in dependencies.yaml was just bumped, the matching
       asset may not be published yet -- publish it with
       scripts/pack_native_deps.sh --publish (run on Ubuntu $BUILD_ON_HINT), or
       use --mode source to build it locally."
    curl -fsSL --retry 3 -o "$TMP/$ASSET.sha256" "$BASE/$ASSET.sha256"         || die "asset downloaded but its .sha256 sidecar is missing; refusing to trust it"

    ( cd "$TMP" && sha256sum -c "$ASSET.sha256" >/dev/null 2>&1 )         || die "SHA-256 mismatch for $ASSET; refusing to extract"
    note "checksum OK"

    # The zip carries a leading libsumo/ directory, so extract into the PARENT
    # of the configured location.
    DEST_PARENT="$(dirname "$LIBSUMO_DIR")"
    mkdir -p "$DEST_PARENT"
    if command -v unzip >/dev/null 2>&1; then
        unzip -qo "$TMP/$ASSET" -d "$DEST_PARENT" || die "extract failed"
    elif command -v python3 >/dev/null 2>&1; then
        python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])"             "$TMP/$ASSET" "$DEST_PARENT" || die "extract failed"
    else
        die "need 'unzip' or python3 to extract: sudo apt-get install -y unzip"
    fi

    [ -f "$SENTINEL" ] || die "$ASSET did not contain bin/libtracicpp.so"
    if command -v ldd >/dev/null 2>&1 && ldd "$SENTINEL" 2>/dev/null | grep -q 'not found'; then
        ldd "$SENTINEL" | grep 'not found' >&2
        die "libsumo loadability check failed"
    fi
    note "loadability check OK"
    echo "libsumo ready: $LIBSUMO_DIR (prebuilt)"
    LIB_READY=1
fi

# --- toolchain ----------------------------------------------------------------
for tool in git cmake c++; do
    command -v "$tool" >/dev/null 2>&1 || die "'$tool' not found. Install the build toolchain:
       sudo apt-get install -y build-essential cmake ninja-build git"
done

# SUMO's CMake requires these even for the libtraci target. Detect and report
# rather than sudo-installing behind the user's back.
MISSING=""
[ -f /usr/include/xercesc/util/XercesVersion.hpp ] || MISSING="$MISSING libxerces-c-dev"
command -v proj >/dev/null 2>&1 || [ -f /usr/include/proj.h ] || MISSING="$MISSING libproj-dev"
if [ -n "$MISSING" ]; then
    die "SUMO's build needs packages that are not installed:$MISSING
       sudo apt-get install -y --no-install-recommends \\
         libxerces-c-dev libproj-dev libgdal-dev zlib1g-dev python3-dev"
fi

GENERATOR=""
command -v ninja >/dev/null 2>&1 && GENERATOR="-G Ninja"

# --- source -------------------------------------------------------------------
CACHE_ROOT="${FIXS_CACHE_DIR:-$HOME/.fixs}"
if [ -z "$SUMO_SRC" ]; then
    SUMO_SRC="$CACHE_ROOT/sumo-$SUMO_VERSION"
    if [ ! -d "$SUMO_SRC/.git" ]; then
        note "cloning SUMO $SUMO_TAG from $SUMO_SOURCE_URL into $SUMO_SRC"
        mkdir -p "$(dirname "$SUMO_SRC")"
        git clone --depth 1 --branch "$SUMO_TAG" \
            "$SUMO_SOURCE_URL" "$SUMO_SRC" >/dev/null 2>&1 \
            || die "clone of SUMO $SUMO_TAG failed"
    else
        note "reusing SUMO checkout at $SUMO_SRC"
    fi
else
    [ -d "$SUMO_SRC/src/libsumo" ] || die "--sumo-src '$SUMO_SRC' does not look like a SUMO checkout"
    note "using SUMO checkout at $SUMO_SRC"
fi

BUILD_DIR="$CACHE_ROOT/sumo-build-$SUMO_VERSION"

# A CMake build dir is bound to the source dir it was configured from. Switching
# between --sumo-src and the default checkout otherwise fails with "does not
# match the source used to generate cache", so drop a cache that points
# elsewhere instead of making the user delete it by hand.
if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
    cached_src="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$BUILD_DIR/CMakeCache.txt" | head -1)"
    if [ -n "$cached_src" ] && [ "$cached_src" != "$SUMO_SRC" ]; then
        note "build cache was configured from $cached_src; reconfiguring"
        rm -rf "$BUILD_DIR"
    fi
fi

note "configuring (libtraci only)"
cmake -S "$SUMO_SRC" -B "$BUILD_DIR" $GENERATOR \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -DCHECK_OPTIONAL_LIBS=OFF \
      -DENABLE_PYTHON_BINDINGS=OFF \
      -DENABLE_JAVA_BINDINGS=OFF >"$BUILD_DIR.cfg.log" 2>&1 \
      || { tail -20 "$BUILD_DIR.cfg.log"; die "SUMO configure failed (log: $BUILD_DIR.cfg.log)"; }

if [ "$LIB_READY" -eq 0 ]; then
    note "building libtracicpp with $JOBS jobs"
    cmake --build "$BUILD_DIR" --target libtracicpp -j "$JOBS" >"$BUILD_DIR.bld.log" 2>&1 \
          || { grep -E 'error:' "$BUILD_DIR.bld.log" | head -20; die "libtracicpp build failed (log: $BUILD_DIR.bld.log)"; }
fi

# --- optional: the SUMO SERVER ------------------------------------------------
# libtraci is only a CLIENT; a co-simulation needs a sumo process to talk to.
# Neither platform's release ships one, so this builds the PINNED server from
# the same checkout -- guaranteeing client and server are the same version,
# which `apt install sumo` cannot (the distro and PPA track other versions).
# Headless 'sumo' only: sumo-gui would drag in fox/OpenGL for no test benefit.
if [ "$WITH_SERVER" -eq 1 ]; then
    if [ -x "$LIBSUMO_DIR/bin/sumo" ] && [ "$FORCE" -eq 0 ]; then
        note "sumo server already present at $LIBSUMO_DIR/bin/sumo"
    else
        note "building the sumo server with $JOBS jobs (slower: this is SUMO itself)"
        cmake --build "$BUILD_DIR" --target sumo -j "$JOBS" >"$BUILD_DIR.sumo.log" 2>&1 \
              || { grep -E "error:" "$BUILD_DIR.sumo.log" | head -20; die "sumo build failed (log: $BUILD_DIR.sumo.log)"; }
        SRV="$SUMO_SRC/bin/sumo"
        [ -x "$SRV" ] || SRV="$(find "$BUILD_DIR" -name sumo -type f -perm -u+x | head -1)"
        [ -n "$SRV" ] && [ -x "$SRV" ] || die "sumo binary was not produced"
        mkdir -p "$LIBSUMO_DIR/bin"
        cp "$SRV" "$LIBSUMO_DIR/bin/"
        note "sumo server installed: $LIBSUMO_DIR/bin/sumo"
    fi
fi

# Library came from the prebuilt asset; only the server needed building.
if [ "$LIB_READY" -eq 1 ]; then
    exit 0
fi

# --- install ------------------------------------------------------------------
BUILT_LIB="$SUMO_SRC/bin/libtracicpp.so"
[ -f "$BUILT_LIB" ] || BUILT_LIB="$(find "$BUILD_DIR" -name 'libtracicpp.so*' -type f | head -1)"
[ -n "$BUILT_LIB" ] && [ -f "$BUILT_LIB" ] || die "libtracicpp.so was not produced"

mkdir -p "$LIBSUMO_DIR/bin"
# Headers from the SAME checkout we just linked against, so the two cannot drift.
cp "$SUMO_SRC/src/libsumo/"*.h "$LIBSUMO_DIR/"
cp "$BUILT_LIB" "$LIBSUMO_DIR/bin/"

# --- verify -------------------------------------------------------------------
# The .ps1 load-tests its DLLs because the old vendored copy silently lacked
# geos_c.dll for months (#70). Same intent here: an unresolved NEEDED entry
# means the library is present but unusable.
[ -f "$LIBSUMO_DIR/libtraci.h" ] || die "libtraci.h missing after install"
if command -v ldd >/dev/null 2>&1; then
    if ldd "$SENTINEL" 2>/dev/null | grep -q 'not found'; then
        echo "WARNING: libtracicpp.so has unresolved dependencies:" >&2
        ldd "$SENTINEL" | grep 'not found' >&2
        die "libsumo loadability check failed"
    fi
    note "loadability check OK"
fi

echo "libsumo ready: $LIBSUMO_DIR"
echo "  headers: $(ls "$LIBSUMO_DIR"/*.h | wc -l) files"
echo "  library: $SENTINEL"
