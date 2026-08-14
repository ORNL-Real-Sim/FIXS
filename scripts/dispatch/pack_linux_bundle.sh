#!/usr/bin/env bash
# =============================================================================
# pack_linux_bundle.sh -- the Linux counterpart of 8_create_zip.ps1
#
#   fixs-build-<version>-linux-x86_64.zip
#
# A SECOND asset on the same release as the Windows bundle, not a replacement:
# the two are consumed by update_fixs.{sh,ps1}, which pick by platform tag. That
# split exists for a reason Compress-Archive cannot solve -- it does not store
# the Unix execute bit, so a Linux binary packed on Windows extracts
# unrunnable. This zip is therefore built on Linux, by zip(1), which does.
#
# Contents mirror the Windows bundle minus what cannot exist here:
#
#   TrafficLayer, VirCarlaEnv     the two Linux targets (VirCarlaEnv optional)
#   CommonLib/                    python + libsumo HEADERS (bin/ is fetched from
#                                 the native-deps release, exactly as on Windows)
#   Carla/                        the co-sim component, minus wheels/__pycache__
#   environment.yml, BUILD_INFO.txt
#
#   NOT here: carmaker/ and vissim/ (licensed Windows toolchains), and
#   VirtualEnvironment.lib (an MSVC static library).
#
# BUILD ON THE OLDEST SUPPORTED DISTRO. glibc is forward-compatible only, so a
# focal-built binary runs on 20.04/22.04/24.04 while a noble-built one runs on
# noble alone -- verified by running both across all three. One bundle serves
# every supported Ubuntu; this script refuses to pack elsewhere without
# --allow-any-distro, and re-checks the binaries themselves below.
#
# Usage:
#   scripts/dispatch/pack_linux_bundle.sh                    # -> dist/
#   scripts/dispatch/pack_linux_bundle.sh --version v0.9.0-alpha
#   scripts/dispatch/pack_linux_bundle.sh --build-dir build --out-dir /tmp/x
# =============================================================================
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="$REPO_ROOT/build"
OUT_DIR="$REPO_ROOT/dist"
VERSION=""
ALLOW_ANY_DISTRO=0
BUILD_ON="20.04"
# The floor focal provides. A bundle above it does not run on the distro the
# bundle exists to serve -- the same check pack_native_deps.sh applies to the .so.
BASE_GLIBC=2.31
BASE_GLIBCXX=3.4.28

die()  { echo "ERROR: $*" >&2; exit 1; }
note() { echo "  $*"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --version)          VERSION="$2";  shift 2 ;;
        --build-dir)        BUILD_DIR="$2"; shift 2 ;;
        --out-dir)          OUT_DIR="$2";  shift 2 ;;
        --allow-any-distro) ALLOW_ANY_DISTRO=1; shift ;;
        -h|--help)          sed -n '2,36p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) die "unknown argument: $1 (try --help)" ;;
    esac
done

command -v zip >/dev/null 2>&1 || die "'zip' not found: sudo apt-get install -y zip"
[ -x "$BUILD_DIR/TrafficLayer" ] || die "no TrafficLayer in $BUILD_DIR
       Build it first:  scripts/dispatch/dispatch.sh --with-carla"

# NOT `. /etc/os-release`: it defines VERSION itself ("20.04.6 LTS (Focal
# Fossa)"), which silently overwrote the release label and produced a zip named
# fixs-build-20.04.6 LTS (Focal Fossa)-linux-x86_64.zip. Read the two fields in
# a subshell so the file's variables never land in this scope.
VERSION_ID="$(. /etc/os-release; echo "${VERSION_ID:-}")"
PRETTY_NAME="$(. /etc/os-release; echo "${PRETTY_NAME:-linux}")"
if [ "${VERSION_ID:-}" != "$BUILD_ON" ] && [ "$ALLOW_ANY_DISTRO" -eq 0 ]; then
    die "refusing to pack on Ubuntu ${VERSION_ID:-unknown}.
       glibc is forward-compatible only, so the published bundle must be built
       on the OLDEST supported distro ($BUILD_ON) or it will not run there.
       Override with --allow-any-distro if you know what you are doing."
fi

# --- version label ------------------------------------------------------------
# Same source of truth as generate_version.ps1: the nearest v* tag. The zip is
# named after the RELEASE CHANNEL when one is given (--version latest), because
# a git-derived name like v0.9.0-alpha-1-g65f2970d reads as though a different
# alpha was published.
if [ -z "$VERSION" ]; then
    VERSION="$(git -C "$REPO_ROOT" describe --tags --match 'v[0-9]*' --abbrev=0 2>/dev/null)"
    [ -n "$VERSION" ] || VERSION="v0.0.0"
fi
ZIP_NAME="fixs-build-${VERSION}-linux-x86_64.zip"

echo "Packing $ZIP_NAME"
note "from: $BUILD_DIR"
note "on:   $PRETTY_NAME (glibc $(ldd --version | head -1 | grep -oE '[0-9]+\.[0-9]+$'))"

STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

# --- binaries -----------------------------------------------------------------
cp "$BUILD_DIR/TrafficLayer" "$STAGE/"
if [ -x "$BUILD_DIR/VirCarlaEnv" ]; then
    cp "$BUILD_DIR/VirCarlaEnv" "$STAGE/"
    note "binaries: TrafficLayer, VirCarlaEnv"
else
    # Not fatal: VirCarlaEnv needs the CARLA client SDK, which is optional.
    # Silently shipping a bundle that lacks it would be, though.
    echo "  WARNING: no VirCarlaEnv in $BUILD_DIR -- the bundle will not contain it." >&2
    echo "           Build it with scripts/dispatch/dispatch.sh --with-carla." >&2
    note "binaries: TrafficLayer"
fi
chmod +x "$STAGE"/TrafficLayer "$STAGE"/VirCarlaEnv 2>/dev/null

# --- the same baseline gate the native-deps packer applies --------------------
# A dev box can BE 20.04 and still carry a newer libstdc++ (this one does: 13.1),
# in which case the distro check above passes and the binary still fails to load
# on stock focal. Read what the artifact actually requires.
max_ver() { objdump -p "$1" 2>/dev/null | grep -oE "$2_[0-9][0-9.]*" | sed "s/^$2_//" | sort -V | tail -1; }
for bin in "$STAGE"/TrafficLayer "$STAGE"/VirCarlaEnv; do
    [ -f "$bin" ] || continue
    for fam in "GLIBC:$BASE_GLIBC" "GLIBCXX:$BASE_GLIBCXX"; do
        f="${fam%%:*}"; base="${fam##*:}"; req="$(max_ver "$bin" "$f")"
        [ -n "$req" ] || continue
        [ "$(printf '%s\n%s\n' "$base" "$req" | sort -V | tail -1)" = "$base" ] || die \
"$(basename "$bin") requires ${f}_${req}, but Ubuntu $BUILD_ON provides at most ${f}_${base}.
       It was linked against a newer toolchain than the oldest supported distro
       has, so it cannot load there -- even though this box reports $BUILD_ON.
       Build it in a stock ubuntu:$BUILD_ON container (which is what CI does)."
    done
    note "$(basename "$bin"): GLIBC_$(max_ver "$bin" GLIBC) / GLIBCXX_$(max_ver "$bin" GLIBCXX) <= focal baseline"
done

# --- CommonLib: everything but the libsumo runtime ----------------------------
# bin/ is fetched from the native-deps release by update_fixs.sh, exactly as on
# Windows, so the bundle carries headers only. YAMLMatlab/Tests is dropped for
# the reason 8_create_zip.ps1 gives: dead weight whose directories extract
# without the execute bit and then break `rm -rf FIXS/` (#190).
mkdir -p "$STAGE/CommonLib"
for item in "$REPO_ROOT"/CommonLib/*; do
    name="$(basename "$item")"
    case "$name" in
        libsumo|libcarla) continue ;;
        *) cp -r "$item" "$STAGE/CommonLib/" ;;
    esac
done
if [ -d "$REPO_ROOT/CommonLib/libsumo" ]; then
    mkdir -p "$STAGE/CommonLib/libsumo"
    find "$REPO_ROOT/CommonLib/libsumo" -maxdepth 1 -type f -exec cp {} "$STAGE/CommonLib/libsumo/" \;
fi
rm -rf "$STAGE/CommonLib/YAMLMatlab/Tests"
find "$STAGE/CommonLib" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null

# --- Carla/ co-sim component ---------------------------------------------------
if [ -d "$REPO_ROOT/Carla" ]; then
    mkdir -p "$STAGE/Carla"
    for item in "$REPO_ROOT"/Carla/*; do
        case "$(basename "$item")" in *.whl) continue ;; esac
        cp -r "$item" "$STAGE/Carla/"
    done
    find "$STAGE/Carla" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
    note "+ Carla/ co-sim component"
fi

[ -f "$REPO_ROOT/environment.yml" ] && cp "$REPO_ROOT/environment.yml" "$STAGE/"

# --- BUILD_INFO.txt -------------------------------------------------------------
# Not decoration: update_fixs.{sh,ps1} read the SUMO version from this file to
# pin which native runtime to install, and a bundle without it silently falls
# back to a looser match. Same keys as 7_build_info.ps1 writes, so one parser
# serves both bundles.
if [ -f "$BUILD_DIR/BUILD_INFO.txt" ]; then
    cp "$BUILD_DIR/BUILD_INFO.txt" "$STAGE/"
else
    yaml_key() {
        awk -v blk="  $1:" '$0 ~ "^" blk {i=1; next} /^  [a-z]/{i=0} i' "$REPO_ROOT/dependencies.yaml" \
            | sed -n "s/^[[:space:]]*$2:[[:space:]]*\"\{0,1\}\([^\"]*\)\"\{0,1\}[[:space:]]*$/\1/p" | head -1 | tr -d '\r'
    }
    {
        echo "================================================================================"
        echo "RealSim FIXS Build Information"
        echo "================================================================================"
        echo
        echo "BUILD METADATA"
        echo "--------------"
        echo "Build Date:           $(date '+%a %m/%d/%Y %H:%M:%S')"
        echo "Build Configuration:  Release"
        echo "Build Platform:       linux-x86_64 ($PRETTY_NAME, glibc $(ldd --version | head -1 | grep -oE '[0-9]+\.[0-9]+$'))"
        echo
        echo "SOURCE INFORMATION"
        echo "------------------"
        echo "Git Branch:           $(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null)"
        echo "Git Commit:           $(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null)"
        echo "Git Tag:              $VERSION"
        echo
        echo "DEPENDENCY VERSIONS"
        echo "-------------------"
        echo "Simulators:"
        echo "  SUMO:               $(yaml_key sumo version)"
        echo "  CARLA:              $(yaml_key carla version)"
    } > "$STAGE/BUILD_INFO.txt"
    note "generated BUILD_INFO.txt (no build/BUILD_INFO.txt present)"
fi

# --- zip ------------------------------------------------------------------------
mkdir -p "$OUT_DIR"
ZIP_PATH="$OUT_DIR/$ZIP_NAME"
rm -f "$ZIP_PATH"
( cd "$STAGE" && zip -qr "$ZIP_PATH" . )

# Prove the execute bit survived rather than assuming it: that property is the
# entire reason this bundle is packed on Linux instead of by Compress-Archive.
MODE="$(unzip -Z "$ZIP_PATH" TrafficLayer 2>/dev/null | awk 'NR==1{print $1}')"
case "$MODE" in
    *x*) note "execute bit preserved in the zip ($MODE TrafficLayer)" ;;
    *)   die "TrafficLayer is not executable inside the zip ($MODE); consumers would have to chmod it." ;;
esac

echo "Packed $ZIP_NAME ($(du -h "$ZIP_PATH" | cut -f1))"
echo "  sha256=$(sha256sum "$ZIP_PATH" | cut -d' ' -f1)"
