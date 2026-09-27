#!/usr/bin/env bash
# =============================================================================
# pack_native_deps.sh -- Linux counterpart of scripts/dispatch/pack_native_deps.ps1
#
# Produces (and optionally publishes) a Linux native-deps asset for the public
# rolling 'fixs-native-deps' release, so a Linux clone can fetch a prebuilt
# library instead of building it -- which is what the Windows side already does.
#
#   --component sumo   (default)
#       libsumo-<sumo_ver>-linux-x86_64.zip
#       Upstream publishes no Linux binary for a PINNED SUMO version (the only
#       prebuilt Linux distribution is ppa:sumo/stable, which tracks a DIFFERENT
#       version), so we build the pinned one and publish it ourselves.
#
#           libsumo/*.h                   headers  (build time)
#           libsumo/bin/libtracicpp.so    library  (build + run time)
#
#   --component carla
#       libcarla-<carla_ver>-linux-x86_64.zip
#       CARLA's GitHub releases carry NO assets at all, so the C++ client SDK is
#       published nowhere; it exists only inside a built CARLA source tree, at
#       <carla_root>/PythonAPI/carla/dependencies. Building it needs UE4's clang
#       (Setup.sh derives CC/CXX from $UE4_ROOT), which no CI runner has -- so
#       the asset is produced HERE, on a box that has one, and consumed
#       everywhere else. Same acquisition shape as the Windows -Mode source
#       path, except that path copies all 468 MB of that directory; this ships
#       only what VirCarlaEnv compiles and links against (~20 MB):
#
#           libcarla/include/carla/**             the client API
#           libcarla/include/system/{boost,rpc,recast}/**
#           libcarla/lib/*.a                      stripped static archives
#
#       The static archives carry -g debug info: libcarla_client.a alone is
#       217 MB unstripped and 12.6 MB after --strip-debug, and still links.
#
# BUILD ON THE OLDEST SUPPORTED DISTRO. glibc is forward-compatible only, so an
# asset built on 20.04 (glibc 2.31) runs on 22.04 and 24.04, but not the
# reverse. One asset therefore covers the whole matrix; this script refuses to
# publish from a newer distro unless --allow-any-distro is given.
#
# Usage:
#   scripts/pack_native_deps.sh                              # sumo, into dist/
#   scripts/pack_native_deps.sh --publish                    # ... and upload
#   scripts/pack_native_deps.sh --component carla --publish
#   scripts/pack_native_deps.sh --component carla --carla-root ~/carla_0.9.15
#   scripts/pack_native_deps.sh --out-dir /tmp/x
# =============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG='fixs-native-deps'          # rolling, version-LESS on purpose (as in the .ps1)
GH_REPO='ORNL-Real-Sim/FIXS'
OUT_DIR="$REPO_ROOT/dist"
COMPONENT=sumo
CARLA_ROOT="${CARLA_ROOT:-}"    # env is a fallback; --carla-root overrides it
PUBLISH=0
ALLOW_ANY_DISTRO=0
NO_CONTAINER_CHECK=0
BUILD_ON="20.04"

die()  { echo "ERROR: $*" >&2; exit 1; }
note() { echo "  $*"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --component)         COMPONENT="$2"; shift 2 ;;
        --carla-root)        CARLA_ROOT="$2"; shift 2 ;;
        --publish)           PUBLISH=1; shift ;;
        --out-dir)           OUT_DIR="$2"; shift 2 ;;
        --repo)              GH_REPO="$2"; shift 2 ;;
        --allow-any-distro)  ALLOW_ANY_DISTRO=1; shift ;;
        --no-container-check) NO_CONTAINER_CHECK=1; shift ;;
        -h|--help)           sed -n '2,50p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) die "unknown argument: $1 (try --help)" ;;
    esac
done

case "$COMPONENT" in
    sumo|carla) ;;
    *) die "unknown --component '$COMPONENT' (expected sumo or carla)" ;;
esac

# --- versions come from dependencies.yaml, never a literal -------------------
# tr strips \r as well: dependencies.yaml is CRLF in the repo, and a trailing
# carriage return would silently poison the asset name and the paths below.
yaml_key() {
    awk -v blk="  $1:" '$0 ~ "^" blk {inblk=1; next} /^  [a-z]/{inblk=0} inblk' \
        "$REPO_ROOT/dependencies.yaml" \
        | sed -n "s/^[[:space:]]*$2:[[:space:]]*\"\{0,1\}\([^\"]*\)\"\{0,1\}[[:space:]]*$/\1/p" \
        | head -1 | tr -d '\r'
}

DEP_VERSION="$(yaml_key "$COMPONENT" version)"
DEP_LOCATION="$(yaml_key "$COMPONENT" location)"
[ -n "$DEP_VERSION" ]  || die "could not read $COMPONENT.version from dependencies.yaml"
[ -n "$DEP_LOCATION" ] || die "could not read $COMPONENT.location from dependencies.yaml"

if [ "$COMPONENT" = "sumo" ]; then
    ZIP_NAME="libsumo-${DEP_VERSION}-linux-x86_64.zip"
else
    ZIP_NAME="libcarla-${DEP_VERSION}-linux-x86_64.zip"
fi

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

STAGE="$(mktemp -d)"     # zipped verbatim -- only payload goes in here
SCRATCH="$(mktemp -d)"   # probes, logs, anything transient
trap 'rm -rf "$STAGE" "$SCRATCH"' EXIT

# =============================================================================
# Symbol-version baseline
# =============================================================================
# "Built on $BUILD_ON" is checked above by reading /etc/os-release, which is NOT
# the same thing as "links against $BUILD_ON's libraries". A developer box that
# has a newer toolchain installed -- gcc-13 from a PPA is common, and CARLA
# needs a modern one -- still says VERSION_ID=20.04 while its libstdc++ exports
# symbols focal never had. An artifact built there fails to LINK on stock focal,
# and neither the distro check nor the link probe (which runs against the local,
# newer libstdc++) notices.
#
# That is not hypothetical: the first libcarla asset packed here referenced
# std::__exception_ptr::exception_ptr::_M_addref (gcc 12+) from carla_client and
# std::__throw_bad_array_new_length (gcc 11+) from rpclib. It linked on the
# packing box and on 22.04/24.04, and failed on stock 20.04 -- the very distro
# the asset exists to serve.
#
# The check differs by artifact kind, and the difference is not cosmetic:
#
#   a SHARED LIBRARY records the symbol VERSION NODES it needs, and the loader
#   demands exactly those, so reading them off the .so is the whole answer.
#
#   a STATIC ARCHIVE records unversioned names; the version is chosen by
#   whoever links it. Reading versions off a probe binary linked HERE therefore
#   over-reports -- std::condition_variable::wait binds to GLIBCXX_3.4.30 on
#   this box and to an older node on focal, and both are correct. What matters
#   for an archive is whether the NAMES it needs exist on the baseline at all,
#   which only the baseline can answer: hence the container probe below.
BASE_GLIBC=2.31           # Ubuntu 20.04
BASE_GLIBCXX=3.4.28       # libstdc++6 10.x, focal's stock
BASE_CXXABI=1.3.12

# Highest version required from a symbol-version family (GLIBC/GLIBCXX/CXXABI).
# Empty when the artifact requires none, which is a pass, not a failure.
max_required_version() {
    objdump -p "$1" 2>/dev/null | grep -oE "$2_[0-9][0-9.]*" | sed "s/^$2_//" | sort -V | tail -1
}

check_baseline_symbols() {
    local file="$1" what="$2" spec fam base req
    command -v objdump >/dev/null 2>&1 || { note "objdump not found; skipping symbol-baseline check"; return 0; }
    for spec in "GLIBC:$BASE_GLIBC" "GLIBCXX:$BASE_GLIBCXX" "CXXABI:$BASE_CXXABI"; do
        fam="${spec%%:*}"; base="${spec##*:}"
        req="$(max_required_version "$file" "$fam")"
        [ -n "$req" ] || continue
        if [ "$(printf '%s\n%s\n' "$base" "$req" | sort -V | tail -1)" != "$base" ]; then
            die "$what requires ${fam}_${req}, but Ubuntu $BUILD_ON provides at most ${fam}_${base}.
       The artifact was linked against a NEWER toolchain than the oldest
       supported distro has, so it would fail to link or load there -- even
       though this box reports VERSION_ID=$BUILD_ON.
       Check what is installed:  dpkg -l | grep libstdc++6
       Rebuild the payload with that distro's stock compiler (for libcarla:
       cmake -DCMAKE_BUILD_TYPE=Client with a toolchain file naming g++-9, and
       rebuild rpclib the same way), or pack inside a stock container."
        fi
        note "$what: max ${fam}_${req} <= ${fam}_${base}"
    done
}

# =============================================================================
# libsumo
# =============================================================================
pack_sumo() {
    # --- produce the payload --------------------------------------------------
    # Reuse the fetch script rather than duplicating the SUMO build, so the
    # packed artifact is byte-for-byte what a developer's source build produces.
    note "building libtracicpp via fetch_native_deps.sh"
    "$REPO_ROOT/scripts/fetch_native_deps.sh" --force >/dev/null || die "fetch_native_deps.sh failed"

    local src_dir="$REPO_ROOT/$DEP_LOCATION"
    [ -f "$src_dir/bin/libtracicpp.so" ] || die "libtracicpp.so missing from $src_dir/bin"
    [ -f "$src_dir/libtraci.h" ]         || die "headers missing from $src_dir"

    # --- stage, exactly mirroring the Windows layout --------------------------
    mkdir -p "$STAGE/libsumo/bin"
    cp "$src_dir"/*.h                "$STAGE/libsumo/"
    cp "$src_dir/bin/libtracicpp.so" "$STAGE/libsumo/bin/"

    # --- verify BEFORE publishing ---------------------------------------------
    # The .ps1 load-tests its DLLs because a vendored copy once silently lacked
    # geos_c.dll for months (#70). Same intent: refuse to ship a library whose
    # NEEDED entries cannot resolve.
    if ldd "$STAGE/libsumo/bin/libtracicpp.so" | grep -q 'not found'; then
        ldd "$STAGE/libsumo/bin/libtracicpp.so" | grep 'not found' >&2
        die "libtracicpp.so has unresolved dependencies; refusing to pack"
    fi
    note "loadability check OK ($(ls "$STAGE/libsumo"/*.h | wc -l) headers)"

    # Same exposure as libcarla: this .so is built by the packing box's
    # compiler, so a newer-than-focal toolchain would produce a library that
    # cannot load on focal.
    check_baseline_symbols "$STAGE/libsumo/bin/libtracicpp.so" "libtracicpp.so"
}

# =============================================================================
# libcarla
# =============================================================================

# The archives VirCarlaEnv.vcxproj links, minus the ones with no Linux
# counterpart. zlib/zlibstatic are the system libz here; boost_python310 and
# carla_client_debug are vestigial on Windows too (no TU references them).
# carla_client/rpc/boost_filesystem/Recast/Detour/DetourCrowd are the set the
# link probe below actually needs; the rest are shipped because the Windows
# project links them and they cost 0.3 MB in total.
CARLA_LIBS="libcarla_client.a librpc.a libRecast.a libDetour.a libDetourCrowd.a
            libDetourTileCache.a libDebugUtils.a
            libboost_filesystem.a libboost_system.a libboost_atomic.a"

# Header trees reachable from the CARLA API VirCarlaEnv includes, established
# with `g++ -M` rather than by eye. Deliberately NOT shipped: libpng16/, png*.h,
# moodycamel/, pugixml/, odrSpiral/, OSM2ODR.h -- nothing includes them, and
# they belong to the server/osm2odr side of that directory.
CARLA_SYSTEM_INCLUDES="boost rpc recast"

resolve_carla_root() {
    # explicit flag > env > ~/.fixs/carla.json. The json is the same per-machine
    # config the Windows fetch script reads, but its carla_root may point at a
    # PACKAGED CARLA (mode: packaged), which has no PythonAPI/carla/dependencies
    # at all -- so a candidate only counts if that directory is really there.
    local candidates=() c
    [ -n "$CARLA_ROOT" ] && candidates+=("$CARLA_ROOT")
    local json="${FIXS_CACHE_DIR:-$HOME/.fixs}/carla.json"
    if [ -f "$json" ]; then
        c="$(sed -n 's/.*"carla_root"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$json" | head -1)"
        [ -n "$c" ] && candidates+=("$c")
    fi

    for c in ${candidates+"${candidates[@]}"}; do
        if [ -d "$c/PythonAPI/carla/dependencies/lib" ]; then
            echo "$c"; return 0
        fi
    done

    die "no CARLA source tree with a built client SDK was found.
       Looked for <root>/PythonAPI/carla/dependencies/lib in:
         ${candidates[*]:-(nothing configured)}
       Pass --carla-root <path to a CARLA source checkout>, or build the client
       first (see doc/Carla_Linux_building.md):
         cd <carla_root> && make setup && make LibCarla"
}

pack_carla() {
    local root deps ver
    root="$(resolve_carla_root)"
    deps="$root/PythonAPI/carla/dependencies"
    note "CARLA source tree: $root"

    # --- the SDK must be the version dependencies.yaml pins -------------------
    # carla/Version.h is generated from `git describe` of the tree that built
    # it, so it is the only in-band provenance the artifact carries.
    ver="$(sed -n 's/.*return "\(.*\)";.*/\1/p' "$deps/include/carla/Version.h" | head -1)"
    [ -n "$ver" ] || die "could not read a version from $deps/include/carla/Version.h"
    case "$ver" in
        "$DEP_VERSION"*) ;;
        *) die "the built SDK is CARLA $ver but dependencies.yaml pins $DEP_VERSION.
       Check out the pinned version and rebuild the client, or bump the yaml." ;;
    esac
    note "SDK version stamp: $ver"
    case "$ver" in
        *dirty*) echo "  WARNING: built from a MODIFIED CARLA checkout ($ver)." >&2
                 echo "           Record what was changed -- the asset is not reproducible from" >&2
                 echo "           the upstream tag alone. See doc/Carla_Linux_building.md." >&2 ;;
    esac

    # --- stage ----------------------------------------------------------------
    # Leading libcarla/ directory, so the zip extracts into the PARENT of the
    # configured location -- the same shape as the libsumo asset and as what
    # Expand-Archive produces on Windows.
    local out="$STAGE/libcarla"
    mkdir -p "$out/lib" "$out/include/system"
    cp -a "$deps/include/carla" "$out/include/"
    local sub
    for sub in $CARLA_SYSTEM_INCLUDES; do
        [ -d "$deps/include/system/$sub" ] || die "missing header tree: include/system/$sub"
        cp -a "$deps/include/system/$sub" "$out/include/system/"
    done

    local l
    for l in $CARLA_LIBS; do
        [ -f "$deps/lib/$l" ] || die "missing archive: lib/$l (was LibCarla built for the CLIENT target?)"
        cp "$deps/lib/$l" "$out/lib/"
    done

    # Debug info is 92% of the payload and nothing consumes it: VirCarlaEnv is
    # not debugged through CARLA's internals, and the archives still link.
    strip --strip-debug "$out"/lib/*.a
    note "staged $(ls "$out/lib" | wc -l) archives ($(du -sh "$out/lib" | cut -f1) stripped), headers $(du -sh "$out/include" | cut -f1)"

    verify_carla "$out"
}

# Compile AND link a TU against the STAGED tree with the system compiler. This
# is the analogue of libsumo's loadability check, and it is the check that
# matters here: the SDK is built by UE4's clang while every consumer uses the
# distro's g++, so the one thing that can silently go wrong is an ABI or
# missing-archive mismatch that only surfaces at link time. The probe uses the
# same CARLA API surface VirCarlaEnv does, so a shipped asset that cannot build
# VirCarlaEnv cannot pass.
verify_carla() {
    local out="$1" probe="$SCRATCH/probe"
    command -v g++ >/dev/null 2>&1 || die "'g++' not found; cannot verify the staged SDK"
    mkdir -p "$probe"
    cat > "$probe/probe.cpp" <<'PROBE'
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/client/ActorList.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Vehicle.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/DebugHelper.h>
#include <carla/trafficmanager/TrafficManager.h>
#include <carla/rpc/Command.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/geom/Transform.h>
int main() {
    carla::client::Client client("127.0.0.1", 2000);
    client.SetTimeout(std::chrono::seconds(1));
    auto world = client.GetWorld();
    auto map = world.GetMap();
    auto wp = map->GetWaypoint(carla::geom::Location(0, 0, 0));
    auto bp = world.GetBlueprintLibrary();
    auto actors = world.GetActors();
    carla::traffic_manager::TrafficManager tm(client.GetInstanceTM(8000));
    auto dbg = world.MakeDebugHelper();
    dbg.DrawPoint(carla::geom::Location(0, 0, 0));
    carla::rpc::VehicleControl ctl;
    return (int)client.GetClientVersion().size() + (bp ? 1 : 0)
         + (int)actors->size() + (wp ? 1 : 0) + (int)ctl.throttle;
}
PROBE
    note "verifying: compiling and linking a probe against the staged SDK"
    g++ -std=c++17 -pthread -O1 -DNDEBUG \
        -I"$out/include" -isystem "$out/include/system" \
        -c "$probe/probe.cpp" -o "$probe/probe.o" 2>"$probe/cc.log" \
        || { tail -20 "$probe/cc.log" >&2; die "the staged headers do not compile"; }

    g++ -std=c++17 -pthread -o "$probe/probe" "$probe/probe.o" -L"$out/lib" \
        -Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic \
        -L"$out/lib" -lRecast -lDetour -lDetourCrowd 2>"$probe/ld.log" \
        || { tail -20 "$probe/ld.log" >&2; die "the staged archives do not link"; }

    # A libc++-built client would fail the link above, but say so explicitly:
    # a silent std::string ABI mismatch is the failure mode this asset is most
    # exposed to (#65, Q1).
    if nm "$out/lib/libcarla_client.a" 2>/dev/null | grep -q '_ZNSt3__1'; then
        die "libcarla_client.a carries libc++ symbols (_ZNSt3__1).
       The CLIENT must be built against libstdc++; libc++ is the UE4/server
       toolchain. Rebuild with the client target and re-pack."
    fi
    note "link probe OK (libstdc++ ABI, $(du -h "$probe/probe" | cut -f1) binary)"

    # Cheap always-on screen for the two failures we have actually hit, so a
    # box without a container runtime is not left with no check at all. It is a
    # heuristic -- a list of known post-baseline symbols, not a proof -- which
    # is why the container probe below is the one that decides.
    local bad
    # '|| true': a clean payload means grep matches nothing and exits 1, which
    # set -e would otherwise treat as the script failing at its own success.
    bad="$(nm -u "$out"/lib/*.a 2>/dev/null \
           | grep -oE '_M_addref|_M_release|__throw_bad_array_new_length' | sort -u | tr '\n' ' ' || true)"
    if [ -n "$bad" ]; then
        die "the staged archives reference libstdc++ symbols newer than Ubuntu $BUILD_ON: $bad
       They come from an archive compiled against a NEWER libstdc++ than focal's
       (gcc 11 added __throw_bad_array_new_length, gcc 12 exception_ptr::_M_addref),
       which links here and fails on stock $BUILD_ON. Rebuild the offending
       archive with that distro's stock compiler -- see doc/Carla_Linux_building.md."
    fi

    verify_carla_on_baseline "$out"
    rm -rf "$probe"
}

# The authoritative check: link the same probe inside a STOCK Ubuntu $BUILD_ON
# container. Only the baseline itself can answer whether the archives' undefined
# symbols exist there, and this is the check that caught the first asset -- which
# passed everything above and still failed to link on stock focal.
#
# Skipped, loudly, when no container runtime is available: a machine without one
# can still pack, it just cannot prove the result. --no-container-check makes
# that deliberate rather than accidental.
verify_carla_on_baseline() {
    local out="$1" engine=""
    if [ "$NO_CONTAINER_CHECK" -eq 1 ]; then
        echo "  WARNING: baseline container check SKIPPED (--no-container-check)." >&2
        return 0
    fi
    for e in docker podman; do
        if command -v "$e" >/dev/null 2>&1 && "$e" info >/dev/null 2>&1; then engine="$e"; break; fi
    done
    if [ -z "$engine" ]; then
        echo "  WARNING: no usable docker/podman, so the staged SDK was NOT link-tested on" >&2
        echo "           stock Ubuntu $BUILD_ON. The local probe passes even when the asset" >&2
        echo "           cannot link there, so verify before trusting this artifact." >&2
        return 0
    fi

    cat > "$SCRATCH/baseline_probe.sh" <<'BASELINE'
set -e
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/dev/null
apt-get install -y -qq --no-install-recommends build-essential >/dev/null
cd /stage
g++ -std=c++17 -pthread -O1 -DNDEBUG -Ilibcarla/include -isystem libcarla/include/system \
    -c /scratch/probe/probe.cpp -o /tmp/probe.o
g++ -std=c++17 -pthread -o /tmp/probe /tmp/probe.o -Llibcarla/lib \
    -Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic \
    -Llibcarla/lib -lRecast -lDetour -lDetourCrowd
BASELINE

    note "verifying on stock Ubuntu $BUILD_ON via $engine (authoritative)"
    if ! "$engine" run --rm \
            -v "$STAGE:/stage:ro" \
            -v "$SCRATCH:/scratch:ro" \
            "ubuntu:$BUILD_ON" bash /scratch/baseline_probe.sh 2>"$SCRATCH/baseline.log"; then
        grep -E 'undefined reference|error:' "$SCRATCH/baseline.log" | head -10 >&2
        die "the staged SDK does not link on stock Ubuntu $BUILD_ON (full log: $STAGE/baseline.log
       -- copy it before this script exits, the staging directory is temporary).
       It links HERE only because this box has a newer libstdc++ than focal
       ships. Rebuild the payload with the baseline compiler."
    fi
    note "baseline link OK on stock Ubuntu $BUILD_ON"
}

if [ "$COMPONENT" = "sumo" ]; then pack_sumo; else pack_carla; fi

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
