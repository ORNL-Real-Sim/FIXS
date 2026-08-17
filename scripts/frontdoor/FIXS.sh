#!/usr/bin/env bash
# FIXS_FRONTDOOR: 1
# ============================================================================
#  FIXS co-simulation - the front door (Linux/macOS).
#
#  Download this one file into a repo, run it, and it installs FIXS and runs a
#  co-sim. Nothing else is required to start:
#
#      curl -fsSLO https://github.com/ORNL-Real-Sim/FIXS/releases/latest/download/FIXS.sh
#      chmod +x FIXS.sh && ./FIXS.sh
#
#  WHAT THIS FILE IS ALLOWED TO KNOW is deliberately almost nothing: where FIXS
#  comes from, whether it is installed, and which python to hand over to. Every
#  option, every menu and every error message belongs to the engine, and this
#  forwards its whole command line untouched - so there is no flag list here to
#  drift from the engine's, which is exactly how the old per-repo wrappers
#  accumulated bugs (see FIXS#313).
#
#  It is authored in FIXS (scripts/frontdoor/) and published as a release asset.
#  Do not edit your copy: an update will tell you when the contract version above
#  has moved, and the answer is to re-download rather than to patch.
#
#      ./FIXS.sh --help              every option, from the engine
#      ./FIXS.sh --update-fixs       fetch or refresh the FIXS build
# ============================================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

FRONTDOOR_CONTRACT=1
DEFAULT_REPO="ORNL-Real-Sim/FIXS"

# ---------------------------------------------------------------------------
# The two values needed before any engine code exists on disk: where to fetch
# FIXS from, and which release this repo pins. They live in fixs.json, the same
# file that declares the applications - one file per integration.
#
# Read with grep, not a JSON parser, and ONLY these two keys. That is the whole
# contract: no matter how the manifest schema grows, nothing here has to grow
# with it, because everything else is read by app_catalog in python after the
# engine is installed. fixs_sources.txt is still honoured for repos integrated
# before fixs.json existed.
# ---------------------------------------------------------------------------
json_field() {   # json_field <file> <block> <key>
    [[ -f "$1" ]] || return 0
    tr -d '\n' < "$1" \
        | grep -o "\"$2\"[[:space:]]*:[[:space:]]*{[^}]*}" \
        | grep -o "\"$3\"[[:space:]]*:[[:space:]]*\"[^\"]*\"" \
        | head -n1 | sed 's/.*"\([^"]*\)"$/\1/'
}

txt_field() {    # txt_field <file> <key>   (legacy fixs_sources.txt)
    [[ -f "$1" ]] || return 0
    sed -n "s/^[[:space:]]*$2[[:space:]]*=[[:space:]]*//p" "$1" | head -n1 | tr -d '[:space:]'
}

MANIFEST="$ROOT/fixs.json"
LEGACY="$ROOT/fixs_sources.txt"
FIXS_REPO="$(json_field "$MANIFEST" fixs repo)"
FIXS_VERSION="$(json_field "$MANIFEST" fixs version)"
[[ -n "$FIXS_REPO"    ]] || FIXS_REPO="$(txt_field "$LEGACY" fixs_repo)"
[[ -n "$FIXS_VERSION" ]] || FIXS_VERSION="$(txt_field "$LEGACY" fixs_default_version)"
FIXS_REPO="${FIXS_REPO:-$DEFAULT_REPO}"

# The env applications run in. FIXS defaults to 'realsim', the name its own
# environment.yml carries; a repo that wants its apps' extra packages kept out of
# the engine's env names its own here. Already exported? that wins.
MANIFEST_ENV="$(json_field "$MANIFEST" fixs env)"
[[ -n "${FIXS_ENV_NAME:-}" ]] || [[ -z "$MANIFEST_ENV" ]] || export FIXS_ENV_NAME="$MANIFEST_ENV"

# ---------------------------------------------------------------------------
# Bootstrap. The updater lives in FIXS and is fetched from the release being
# installed, so the unpacker always matches the bundle it unpacks (#272). What
# stays here is only which repo, which ref, and run it against our root - a
# contract that does not change when the release format does, which is what makes
# it safe for this file to sit in every application repo.
# ---------------------------------------------------------------------------
fetch_fixs() {   # fetch_fixs [VERSION]
    local want="${1:-}" ref tmp url rc
    # 'main' is the last resort, not a version: a script taken from main can still
    # list the releases and hand off to whichever one is chosen.
    ref="${want:-${FIXS_VERSION:-main}}"

    command -v curl >/dev/null 2>&1 || {
        echo "[FIXS] curl is required to fetch the updater." >&2; return 1; }

    tmp="$(mktemp "${TMPDIR:-/tmp}/update_fixs-XXXXXX.sh")"
    url="https://raw.githubusercontent.com/$FIXS_REPO/$ref/scripts/update_fixs.sh"
    if ! curl -fsSL "$url" -o "$tmp"; then
        if [[ "$ref" != "main" ]]; then
            echo "[FIXS] no updater at '$ref'; falling back to 'main'."
            ref="main"
            url="https://raw.githubusercontent.com/$FIXS_REPO/main/scripts/update_fixs.sh"
        fi
        if ! curl -fsSL "$url" -o "$tmp"; then
            rm -f "$tmp"
            echo "[FIXS] Could not download the FIXS updater from $url" >&2
            echo "[FIXS] Check your network, or that '$FIXS_REPO' is reachable." >&2
            return 1
        fi
    fi

    local args=(--root "$ROOT" --repo "$FIXS_REPO" --self-ref "$ref")
    [[ -n "$want"         ]] && args+=(--version "$want")
    [[ -n "$FIXS_VERSION" ]] && args+=(--default-version "$FIXS_VERSION")
    bash "$tmp" "${args[@]}"; rc=$?
    rm -f "$tmp"
    return $rc
}

# Record what was installed, so a fresh clone of this repo bootstraps the same
# engine without anyone hand-writing config. Only ever CREATED, never edited: once
# the file exists it is the repo's, and it is where apps get declared.
seed_manifest() {
    [[ -f "$MANIFEST" ]] && return 0
    [[ -f "$LEGACY"   ]] && return 0     # an older integration already has its config
    local ver
    ver="$(sed -n '1s/ .*//p' "$ROOT/FIXS/FIXS_VERSION.txt" 2>/dev/null)"
    [[ -n "$ver" ]] || return 0
    cat > "$MANIFEST" <<JSON
{
  "schema": 2,
  "fixs": { "repo": "$FIXS_REPO", "version": "$ver" },
  "apps": []
}
JSON
    echo "[FIXS] wrote $MANIFEST - commit it; it pins the engine this repo runs."
    echo "[FIXS] Declare your applications in its \"apps\" list when you have some."
}

# --update-fixs is the ONE name this file answers, because it is the one action
# that must work before there is an engine to answer it. Everything else falls
# through to run_cosim, whose --help is the reference.
if [[ "${1:-}" == "--update-fixs" ]]; then
    shift
    fetch_fixs "${1:-}" || exit $?
    seed_manifest
    exit 0
fi

# The gate is FIXS_VERSION.txt, not any .py: the updater writes that marker LAST
# and only on a complete install, whereas the python ships inside the build zip
# and exists the moment it is unpacked - before the native runtime is fetched. A
# fetch that died at the runtime step used to leave a headers-only bundle that
# still satisfied the old gate, and the co-sim then failed with "Unable to locate
# SUMO library directory" instead of anything about the failed update.
if [[ ! -f "$ROOT/FIXS/FIXS_VERSION.txt" ]]; then
    if [[ -e "$ROOT/FIXS" ]]; then
        echo "[FIXS] the FIXS build is incomplete (no FIXS_VERSION.txt) - refetching ..."
    else
        echo "[FIXS] FIXS is not installed here - fetching it first ..."
    fi
    fetch_fixs || { echo "[FIXS] setup failed - see above. Not continuing." >&2; exit 1; }
    seed_manifest
fi

# Say so when this file is older than the engine it just installed. It is never
# overwritten in place: a running shell reads its own script incrementally, and
# this file is also the repo's committed entry point. Re-downloading is the fix.
SHIPPED="$ROOT/FIXS/frontdoor/FIXS.sh"
if [[ -f "$SHIPPED" ]]; then
    want="$(sed -n 's/^# FIXS_FRONTDOOR: *\([0-9]*\).*/\1/p' "$SHIPPED" | head -n1)"
    if [[ -n "$want" && "$want" != "$FRONTDOOR_CONTRACT" ]]; then
        echo "[FIXS] this FIXS.sh is contract v$FRONTDOOR_CONTRACT; the installed"
        echo "[FIXS] build expects v$want. Copy FIXS/frontdoor/FIXS.sh over it."
    fi
fi

# Any python 3 is enough: run_cosim re-execs under the interpreter carla.json
# names, so conda never needs to be active and this need not know the env.
PY="$(command -v python3 || command -v python || true)"
if [[ -z "$PY" ]]; then
    echo "[FIXS] No python 3 found on PATH. Install one, then run ./FIXS.sh --setup" >&2
    exit 1
fi

exec "$PY" "$ROOT/FIXS/cosim/run_cosim.py" "$@"
