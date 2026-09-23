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
#  WHAT THIS FILE IS ALLOWED TO KNOW is deliberately little: where FIXS comes
#  from, whether it is installed, which python to hand over to, and how to be
#  pleasant to someone who started it with no arguments. Every option, every menu
#  and every error message belongs to the engine, and this forwards its command
#  line untouched - there is no flag translation here to drift from the engine's,
#  which is exactly how the old per-repo wrappers accumulated bugs (see FIXS#313).
#  The option list below is a short guide to engine names, not a second parser.
#
#  The no-arguments prompt and the python search come from FIXS_Applications'
#  run_cosim.sh, where each one answered a real support question; they move here
#  so that repo loses nothing by switching to this file.
#
#  It is authored in FIXS (scripts/frontdoor/) and published as a release asset.
#  Do not edit your copy: an update will tell you when the contract version above
#  has moved, and the answer is to re-download rather than to patch.
#
#      ./FIXS.sh                     run (from a terminal: asks for options first)
#      ./FIXS.sh --gui               the FIXS window
#      ./FIXS.sh --help              the common options
#      ./FIXS.sh --update-fixs       fetch or refresh the FIXS build
# ============================================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

FRONTDOOR_CONTRACT=1
DEFAULT_REPO="ORNL-Real-Sim/FIXS"

# ---------------------------------------------------------------------------
# The values needed before any engine code exists on disk: where to fetch FIXS
# from, which release this repo pins, and which env its apps run in. They live in
# fixs.json, the same file that declares the applications.
#
# Read with grep, not a JSON parser, and ONLY these keys. That is the whole
# contract: however the manifest schema grows, nothing here has to grow with it,
# because everything else is read by app_catalog in python after the engine is
# installed. fixs_sources.txt is still honoured for repos integrated before
# fixs.json existed.
# ---------------------------------------------------------------------------
# An ABSENT key is a normal answer, not a failure - most of these are optional.
# grep exits 1 when it matches nothing, and under `set -e` with pipefail that
# would abort the whole script at the assignment, silently and with rc=1. So each
# reader swallows its own miss and returns the empty string.
json_field() {   # json_field <file> <block> <key>  -> value, or ""
    local out=""
    [[ -f "$1" ]] || return 0
    out="$(tr -d '\n' < "$1" \
        | grep -o "\"$2\"[[:space:]]*:[[:space:]]*{[^}]*}" \
        | grep -o "\"$3\"[[:space:]]*:[[:space:]]*\"[^\"]*\"" \
        | head -n1 | sed 's/.*"\([^"]*\)"$/\1/')" || out=""
    printf '%s' "$out"
}

txt_field() {    # txt_field <file> <key> -> value, or ""   (legacy fixs_sources.txt)
    local out=""
    [[ -f "$1" ]] || return 0
    out="$(sed -n "s/^[[:space:]]*$2[[:space:]]*=[[:space:]]*//p" "$1" \
        | head -n1 | tr -d '[:space:]')" || out=""
    printf '%s' "$out"
}

MANIFEST="$ROOT/fixs.json"
LEGACY="$ROOT/fixs_sources.txt"
FIXS_REPO="$(json_field "$MANIFEST" fixs repo)"
FIXS_VERSION="$(json_field "$MANIFEST" fixs version)"
MANIFEST_ENV="$(json_field "$MANIFEST" fixs env)"
[[ -n "$FIXS_REPO"    ]] || FIXS_REPO="$(txt_field "$LEGACY" fixs_repo)"
[[ -n "$FIXS_VERSION" ]] || FIXS_VERSION="$(txt_field "$LEGACY" fixs_default_version)"
[[ -n "$MANIFEST_ENV" ]] || MANIFEST_ENV="$(txt_field "$LEGACY" fixs_env)"
FIXS_REPO="${FIXS_REPO:-$DEFAULT_REPO}"

# The env applications run in. FIXS defaults to 'realsim', the name its own
# environment.yml carries; a repo that wants its apps' extra packages kept out of
# the engine's env names its own here. Already exported? that wins.
[[ -n "${FIXS_ENV_NAME:-}" ]] || [[ -z "$MANIFEST_ENV" ]] || export FIXS_ENV_NAME="$MANIFEST_ENV"

# The option list, shared by --help and the no-arguments prompt: one list, so a
# name cannot appear in one and not the other. Three groups, each one question:
# what to do instead of a run, where CARLA is, what to run.
print_options() {
    cat <<'OPTIONS_TEXT'
  --gui                         the FIXS window: pick, run, stop, watch the log
  --setup [carla]               configure a simulator here (default: carla)
  --update-python               rebind the python env, keeping the CARLA setup
  --import-map [MAP]            install a map (no MAP: list the published ones)
  --update-fixs [VERSION]       fetch or refresh the FIXS build
                                (no VERSION: pick from a menu)
  --version                     what is installed here
  --doctor                      check this machine can run a co-sim
  --cleanup                     stop what a crashed run left behind

  --peer HOST[:PORT]            CARLA runs there; this machine runs the traffic half
  --serve                       CARLA runs here; wait for the traffic machine to call
  --sumo-only                   traffic only, no CARLA, nothing rendered

  --map NAME                    the map, instead of the menu asking
  --sumocfg PATH                the SUMO scenario, instead of the menu asking
  --app-args "ARGS"             extra arguments for the app's own controller
OPTIONS_TEXT
}

usage() {
    cat <<'HELP_HEAD'
FIXS - co-simulation

USAGE
  ./FIXS.sh                     run it. Asks what to run, remembers, replays.
                                Everything is changed from that menu.

HELP_HEAD
    print_options
    cat <<'HELP_TAIL'

These are the engine's own option names; this file passes them through. The
full list, for scripts and developers:
  python3 FIXS/cosim/run_cosim.py --help
HELP_TAIL
}

# ---------------------------------------------------------------------------
# The python to hand over to. Any python 3 works as a bootstrap - run_cosim
# re-execs under the one ~/.fixs/carla.json names - but it has to BE a python 3,
# established by ASKING rather than by the name: `command -v python` answers on a
# box whose python is still a 2.x, and that one met the engine's first f-string as
# a SyntaxError citing PEP 263, naming a line that never mentions the interpreter.
# ---------------------------------------------------------------------------
# Echoes the candidate's own sys.executable if it is a python 3, nothing otherwise.
# sys.executable, not the name probed: that is the path that survives a symlink
# chain (conda ships bin/python -> python3 -> python3.N). </dev/null so a
# candidate that wants to be interactive cannot block the launcher.
_py3_exe() {
    "$1" -c 'import sys; sys.stdout.write(sys.executable if sys.version_info[0] == 3 else "")' \
        </dev/null 2>/dev/null || true
}

pyexe() {
    local py="" cand cfg="$HOME/.fixs/carla.json"
    # An explicit pin wins over everything: the escape hatch for a box whose system
    # python must stay a 2.x, or with several python 3s where one is wanted. It
    # names only the BOOTSTRAP interpreter. A pin that is not a python 3 is an
    # error, not a reason to fall back - quietly using another python is how you
    # end up debugging the wrong one.
    if [[ -n "${FIXS_BOOTSTRAP_PYTHON:-}" ]]; then
        py="$(_py3_exe "$FIXS_BOOTSTRAP_PYTHON")"
        if [[ -z "$py" || ! -x "$py" ]]; then
            {
                echo "[FIXS] FIXS_BOOTSTRAP_PYTHON is set, but that is not a usable python 3:"
                echo "         $FIXS_BOOTSTRAP_PYTHON"
                echo "       Point it at a python 3.10 executable, or clear it to search PATH."
            } >&2
            return 1
        fi
        printf '%s' "$py"; return 0
    fi
    # The configured env first: it is the one the engine would re-exec into anyway.
    if [[ -f "$cfg" ]]; then
        cand="$(grep -o '"python"[[:space:]]*:[[:space:]]*"[^"]*"' "$cfg" \
                | sed 's/.*"\([^"]*\)"$/\1/')" || cand=""
        [[ -n "$cand" && -x "$cand" ]] && py="$(_py3_exe "$cand")"
    fi
    # python3.10 before python3: the stack supports 3.7-3.10, and on a box without
    # conda, setup pip-installs into whatever interpreter it runs under - where
    # there is no carla wheel for 3.11+.
    if [[ -z "$py" ]]; then
        for cand in python3.10 python3 python; do
            command -v "$cand" >/dev/null 2>&1 || continue
            py="$(_py3_exe "$cand")"
            [[ -n "$py" && -x "$py" ]] && break
            py=""
        done
    fi
    # A conda install is the python 3 most likely to exist and NOT be on PATH: the
    # Miniconda installer recommends against adding itself. conda's own registry
    # first - it finds an install on another disk that no guessed root would - then
    # the usual roots (the same as env_setup._conda_roots, kept in step by hand
    # because this runs before any python does).
    if [[ -z "$py" && -f "$HOME/.conda/environments.txt" ]]; then
        while IFS= read -r cand; do
            [[ -n "$cand" && -x "$cand/bin/python3" ]] || continue
            py="$(_py3_exe "$cand/bin/python3")"
            [[ -n "$py" && -x "$py" ]] && break
            py=""
        done < "$HOME/.conda/environments.txt"
    fi
    if [[ -z "$py" ]]; then
        for cand in "${CONDA_PREFIX:-}" "$HOME/miniconda3" "$HOME/anaconda3" \
                    "$HOME/miniforge3" "$HOME/mambaforge" /opt/conda; do
            [[ -n "$cand" && -x "$cand/bin/python3" ]] || continue
            py="$(_py3_exe "$cand/bin/python3")"
            [[ -n "$py" && -x "$py" ]] && break
            py=""
        done
    fi
    if [[ -z "$py" ]]; then
        cat >&2 <<'EOM'
[FIXS] No Python 3 found.

       Install Python 3.10, then run ./FIXS.sh again - it does the rest,
       including building the FIXS python env.
         Miniconda      https://www.anaconda.com/docs/getting-started/miniconda/install
         or python.org  https://www.python.org/downloads/release/python-3109/

       3.10 specifically: the CARLA client wheel is published only for CPython
       3.7-3.10, so pip finds nothing to install on 3.11+.
       Already have one somewhere unusual? export FIXS_BOOTSTRAP_PYTHON=/path/to/it
EOM
        # return, not exit: the caller reads this through $(...), and an exit there
        # only ends the subshell.
        return 1
    fi
    printf '%s' "$py"
}

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
    rc=0; bash "$tmp" "${args[@]}" || rc=$?
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
    ver="$(sed -n '1s/ .*//p' "$ROOT/FIXS/FIXS_VERSION.txt" 2>/dev/null)" || ver=""
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

# ---------------------------------------------------------------------------
# Started with no arguments from a terminal: ask once, the way FIXS.bat asks a
# double-click. The options would otherwise be reachable only by knowing them.
# Whatever is typed is re-entered as an ordinary command line, so it means exactly
# what the command line means and there is no second grammar here. 'gui' alone is
# the one shorthand. No terminal on stdin (a file manager that opens none, a
# script redirecting stdin) -> no prompt, and the run proceeds as before.
# ---------------------------------------------------------------------------
if [[ $# -eq 0 && -t 0 ]]; then
    echo
    echo "FIXS - co-simulation"
    echo
    print_options
    echo
    echo "Press Enter to run, type  gui  for the FIXS window, or type any options above."
    read -r -p "  options: " OPTS || OPTS=""
    # Empty -- or EOF -- falls straight through to the ordinary run.
    if [[ -n "${OPTS// }" ]]; then
        [[ "${OPTS// }" == "gui" ]] && OPTS="--gui"
        # shellcheck disable=SC2086  # word-splitting IS the point: typed options
        exec "$0" $OPTS
    fi
fi

# The only names this file answers itself: help, because it must work before
# there is an engine to ask; and --update-fixs, the one action that must work
# before there is an engine at all. Everything else goes to run_cosim untouched.
case "${1:-}" in
    --help|-h) usage; exit 0 ;;
    --update-fixs)
        shift
        fetch_fixs "${1:-}" || exit $?
        seed_manifest
        exit 0 ;;
esac

# Python BEFORE the fetch. Both are prerequisites, and this is the one answered in
# a second - a box with no python 3 used to sit through the whole FIXS download
# and only then be told to go and install python.
PY="$(pyexe)" || exit 1

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
    # INSTALL THE DECLARED PIN, not "whatever the picker defaults to": a repo that
    # says which engine it runs has already made the choice. Passing the pin also
    # makes this one lookup of one release by tag, which still works when the
    # releases INDEX is down (seen live: GitHub 504 on the index while
    # /releases/tags/<tag> served normally). No pin -> the picker, correctly.
    fetch_fixs "${FIXS_VERSION:-}" \
        || { echo "[FIXS] setup failed - see above. Not continuing." >&2; exit 1; }
fi

# Every run, not only after a fetch: a repo can arrive at an installed FIXS/ some
# other way - migrating off run_cosim.sh, or a colleague's copy - and it should
# still end up with its pin recorded. Returns at once when a manifest (or a legacy
# fixs_sources.txt) exists, so this costs one stat on every later run.
seed_manifest

# Say so when this file is older than the engine it just installed. It is never
# overwritten in place: a running shell reads its own script incrementally, and
# this file is also the repo's committed entry point. Re-downloading is the fix.
SHIPPED="$ROOT/FIXS/frontdoor/FIXS.sh"
if [[ -f "$SHIPPED" ]]; then
    want="$(sed -n 's/^# FIXS_FRONTDOOR: *\([0-9]*\).*/\1/p' "$SHIPPED" | head -n1)" || want=""
    if [[ -n "$want" && "$want" != "$FRONTDOOR_CONTRACT" ]]; then
        echo "[FIXS] this FIXS.sh is contract v$FRONTDOOR_CONTRACT; the installed"
        echo "[FIXS] build expects v$want. Copy FIXS/frontdoor/FIXS.sh over it."
    fi
fi

exec "$PY" "$ROOT/FIXS/cosim/run_cosim.py" "$@"
