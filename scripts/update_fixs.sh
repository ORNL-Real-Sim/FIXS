#!/usr/bin/env bash
# ============================================================================
#  Fetch/refresh a FIXS build into a consuming application's checkout (POSIX).
#
#  NOT a user entry point - the application's front door owns that
#  (./run_cosim.sh --update-fixs). That front door downloads this script from a
#  FIXS release tag and runs it; if the user then picks a different release, this
#  script hands off to THAT release's copy of itself, so the unpacker always
#  matches the bundle it unpacks. Keeping it here rather than in each app repo is
#  #272: release-format knowledge (zip layout, where the native runtime comes
#  from, the freshness rule) is engine knowledge, and a per-app copy drifts.
#  scripts/update_fixs.ps1 is the exact counterpart; change them together.
#
#  Usage:
#    update_fixs.sh --root /path/to/app                    # pick interactively
#    update_fixs.sh --root /path/to/app --version v0.9.0   # a specific release
#    update_fixs.sh --root . --repo my-fork/FIXS
#
#  Installs into <root>/FIXS. Requires bash, curl, unzip. No git, no GitHub CLI,
#  no auth - FIXS is public.
#
#  Exit codes: 0 = a complete bundle is in place; 1 = nothing usable installed.
# ============================================================================
set -euo pipefail

ROOT=""
VERSION=""
REPO="ORNL-Real-Sim/FIXS"
SELF_REF=""            # the ref this copy came from; enables the hand-off below
DEFAULT_VERSION=""     # the app's preferred channel, offered as the Enter-default

while [[ $# -gt 0 ]]; do
    case "$1" in
        --root)            ROOT="$2";            shift 2 ;;
        --version)         VERSION="$2";         shift 2 ;;
        --repo)            REPO="$2";            shift 2 ;;
        --self-ref)        SELF_REF="$2";        shift 2 ;;
        --default-version) DEFAULT_VERSION="$2"; shift 2 ;;
        *) echo "[ERROR] unknown argument: $1" >&2; exit 1 ;;
    esac
done
[[ -n "$ROOT" ]] || { echo "[ERROR] --root is required." >&2; exit 1; }

# git is deliberately NOT required. It used to be, for a sparse clone of
# CommonLib/libsumo/bin; #238 deleted that path from git and moved the runtime to
# a release asset, so the clone silently produced an empty tree.
for tool in curl unzip; do
    command -v "$tool" >/dev/null 2>&1 || { echo "[ERROR] '$tool' is required but not on PATH." >&2; exit 1; }
done

ROOT="$(cd "$ROOT" && pwd)"
OUTPUT_DIR="$ROOT/FIXS"
VERSION_FILE="$OUTPUT_DIR/FIXS_VERSION.txt"
API="https://api.github.com/repos/$REPO"
DEPS_TAG="fixs-native-deps"     # rolling release carrying the packed native runtimes
CURL=(curl -fsSL -H 'User-Agent: fixs-fetch' -H 'Accept: application/vnd.github+json')

TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/fixs-fetch-XXXXXX")"
cleanup() { rm -rf "$TMP_DIR"; }
trap cleanup EXIT

json_field() {  # json_field <file> <field> -> first top-level "field":"value"
    grep -o "\"$2\":[[:space:]]*\"[^\"]*\"" "$1" | sed 's/.*"\([^"]*\)"$/\1/' | head -n1
}

# ---------------------------------------------------------------------------
# A release is INSTALLABLE iff it carries a fixs-build-*.zip. That one rule keeps
# the picker honest with no tag list to maintain: it excludes the native-deps
# release, every Binaries-<sha> bundle (there can be arbitrarily many), and old
# tags like v0.6.0/v0.7.0 cut before the build zip existed, which have no assets
# at all. Four of the six tags visible today are unusable; the menu offered all six.
#
# The releases JSON is walked as an ORDERED key stream rather than parsed: within
# each release object GitHub emits tag_name, target_commitish, draft, prerelease
# and published_at before that release's assets, and browser_download_url occurs
# only inside assets - so a new tag_name reliably starts a new record. This avoids
# a jq dependency without hand-rolling a JSON parser.
# ---------------------------------------------------------------------------
installable_releases() {  # -> TAB-separated: tag, sha7, published(date), rolling|pinned
    "${CURL[@]}" "$API/releases?per_page=30" \
      | grep -o '"tag_name":[[:space:]]*"[^"]*"\|"target_commitish":[[:space:]]*"[^"]*"\|"draft":[[:space:]]*[a-z]*\|"prerelease":[[:space:]]*[a-z]*\|"published_at":[[:space:]]*"[^"]*"\|"browser_download_url":[[:space:]]*"[^"]*"' \
      | awk -F'"' '
          function flush() {
              if (tag != "" && ok == 1 && draft != "true")
                  printf "%s\t%s\t%s\t%s\n", tag, substr(sha, 1, 7), substr(pub, 1, 10),
                         (pre == "true" ? "rolling" : "pinned")
          }
          /"tag_name"/          { flush(); tag = $4; sha = ""; pub = ""; ok = 0; pre = ""; draft = ""; next }
          /"target_commitish"/  { sha = $4; next }
          /"published_at"/      { pub = $4; next }
          /"draft"/             { draft = ($0 ~ /true/ ? "true" : "false"); next }
          /"prerelease"/        { pre   = ($0 ~ /true/ ? "true" : "false"); next }
          /"browser_download_url"/ { if ($4 ~ /fixs-build-.*\.zip$/) ok = 1; next }
          END { flush() }'
}

select_release() {  # menu -> chosen tag on stdout, menu on stderr
    local lines=() i tag kind sha pub choice default
    while IFS= read -r l; do [[ -n "$l" ]] && lines+=("$l"); done
    [[ ${#lines[@]} -gt 0 ]] || return 1
    # The Enter-default is the CONSUMING APP's channel preference, not ours: an
    # app built against 0.9.0 features wants v0.9.0-alpha even though 'latest' may
    # be newer. It arrives via --default-version (fixs_default_version in the
    # app's fixs_sources.txt) so the choice lives in the app's config rather than
    # being hardcoded in two scripts per app repo, which is where it used to live.
    # Falling back to the first entry means API order, NOT newest - GitHub orders
    # releases by creation, and a rolling tag keeps its original created_at when
    # republished, so the list is not sorted by publish time.
    default="$(cut -f1 <<< "${lines[0]}")"
    if [[ -n "$DEFAULT_VERSION" ]] && cut -f1 <<< "$(printf '%s\n' "${lines[@]}")" | grep -qx "$DEFAULT_VERSION"; then
        default="$DEFAULT_VERSION"
    fi
    # Annotate every entry with what it resolves to. Bare tag names were not
    # enough to tell the rolling channels apart: 'latest' and 'v0.9.0-alpha' look
    # identical in a menu, and 'latest' then unpacks a zip named after the nearest
    # tag (fixs-build-v0.9.0-alpha-1-g65f2970d.zip), which reads as though the
    # wrong release was installed. Commit + date make them distinguishable BEFORE
    # the choice, not inferable from a filename afterwards.
    {
        echo "Available FIXS releases:"
        for i in "${!lines[@]}"; do
            IFS=$'\t' read -r tag sha pub kind <<< "${lines[$i]}"
            printf "   %d) %-18s %-7s  %s  %s\n" "$((i + 1))" "$tag" "$kind" "$sha" "$pub"
        done
        printf "Which release? [1-%d], Enter = %s (default): " "${#lines[@]}" "$default"
    } >&2
    read -r choice
    if [[ "$choice" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#lines[@]} )); then
        cut -f1 <<< "${lines[$((choice - 1))]}"
    else
        echo "$default"
    fi
}

sha256_of() {
    if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | awk '{print tolower($1)}'
    elif command -v shasum   >/dev/null 2>&1; then shasum -a 256 "$1" | awk '{print tolower($1)}'
    else echo ""; fi
}

get_asset() {  # get_asset <url> <dest> - download and verify against the .sha256 sidecar
    local url="$1" dest="$2" expected actual
    curl -fsSL -H 'User-Agent: fixs-fetch' -o "$dest" "$url"
    # A missing/unreachable sidecar warns and proceeds (older assets may predate
    # them); a MISMATCH is always fatal - that is a corrupt or tampered download.
    if expected="$(curl -fsSL "$url.sha256" 2>/dev/null | awk '{print tolower($1); exit}')" && [[ -n "$expected" ]]; then
        actual="$(sha256_of "$dest")"
        if [[ -z "$actual" ]]; then
            echo "  [warn] no sha256sum/shasum on PATH - proceeding WITHOUT verification." >&2
        elif [[ "$expected" != "$actual" ]]; then
            echo "[ERROR] checksum mismatch for $(basename "$url"): expected $expected, got $actual" >&2
            exit 1
        else
            echo "    checksum OK ($actual)"
        fi
    else
        echo "  [warn] could not fetch $(basename "$url").sha256 - proceeding WITHOUT verification." >&2
    fi
}

# ---------------------------------------------------------------------------
# Resolve the release.
#
# INVARIANT: --version given => never prompt, never hand off. That is what bounds
# the hand-off below to a single step and makes an infinite bounce impossible.
# ---------------------------------------------------------------------------
if [[ -z "$VERSION" ]]; then
    LIST="$(installable_releases)"
    [[ -n "$LIST" ]] || { echo "[ERROR] No installable FIXS release at $REPO (none carries a 'fixs-build-*.zip')." >&2; exit 1; }
    if [[ -t 0 ]]; then
        VERSION="$(select_release <<< "$LIST")"
    elif [[ -n "$DEFAULT_VERSION" ]] && cut -f1 <<< "$LIST" | grep -qx "$DEFAULT_VERSION"; then
        VERSION="$DEFAULT_VERSION"
    else
        VERSION="$(head -n1 <<< "$LIST" | cut -f1)"
    fi

    # Hand off to the chosen release's own copy of this script, so the unpacker
    # always matches the bundle it unpacks (#272). Only reachable when the
    # bootstrap told us which ref we came from and the user picked a different
    # one; --version on the child makes it terminal by the invariant above.
    if [[ -n "$SELF_REF" && "$SELF_REF" != "$VERSION" ]]; then
        echo "Switching to $VERSION's own updater (this copy came from $SELF_REF) ..."
        CHILD="$TMP_DIR/update_fixs_$VERSION.sh"
        if curl -fsSL "https://raw.githubusercontent.com/$REPO/$VERSION/scripts/update_fixs.sh" -o "$CHILD"; then
            exec bash "$CHILD" --root "$ROOT" --version "$VERSION" --repo "$REPO"
        fi
        # Not fatal: a release predating this script still has a build zip we know
        # how to unpack, so carry on with the copy already running.
        echo "  $VERSION has no scripts/update_fixs.sh; continuing with this one."
    fi
fi

RELEASE_JSON="$TMP_DIR/release.json"
"${CURL[@]}" "$API/releases/tags/$VERSION" -o "$RELEASE_JSON" \
    || { echo "[ERROR] Could not find FIXS release '$VERSION' at $REPO." >&2; exit 1; }

ASSET_URL="$(grep -o '"browser_download_url":[[:space:]]*"[^"]*"' "$RELEASE_JSON" \
    | sed 's/.*"\(https[^"]*\)".*/\1/' | grep -E 'fixs-build-.*\.zip$' | head -n1)"
[[ -n "$ASSET_URL" ]] || { echo "[ERROR] Release '$VERSION' carries no 'fixs-build-*.zip' and cannot be installed." >&2; exit 1; }
ASSET_NAME="$(basename "$ASSET_URL")"
SHA="$(json_field "$RELEASE_JSON" target_commitish)"
PUBLISHED_AT="$(json_field "$RELEASE_JSON" published_at)"
PRERELEASE="$(grep -o '"prerelease":[[:space:]]*[a-z]*' "$RELEASE_JSON" | head -n1 | grep -o 'true\|false')"

# State the resolution in full. A rolling tag's asset is named after the nearest
# annotated tag plus a git-describe suffix, so 'latest' unpacks a zip called
# fixs-build-v0.9.0-alpha-1-g65f2970d.zip - which reads like the alpha was
# installed instead of what was asked for. Printing tag + commit + asset together
# is what makes the two distinguishable.
echo "=== Fetching the FIXS build from $REPO (public, no auth) ==="
echo "  release:   $VERSION$([[ "$PRERELEASE" == "true" ]] && echo '  (rolling prerelease - always re-fetched)')"
echo "  commit:    ${SHA:0:7}"
echo "  asset:     $ASSET_NAME"
echo "  published: $PUBLISHED_AT"
echo "  into:      $OUTPUT_DIR"

# Skip only when the release is IMMUTABLE. The rolling prereleases republish in
# place under a fixed tag, so a matching tag does NOT mean the local bundle is
# current. This reads the 'prerelease' flag rather than hardcoding which tags are
# rolling - the hardcoded list this replaces agreed with the flag only by
# coincidence, and would have skipped a re-fetch for any newly named channel.
if [[ -f "$VERSION_FILE" && "$PRERELEASE" != "true" ]]; then
    current="$(head -n1 "$VERSION_FILE" | tr -d '[:space:]')"
    if [[ "$current" == "$VERSION" ]]; then
        echo "FIXS $VERSION is already installed (delete $VERSION_FILE to force)."
        exit 0
    fi
fi

# --- The build zip ---------------------------------------------------------
ZIP_PATH="$TMP_DIR/$ASSET_NAME"
echo "Downloading $ASSET_NAME ..."
get_asset "$ASSET_URL" "$ZIP_PATH"

echo "Extracting to $OUTPUT_DIR ..."
# The Windows-packed zip ships some directories without their EXECUTE bit (e.g.
# CommonLib/YAMLMatlab/Tests/Data is drw-rw-r--). Without x on a directory it
# cannot be traversed, so rm cannot reach - let alone unlink - what is inside,
# and a SECOND run died with "Permission denied". No sudo involved: we own the
# tree, it is just un-traversable. u+rwX restores read/write everywhere and
# execute on DIRECTORIES only (capital X). Proper fix is packing traversable
# directories upstream.
[[ -e "$OUTPUT_DIR" ]] && chmod -R u+rwX "$OUTPUT_DIR" 2>/dev/null || true
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"
# The release zip is built on Windows (backslash path separators); unzip converts
# them to a proper tree but returns 1 (warning). Only exit >=2 is a real error.
set +e
unzip -q "$ZIP_PATH" -d "$OUTPUT_DIR"; unzip_rc=$?
set -e
[[ "$unzip_rc" -le 1 ]] || { echo "[ERROR] unzip failed (exit $unzip_rc)." >&2; exit "$unzip_rc"; }

# --- Normalize bundled shell scripts to LF + restore the exec bit ----------
# The release zip is built on Windows: bundled *.sh files carry CRLF endings and
# no Unix exec bit. A trailing CR corrupts the shebang ("bash\r", reported by the
# kernel as "no such file or directory"). Remove once FIXS pins *.sh to LF via
# .gitattributes eol=lf.
echo "Normalizing bundled shell scripts (CRLF -> LF, +x) ..."
normalized=0
while IFS= read -r -d '' sh; do
    tmp="$sh.lffix.$$"
    if tr -d '\r' < "$sh" > "$tmp" && ! cmp -s "$sh" "$tmp"; then
        mv "$tmp" "$sh"; normalized=$((normalized + 1))
    else
        rm -f "$tmp"
    fi
    chmod +x "$sh" 2>/dev/null || true
done < <(find "$OUTPUT_DIR" -type f -name '*.sh' -print0)
echo "  normalized $normalized script(s) to LF"

# --- The native runtime ----------------------------------------------------
# The build zip ships libsumo HEADERS only (bin/ is ~413 MB), but TrafficLayer
# links libsumocpp.lib and loads the runtime at startup, so a headers-only
# install dies with "Unable to locate SUMO library directory".
echo "Fetching the native runtime from the '$DEPS_TAG' release ..."
DEPS_JSON="$TMP_DIR/native-deps.json"
"${CURL[@]}" "$API/releases/tags/$DEPS_TAG" -o "$DEPS_JSON" \
    || { echo "[ERROR] Could not read the '$DEPS_TAG' release at $REPO." >&2; exit 1; }
# Version-named: take whatever the release currently carries, so bumping the SUMO
# version in dependencies.yaml + publishing the asset is the whole change.
DEP_URL="$(grep -o '"browser_download_url":[[:space:]]*"[^"]*libsumo-[^"]*\.zip"' "$DEPS_JSON" \
           | head -n1 | sed 's/.*"\(https[^"]*\)"$/\1/')"
[[ -n "$DEP_URL" ]] || { echo "[ERROR] no 'libsumo-*.zip' asset on the '$DEPS_TAG' release." >&2; exit 1; }
echo "  downloading $(basename "$DEP_URL") ..."
get_asset "$DEP_URL" "$TMP_DIR/libsumo.zip"
# Extracting into CommonLib/ reproduces the <component>/bin layout the
# executables already search; any future runtime asset lands the same way.
set +e
unzip -q -o "$TMP_DIR/libsumo.zip" -d "$OUTPUT_DIR/CommonLib"; unzip_rc=$?
set -e
[[ "$unzip_rc" -le 1 ]] || { echo "[ERROR] unzip of the native runtime failed (exit $unzip_rc)." >&2; exit "$unzip_rc"; }

SUMO_BIN="$OUTPUT_DIR/CommonLib/libsumo/bin"
n="$(find "$SUMO_BIN" -maxdepth 1 -name '*.dll' 2>/dev/null | wc -l | tr -d '[:space:]')"
if [[ ! -f "$SUMO_BIN/libsumocpp.lib" || "$n" -eq 0 ]]; then
    echo "[ERROR] the native runtime extracted but $SUMO_BIN has no libsumocpp.lib / no DLLs." >&2
    exit 1
fi
echo "  native runtime ready ($n DLLs)"

# --- Version marker --------------------------------------------------------
# Written LAST and only on success: an incomplete bundle stamped as good would be
# reported as already-installed by the skip above, and TrafficLayer would not
# fail until startup.
if [[ "$PRERELEASE" == "true" ]]; then STAMP="$VERSION ($PUBLISHED_AT)"; else STAMP="$VERSION"; fi
cat > "$VERSION_FILE" <<EOF
$STAMP
Fetched: $(date '+%Y-%m-%d %H:%M:%S')
Source: $REPO
Commit: ${SHA:0:7}
Zip: $ASSET_NAME
EOF

echo ""
echo "FIXS $VERSION is ready in $OUTPUT_DIR"
if [[ -f "$OUTPUT_DIR/BUILD_INFO.txt" ]]; then
    echo ""
    echo "Build details:"
    head -n 15 "$OUTPUT_DIR/BUILD_INFO.txt"
fi
