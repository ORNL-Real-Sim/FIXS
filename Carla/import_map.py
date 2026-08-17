"""
import_map.py - import a RoadRunner/OpenDRIVE map into the configured CARLA.

Generalizes the proven one-click import (run CARLA's
`Util/BuildTools/Import.py --package=<name>` directly, with UE4_ROOT set) into a
shippable, parameterized helper. Used by run_cosim's --auto-import and by the
per-app one-click import_*.bat / .sh wrappers.

A map "package" is the `<name>.json` descriptor plus its fbx/xodr/fbm asset
folder. This helper makes sure that package is staged under <carla_root>/Import,
then runs the cook.

A raw RoadRunner export is accepted too: it ships no `<name>.json`, so one is
generated from its filenames, and its geometry is renamed to the map name you
are importing as (see generate_descriptor). That way `--package mlk_no_signal`
cooks `mlk_no_signal` whether the export inside was called `MLK_no_signal_0805`
or anything else, and next month's re-export lands on the same map rather than a
new one.

The package is sourced, in order:
  --pick-release        list the repo's map releases and prompt which VERSION to
                        import, then download it via gh (no URL/version to
                        hand-maintain - you pick from what's published). gh is
                        needed to LIST and to DOWNLOAD, but not to import: when it
                        is missing or cannot see the library, the picker says so,
                        lists nothing online, and still offers the local file, or
  already staged        files already present under <carla_root>/Import, or
  --package-dir <dir>   a local folder holding the package files, or
  --package-url <url>   tried via the GitHub CLI if installed (handles private
                        repos), else you are prompted to point at a copy you
                        downloaded by hand from the release - the portable path,
                        needing only browser access, no gh / auth.

Both CARLA flavours are served, by the route each one supports - the flavour is
read from the saved config and dispatched on, not used to refuse:

  source    COOKS the package. The cook runs the Unreal editor, so this is the
            only flavour that can turn an fbx/xodr into a map.
  packaged  INSTALLS the precooked package (the Digital-Twin Library's
            `*_cooked.tar.gz`) into the package root - a plain extract, no editor
            (`install_precooked` -> `install_cooked`).

run_cosim's preflight calls the same `install_precooked`, so the two front doors
cannot disagree about what a packaged build can install. carla_root / ue4_root
default to the saved env config (~/.fixs/carla.json) written by carla_env_setup.py.

Examples:
  # pick which published version to import (lists releases tagged map-*):
  python import_map.py --pick-release --repo ORNL-Real-Sim/FIXS_Applications --tag-prefix map-
  python import_map.py --package RP_Ver0529 --package-dir C:/src_ext/Carla/Import
  python import_map.py --package RP_Ver0529 --package-url https://.../RP_Ver0529_import.zip
"""
import argparse
import fnmatch
import os
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import zipfile

import carla_env_setup as env
import fixs_paths


def _mode(mode=None):
    """The CARLA flavour to resolve paths for: the caller's, else the saved
    config's, else 'source'. Defaulting from the config is what lets every
    existing (source-build) call site stay unchanged."""
    if mode:
        return mode
    return (env.load_config() or {}).get("mode") or "source"


def package_root(carla_root):
    """The folder a packaged CARLA's own ImportAssets.sh cds into - the one that
    directly holds CarlaUE4/Content.

    env.packaged_exe accepts carla_root pointing EITHER at that folder or at a
    wrapper holding WindowsNoEditor/ (LinuxNoEditor/), which are one level apart;
    resolving through the launcher we already found is what keeps the two spellings
    from producing two different Content roots."""
    exe = env.packaged_exe(carla_root)
    return os.path.dirname(exe) if exe else carla_root


def content_root(carla_root, mode=None):
    """The Content/ dir holding cooked assets for this CARLA flavour.

    A source build keeps it under Unreal/CarlaUE4/; a packaged build has it beside
    the launcher. Everything below resolves through here, so the packaged flavour
    needed no new naming rules - only this one prefix differs."""
    if _mode(mode) == "packaged":
        return os.path.join(package_root(carla_root), "CarlaUE4", "Content")
    return os.path.join(carla_root, "Unreal", "CarlaUE4", "Content")


def cooked_content_dir(carla_root, name, mode=None):
    """The Content/<name> folder produced by a successful import."""
    return os.path.join(content_root(carla_root, mode), name)


def cooked_map_path(carla_root, name, mode=None):
    """Where the cooked .umap lands after a successful import."""
    return os.path.join(cooked_content_dir(carla_root, name, mode),
                        "Maps", name, name + ".umap")


def map_is_imported(carla_root, name, mode=None):
    return os.path.isfile(cooked_map_path(carla_root, name, mode))


def package_descriptor(carla_root, name, mode=None):
    """The <name>.Package.json the cook writes beside the imported content. It is
    the authoritative record of what the package actually holds - the map's real
    name and its /Game/... path - neither of which is *guaranteed* to equal the
    package name (see _package_from_tag: that is a release convention, not a rule).
    A precooked package ships the same file, so a packaged build is resolved by the
    same evidence a source build is."""
    return os.path.join(cooked_content_dir(carla_root, name, mode),
                        "Config", name + ".Package.json")


def declared_maps(carla_root, name, mode=None):
    """[(map_name, /Game/... level path)] declared by the package descriptor.
    Entries whose .umap is missing are dropped: a descriptor outlives the content
    it describes (a half-wiped re-import leaves one behind), and a name we hand to
    load_world must point at a level that is really on disk."""
    import json
    try:
        with open(package_descriptor(carla_root, name, mode), encoding="utf-8") as f:
            declared = json.load(f).get("maps") or []
    except (OSError, ValueError):
        return []
    found = []
    for m in declared:
        mname, mpath = m.get("name"), m.get("path")
        if not mname or not str(mpath).startswith("/Game/"):
            continue
        umap = os.path.join(content_root(carla_root, mode),
                            *mpath[len("/Game/"):].split("/"), mname + ".umap")
        if os.path.isfile(umap):
            found.append((mname, f"{mpath}/{mname}"))
    return found


def find_level_paths(carla_root, name, mode=None):
    """Every cooked level called <name>.umap, as /Game/... object paths.

    More than one means load_world(<name>) is a coin flip: CARLA resolves a bare
    name by recursive file search and silently takes the first hit (see
    UCarlaEpisode::LoadNewEpisode -> PathList[0]), so which copy you get is
    directory-traversal order, not a choice. A full /Game/... path is exact."""
    content = content_root(carla_root, mode)
    umap = name + ".umap"
    found = []
    for root, _dirs, files in os.walk(content):
        if umap in files:
            rel = os.path.relpath(root, content).replace(os.sep, "/")
            found.append((f"/Game/{rel}/{name}", os.path.getsize(os.path.join(root, umap))))
    return sorted(found)


def choose_level_path(carla_root, name, mode=None, interactive=None):
    """Full /Game/... path to load for `name`. Only reached when the path could NOT
    be worked out from the package itself, so there is nothing to default to: if
    several cooked levels share the name, only the user can say which was meant.

    `interactive` is the caller's answer to "may this stop and ask?"; isatty() is
    only the default. A --serve host has a terminal and no one at it."""
    found = find_level_paths(carla_root, name, mode)
    if len(found) <= 1:
        return found[0][0] if found else None

    print(f"\n[cosim] {len(found)} cooked maps are named '{name}' and nothing "
          f"identifies which is wanted - pick one:")
    for i, (path, size) in enumerate(found, 1):
        print(f"   {i}) {path}  ({size / 1048576:.1f} MB)")
    if not (sys.stdin.isatty() if interactive is None else interactive):
        sys.exit("[cosim] non-interactive and the name is ambiguous; pass --map "
                 "<package> so the path resolves, or delete the stale copies.")
    while True:
        ans = _prompt(f"[cosim] Which one? [1-{len(found)}]: ").strip()
        if ans.isdigit() and 1 <= int(ans) <= len(found):
            return found[int(ans) - 1][0]
        print("[cosim] invalid choice; enter a number from the list.")


def duplicate_level_note(carla_root, name, using, mode=None):
    """Heads-up text when other cooked levels share `name`, else "". Informational
    only - we load by exact /Game/... path, so the copies cannot be picked up by
    mistake; they are just leftovers from repeat imports, worth deleting."""
    others = [p for p, _ in find_level_paths(carla_root, name, mode) if p != using]
    if not others:
        return ""
    return (f"[cosim] note: {len(others)} other cooked map(s) also named '{name}'; "
            f"ignored (loading by full path):\n"
            + "\n".join(f"           {p}" for p in others))


def resolve_cooked_map(carla_root, name, mode=None):
    """(map_name, /Game/... level path) for an imported package, or None when it
    cannot be resolved *unambiguously*.

    Resolve, never assume. The picked release gives a PACKAGE name; what CARLA
    loads is a MAP name, and the two agree only by naming convention. So: accept
    the conventional layout when it is genuinely on disk, else believe the
    package's own descriptor, else give up - so the caller can ask the user
    rather than invent a name that load_world will fail to open."""
    if map_is_imported(carla_root, name, mode):
        return name, f"/Game/{name}/Maps/{name}/{name}"
    found = declared_maps(carla_root, name, mode)
    return found[0] if len(found) == 1 else None


def _descriptor(carla_root, name):
    return os.path.join(carla_root, "Import", name + ".json")


def _stage_from_path(path, import_dir):
    """Copy/extract a downloaded package (a .zip or an extracted folder) into
    <carla_root>/Import, preserving the descriptor + asset-folder layout."""
    if os.path.isfile(path) and path.lower().endswith(".zip"):
        print(f"[import] extracting {path} -> {import_dir}")
        with zipfile.ZipFile(path) as z:
            z.extractall(import_dir)
    elif os.path.isdir(path):
        print(f"[import] copying {path} -> {import_dir}")
        for item in os.listdir(path):
            src = os.path.join(path, item)
            dst = os.path.join(import_dir, item)
            if os.path.isdir(src):
                shutil.copytree(src, dst, dirs_exist_ok=True)
            else:
                shutil.copy2(src, dst)
    else:
        sys.exit(f"[import] package path is not a .zip or a folder: {path}")


def _gh_release_ref(url):
    """(repo, tag, asset) if url is a github release-asset URL, else None."""
    m = re.match(r"https?://github\.com/([^/]+/[^/]+)/releases/download/([^/]+)/(.+)$", url)
    return (m.group(1), m.group(2), m.group(3)) if m else None


def _try_gh_download(package_url):
    """Opportunistic auto-download via the GitHub CLI (handles private repos).
    Returns (zip_path, tmpdir) on success, else (None, None) so we fall back to a
    manual prompt - many machines won't have gh installed/authenticated."""
    ref = _gh_release_ref(package_url or "")
    gh = shutil.which("gh")
    if not ref or not gh:
        return None, None
    repo, tag, asset = ref
    tmp = tempfile.mkdtemp(prefix="fixs-map-")
    print(f"[import] trying gh to download {asset} from {repo}@{tag} ...")
    rc = subprocess.call([gh, "release", "download", tag, "-R", repo,
                          "-p", asset, "-D", tmp, "--clobber"])
    got = os.path.join(tmp, asset)
    if rc == 0 and os.path.isfile(got):
        return got, tmp
    print("[import] gh download unavailable; will ask for a local copy instead.")
    shutil.rmtree(tmp, ignore_errors=True)
    return None, None


def _select_package(name, package_url, precooked=False):
    """Let the user point at a package they downloaded by hand - a native file
    picker, falling back to a typed path. This is the portable path: no GitHub
    CLI / auth needed, just browser access to the release.

    `precooked` says which artefact this CARLA can actually take, and it has to be
    asked for by name. A PACKAGED build cooks nothing, so the only thing it can
    install is the library's `*_cooked.tar.gz` (install_precooked) - yet this
    dialog offered "[Z = zip, F = folder]" and filtered `*.zip`, steering the one
    flavour that needs the tarball towards the two artefacts it must reject (#283).
    A SOURCE build is the mirror image: it cooks, so it wants the .zip or the
    extracted folder and cannot use a cooked tarball.

    For a source build we still ask zip-or-folder first, because the two need
    different dialogs and askopenfilename cannot select a directory. Everything
    downstream already accepts a folder (_stage_from_path copies a tree); only this
    dialog could not offer one, which made the picker's "select a local .zip /
    folder" a half truth - a typed path was the sole way to hand it a raw
    extracted export. A precooked package is always one file, so it asks nothing."""
    what = "precooked package (*_cooked.tar.gz)" if precooked else ".zip (or extracted folder)"
    print(f"\n[import] Select the downloaded '{name}' {what}.")
    if package_url:
        print("[import] If you don't have it yet, download it (browser is fine - "
              "you need access to the release):")
        print(f"             {package_url}")
    folder = (not precooked) and sys.stdin.isatty() and _prompt(
        "[import] Is it a .zip or an extracted folder? "
        "[Z = zip, F = folder, Enter = zip]: ").strip().lower().startswith("f")
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        if folder:
            path = filedialog.askdirectory(
                title=f"Select the extracted {name} package folder")
        elif precooked:
            path = filedialog.askopenfilename(
                title=f"Select the downloaded precooked {name} package (*_cooked.tar.gz)",
                filetypes=[("Precooked map packages", "*.tar.gz"), ("All files", "*.*")])
        else:
            path = filedialog.askopenfilename(
                title=f"Select the downloaded {name} package (.zip)",
                filetypes=[("Zip archives", "*.zip"), ("All files", "*.*")])
        root.destroy()
        if path:
            return path
    except Exception as exc:  # no display / no tkinter
        print(f"[import] file picker unavailable ({exc}); type the path instead.")
    if not sys.stdin.isatty():
        sys.exit(f"[import] non-interactive session: pass --package-dir with the "
                 f"downloaded {what}.")
    # _prompt, not input: isatty() is unreliable through some Windows shells, so the
    # guard above can let a closed stdin through. A source build never noticed - the
    # zip-or-folder question above absorbed the EOF first - but a precooked pick asks
    # nothing, which made this the first read and turned it into an EOFError traceback.
    path = _prompt(f"[import] Path to the downloaded {what}: ").strip().strip('"')
    if not path or not os.path.exists(path):
        sys.exit(f"[import] path not found: {path!r}")
    return path


def _has_descriptor(src):
    """True if the package at `src` (a .zip or a folder) already carries a CARLA
    map descriptor - any *.json other than the road-painter decals file. A raw
    RoadRunner export carries none (only .fbx/.xodr/.geojson/.rrdata.xml), which
    is how stage_package tells a hand-authored package from one whose descriptor
    must be generated.

    `.geojson` deliberately does not count: like CARLA's own `fnmatch("*.json")`,
    the check needs the literal '.json', so `<map>.geojson` is ignored."""
    def is_desc(fname):
        base = os.path.basename(fname).lower()
        return base.endswith(".json") and base != "roadpainter_decals.json"
    if os.path.isfile(src) and src.lower().endswith(".zip"):
        with zipfile.ZipFile(src) as z:
            return any(is_desc(n) for n in z.namelist())
    if os.path.isdir(src):
        for _root, _dirs, files in os.walk(src):
            if any(is_desc(f) for f in files):
                return True
    return False


def _tile_split(fbx_name):
    """(stem, x, y) parsed from a strict `<stem>_Tile_<x>_<y>.fbx`, else None.

    Strict on purpose: CARLA reads the streaming-grid index off the last two
    underscore tokens of the tile name (LoadAssetMaterialsCommandlet.cpp), so a
    loose `<map>_Tile_0_0_final.fbx` would silently cook as tile (0,0) and
    collide. Anything not exactly `<stem>_Tile_<int>_<int>.fbx` is not a tile -
    the caller warns about it rather than guessing.

    Returns the stem too, so a tile set can be recognised without knowing its
    name in advance: a raw export names its tiles after itself, not after the
    map we are about to cook."""
    if not fbx_name.lower().endswith(".fbx"):
        return None
    m = re.fullmatch(r"(.+)_Tile_(\d+)_(\d+)", fbx_name[:-4])
    return (m.group(1), int(m.group(2)), int(m.group(3))) if m else None


def _find_export(scan_root, map_name):
    """(asset_dir, stem) of the RoadRunner export staged under `scan_root`, where
    `stem` is the name the export gave itself - the stem of its `.xodr`.

    Discovered, never assumed. A RoadRunner export carries the author's working
    name (`MLK_no_signal_0805`, `JC_Newest_Ver_MLK_noped1002_final_debug`), which
    is essentially never the name the map should be cooked under, so requiring
    `<map_name>.xodr` here just fails on every real export.

    One package holds one map. That is the rule for a Digital-Twin-Library bundle
    and for a folder picked by hand, and a tiled map is still one map - so a
    second .xodr is not a choice to offer, it is a packaging mistake to report.
    Both shapes of it land on the same destination anyway: this import is headed
    for a single `/Game/<pkg>/Maps/<map_name>`, which two maps would overwrite in
    turn."""
    hits = []
    for root, _dirs, files in os.walk(scan_root):
        for f in sorted(files):
            if f.lower().endswith(".xodr"):
                hits.append((root, f[:-5]))
    if not hits:
        sys.exit(f"[import] cannot describe '{map_name}': no .xodr under {scan_root}. "
                 f"A raw RoadRunner export must ship its OpenDRIVE road network "
                 f"(<export>.xodr) beside its <export>.fbx.")
    if len(hits) == 1:
        return hits[0]

    rel = lambda h: os.path.relpath(os.path.join(h[0], h[1] + ".xodr"), scan_root)
    what = (f"{hits[0][1]}.xodr is staged in {len(hits)} places (staged more than once?)"
            if len({h[1] for h in hits}) == 1 else
            f"{len(hits)} maps are staged here, and one package holds one map")
    where = "\n".join(f"             {rel(h)}" for h in sorted(hits))
    sys.exit(f"[import] cannot describe '{map_name}': {what}. Keep one, so a single "
             f"descriptor maps to a single destination:\n{where}")


def _export_fbx(asset_dir, stem):
    """(source_fbx, [tile_fbx, ...]) - bare filenames - for the export `stem` in
    `asset_dir`. Exactly one of the two is populated: a map is single-source or
    tiled, never both.

    Normally the fbx is named after the .xodr - that is what RoadRunner writes.
    When it is not, one-map-per-package (_find_export) still identifies it: a
    lone .fbx, or a single tile set, can only be this map's geometry. It is taken
    with a WARNING rather than silently, because a stem mismatch usually means
    the package was assembled by hand, and the one .fbx staged might be scenery
    rather than the road network - which the cook would not catch.

    Several unrelated .fbx (a RoadRunner layer-split export) cannot be resolved
    that way, so it is reported: cooking the wrong layer as the whole map only
    shows up after the cook, and the fix belongs in the package."""
    fbx = sorted(f for f in os.listdir(asset_dir) if f.lower().endswith(".fbx"))
    single = stem + ".fbx" if stem + ".fbx" in fbx else None
    tiles = [f for f in fbx if (_tile_split(f) or (None,))[0] == stem]
    if single and tiles:
        sys.exit(f"[import] cannot describe '{stem}': both {stem}.fbx and "
                 f"{stem}_Tile_*.fbx are present - a map is single-source or tiled, "
                 f"not both.")
    if single or tiles:
        return single, tiles
    if not fbx:
        sys.exit(f"[import] cannot describe '{stem}': found {stem}.xodr but no .fbx "
                 f"beside it in {asset_dir}.")

    # The .xodr and the .fbx were named apart.
    plain = [f for f in fbx if _tile_split(f) is None]
    prefixes = sorted({t[0] for t in (_tile_split(f) for f in fbx) if t})
    if len(prefixes) == 1 and not plain:
        chosen = [f for f in fbx if _tile_split(f)[0] == prefixes[0]]
        print(f"[import] warning: {stem}.xodr has no {stem}_Tile_<x>_<y>.fbx. Taking "
              f"the only tile set staged, {prefixes[0]}_Tile_*.fbx ({len(chosen)} "
              f"tiles) - check that is this map's geometry.")
        return None, chosen
    if len(plain) == 1 and not prefixes:
        print(f"[import] warning: {stem}.xodr has no {stem}.fbx. Taking the only "
              f"geometry staged, {plain[0]} - check that is this map's road network "
              f"and not, say, scenery.")
        return plain[0], []
    listed = "\n".join(f"             {f}" for f in fbx)
    sys.exit(f"[import] cannot describe '{stem}': {stem}.xodr has no {stem}.fbx or "
             f"{stem}_Tile_<x>_<y>.fbx, and several geometries could be it:\n{listed}\n"
             f"         One package holds one map: name the .fbx after the .xodr, or "
             f"stage only that map's geometry.")


def _rename_export(asset_dir, stem, map_name, source, tiles):
    """Rename the staged export's geometry from `stem` to `map_name`, in place.
    Returns (source, tiles) under their new names.

    Why rename rather than name the map after the export: the cook writes the
    walker navmesh as `Maps/<name>/Nav/<fbx_stem>.bin` (CARLA's
    Util/BuildTools/Import.py build_binary_for_navigation), while at runtime
    FNavigationMesh::Load asks for `<MapName>.bin` - so geometry whose stem is
    not the map name costs that map its pedestrian navigation, silently. Keeping
    package == map == umap == fbx stem also keeps a re-export importable under
    the SAME name, so run profiles, app manifests and the library catalog keep
    resolving instead of orphaning on every export date.

    Only the .fbx moves. CARLA copies the .xodr to `<name>.xodr` itself, and the
    .geojson / .rrdata.xml stay put so Import/<map_name>/ still shows which
    export it came from. A sibling `<stem>.fbm` (embedded textures - normally
    created by the importer, occasionally shipped) moves with its .fbx so the
    pair cannot drift."""
    def move(old, new):
        if old == new:
            return new
        src, dst = os.path.join(asset_dir, old), os.path.join(asset_dir, new)
        if os.path.exists(dst):
            sys.exit(f"[import] cannot rename {old} -> {new}: {new} already exists in "
                     f"{asset_dir}. Stage this export on its own.")
        os.rename(src, dst)
        fbm = os.path.join(asset_dir, old[:-4] + ".fbm")
        if os.path.isdir(fbm):
            os.rename(fbm, os.path.join(asset_dir, new[:-4] + ".fbm"))
        return new

    if source:
        source = move(source, map_name + ".fbx")
    tiles = [move(t, f"{map_name}_Tile_{xy[1]}_{xy[2]}.fbx")
             for t, xy in ((t, _tile_split(t)) for t in tiles)]
    print(f"[import] renamed the export geometry '{stem}' -> '{map_name}' so the cooked "
          f"map, its assets and its navmesh all answer to one name.")
    return source, tiles


def _tile_size(asset_dir):
    """RoadRunner tile edge length, in metres, for a tiled export in `asset_dir`.

    Prefer the export's own TilesInfo.txt (`firstTileCenterX,firstTileCenterY,
    tileSize`); fall back to 2000 when it is absent - the common case. 2000 is
    both the cook commandlet's own default (PrepareAssetsForCookingCommandlet)
    and the maximum Unreal honours (FbxStaticMeshImport clamps TileSize > 2000),
    so the descriptor and the cook agree with nothing to infer."""
    info = os.path.join(asset_dir, "TilesInfo.txt")
    if os.path.isfile(info):
        try:
            with open(info, encoding="utf-8") as f:
                for line in f:
                    nums = re.findall(r"-?\d+(?:\.\d+)?", line)
                    if len(nums) >= 3:
                        return int(round(float(nums[2])))
        except (OSError, ValueError):
            pass
        print(f"[import] warning: {info} present but unparseable; using tile_size=2000")
    return 2000


def generate_descriptor(import_dir, map_name):
    """Synthesise Import/<map_name>.json for a raw RoadRunner export that shipped
    no CARLA descriptor, deriving everything from the staged filenames (see
    FIXS_Applications #4). stage_package isolates a raw export under
    Import/<map_name>/, so that subtree is scanned - never the shared Import/
    root. Returns the descriptor path.

    The export names itself and we rename to match. RoadRunner writes the
    author's working name, so nothing here may assume `<map_name>.xodr` exists:
    _find_export discovers the export, _rename_export renames its geometry to
    `map_name`, and the descriptor is written under that single name. Cooking
    under the export's own name instead would make every re-export a *different*
    CARLA map and orphan the run profiles, app manifests and catalog entries
    pointing at the old one - and would break the walker navmesh (_rename_export).

    Resolve or report, never guess. One package holds one map (a tiled map is
    still one map), so anything that breaks that rule - no .xodr, several .xodr,
    geometry that cannot be tied to the road network - is a packaging mistake
    named on the spot rather than a prompt or a coin flip."""
    import json

    subtree = os.path.join(import_dir, map_name)
    scan_root = subtree if os.path.isdir(subtree) else import_dir
    asset_dir, stem = _find_export(scan_root, map_name)
    source, tiles = _export_fbx(asset_dir, stem)
    if stem != map_name:
        source, tiles = _rename_export(asset_dir, stem, map_name, source, tiles)

    rel = lambda f: os.path.relpath(os.path.join(asset_dir, f),
                                    import_dir).replace(os.sep, "/")
    entry = {"name": map_name,
             # Left under the export's own name: CARLA copies it to <name>.xodr
             # on import (Import.py, "Move maps XODR files if any").
             "xodr": rel(stem + ".xodr"),
             "use_carla_materials": False}  # RoadRunner ships its own materials
    if source:
        entry["source"] = rel(source)
        kind = "single-source"
    else:
        entry["tile_size"] = _tile_size(asset_dir)
        entry["tiles"] = [rel(t) for t in sorted(tiles)]
        kind = f"tiled, {len(tiles)} tiles, tile_size={entry['tile_size']}"
    if stem != map_name:
        # Provenance: which export this map was cooked from. CARLA's importer
        # copies keys it does not know straight through, so this costs nothing
        # and survives in the descriptor for whoever asks "which export is this?".
        entry["exported_as"] = stem

    # Surface, don't silently drop, RoadRunner layer-split exports: extra .fbx
    # that are neither the source nor a strict tile. CARLA's own generator
    # ignores them; a dropped layer should at least be visible.
    kept = {source} if source else set(tiles)
    for f in sorted(os.listdir(asset_dir)):
        if f.lower().endswith(".fbx") and f not in kept:
            print(f"[import] warning: ignoring unexpected fbx {f!r} (not {map_name}.fbx "
                  f"or {map_name}_Tile_<x>_<y>.fbx); not part of the descriptor.")

    descriptor = os.path.join(import_dir, map_name + ".json")
    with open(descriptor, "w", encoding="utf-8") as f:
        json.dump({"maps": [entry], "props": []}, f, indent=2)
        f.write("\n")
    print(f"[import] generated {descriptor} ({kind})")
    return descriptor


def stage_package(carla_root, name, package_url=None, package_dir=None, package_pick=False):
    """Ensure <carla_root>/Import has <name>.json (+ its assets). Sources the
    package, in order: an already-staged descriptor, an explicit --package-dir,
    a hand-picked download (--package-pick, always asks), an opportunistic gh
    download of --package-url, else a file picker for a hand-downloaded copy."""
    import_dir = os.path.join(carla_root, "Import")
    descriptor = _descriptor(carla_root, name)
    if os.path.isfile(descriptor) and not package_url and not package_dir and not package_pick:
        print(f"[import] package already staged: {descriptor}")
        return import_dir

    os.makedirs(import_dir, exist_ok=True)
    tmpdir = None
    try:
        if package_dir:
            src = package_dir
        elif package_pick:
            src = _select_package(name, package_url)  # forced manual select
        else:
            src, tmpdir = _try_gh_download(package_url)
            if src is None:
                src = _select_package(name, package_url)
        if _has_descriptor(src):
            # Hand-authored package: <name>.json + its <name>/ asset folder land
            # directly under Import/.
            _stage_from_path(src, import_dir)
        else:
            # Raw RoadRunner export (no CARLA .json): isolate it under
            # Import/<name>/ so repeat imports of different maps cannot collide,
            # then synthesise Import/<name>.json from the fbx/xodr (see #4).
            print(f"[import] '{name}' ships no CARLA descriptor; treating it as a "
                  f"raw RoadRunner export and generating one.")
            raw_dest = os.path.join(import_dir, name)
            fresh = not os.path.isdir(raw_dest)
            os.makedirs(raw_dest, exist_ok=True)
            _stage_from_path(src, raw_dest)
            try:
                generate_descriptor(import_dir, name)
            except SystemExit:
                # Don't leave a half-staged export behind for the next attempt to
                # trip over: an abandoned copy reads as the same map staged twice.
                if fresh:
                    shutil.rmtree(raw_dest, ignore_errors=True)
                    print(f"[import] removed the partially staged {raw_dest}")
                raise
    finally:
        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)

    if not os.path.isfile(descriptor):
        sys.exit(f"[import] after staging, descriptor still missing: {descriptor}\n"
                 f"         (a packaged map must contain {name}.json; a raw export "
                 f"must be named after the map so one can be generated)")
    return import_dir


def _looks_like_bundle(names):
    """True if a package's entries are rooted under a top-level `carla/` - the
    Digital-Twin-Library combined layout (`carla/` + `sumo/`) - rather than a
    flat/legacy CARLA-only package."""
    tops = {n.replace("\\", "/").split("/", 1)[0] for n in names if n.strip()}
    return "carla" in tops


def _bundle_members(z, sumo_only):
    """Which entries of a bundle zip to extract: everything, or only `sumo/`."""
    if not sumo_only:
        return None                      # extractall's own default
    return [n for n in z.namelist()
            if n.replace("\\", "/").split("/", 1)[0] == "sumo"]


def open_bundle(src, cache_name=None):
    """Split a map source into its CARLA package and its SUMO scenario.

    A Digital-Twin-Library map ships as one bundle - a zip (or folder) with a
    top-level `carla/` (the CARLA import package) and `sumo/` (the scenario).
    Returns `(carla_src, sumo_dir)`:
      - bundle  -> (`<cache>/carla`, `<cache>/sumo`); a zip is extracted once
                   (re-extracted only when the zip is newer). `cache_name` (the
                   cooked map name) extracts into ~/.fixs/maps/<cache_name>/, else
                   a sibling `<stem>_unpacked/`. A folder is used in place.
      - bundle, CLIENT mode -> `(None, <cache>/sumo)`; the CARLA half is not
                   extracted at all, because a machine with no local CARLA has
                   nothing to import it into (FIXS#309).
      - legacy CARLA-only zip/folder -> `(src, None)`, unchanged behavior.
    `carla_src` is what to hand `ensure_map`/`stage_package`; `sumo_dir` (or None)
    is where run_cosim finds the `.sumocfg`."""
    if os.path.isdir(src):
        carla = os.path.join(src, "carla")
        sumo = os.path.join(src, "sumo")
        if os.path.isdir(carla):
            return carla, (sumo if os.path.isdir(sumo) else None)
        return src, None
    if os.path.isfile(src) and src.lower().endswith(".zip"):
        with zipfile.ZipFile(src) as z:
            if not _looks_like_bundle(z.namelist()):
                return src, None
            unpacked = _map_cache_dir(cache_name) if cache_name else os.path.join(
                os.path.dirname(os.path.abspath(src)),
                os.path.splitext(os.path.basename(src))[0] + "_unpacked")
            # A client-mode machine has no local CARLA, so it never imports: the
            # bundle's carla/ half is written once and read never (368 MB of
            # roosevelt_full's 370). Skip it. Nothing to record about the omission -
            # a machine that later gains a CARLA finds no cooked map, so its
            # preflight re-downloads and extracts the bundle in full.
            sumo_only = _mode() == "client"
            # Freshness is judged against a directory we ACTUALLY extract: sentinel
            # on carla/ while extracting sumo-only would be permanently absent, so
            # every run would re-extract. `unpacked` itself cannot serve, since in a
            # per-map cache it also holds the downloaded zip. Clear only the bundle's
            # own subdirs, so that sibling zip survives - and props/ is in the list
            # because a prop left behind by an older bundle would otherwise be
            # installed forever (FIXS#223).
            landmark = os.path.join(unpacked, "sumo" if sumo_only else "carla")
            if not os.path.isdir(landmark) or \
                    os.path.getmtime(src) > os.path.getmtime(landmark):
                for sub in ("carla", "sumo", "props"):
                    shutil.rmtree(os.path.join(unpacked, sub), ignore_errors=True)
                os.makedirs(unpacked, exist_ok=True)
                z.extractall(unpacked, members=_bundle_members(z, sumo_only))
                print(f"[import] unpacked {'sumo half of ' if sumo_only else ''}"
                      f"bundle -> {unpacked}")
        carla = os.path.join(unpacked, "carla")
        sumo = os.path.join(unpacked, "sumo")
        if os.path.isdir(carla):
            carla_src = carla
        else:
            # None rather than `unpacked` when the CARLA half was skipped on purpose:
            # handing back a directory that does not contain a CARLA package would
            # send an importer off to fail on it. Absent by choice is not the same as
            # a legacy layout.
            carla_src = None if sumo_only else unpacked
        return carla_src, (sumo if os.path.isdir(sumo) else None)
    return src, None


def bundle_sumocfg(sumo_dir):
    """The single `.sumocfg` inside a bundle's `sumo/` dir, or None if there is no
    `sumo/`. Exits if the dir holds more than one (ambiguous - caller should pass
    an explicit --sumocfg)."""
    if not sumo_dir or not os.path.isdir(sumo_dir):
        return None
    cfgs = sorted(f for f in os.listdir(sumo_dir) if f.lower().endswith(".sumocfg"))
    if not cfgs:
        return None
    if len(cfgs) > 1:
        sys.exit(f"[import] {sumo_dir} has {len(cfgs)} .sumocfg files {cfgs}; "
                 f"pass --sumocfg to choose one.")
    return os.path.join(sumo_dir, cfgs[0])


def map_name_in(carla_src):
    """The real map/package name a staged CARLA source describes - the stem of its
    lone `<name>.json` descriptor, else its lone `<name>.xodr`. This is the name
    CARLA actually cooks/loads, which need NOT equal a release/location tag (e.g.
    the `roosevelt` bundle's carla/ describes `Roosevelt_07142026`). None if it
    cannot be told unambiguously (0 or >1 candidates)."""
    if not carla_src or not os.path.isdir(carla_src):
        return None

    def stems(ext):
        found = []
        for root, _dirs, files in os.walk(carla_src):
            for f in files:
                if f.lower().endswith(ext):
                    found.append(f[:-len(ext)])
        return found

    jsons = [j for j in stems(".json") if j.lower() != "roadpainter_decals"]
    if len(jsons) == 1:
        return jsons[0]
    xodrs = list(set(stems(".xodr")))
    if len(xodrs) == 1:
        return xodrs[0]
    return None


def _dir_with_sumocfg(root):
    """The directory under `root` that directly holds a .sumocfg (e.g. a lone
    'SUMO files/' wrapper inside a scenario zip), or None."""
    if not root or not os.path.isdir(root):
        return None
    for cur, _dirs, files in os.walk(root):
        if any(f.lower().endswith(".sumocfg") for f in files):
            return cur
    return None


def _sumo_scenario_dir(src, cache_name=None):
    """If `src` (a .zip or folder) is a SUMO-only scenario - a .sumocfg present,
    no CARLA asset (.xodr/.fbx) - return the dir holding the .sumocfg. A zip is
    unpacked once into the map's folder ~/.fixs/maps/<cache_name>/sumo/ (so a
    separately-picked scenario lands under its map, cooked-name), else a sibling
    `<stem>_unpacked/`. Else None. The sumo half of classify_source."""
    if os.path.isfile(src) and src.lower().endswith(".zip"):
        with zipfile.ZipFile(src) as z:
            names = [n.lower() for n in z.namelist()]
            if not any(n.endswith(".sumocfg") for n in names):
                return None
            if any(n.endswith((".xodr", ".fbx")) for n in names):
                return None  # carries CARLA geometry -> not sumo-only
            unpacked = os.path.join(_map_cache_dir(cache_name), "sumo") if cache_name else \
                os.path.join(os.path.dirname(os.path.abspath(src)),
                             os.path.splitext(os.path.basename(src))[0] + "_unpacked")
            if not os.path.isdir(unpacked) or os.path.getmtime(src) > os.path.getmtime(unpacked):
                shutil.rmtree(unpacked, ignore_errors=True)
                os.makedirs(unpacked, exist_ok=True)
                z.extractall(unpacked)
        return _dir_with_sumocfg(unpacked)
    if os.path.isdir(src):
        has_cfg = has_carla = False
        for _cur, _dirs, files in os.walk(src):
            for f in files:
                fl = f.lower()
                has_cfg = has_cfg or fl.endswith(".sumocfg")
                has_carla = has_carla or fl.endswith((".xodr", ".fbx"))
        if has_cfg and not has_carla:
            src_dir = _dir_with_sumocfg(src)
            # Cache a folder pick under the map (cooked-name), like the zip branch
            # above, so a later Local (already-imported) pick of this map reuses it
            # without a re-prompt. Skip the copy if it is already the cached dir.
            if cache_name and src_dir:
                dest = os.path.join(_map_cache_dir(cache_name), "sumo")
                if os.path.abspath(src_dir) != os.path.abspath(dest):
                    shutil.rmtree(dest, ignore_errors=True)
                    shutil.copytree(src_dir, dest)
                return _dir_with_sumocfg(dest)
            return src_dir
    return None


def classify_source(src, cache_name=None):
    """What a map source provides, as (carla_src, sumo_src):
      - bundle (carla/ + sumo/) -> (carla dir, sumo dir)
      - carla-only pkg/export   -> (carla src, None)
      - sumo-only scenario      -> (None, sumo dir)
    A source fills the CARLA slot, the SUMO slot, or both; run_cosim fills any slot
    a pick leaves empty. Zips are unpacked (into ~/.fixs/maps/<cache_name>/ when the
    cooked map name is known) as needed."""
    carla_src, sumo_dir = open_bundle(src, cache_name)   # bundle split, else (src, None)
    if sumo_dir is not None:
        return carla_src, sumo_dir                  # bundle: both slots
    sdir = _sumo_scenario_dir(src, cache_name)
    if sdir is not None:
        return None, sdir                           # sumo-only
    return carla_src, None                          # carla-only


def fetch_catalog(repo):
    """The DT-Library catalog (list of map entries), fetched fresh via gh from
    `repo`'s catalog.json and cached at ~/.fixs/catalog.json. Falls back to the
    cache when offline / gh is unavailable; [] if neither works. Read every run so
    the map list, real names, and per-map settings stay current."""
    import json
    cache = os.path.join(os.path.dirname(env.CONFIG_PATH), "catalog.json")
    gh = shutil.which("gh")
    if gh:
        try:
            out = subprocess.run(
                [gh, "api", f"repos/{repo}/contents/catalog.json",
                 "-H", "Accept: application/vnd.github.raw+json"],
                capture_output=True, text=True, timeout=15)
            if out.returncode == 0 and out.stdout.strip():
                maps = json.loads(out.stdout).get("maps", [])
                try:
                    with open(cache, "w", encoding="utf-8") as f:
                        f.write(out.stdout)
                except OSError:
                    pass
                return maps
        except Exception:
            pass
    if os.path.isfile(cache):
        try:
            with open(cache, encoding="utf-8") as f:
                return json.load(f).get("maps", [])
        except (OSError, ValueError):
            pass
    return []


def catalog_entry(catalog, name):
    """The catalog entry known by `name`, or None.

    Matches any of the three names one entry answers to: its `location` (the
    picker's Online label, and what --map takes), its `map_name` (the REAL cooked
    CARLA name, which is what a Local pick and resolve_cooked_map deal in), and
    its `release` tag. These deliberately differ - location 'roosevelt' vs
    map_name 'roosevelt_full' - so matching `location` alone silently dropped the
    entry whenever the caller held a cooked name. That made one map resolve two
    different ways depending on which picker entry you came in through: picking
    'roosevelt' applied the entry's settings (net_offset: zero), while picking
    'roosevelt_full' matched nothing, fell back to defaults, and left the CARLA
    spectator framed on a sign-flipped y."""
    if not name:
        return None
    for m in catalog or []:
        if name in (m.get("location"), m.get("map_name"), m.get("release")):
            return m
    return None


def run_import(carla_root, ue4_root, name):
    """Run CARLA's Import.py to cook the staged package. Returns its exit code."""
    import_py = os.path.join(carla_root, "Util", "BuildTools", "Import.py")
    if not os.path.isfile(import_py):
        sys.exit(f"[import] {import_py} not found - is {carla_root} a CARLA source build?")
    proc_env = dict(os.environ)
    if ue4_root:
        proc_env["UE4_ROOT"] = ue4_root
    if not proc_env.get("UE4_ROOT"):
        print("[import] WARNING: UE4_ROOT not set; the cook commandlet may fail.")
    cmd = [sys.executable, import_py, f"--package={name}"]
    print(f"[import] running: {' '.join(cmd)}  (cwd={carla_root})")
    print("[import] cooking the map can take several minutes ...")
    # CARLA's Import.py cooks EVERY *.json under Import/ (its --package flag does
    # not filter), so a leftover package from an earlier import cross-contaminates
    # this cook - and if it names an already-cooked map, the whole run aborts.
    # Isolate: stash the other descriptors (+ their asset folders) for the cook,
    # restore them after.
    restore = _isolate_import(os.path.join(carla_root, "Import"), name)
    try:
        return subprocess.call(cmd, cwd=carla_root, env=proc_env)
    finally:
        restore()


def _isolate_import(import_dir, keep):
    """Temporarily move every package under `import_dir` except `keep` aside, so
    CARLA's Import.py cooks only `keep`. Returns a restore() to move them back
    (call it in a finally). `keep`'s own descriptor + asset folder and the shared
    roadpainter_decals.json stay put."""
    if not os.path.isdir(import_dir):
        return lambda: None
    stash = tempfile.mkdtemp(prefix="fixs-import-stash-")
    moved = []
    for f in sorted(os.listdir(import_dir)):
        if not f.lower().endswith(".json") or f.lower() == "roadpainter_decals.json":
            continue
        base = f[:-len(".json")]
        if base == keep:
            continue
        shutil.move(os.path.join(import_dir, f), os.path.join(stash, f))
        moved.append(f)
        folder = os.path.join(import_dir, base)
        if os.path.isdir(folder):
            shutil.move(folder, os.path.join(stash, base))
            moved.append(base)
    if moved:
        print(f"[import] isolating '{keep}' for the cook (set aside {len(moved)} "
              f"other Import/ item(s), restored after)")

    def restore():
        for m in moved:
            src = os.path.join(stash, m)
            if os.path.exists(src):
                shutil.move(src, os.path.join(import_dir, m))
        shutil.rmtree(stash, ignore_errors=True)
    return restore


def _resolve_carla(carla_root=None, ue4_root=None, need_source=True):
    """Resolve (carla_root, ue4_root) from the saved env config, running first-time
    setup if nothing is configured yet. Exits with a clear message if no CARLA is
    configured. Explicit args win over the saved config.

    `need_source` is the operation's requirement, not a property of the machine:
    COOKING an fbx/xodr runs the Unreal editor and genuinely needs a source build,
    but INSTALLING an already-cooked package (install_cooked) is a file copy and
    works on either flavour. The two were conflated here, which is why a packaged
    CARLA could not consume a precooked map at all."""
    cfg = env.load_config()
    if cfg is None and not carla_root:
        # first use on a fresh clone: configure the CARLA env, just like run_cosim
        print("[import] no CARLA env configured; running first-time setup ...")
        cfg = env.run_setup()
        # ... and continue under the interpreter setup just bound. main() already
        # re-exec'd on the config as it stood on entry; there WAS no config then, so
        # this is the first moment the answer exists - and run_import below hands
        # sys.executable to CARLA's Import.py.
        env.reexec_under_configured(__file__, cfg, tag="import")
    cfg = cfg or {}
    carla_root = carla_root or cfg.get("carla_root")
    ue4_root = ue4_root or cfg.get("ue4_root")
    if not carla_root:
        if cfg.get("mode") == "client":
            sys.exit("[import] this machine is configured as a CARLA CLIENT - CARLA runs "
                     "on another host, so there is no local install to import into.\n"
                     "        Import the map on that host, or re-run setup_carla here and "
                     "pick a local packaged or source build.")
        sys.exit("[import] no CARLA configured - run setup_carla first.")
    if need_source and cfg.get("mode") == "packaged":
        sys.exit("[import] the configured CARLA is PACKAGED; cooking a custom map from "
                 "fbx/xodr needs a SOURCE build (run setup_carla and pick source).\n"
                 "        A map published with a precooked asset installs here without "
                 "one - run it through run_cosim, which picks that path automatically.")
    return carla_root, ue4_root


def ensure_map(name, carla_root=None, ue4_root=None, package_url=None,
               package_dir=None, force=False, prompt_if_exists=False,
               package_pick=False, source_sha=None):
    """Make sure `name` is imported into the (source-build) CARLA, importing it
    if needed. Returns 0 on success; exits with a clear message otherwise.

    If the map already exists: `force` re-imports unconditionally; otherwise, when
    `prompt_if_exists` and the session is interactive, the user is asked whether to
    re-import (re-download + re-cook) - handy for updating a map or testing.

    `source_sha` is the digest of the release asset being cooked; it is stamped on
    the result so a later run can tell which bundle this map came from."""
    carla_root, ue4_root = _resolve_carla(carla_root, ue4_root)

    umap = cooked_map_path(carla_root, name)
    already = os.path.isfile(umap)
    if already and not force:
        print(f"[import] '{name}' already imported: {umap}")
        if not (prompt_if_exists and sys.stdin.isatty()):
            return 0
        ans = input("[import] re-import it now (re-download + re-cook)? [y/N]: ").strip().lower()
        if ans != "y":
            print("[import] keeping the existing map.")
            return 0
        print("[import] re-importing ...")

    stage_package(carla_root, name, package_url, package_dir, package_pick)

    # CARLA's ImportAssets commandlet imports cleanly into an empty destination
    # but often fails to *replace* existing cooked content (the .umap is left
    # untouched). So for a re-import we move the old content aside first and
    # import fresh; the backup is restored if the cook fails, so a working map is
    # never lost.
    content_dir = cooked_content_dir(carla_root, name)
    backup = content_dir + ".bak_reimport"
    if already and os.path.isdir(content_dir):
        shutil.rmtree(backup, ignore_errors=True)
        os.rename(content_dir, backup)
        print(f"[import] moved existing content aside for a clean re-import (restored on failure)")

    rc = run_import(carla_root, ue4_root, name)
    ok = os.path.isfile(umap)

    if os.path.isdir(backup):
        if ok:
            shutil.rmtree(backup, ignore_errors=True)
        else:
            shutil.rmtree(content_dir, ignore_errors=True)
            os.rename(backup, content_dir)
            print("[import] import did not produce the map; restored the previous one.")

    if not ok:
        sys.exit(f"[import] import failed: Import.py exited {rc} and {umap} was not produced.")
    if rc != 0:
        print(f"[import] note: Import.py exited {rc} (CARLA's cook commonly flags non-fatal "
              f"warnings as errors), but the map .umap was written.")
    # After the restore-on-failure dance above, so the stamp only ever describes
    # content that actually got cooked - a failed re-import restores the previous
    # map together with its previous stamp.
    _write_sha(cooked_content_dir(carla_root, name), source_sha)
    print(f"[import] done: '{name}' imported -> {umap}")
    return 0


# ------------------------------------------------- precooked (packaged CARLA)

def _safe_members(tar, dest):
    """Yield `tar`'s members, refusing any that would write outside `dest`.

    tarfile.extractall follows absolute paths, `..` and symlinks straight out of
    the destination; we extract release assets, so the archive is only as
    trustworthy as whoever can publish one. Python 3.12's filter='data' does this,
    but the shipped env is 3.10 (environment.yml), so it is done here."""
    root = os.path.realpath(dest)
    for m in tar.getmembers():
        target = os.path.realpath(os.path.join(dest, m.name))
        if target != root and not target.startswith(root + os.sep):
            sys.exit(f"[import] refusing to extract '{m.name}': it escapes {dest}")
        if m.issym() or m.islnk():
            link = os.path.realpath(os.path.join(os.path.dirname(target), m.linkname))
            if not link.startswith(root + os.sep):
                sys.exit(f"[import] refusing link '{m.name}' -> '{m.linkname}': "
                         f"it escapes {dest}")
        yield m


def cooked_asset_name(asset):
    """The precooked asset that pairs with source bundle `asset`, by convention:
    `<stem>_cooked.tar.gz` (mlk_no_signal.zip -> mlk_no_signal_cooked.tar.gz).

    A fallback, not the truth. The catalog is meant to name the cooked asset
    outright (FIXS_Applications#27); until that field lands this derives it, and
    a derived name that is not actually published fails the same way a wrong
    catalog entry would - at the download, by name, before anything is cooked or
    launched."""
    if not asset:
        return None
    stem = asset[:-4] if asset.lower().endswith(".zip") else asset
    return stem + "_cooked.tar.gz"


def catalog_cooked_asset(entry):
    """The precooked asset name for a catalog entry, or None if that map has no
    precooked build published. `cooked_asset` when the catalog says so; else the
    conventional name derived from the source bundle."""
    if not entry:
        return None
    named = entry.get("cooked_asset")
    if named:
        return named
    # An explicit false/empty cooked_asset is a deliberate "this map has none";
    # only a MISSING key falls through to the convention.
    if "cooked_asset" in entry:
        return None
    return cooked_asset_name(entry.get("asset"))


def install_precooked(carla_root, name, repo=None, tag=None, entry=None,
                      local=None, force=False, log="import"):
    """Resolve and install an ALREADY-cooked map package into a PACKAGED CARLA.
    Returns the map name the package actually provides (may differ from `name`).

    This is the packaged flavour's ENTIRE import path. A packaged build cannot cook
    - cooking runs the Unreal editor it does not ship - so the map has to arrive
    cooked, and putting it in place is a plain extract (install_cooked). What sits
    in front of that extract is the part worth sharing: which asset to fetch, and
    the three cases where the honest answer is "not from here".

    Shared by run_cosim's packaged preflight and import_map's picker. It lived only
    in run_cosim, which is why `--import-map` - the command whose whole name is
    importing a map - could not import one on a packaged build (#276).

    `local` is a path the user picked; only a precooked package is usable, and it is
    recognised by what is inside it rather than by its name. Else the asset name
    comes from the catalog `entry` and is downloaded from `repo`/`tag`.
    `log` is the caller's message prefix, so the user reads one voice."""
    # Ask the archive, not the file name. The suffix was standing in for "is a
    # precooked package", and the two disagree at the edges: a `.tgz`, or a tarball
    # renamed on the way through a browser, is perfectly installable and was
    # refused as a "source bundle", while any `.tar.gz` at all was accepted and
    # failed later inside install_cooked. cooked_name_in_tar is the same evidence
    # install_cooked itself uses, so the two can no longer disagree.
    if local and cooked_name_in_tar(local):
        tar_path = local
    elif local:
        # A source bundle (carla/ with fbx+xodr) would have to be cooked, which is
        # the one thing this flavour cannot do.
        sys.exit(f"[{log}] '{local}' is a source bundle and this CARLA is PACKAGED, "
                 f"which cannot cook one. Point at the map's precooked "
                 f"*_cooked.tar.gz, or configure a source build (run_cosim --setup).")
    elif not tag:
        sys.exit(f"[{log}] map '{name}' is not installed in {carla_root} and is not a "
                 f"Digital-Twin-Library map, so there is no precooked package to "
                 f"install. Pick a library map, or point --package-dir at a precooked "
                 f"*_cooked.tar.gz.")
    else:
        asset = catalog_cooked_asset(entry)
        # Say "not published" by NAME, before downloading anything. The alternative
        # is a gh pattern-miss the user has to decode - and for a map whose release
        # genuinely has no cooked asset (atlanta, at the time of writing) that is the
        # expected path, not the exceptional one.
        published = release_assets(repo, tag)
        if asset is None or (published is not None and asset not in published):
            have = ", ".join(published) if published else "unknown"
            sys.exit(
                f"[{log}] no precooked package published for '{name}' "
                f"(release '{tag}' of {repo}).\n"
                f"        A PACKAGED CARLA can only run maps that ship one.\n"
                f"        expected asset: {asset or '(none named in the catalog)'}\n"
                f"        published:      {have}\n"
                f"        Use a source build (run_cosim --setup) to cook this map "
                f"yourself, or ask for a precooked build of it.")
        tar_path = download_cooked_tar(repo, tag, asset, force_redownload=force,
                                       cache_name=name)

    real = install_cooked(carla_root, tar_path, force=force)
    if real != name:
        print(f"[{log}] precooked package provides map '{real}'")
    # Same stamp as the source path, from the .tar.gz asset instead of the .zip:
    # a packaged install goes stale exactly the same way a cook does. A local
    # package leaves it unrecorded, which is the "not checked" state.
    _write_sha(cooked_content_dir(carla_root, real, mode="packaged"),
               _read_sha(os.path.dirname(tar_path)))
    return real


def install_cooked(carla_root, tar_path, name=None, force=False):
    """Install a precooked map package into a PACKAGED CARLA. Returns the map name.

    CARLA's own Util/ImportAssets.sh is exactly

        for f in `find Import/ -name "*.tar.gz"`; do tar --keep-newer-files -xvf $f; done

    run from the package root - a plain extract, no manifest, no commandlet, no
    registration step. That is worth stating because it is the whole basis for
    doing it here in `tarfile` instead: ImportAssets.sh is bash-only and CARLA
    ships no .bat counterpart, so shelling out would leave Windows with no
    packaged path at all. Reimplementing the extract gives both OSes the same one.

    We deliberately do NOT copy `--keep-newer-files`. That flag exists because the
    script re-extracts every tarball sitting in Import/ on each run and must not
    clobber newer local edits; we extract one named asset on purpose, and an
    install that silently skipped files because of mtimes would be a confusing
    half-map. `force` instead removes the old Content/<name> first, mirroring
    ensure_map's clean-reimport."""
    dest = package_root(carla_root)
    if not os.path.isdir(dest):
        sys.exit(f"[import] packaged CARLA root not found: {dest}")

    with tarfile.open(tar_path, "r:*") as tar:
        members = list(_safe_members(tar, dest))
        if name is None:
            name = _cooked_name_in(members)
            if name is None:
                sys.exit(f"[import] cannot tell which map {tar_path} provides "
                         f"(expected CarlaUE4/Content/<name>/Config/<name>.Package.json); "
                         f"pass the name explicitly.")
        content_dir = cooked_content_dir(carla_root, name, mode="packaged")
        if os.path.isdir(content_dir):
            if not force:
                print(f"[import] '{name}' already installed: {content_dir}")
                return name
            print(f"[import] removing existing content for a clean re-install: {content_dir}")
            shutil.rmtree(content_dir, ignore_errors=True)
        print(f"[import] installing precooked '{name}' -> {dest}")
        tar.extractall(dest, members=members)

    umap = cooked_map_path(carla_root, name, mode="packaged")
    if not os.path.isfile(umap):
        sys.exit(f"[import] install finished but {umap} is missing - "
                 f"{tar_path} does not look like a precooked package for '{name}'.")
    print(f"[import] done: '{name}' installed -> {umap}")
    return name


_SHADER_MARKERS = {"d3d": b"DXBC", "vulkan": b"GLSL.std.450"}


def shader_platforms_in(content_dir, size_cap=2 * 1024 * 1024):
    """Which shader platforms a cooked package carries shaders for - a subset of
    {'d3d', 'vulkan'}, empty if none is recognised.

    UE4 compiles a material's shaders per shader platform and bakes the result
    into the cooked asset, so a Linux cook ships SPIR-V and a Windows one ships
    D3D bytecode; CARLA's own Windows package ships BOTH. A packaged build has no
    shader compiler, so a material carrying no map for the running platform
    silently falls back to the default material - the level loads, geometry and
    placed actors are correct, and the road surface renders as grey checkerboard.
    Nothing downstream can detect that, which is why it is detected here.

    Every *_cooked.tar.gz the Digital-Twin-Library publishes today is a Linux cook
    (FIXS_Applications#29), so on Windows this fires.

    A heuristic by necessity: this pattern-matches container magic rather than
    parsing UE4 packages, so it is used to WARN, never to refuse. Only small .uexp
    are read - shader maps live in materials (~80KB), never in the multi-MB .umap
    or mesh blobs - which keeps a full scan to a few hundred KB."""
    found = set()
    for root, _dirs, files in os.walk(content_dir):
        for f in files:
            if not f.endswith(".uexp"):
                continue
            path = os.path.join(root, f)
            try:
                if os.path.getsize(path) > size_cap:
                    continue
                with open(path, "rb") as fh:
                    blob = fh.read()
            except OSError:
                continue
            found.update(k for k, marker in _SHADER_MARKERS.items() if marker in blob)
            if len(found) == len(_SHADER_MARKERS):
                return found          # nothing more to learn
    return found


def host_shader_platform():
    """The shader platform a packaged CARLA uses here by default: D3D on Windows,
    Vulkan elsewhere. Windows CarlaUE4.exe also accepts -vulkan, so a 'vulkan'-only
    package is not necessarily unusable there - only unusable as launched."""
    return "d3d" if platform.system() == "Windows" else "vulkan"


def _cooked_name_in(members):
    """The map name a precooked archive describes - the stem of its lone
    CarlaUE4/Content/<name>/Config/<name>.Package.json. None if 0 or >1, for the
    same reason map_name_in gives up: a name we hand to load_world must be
    resolved from the package, never guessed from the file name."""
    found = set()
    for m in members:
        parts = m.name.replace("\\", "/").split("/")
        if len(parts) >= 5 and parts[-2] == "Config" and parts[-1].endswith(".Package.json"):
            found.add(parts[-1][:-len(".Package.json")])
    return found.pop() if len(found) == 1 else None


def cooked_name_in_tar(path):
    """The map a precooked *.tar.gz provides, read out of the archive itself, or
    None if it is not one / does not say.

    The file name is not the answer: `mlk_no_signal_cooked.tar.gz` happens to carry
    `mlk_no_signal`, but that is a publishing convention, not a rule - which is why
    install_cooked resolves the name from the .Package.json inside. Peeking here
    lets a hand-picked tarball be named before it is installed, so the picker can
    say which map it selected instead of asking the user to guess a name that the
    install would then override anyway."""
    try:
        with tarfile.open(path, "r:*") as tar:
            return _cooked_name_in(tar.getmembers())
    except (OSError, tarfile.TarError):
        return None


def read_map_config(path):
    """Parse a simple `key=value` map config (keys: package, url). Lets an app
    declare its map in one text file instead of hard-coding the URL in a wrapper."""
    cfg = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, val = line.split("=", 1)
            cfg[key.strip().lower()] = val.strip()
    return cfg


GH_HOWTO = ("Install gh (https://cli.github.com) and run `gh auth login`; if it is "
            "already\n        installed, `gh auth status` shows which account and "
            "scopes it is using -\n        a token without `repo` cannot see a "
            "private library and reports it as 404.")


def _require_gh():
    """Path to the GitHub CLI, or exit saying how to proceed without it.

    Only DOWNLOADING needs this. Listing degrades instead (see list_map_releases):
    a machine that cannot reach the library must still be able to import a package
    the user fetched by hand, and that path needs no gh at all."""
    gh = shutil.which("gh")
    if not gh:
        sys.exit(f"[import] downloading a published map needs the GitHub CLI (gh).\n"
                 f"        {GH_HOWTO}\n"
                 f"        Or import a copy you downloaded yourself: --package <name> "
                 f"--package-dir <file>.")
    return gh


def _package_from_tag(tag, tag_prefix=""):
    """Package name = release tag with its prefix stripped. Convention:
    tag `map-<pkg>` <-> asset `<pkg>_carla_import.zip` <-> descriptor `<pkg>.json`."""
    return tag[len(tag_prefix):] if tag_prefix and tag.startswith(tag_prefix) else tag


def list_map_releases(repo, tag_prefix=""):
    """Map releases in `repo` (tag starts with `tag_prefix`), newest first, via gh.
    Each item: {tag, title, date 'YYYY-MM-DD', prerelease}. Drafts are skipped -
    their assets aren't downloadable.

    Returns [] - after saying why - when the releases cannot be listed: no gh, gh
    not authenticated, no network, or a token that cannot see a private library.
    This used to sys.exit, which made a gh problem fatal to the whole picker: the
    menu is printed by choose_map AFTER this call, so the `L) local file` row - the
    one path built to need nothing but browser access - was never reached on
    exactly the failure it exists for (#283). Not listing the library is a smaller
    fact than not being able to import, and only the first one is true here.
    fetch_catalog already degrades the same way, for the same reason."""
    import json
    gh = shutil.which("gh")
    if not gh:
        print(f"[import] the GitHub CLI (gh) is not installed, so the published maps "
              f"cannot be listed.\n        {GH_HOWTO}")
        return []
    try:
        out = subprocess.check_output(
            [gh, "release", "list", "-R", repo, "-L", "100", "--json",
             "tagName,name,publishedAt,isPrerelease,isDraft"],
            text=True, stderr=subprocess.PIPE)
    except (OSError, subprocess.CalledProcessError) as exc:
        detail = (getattr(exc, "stderr", "") or "").strip() or str(exc)
        print(f"[import] `gh release list` failed for {repo}, so the published maps "
              f"cannot be listed:\n{detail}\n        {GH_HOWTO}")
        return []
    rels = []
    for r in json.loads(out):
        tag = r.get("tagName", "")
        if r.get("isDraft") or (tag_prefix and not tag.startswith(tag_prefix)):
            continue
        rels.append({"tag": tag, "title": r.get("name") or "",
                     "date": (r.get("publishedAt") or "")[:10],
                     "prerelease": bool(r.get("isPrerelease"))})
    rels.sort(key=lambda r: r["date"], reverse=True)
    return rels


def _infer_package_name(path):
    """Best-effort map/package name from a downloaded zip or folder: the stem of
    the <name>.json descriptor it contains, else - for a raw RoadRunner export,
    which ships no descriptor - the stem of its lone <name>.xodr. Returns None if
    neither is unambiguous. (Handles Windows-built zips with backslash entries.)

    The .xodr fallback is what map_name_in() already does for a staged bundle;
    without it the picker had to ask for "the name matching <name>.json" on a
    package that has no .json, and any answer but the export's own name then
    failed downstream."""
    entries = []
    if os.path.isfile(path) and path.lower().endswith(".zip"):
        with zipfile.ZipFile(path) as z:
            entries = z.namelist()
    elif os.path.isdir(path):
        entries = [os.path.join(r, f) for r, _d, fs in os.walk(path) for f in fs]
    bases = [os.path.basename(e.replace("\\", "/").rstrip("/")) for e in entries]
    for base in bases:
        low = base.lower()
        if low.endswith(".json") and low != "roadpainter_decals.json":
            return base[:-5]
    xodrs = {b[:-5] for b in bases if b.lower().endswith(".xodr")}
    return xodrs.pop() if len(xodrs) == 1 else None


def _prompt(msg):
    """input() that converts a non-interactive / closed stdin (EOF) into a clean
    exit instead of an EOFError traceback. isatty() is unreliable through some
    Windows shells, so we guard the call itself rather than trust isatty alone."""
    try:
        return input(msg)
    except EOFError:
        sys.exit("[import] non-interactive session (no input); pass an explicit "
                 "--map / --package to run without prompting.")


def _app_map_choice(name, catalog, cooked):
    """How an app-declared map name resolves, as (kind, label, flag, tag), or None
    if it resolves to nothing:
      'release' -> the name matches a Digital-Twin-Library entry (by location,
                   cooked map_name or release tag - see catalog_entry)
      'cooked'  -> not in the library, but already cooked into this CARLA
    A name that is neither costs the app its shortcut and nothing else: the picker
    then behaves exactly as it would with no application selected. That is
    deliberate - a map that has not been published yet must not be able to block a
    run, and an app manifest is data, so a typo in it should not be fatal."""
    ent = catalog_entry(catalog, name)
    if ent:
        flag = "  (Digital-Twin-Library)"
        if (ent.get("map_name") or "") in cooked:
            flag += "  (already imported)"
        return "release", (ent.get("map_name") or name), flag, ent.get("release")
    if name in cooked:
        return "cooked", name, "  (already imported)", None
    return None


def choose_map(repo, tag_prefix="", carla_root=None, catalog=None,
               preferred=None, app_label=None, current=None):
    """Interactive chooser. Two sections plus `L` for a file the user downloaded by
    hand - a .zip / folder on a source build, a precooked *_cooked.tar.gz on a
    packaged one, which is also the only route left when gh cannot reach the
    library (list_map_releases then returns [] and section 1 is simply empty):
      1. ONLINE - Digital-Twin-Library releases in `repo`. When an application is
                  selected and the library HAS its map(s), this section is narrowed
                  to those: an app pinned to Roosevelt should not be offered
                  Atlanta as if the two were interchangeable. If the app's map is
                  not in the library (not published yet, or cooked by hand), there
                  is nothing to narrow to and the full library is listed instead.
      2. LOCAL  - every map already cooked into `carla_root`. NEVER filtered: what
                  is cooked into this CARLA is a property of the machine, not of
                  the application, so an app can never hide a map you could
                  actually run. App maps are marked here, not promoted.
    To browse the whole library while a narrowing app is selected, pick "none" at
    the application prompt.

    Enter takes `current` - the map the setup is already running - wherever it
    landed; then the app's map, wherever IT landed (library or cooked); else item 1.
    A setup pinned to a map has to be able to open this row and back out of it
    unchanged, which "Enter = item 1" did not allow. The cooked copy wins the
    default over the library one: same map, but no re-import to reach it.

    Returns (name, tag, local_path):
      online release -> (name, tag,  None)
      local cooked   -> (name, None, None)   # already imported: run as-is
      local file     -> (name, None, path)
    Exits if the session is non-interactive."""
    releases = list_map_releases(repo, tag_prefix)
    cooked = list_imported_maps(carla_root) if carla_root else []
    preferred = preferred or []
    # Which artefact `L` may hand back is a property of the CARLA installed here,
    # not of the choice - a packaged build can only install a precooked tarball.
    # Resolved once, and used for both the menu wording and the dialog.
    packaged = _mode() == "packaged"

    resolved_app = [r for r in (_app_map_choice(n, catalog, cooked) for n in preferred) if r]
    app_tags = {tag for _k, _l, _f, tag in resolved_app if tag}
    app_names = {label for _k, label, _f, _t in resolved_app}
    if app_tags:
        releases = [r for r in releases if r["tag"] in app_tags]

    print("\n[import] Pick a map to run:")
    menu = []          # menu number -> ("release", release) | ("cooked", name)
    default_idx = 1    # menu number Enter selects; the app's map when there is one
    current_idx = 0    # ... unless the setup already runs one of these
    if releases:
        who = f" for {app_label}" if app_tags and app_label else ""
        print(f"  Online (Digital-Twin-Library){who}:")
        for r in releases:
            menu.append(("release", r))
            pkg = _package_from_tag(r["tag"], tag_prefix)
            # Label with the REAL cooked map name when the catalog knows one, so an
            # Online entry and its Local twin read as the same map ('roosevelt_full'
            # in both lists) instead of two unrelated things ('roosevelt' vs
            # 'roosevelt_full'). The release tag stays the identifier we return and
            # what --map accepts; only the display changes. Safe because
            # catalog_entry() resolves location, map_name and release alike.
            ent = catalog_entry(catalog, pkg)
            label = (ent or {}).get("map_name") or pkg
            flag = "  (pre-release)" if r["prerelease"] else ""
            if label in cooked:
                flag += "  (already imported)"
            if label == current:
                flag += "  (current)"
                current_idx = len(menu)
            print(f"   {len(menu):>2}) {label:<26} {r['date']}{flag}")
    if cooked:
        print("  Local (already imported into CARLA):")
        for name in cooked:
            menu.append(("cooked", name))
            mark = ""
            if name in app_names:
                mark = "  (app map)"
                # An app map the library does not carry still gets to be the
                # default - it just lives in this section instead of the one above.
                if not app_tags:
                    default_idx = len(menu)
            if name == current:
                mark += "  (current)"
                current_idx = len(menu)     # cooked beats the library copy
            print(f"   {len(menu):>2}) {name}{mark}")
    if not menu:
        print("   (no online releases or imported maps found)")
    print("   L) select a local precooked *_cooked.tar.gz instead" if packaged
          else "   L) select a local .zip / folder instead")
    if current_idx:
        default_idx = current_idx

    if not sys.stdin.isatty():
        sys.exit("[import] non-interactive session: cannot prompt. Pass --map / --package "
                 "(+ --package-url/--package-dir) to choose non-interactively.")
    while True:
        hint = f"[1-{len(menu)} / L]" if menu else "[L]"
        default = f", Enter = {default_idx}" if menu else ""
        ans = _prompt(f"[import] Which? {hint}{default}: ").strip().lower()
        if ans == "" and menu:
            ans = str(default_idx)
        if ans == "l":
            path = _select_package("map", None, precooked=packaged)
            if packaged:
                # Nothing to ask: a precooked package carries its own name, and
                # install_cooked reads it from the same .Package.json this does. A
                # name typed here could only disagree with the archive, and the
                # archive would win - so state what was picked instead of asking.
                name = cooked_name_in_tar(path)
                if not name:
                    sys.exit(f"[import] {path} does not look like a precooked map "
                             f"package (no CarlaUE4/Content/<name>/Config/"
                             f"<name>.Package.json inside).\n"
                             f"        A PACKAGED CARLA can only install one of "
                             f"those - it ships no editor to cook a source bundle.")
                print(f"[import] that package provides map '{name}'")
                return name, None, path
            name = _infer_package_name(path)
            if _has_descriptor(path):
                # A packaged map is already named: its descriptor filename IS the
                # package CARLA cooks under, so it is not ours to rename here.
                if not name:
                    name = _prompt("[import] map name (matches <name>.json inside "
                                   "the package): ").strip()
            else:
                # A raw export offers only the author's working name
                # ('MLK_no_signal_0805'). Cooking under it would make the next
                # export a different map, so offer it as a default and let the
                # map be named something that outlives this export.
                hint = f" [Enter = {name}]" if name else ""
                name = _prompt(f"[import] cook this raw export as which map name?"
                               f"{hint}: ").strip() or name
            if not name:
                sys.exit("[import] no package name given; cannot import.")
            return name, None, path
        if ans.isdigit() and 1 <= int(ans) <= len(menu):
            kind, payload = menu[int(ans) - 1]
            if kind == "release":
                return _package_from_tag(payload["tag"], tag_prefix), payload["tag"], None
            return payload, None, None  # cooked: already imported, run as-is
        print("[import] invalid choice; enter a number, or L for a local file.")


def choose_sumo_source(cache_name=None):
    """Prompt for a SUMO scenario (a .zip or folder holding a .sumocfg) and return
    the dir that contains it, or None. Fills the SUMO slot separately when the
    chosen CARLA source ships no sumo/ - e.g. a carla-only local pick or a raw
    export. `cache_name` (the cooked map name) lands the extracted scenario under
    ~/.fixs/maps/<cache_name>/sumo/. Returns None in a non-interactive session so
    the caller can fail with a clear 'pass --sumocfg' message."""
    if not sys.stdin.isatty():
        return None
    print("\n[cosim] the chosen map has no SUMO scenario; select one now "
          "(a .zip or folder containing a .sumocfg).")
    path = _select_package("SUMO scenario", None)  # native picker / typed path
    _carla_src, sumo_dir = classify_source(path, cache_name)
    if sumo_dir is None:
        print(f"[cosim] no .sumocfg found in {path}")
    return sumo_dir


def list_imported_maps(carla_root, mode=None):
    """Names of maps already cooked under this CARLA (i.e. that have a <name>.umap).
    Used to let tools that operate on an existing map (e.g. place_tls) offer a
    local choice without needing gh / the network."""
    content = content_root(carla_root, mode)
    if not os.path.isdir(content):
        return []
    return [name for name in sorted(os.listdir(content))
            if os.path.isfile(cooked_map_path(carla_root, name, mode))]


def choose_imported_map(carla_root, mode=None, interactive=None):
    """Numbered menu of the maps already cooked into this CARLA; returns the chosen
    name. Auto-selects when there is exactly one; exits if there are none, or if a
    choice is needed but the session is non-interactive.

    `interactive` is the caller's answer to "may this stop and ask?"; isatty() is
    only the default. A --serve host has a terminal and no one at it."""
    maps = list_imported_maps(carla_root, mode)
    if not maps:
        sys.exit(f"[import] no cooked maps found under {carla_root}. Import one first "
                 "(e.g. import_<app>_map).")
    if len(maps) == 1:
        return maps[0]
    print("\n[import] Imported maps:")
    for i, m in enumerate(maps, 1):
        print(f"   {i}) {m}")
    if not (sys.stdin.isatty() if interactive is None else interactive):
        sys.exit("[import] non-interactive session: pass --map <name> to choose one.")
    while True:
        ans = _prompt(f"[import] Which map? [1-{len(maps)}]: ").strip()
        if ans.isdigit() and 1 <= int(ans) <= len(maps):
            return maps[int(ans) - 1]
        print("[import] invalid choice; enter a number from the list.")


def _map_cache_dir(name=None):
    """Local map cache: ~/.fixs/maps (next to carla.json), or $FIXS_MAP_CACHE. With
    `name`, the per-map subfolder ~/.fixs/maps/<name>/ - named by the cooked map
    name so it matches CARLA's Content/<name>/; it holds that map's bundle zip,
    extracted carla/ + sumo/, and generated tl_table.csv. Kept outside FIXS/ so
    `initialize` (which wipes FIXS/) never deletes it.

    Everything under here is RE-CREATABLE: downloaded, extracted, or derived from
    what was extracted. Nothing a user hand-edits lives here - scenario yamls are
    app-bounded and live under ~/.fixs/apps/ (see app_catalog.scenario_dir), so
    deleting this tree to reclaim disk can never destroy someone's config. It is
    also shared: one 700MB bundle serves every app that runs that map."""
    d = os.environ.get("FIXS_MAP_CACHE") or os.path.join(os.path.dirname(env.CONFIG_PATH), "maps")
    if name:
        d = os.path.join(d, name)
    os.makedirs(d, exist_ok=True)
    return d


def map_cache_dir(name):
    """The per-map cache dir ~/.fixs/maps/<name>/ - where this map's bundle was
    extracted, and so where its placement.yaml is if it ships one."""
    return _map_cache_dir(name)


def props_cache_dir():
    """Shared props cache ~/.fixs/props/ - ONE copy, for every map.

    Props install to a map-independent content path (/Game/FIXS/Props), so a copy
    shipped inside each map bundle would mean the last map imported silently decides
    which prop every other map places. Fetching one shared copy removes that.

    Sits beside maps/ rather than under it for the same reason, and is equally
    re-creatable: everything here is downloaded, never hand-edited."""
    d = os.environ.get("FIXS_PROPS_CACHE") or os.path.join(
        os.path.dirname(env.CONFIG_PATH), "props")
    os.makedirs(d, exist_ok=True)
    return d


def repo_file(repo, path, ref="main"):
    """Raw bytes of a file in `repo` at `ref`, or None if it cannot be fetched.

    The one place that knows HOW bytes come out of the map library.

    Note the media type: `application/vnd.github.raw`, NOT the `+json` variant
    fetch_catalog uses. On a binary payload the `+json` form fails with
    "transform: short source buffer" because gh tries to transform it as JSON, and
    the caller gets an empty file that then fails md5 verification for a reason
    that has nothing to do with the file. Output is captured as BYTES for the same
    reason - text=True would corrupt a .uasset.

    TODO(no-gh): Digital-Twin-Library is private today, so this needs gh's auth.
    Once it is public this becomes a plain stdlib GET of
        https://raw.githubusercontent.com/{repo}/{ref}/{path}
    with no external binary and no auth, and this function is the only thing that
    changes. FIXS's own repo is already public, so its release downloads can move
    first. Keep new callers going through here rather than shelling to gh directly.
    """
    gh = shutil.which("gh")
    if not gh:
        return None
    try:
        out = subprocess.run(
            [gh, "api", f"repos/{repo}/contents/{path}?ref={ref}",
             "-H", "Accept: application/vnd.github.raw"],
            capture_output=True, timeout=120)     # bytes: no text=True
        if out.returncode == 0 and out.stdout:
            return out.stdout
    except Exception:
        pass
    return None


def props_source(repo):
    """(subdir, ref) for the shared props, from the cached catalog's `props` block.

    Defaults to ("props", "main") so a catalog that predates the block still works.
    Keeping this in the catalog means the ref can be pinned to a tag or sha later
    without a schema change - the catalog IS the pin."""
    import json
    cache = os.path.join(os.path.dirname(env.CONFIG_PATH), "catalog.json")
    block = {}
    if os.path.isfile(cache):
        try:
            with open(cache, encoding="utf-8") as f:
                block = json.load(f).get("props") or {}
        except (OSError, ValueError):
            block = {}
    return block.get("path", "props"), block.get("ref", "main")


def fetch_props(repo, subdir=None, ref=None):
    """Fetch the map library's shared props into ~/.fixs/props. Returns that dir,
    or None if nothing could be fetched and nothing was cached.

    Self-describing, by design: placement.yaml declares install_root and the
    blueprints under it, so the asset list comes from the manifest itself
    (props.assets_declared) rather than a directory listing or a second list in the
    catalog. No listing is needed, which is what lets this move to
    raw.githubusercontent.com unchanged, and no second list means nothing to drift.

    Each .uasset is verified against provenance.json's md5 before it is kept, so a
    truncated or transformed download fails here rather than as a broken asset in
    the editor later. Falls back to whatever is already cached when offline."""
    import props as props_mod

    dest = props_cache_dir()
    want_subdir, want_ref = props_source(repo)
    subdir = subdir or want_subdir
    ref = ref or want_ref

    def cached_ok():
        return os.path.isfile(os.path.join(dest, props_mod.MANIFEST_NAME))

    manifest_bytes = repo_file(repo, f"{subdir}/{props_mod.MANIFEST_NAME}", ref)
    prov_bytes = repo_file(repo, f"{subdir}/provenance.json", ref)
    if not manifest_bytes or not prov_bytes:
        # Name the source. "could not reach" alone is a guess: an unreachable network
        # and a subdir that does not exist at that ref look identical from here, and
        # they need opposite fixes.
        where = f"{repo}:{subdir}@{ref}"
        if cached_ok():
            print(f"[props] no props at {where} (or the library is unreachable); "
                  f"using the cached props in {dest}.")
            return dest
        print(f"[props] no props at {where} (or the library is unreachable), "
              f"and nothing is cached.")
        return None

    with open(os.path.join(dest, props_mod.MANIFEST_NAME), "wb") as fh:
        fh.write(manifest_bytes)
    with open(os.path.join(dest, "provenance.json"), "wb") as fh:
        fh.write(prov_bytes)

    import json
    try:
        declared_md5 = (json.loads(prov_bytes).get("exported") or {}).get("md5")
    except ValueError:
        declared_md5 = None

    try:
        manifest = props_mod.load(os.path.join(dest, props_mod.MANIFEST_NAME))
        names = props_mod.assets_declared(manifest)
    except props_mod.ManifestError as exc:
        print(f"[props] fetched manifest is unreadable: {exc}")
        return dest if cached_ok() else None

    for name in names:
        target = os.path.join(dest, name + ".uasset")
        if os.path.isfile(target) and declared_md5 \
                and props_mod.md5_of(target) == declared_md5:
            continue                                  # already current
        blob = repo_file(repo, f"{subdir}/{name}.uasset", ref)
        if not blob:
            print(f"[props] could not fetch {name}.uasset from {repo}")
            if not os.path.isfile(target):
                return None
            continue
        with open(target, "wb") as fh:
            fh.write(blob)
        got = props_mod.md5_of(target)
        if declared_md5 and got != declared_md5:
            os.remove(target)
            print(f"[props] {name}.uasset failed verification "
                  f"({got} != {declared_md5}); discarded.")
            return None
        print(f"[props] fetched {name}.uasset ({len(blob)} bytes)")
    return dest


def map_sumo_dir(name):
    """The cached SUMO scenario dir for an already-imported map: the folder under
    ~/.fixs/maps/<name>/sumo that directly holds a .sumocfg (where a bundle's sumo/
    was extracted, or a separately-picked sumo landed), else None. Lets a Local
    (already-imported) pick reuse its sumo without re-prompting for one."""
    return _dir_with_sumocfg(os.path.join(_map_cache_dir(name), "sumo"))


# ------------------------------------------------ which bundle produced a map

# A cooked map is identified by a DIRECTORY NAME (resolve_cooked_map), so it has no
# memory of what produced it. That is fine while both halves of a bundle age
# together in one cache, and wrong as soon as they do not: the CARLA half is cooked
# once and kept, while the SUMO half tracks the library. Same map name on both
# sides, no error anywhere, vehicles placed against geometry that moved.
#
# So the map carries the sha256 of the release asset it came from. NOTHING IS
# HASHED HERE - GitHub already publishes a digest per release asset, so it is
# queried and copied along. A map with no sha (a local pick, a hand-prepped map,
# anything imported before this existed) simply is not checked.
SHA_FILE = ".source_sha"


def _read_sha(directory):
    try:
        with open(os.path.join(directory, SHA_FILE), encoding="utf-8") as f:
            return f.read().strip() or None
    except OSError:
        return None                      # no note is the normal case, not an error


def _write_sha(directory, sha):
    """Best effort: an unwritable note costs the check, not the run. Writing
    NOTHING when there is no sha is the part that matters - an empty note would
    read back as a value and make two unrelated maps look like a match."""
    if not sha:
        return
    try:
        with open(os.path.join(directory, SHA_FILE), "w", encoding="utf-8") as f:
            f.write(sha.strip() + "\n")
    except OSError:
        pass


def read_cached_sha(cache_name):
    """Which release asset the bundle cached for `cache_name` came from, or None."""
    return _read_sha(_map_cache_dir(cache_name))


def cooked_sha(carla_root, name, mode=None):
    """Which release asset the imported map `name` was built from, or None."""
    return _read_sha(cooked_content_dir(carla_root, name, mode))


def _release_asset_sha(repo, tag, pattern):
    """The sha256 GitHub publishes for the release asset matching `pattern`, or
    None if it cannot be determined (no gh, offline, or a release old enough that
    the API reports a null digest). Queryable without downloading the asset."""
    gh = shutil.which("gh")
    if not gh:
        return None
    try:
        out = subprocess.check_output(
            [gh, "release", "view", tag, "-R", repo, "--json", "assets",
             "--jq", '.assets[] | .name + " " + (.digest // "")'],
            text=True, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        return None
    for line in out.splitlines():
        name, _, digest = line.strip().partition(" ")
        if digest and fnmatch.fnmatch(name, pattern):
            return digest
    return None


def _download_release_asset(repo, tag, pattern, suffix, force_redownload=False,
                            cache_name=None, what="asset"):
    """Shared body of download_release_zip / download_cooked_tar: return a local
    path to the release asset matching `pattern`, downloading it via gh into the
    ~/.fixs/maps/<cache_name or tag>/ cache. `suffix` is how a cached copy is
    recognised, and is what keeps the two callers from seeing each other's file -
    the source bundle (.zip) and the precooked package (.tar.gz) share a cache
    dir, and a map may well have both downloaded."""
    gh = _require_gh()
    tag_dir = _map_cache_dir(cache_name or tag)
    cached = [f for f in os.listdir(tag_dir) if f.lower().endswith(suffix)] \
        if os.path.isdir(tag_dir) else []

    if cached and not force_redownload:
        path = os.path.join(tag_dir, cached[0])
        if sys.stdin.isatty():
            ans = input(f"[import] cached '{cached[0]}' found for '{tag}'. "
                        "[U]se it / [R]e-download / [C]ancel? [U]: ").strip().lower()
            if ans.startswith("c"):
                sys.exit("[import] cancelled by user.")
            force_redownload = ans.startswith("r")
        if not force_redownload:
            print(f"[import] using cached {path}")
            # Deliberately NOT backfilled with the release's current sha: a cached
            # copy may predate the release it came from, so recording today's digest
            # against yesterday's zip would assert something untrue. Unrecorded is
            # the honest state, and it only costs the check.
            return path

    os.makedirs(tag_dir, exist_ok=True)
    print(f"[import] downloading release '{tag}' from {repo}\n"
          f"[import]   into cache {tag_dir} (set FIXS_MAP_CACHE to relocate) ...")
    rc = subprocess.call([gh, "release", "download", tag, "-R", repo,
                          "-p", pattern, "-D", tag_dir, "--clobber"])
    if rc != 0:
        sys.exit(f"[import] `gh release download {tag} -p {pattern}` failed (rc={rc}).")
    got = [f for f in os.listdir(tag_dir) if f.lower().endswith(suffix)]
    if not got:
        sys.exit(f"[import] release '{tag}' has no {what} matching '{pattern}'.")
    # Note WHICH asset this is, while we still know. The cache dir is named for the
    # map, not the release, so nothing else here can answer that afterwards.
    _write_sha(tag_dir, _release_asset_sha(repo, tag, pattern))
    return os.path.join(tag_dir, got[0])


def download_release_zip(repo, tag, force_redownload=False, cache_name=None):
    """Return a local path to the release's .zip asset, downloading it via gh into
    the ~/.fixs/maps/<cache_name or tag>/ cache (cache_name = the cooked map name,
    so the zip sits beside the extracted carla/+sumo/). If a cached copy already
    exists, ask whether to reuse or re-download (default reuse); force_redownload
    skips the prompt. The zip stays in the cache so re-imports are free."""
    return _download_release_asset(repo, tag, "*.zip", ".zip", force_redownload,
                                   cache_name, what="source bundle (.zip)")


def download_cooked_tar(repo, tag, asset, force_redownload=False, cache_name=None):
    """Return a local path to the release's precooked .tar.gz, downloading it via
    gh into the same per-map cache the source bundle uses. `asset` is the exact
    published name (from the catalog, or derived - see catalog_cooked_asset), so a
    release that carries several is not a lottery."""
    return _download_release_asset(repo, tag, asset, ".tar.gz", force_redownload,
                                   cache_name, what="precooked package (.tar.gz)")


def release_assets(repo, tag):
    """Names of the assets published on release `tag`, or None if they cannot be
    listed. Used to say 'this map has no precooked build published' BEFORE
    downloading anything, and to name what is published instead - the alternative
    is a gh failure the user has to interpret."""
    gh = shutil.which("gh")
    if not gh:
        return None
    try:
        out = subprocess.check_output(
            [gh, "release", "view", tag, "-R", repo, "--json", "assets",
             "--jq", ".assets[].name"], text=True, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        return None
    return [line.strip() for line in out.splitlines() if line.strip()]


def pick_and_import(repo, tag_prefix="", carla_root=None, ue4_root=None, force=False):
    """List `repo`'s map releases (or accept a local .zip), ask which to import,
    then put it into the configured CARLA by the route that flavour supports:
    a SOURCE build cooks it, a PACKAGED build installs the precooked package.
    If the chosen version is already there, skip the work (unless the user opts to
    re-import, or force=True).

    need_source=False on purpose. Which flavour is installed is not a property of
    this operation - "import a map" is meaningful on both - so the flavour is
    resolved first and then dispatched on, instead of being used to refuse. Gating
    the whole command on a source build is what made --import-map dead-end on a
    packaged CARLA while run_cosim quietly did the same job (#276)."""
    carla_root, ue4_root = _resolve_carla(carla_root, ue4_root, need_source=False)
    mode = _mode()
    catalog = fetch_catalog(repo)
    name, tag, local = choose_map(repo, tag_prefix, carla_root, catalog=catalog)

    # The picker returns what the RELEASE is called ('mlk'); the cooked map inside
    # it is often named differently ('mlk_no_signal'). Everything below - the
    # already-imported check, the install, the message - is about the map, so
    # resolve it here, exactly as run_cosim does before its own preflight.
    entry = catalog_entry(catalog, name)
    if entry and entry.get("map_name") and entry["map_name"] != name:
        print(f"[import] map '{name}' -> '{entry['map_name']}'  (DT release {tag})")
        name = entry["map_name"]

    if map_is_imported(carla_root, name, mode) and not force:
        print(f"[import] '{name}' is already imported: "
              f"{cooked_map_path(carla_root, name, mode)}")
        if not sys.stdin.isatty():
            return 0
        if input("[import] re-import it (re-download + re-cook)? [y/N]: ").strip().lower() != "y":
            print("[import] keeping the existing map.")
            return 0
        force = True

    if mode == "packaged":
        # install_cooked reports the finished install; no second "done" here.
        install_precooked(carla_root, name, repo=repo, tag=tag, entry=entry,
                          local=local, force=force)
        return 0

    zip_path = local if local else download_release_zip(repo, tag, force_redownload=force)
    return ensure_map(name, carla_root=carla_root, ue4_root=ue4_root,
                      package_dir=zip_path, force=force,
                      source_sha=_read_sha(os.path.dirname(zip_path)))


def import_named(name, carla_root=None, ue4_root=None, package_url=None,
                 package_dir=None, force=False, prompt_if_exists=False,
                 package_pick=False, repo=None, tag_prefix=None):
    """Import the map called `name` by the route the configured CARLA supports.

    The named counterpart of pick_and_import: same dispatch, no menu. A SOURCE
    build cooks the package (ensure_map); a PACKAGED build installs the precooked
    one. `--package <name>` used to go straight to the cook and so refused on a
    packaged build, for the same reason the picker did (#276)."""
    carla_root, ue4_root = _resolve_carla(carla_root, ue4_root, need_source=False)
    if _mode() != "packaged":
        return ensure_map(name, carla_root, ue4_root, package_url, package_dir,
                          force, prompt_if_exists, package_pick,
                          source_sha=(_read_sha(os.path.dirname(package_dir))
                                      if package_dir else None))

    repo, tag_prefix = resolve_map_source(repo, tag_prefix)
    catalog = fetch_catalog(repo)
    entry = catalog_entry(catalog, name)
    if entry and entry.get("map_name") and entry["map_name"] != name:
        print(f"[import] map '{name}' -> '{entry['map_name']}'  "
              f"(DT release {entry.get('release')})")
        name = entry["map_name"]
    if package_url:
        # There is no cook here to feed, and a source zip is not installable as-is.
        print(f"[import] note: this CARLA is PACKAGED, so --package-url is not used; "
              f"the precooked package is resolved from the map catalog instead.")

    umap = cooked_map_path(carla_root, name, "packaged")
    if os.path.isfile(umap) and not force:
        print(f"[import] '{name}' is already installed: {umap}")
        if not (prompt_if_exists and sys.stdin.isatty()):
            return 0
        if input("[import] re-install it (re-download + re-extract)? [y/N]: ").strip().lower() != "y":
            print("[import] keeping the existing map.")
            return 0
        force = True

    # install_cooked says "done: <name> installed -> <umap>" itself; saying it again
    # here just made every packaged install report success twice.
    install_precooked(carla_root, name, repo=repo, tag=(entry or {}).get("release"),
                      entry=entry, local=package_dir, force=force)
    return 0


DEFAULT_MAP_REPO = "ORNL-Real-Sim/FIXS_Applications"
DEFAULT_MAP_TAG_PREFIX = "map-"


def _app_root():
    """The application repo root that holds the manifest and fixs_sources.txt: the
    directory containing FIXS/. Anchored on the FIXS root rather than counted up
    from this file, so it survives a move. Mirrors app_catalog.app_root()."""
    return fixs_paths.app_root(os.path.dirname(os.path.abspath(__file__)))


def resolve_map_source(repo=None, tag_prefix=None):
    """Resolve (repo, tag_prefix) for the map picker. Precedence: an explicit value
    (CLI) wins, else the root fixs_sources.txt (map_repo / map_tag_prefix), else the
    built-in default. Lets the hosting repo change in one config line, no wrappers
    to edit."""
    cfg = {}
    src = os.path.join(_app_root(), "fixs_sources.txt")
    if os.path.isfile(src):
        cfg = read_map_config(src)
    repo = repo or cfg.get("map_repo") or DEFAULT_MAP_REPO
    if tag_prefix is None:
        tag_prefix = cfg.get("map_tag_prefix", DEFAULT_MAP_TAG_PREFIX)
    return repo, tag_prefix


def main():
    # First act: get onto the interpreter carla.json names. This is started as
    # `python Carla/import_map.py` from whatever shell the user has open, so without
    # this the cook runs under whatever python is on PATH - a different env than the
    # one run_cosim uses, from the same machine and the same config.
    env.reexec_under_configured(__file__, tag="import")

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--package", default=None,
                    help="map/package name (matches Import/<name>.json)")
    ap.add_argument("--map-config", default=None,
                    help="text file declaring the map (package= and url= lines)")
    ap.add_argument("--package-url", default=None,
                    help="URL of the package zip (e.g. a GitHub release asset)")
    ap.add_argument("--package-dir", default=None,
                    help="local folder (or .zip) holding the package files")
    ap.add_argument("--package-pick", action="store_true",
                    help="always open a file picker to select a hand-downloaded package "
                         "(skips the gh auto-download)")
    ap.add_argument("--pick-release", action="store_true",
                    help="list the repo's map releases and prompt which version to import, "
                         "then download + cook it (needs gh). Repo/prefix come from "
                         "--repo/--tag-prefix or from repo=/tag_prefix= in --map-config")
    ap.add_argument("--repo", default=None,
                    help="owner/repo whose releases to list for --pick-release")
    ap.add_argument("--tag-prefix", default=None,
                    help="for --pick-release, only offer releases whose tag starts with "
                         "this (e.g. 'map-'); the package name is the tag minus this prefix")
    ap.add_argument("--carla-root", default=None, help="override the saved carla_root")
    ap.add_argument("--ue4-root", default=None, help="override the saved ue4_root")
    ap.add_argument("--force", action="store_true",
                    help="re-import even if already present (no prompt)")
    args = ap.parse_args()

    mc = read_map_config(args.map_config) if args.map_config else {}

    # Default action is the release/local picker; naming a specific --package (or a
    # package= in --map-config) opts into the direct, non-interactive import path.
    if args.pick_release or not (args.package or args.package_dir or args.package_url or mc.get("package")):
        repo, tag_prefix = resolve_map_source(
            args.repo or mc.get("repo"),
            args.tag_prefix if args.tag_prefix is not None else mc.get("tag_prefix"))
        return pick_and_import(repo, tag_prefix, args.carla_root, args.ue4_root, args.force)

    package = args.package or mc.get("package")
    url = args.package_url or mc.get("url")

    # run standalone -> offer to re-import if the map already exists
    return import_named(package, args.carla_root, args.ue4_root,
                        url, args.package_dir, args.force,
                        prompt_if_exists=True, package_pick=args.package_pick,
                        repo=args.repo or mc.get("repo"),
                        tag_prefix=(args.tag_prefix if args.tag_prefix is not None
                                    else mc.get("tag_prefix")))


if __name__ == "__main__":
    sys.exit(main())
