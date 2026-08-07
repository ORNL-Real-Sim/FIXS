"""
import_map.py - import a RoadRunner/OpenDRIVE map into a *source-build* CARLA.

Generalizes the proven one-click import (run CARLA's
`Util/BuildTools/Import.py --package=<name>` directly, with UE4_ROOT set) into a
shippable, parameterized helper. Used by run_cosim's --auto-import and by the
per-app one-click import_*.bat / .sh wrappers.

A map "package" is the `<name>.json` descriptor plus its fbx/xodr/fbm asset
folder. This helper makes sure that package is staged under <carla_root>/Import,
then runs the cook. The package is sourced, in order:
  --pick-release        list the repo's map releases and prompt which VERSION to
                        import, then download it via gh (needs gh; no URL/version
                        to hand-maintain - you pick from what's published), or
  already staged        files already present under <carla_root>/Import, or
  --package-dir <dir>   a local folder holding the package files, or
  --package-url <url>   tried via the GitHub CLI if installed (handles private
                        repos), else you are prompted to point at a copy you
                        downloaded by hand from the release - the portable path,
                        needing only browser access, no gh / auth.

Importing requires a CARLA **source build** (the cook runs the Unreal editor);
packaged CARLA cannot import custom maps. carla_root / ue4_root default to the
saved env config (~/.fixs/carla.json) written by carla_env_setup.py.

Examples:
  # pick which published version to import (lists releases tagged map-*):
  python import_map.py --pick-release --repo ORNL-Real-Sim/FIXS_Applications --tag-prefix map-
  python import_map.py --package RP_Ver0529 --package-dir C:/src_ext/Carla/Import
  python import_map.py --package RP_Ver0529 --package-url https://.../RP_Ver0529_import.zip
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

import carla_env_setup as env


def cooked_content_dir(carla_root, name):
    """The Content/<name> folder produced by a successful import."""
    return os.path.join(carla_root, "Unreal", "CarlaUE4", "Content", name)


def cooked_map_path(carla_root, name):
    """Where the cooked .umap lands after a successful import."""
    return os.path.join(cooked_content_dir(carla_root, name), "Maps", name, name + ".umap")


def map_is_imported(carla_root, name):
    return os.path.isfile(cooked_map_path(carla_root, name))


def package_descriptor(carla_root, name):
    """The <name>.Package.json the cook writes beside the imported content. It is
    the authoritative record of what the package actually holds - the map's real
    name and its /Game/... path - neither of which is *guaranteed* to equal the
    package name (see _package_from_tag: that is a release convention, not a rule)."""
    return os.path.join(cooked_content_dir(carla_root, name),
                        "Config", name + ".Package.json")


def declared_maps(carla_root, name):
    """[(map_name, /Game/... level path)] declared by the package descriptor.
    Entries whose .umap is missing are dropped: a descriptor outlives the content
    it describes (a half-wiped re-import leaves one behind), and a name we hand to
    load_world must point at a level that is really on disk."""
    import json
    try:
        with open(package_descriptor(carla_root, name), encoding="utf-8") as f:
            declared = json.load(f).get("maps") or []
    except (OSError, ValueError):
        return []
    found = []
    for m in declared:
        mname, mpath = m.get("name"), m.get("path")
        if not mname or not str(mpath).startswith("/Game/"):
            continue
        umap = os.path.join(carla_root, "Unreal", "CarlaUE4", "Content",
                            *mpath[len("/Game/"):].split("/"), mname + ".umap")
        if os.path.isfile(umap):
            found.append((mname, f"{mpath}/{mname}"))
    return found


def find_level_paths(carla_root, name):
    """Every cooked level called <name>.umap, as /Game/... object paths.

    More than one means load_world(<name>) is a coin flip: CARLA resolves a bare
    name by recursive file search and silently takes the first hit (see
    UCarlaEpisode::LoadNewEpisode -> PathList[0]), so which copy you get is
    directory-traversal order, not a choice. A full /Game/... path is exact."""
    content = os.path.join(carla_root, "Unreal", "CarlaUE4", "Content")
    umap = name + ".umap"
    found = []
    for root, _dirs, files in os.walk(content):
        if umap in files:
            rel = os.path.relpath(root, content).replace(os.sep, "/")
            found.append((f"/Game/{rel}/{name}", os.path.getsize(os.path.join(root, umap))))
    return sorted(found)


def choose_level_path(carla_root, name):
    """Full /Game/... path to load for `name`. Only reached when the path could NOT
    be worked out from the package itself, so there is nothing to default to: if
    several cooked levels share the name, only the user can say which was meant."""
    found = find_level_paths(carla_root, name)
    if len(found) <= 1:
        return found[0][0] if found else None

    print(f"\n[cosim] {len(found)} cooked maps are named '{name}' and nothing "
          f"identifies which is wanted - pick one:")
    for i, (path, size) in enumerate(found, 1):
        print(f"   {i}) {path}  ({size / 1048576:.1f} MB)")
    if not sys.stdin.isatty():
        sys.exit("[cosim] non-interactive and the name is ambiguous; pass --map "
                 "<package> so the path resolves, or delete the stale copies.")
    while True:
        ans = _prompt(f"[cosim] Which one? [1-{len(found)}]: ").strip()
        if ans.isdigit() and 1 <= int(ans) <= len(found):
            return found[int(ans) - 1][0]
        print("[cosim] invalid choice; enter a number from the list.")


def duplicate_level_note(carla_root, name, using):
    """Heads-up text when other cooked levels share `name`, else "". Informational
    only - we load by exact /Game/... path, so the copies cannot be picked up by
    mistake; they are just leftovers from repeat imports, worth deleting."""
    others = [p for p, _ in find_level_paths(carla_root, name) if p != using]
    if not others:
        return ""
    return (f"[cosim] note: {len(others)} other cooked map(s) also named '{name}'; "
            f"ignored (loading by full path):\n"
            + "\n".join(f"           {p}" for p in others))


def resolve_cooked_map(carla_root, name):
    """(map_name, /Game/... level path) for an imported package, or None when it
    cannot be resolved *unambiguously*.

    Resolve, never assume. The picked release gives a PACKAGE name; what CARLA
    loads is a MAP name, and the two agree only by naming convention. So: accept
    the conventional layout when it is genuinely on disk, else believe the
    package's own descriptor, else give up - so the caller can ask the user
    rather than invent a name that load_world will fail to open."""
    if map_is_imported(carla_root, name):
        return name, f"/Game/{name}/Maps/{name}/{name}"
    found = declared_maps(carla_root, name)
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


def _select_package(name, package_url):
    """Let the user point at a package they downloaded by hand - a native file
    picker, falling back to a typed path. This is the portable path: no GitHub
    CLI / auth needed, just browser access to the release."""
    print(f"\n[import] Select the downloaded '{name}' map package.")
    if package_url:
        print("[import] If you don't have it yet, download it (browser is fine - "
              "you need access to the release):")
        print(f"             {package_url}")
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        path = filedialog.askopenfilename(
            title=f"Select the downloaded {name} package (.zip)",
            filetypes=[("Zip archives", "*.zip"), ("All files", "*.*")])
        root.destroy()
        if path:
            return path
    except Exception as exc:  # no display / no tkinter
        print(f"[import] file picker unavailable ({exc}); type the path instead.")
    if not sys.stdin.isatty():
        sys.exit("[import] non-interactive session: pass --package-dir "
                 "<zip-or-folder> with the downloaded package.")
    path = input("[import] Path to the downloaded .zip (or extracted folder): ").strip().strip('"')
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


def _tile_xy(fbx_name, map_name):
    """(x, y) parsed from a strict `<map_name>_Tile_<x>_<y>.fbx`, else None.

    Strict on purpose: CARLA reads the streaming-grid index off the last two
    underscore tokens of the tile name (LoadAssetMaterialsCommandlet.cpp), so a
    loose `<map>_Tile_0_0_final.fbx` would silently cook as tile (0,0) and
    collide. Anything not exactly `<map>_Tile_<int>_<int>.fbx` is not a tile we
    own - the caller warns about it rather than guessing."""
    stem = fbx_name[:-4] if fbx_name.lower().endswith(".fbx") else fbx_name
    m = re.fullmatch(re.escape(map_name) + r"_Tile_(\d+)_(\d+)", stem)
    return (int(m.group(1)), int(m.group(2))) if m else None


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

    Resolve, never guess: exit loudly when the export cannot be described - no
    <map_name>.xodr staged, the same map staged in two places, or an .xodr with
    neither a <map_name>.fbx nor <map_name>_Tile_<x>_<y>.fbx beside it."""
    import json

    subtree = os.path.join(import_dir, map_name)
    scan_root = subtree if os.path.isdir(subtree) else import_dir
    # Exactly one <map_name>.xodr is expected in the staged subtree; walk it to
    # also cover an export that carried its own inner folder.
    xodr_dirs = [root for root, _dirs, files in os.walk(scan_root)
                 if map_name + ".xodr" in files]
    if not xodr_dirs:
        sys.exit(f"[import] cannot describe '{map_name}': no {map_name}.xodr under "
                 f"{scan_root}. A raw RoadRunner export must be named after the map "
                 f"({map_name}.xodr + {map_name}.fbx or {map_name}_Tile_<x>_<y>.fbx).")
    if len(xodr_dirs) > 1:
        where = "\n".join(f"             {os.path.relpath(d, import_dir)}"
                          for d in sorted(xodr_dirs))
        sys.exit(f"[import] cannot describe '{map_name}': {map_name}.xodr is staged "
                 f"in {len(xodr_dirs)} places (staged more than once?). Keep one so a "
                 f"single descriptor maps to a single destination:\n{where}")

    asset_dir = xodr_dirs[0]
    rel = lambda p: os.path.relpath(p, import_dir).replace(os.sep, "/")
    single = os.path.join(asset_dir, map_name + ".fbx")
    tiles = sorted(os.path.join(asset_dir, f) for f in os.listdir(asset_dir)
                   if _tile_xy(f, map_name) is not None)
    if os.path.isfile(single) and tiles:
        sys.exit(f"[import] cannot describe '{map_name}': both {map_name}.fbx and "
                 f"{map_name}_Tile_*.fbx present - a map is single-source or tiled, "
                 f"not both.")

    entry = {"name": map_name,
             "xodr": rel(os.path.join(asset_dir, map_name + ".xodr")),
             "use_carla_materials": False}  # RoadRunner ships its own materials
    if os.path.isfile(single):
        entry["source"] = rel(single)
        kind = "single-source"
    elif tiles:
        entry["tile_size"] = _tile_size(asset_dir)
        entry["tiles"] = [rel(t) for t in tiles]
        kind = f"tiled, {len(tiles)} tiles, tile_size={entry['tile_size']}"
    else:
        sys.exit(f"[import] cannot describe '{map_name}': found {map_name}.xodr but "
                 f"no {map_name}.fbx or {map_name}_Tile_<x>_<y>.fbx beside it.")

    # Surface, don't silently drop, RoadRunner layer-split exports: extra .fbx
    # that are neither the source nor a strict tile. CARLA's own generator
    # ignores them; a dropped layer should at least be visible.
    for f in sorted(os.listdir(asset_dir)):
        if f.lower().endswith(".fbx") and f != map_name + ".fbx" \
                and _tile_xy(f, map_name) is None:
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
            os.makedirs(raw_dest, exist_ok=True)
            _stage_from_path(src, raw_dest)
            generate_descriptor(import_dir, name)
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


def open_bundle(src, cache_name=None):
    """Split a map source into its CARLA package and its SUMO scenario.

    A Digital-Twin-Library map ships as one bundle - a zip (or folder) with a
    top-level `carla/` (the CARLA import package) and `sumo/` (the scenario).
    Returns `(carla_src, sumo_dir)`:
      - bundle  -> (`<cache>/carla`, `<cache>/sumo`); a zip is extracted once
                   (re-extracted only when the zip is newer). `cache_name` (the
                   cooked map name) extracts into ~/.fixs/maps/<cache_name>/, else
                   a sibling `<stem>_unpacked/`. A folder is used in place.
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
            carla = os.path.join(unpacked, "carla")
            # Compare the zip against the extracted carla/ (not `unpacked`, which may
            # also hold the downloaded bundle.zip in a per-map cache), and clear only
            # the bundle's own subdirs on re-extract so a sibling bundle.zip is
            # preserved. props/ is in that list because a prop left behind by an older
            # bundle would otherwise be installed forever - the same "stale content
            # nobody notices" failure this whole ticket is about (FIXS#223).
            if not os.path.isdir(carla) or os.path.getmtime(src) > os.path.getmtime(carla):
                for sub in ("carla", "sumo", "props"):
                    shutil.rmtree(os.path.join(unpacked, sub), ignore_errors=True)
                os.makedirs(unpacked, exist_ok=True)
                z.extractall(unpacked)
                print(f"[import] unpacked bundle -> {unpacked}")
        carla = os.path.join(unpacked, "carla")
        sumo = os.path.join(unpacked, "sumo")
        return (carla if os.path.isdir(carla) else unpacked), \
               (sumo if os.path.isdir(sumo) else None)
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


def _resolve_carla(carla_root=None, ue4_root=None):
    """Resolve (carla_root, ue4_root) from the saved env config, running first-time
    setup if nothing is configured yet. Exits with a clear message if no CARLA is
    configured, or if it is a packaged build (custom-map import needs a source
    build). Explicit args win over the saved config."""
    cfg = env.load_config()
    if cfg is None and not carla_root:
        # first use on a fresh clone: configure the CARLA env, just like run_cosim
        print("[import] no CARLA env configured; running first-time setup ...")
        cfg = env.run_setup()
    cfg = cfg or {}
    carla_root = carla_root or cfg.get("carla_root")
    ue4_root = ue4_root or cfg.get("ue4_root")
    if not carla_root:
        sys.exit("[import] no CARLA configured - run setup_carla first.")
    if cfg.get("mode") == "packaged":
        sys.exit("[import] the configured CARLA is PACKAGED; importing a custom map "
                 "needs a SOURCE build (run setup_carla and pick source).")
    return carla_root, ue4_root


def ensure_map(name, carla_root=None, ue4_root=None, package_url=None,
               package_dir=None, force=False, prompt_if_exists=False, package_pick=False):
    """Make sure `name` is imported into the (source-build) CARLA, importing it
    if needed. Returns 0 on success; exits with a clear message otherwise.

    If the map already exists: `force` re-imports unconditionally; otherwise, when
    `prompt_if_exists` and the session is interactive, the user is asked whether to
    re-import (re-download + re-cook) - handy for updating a map or testing."""
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
    print(f"[import] done: '{name}' imported -> {umap}")
    return 0


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


def _require_gh():
    """Path to the GitHub CLI, or exit telling the user how to proceed without it."""
    gh = shutil.which("gh")
    if not gh:
        sys.exit("[import] --pick-release needs the GitHub CLI (gh) to list the map "
                 "releases.\n        Install gh (https://cli.github.com) and run "
                 "`gh auth login`, or import a specific version directly with "
                 "--package + --package-url / --package-dir.")
    return gh


def _package_from_tag(tag, tag_prefix=""):
    """Package name = release tag with its prefix stripped. Convention:
    tag `map-<pkg>` <-> asset `<pkg>_carla_import.zip` <-> descriptor `<pkg>.json`."""
    return tag[len(tag_prefix):] if tag_prefix and tag.startswith(tag_prefix) else tag


def list_map_releases(repo, tag_prefix=""):
    """Map releases in `repo` (tag starts with `tag_prefix`), newest first, via gh.
    Each item: {tag, title, date 'YYYY-MM-DD', prerelease}. Drafts are skipped -
    their assets aren't downloadable."""
    import json
    gh = _require_gh()
    try:
        out = subprocess.check_output(
            [gh, "release", "list", "-R", repo, "-L", "100", "--json",
             "tagName,name,publishedAt,isPrerelease,isDraft"],
            text=True, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as exc:
        sys.exit(f"[import] `gh release list` failed for {repo}:\n{exc.stderr.strip()}\n"
                 "        Check `gh auth status` and that your account can see the repo.")
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
    """Best-effort map/package name from a downloaded zip or folder: the stem of the
    <name>.json descriptor it contains (handles Windows-built zips with backslash
    entries). Returns None if no descriptor is found."""
    entries = []
    if os.path.isfile(path) and path.lower().endswith(".zip"):
        with zipfile.ZipFile(path) as z:
            entries = z.namelist()
    elif os.path.isdir(path):
        entries = os.listdir(path)
    for e in entries:
        base = os.path.basename(e.replace("\\", "/").rstrip("/"))
        if base.lower().endswith(".json"):
            return base[:-5]
    return None


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
    """Interactive chooser. Two sections plus `L` for a hand-picked .zip / folder:
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
    print("   L) select a local .zip / folder instead")
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
            path = _select_package("map", None)  # native file picker / typed path
            name = _infer_package_name(path)
            if not name:
                name = _prompt("[import] map name (matches <name>.json inside the package): ").strip()
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


def list_imported_maps(carla_root):
    """Names of maps already cooked under this CARLA (i.e. that have a <name>.umap).
    Used to let tools that operate on an existing map (e.g. place_tls) offer a
    local choice without needing gh / the network."""
    content = os.path.join(carla_root, "Unreal", "CarlaUE4", "Content")
    if not os.path.isdir(content):
        return []
    return [name for name in sorted(os.listdir(content))
            if os.path.isfile(cooked_map_path(carla_root, name))]


def choose_imported_map(carla_root):
    """Numbered menu of the maps already cooked into this CARLA; returns the chosen
    name. Auto-selects when there is exactly one; exits if there are none, or if a
    choice is needed but the session is non-interactive."""
    maps = list_imported_maps(carla_root)
    if not maps:
        sys.exit(f"[import] no cooked maps found under {carla_root}. Import one first "
                 "(e.g. import_<app>_map).")
    if len(maps) == 1:
        return maps[0]
    print("\n[import] Imported maps:")
    for i, m in enumerate(maps, 1):
        print(f"   {i}) {m}")
    if not sys.stdin.isatty():
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


def download_release_zip(repo, tag, force_redownload=False, cache_name=None):
    """Return a local path to the release's .zip asset, downloading it via gh into
    the ~/.fixs/maps/<cache_name or tag>/ cache (cache_name = the cooked map name,
    so the zip sits beside the extracted carla/+sumo/). If a cached copy already
    exists, ask whether to reuse or re-download (default reuse); force_redownload
    skips the prompt. The zip stays in the cache so re-imports are free."""
    gh = _require_gh()
    tag_dir = _map_cache_dir(cache_name or tag)
    cached = [f for f in os.listdir(tag_dir) if f.lower().endswith(".zip")] \
        if os.path.isdir(tag_dir) else []

    if cached and not force_redownload:
        zip_path = os.path.join(tag_dir, cached[0])
        if sys.stdin.isatty():
            ans = input(f"[import] cached '{cached[0]}' found for '{tag}'. "
                        "[U]se it / [R]e-download / [C]ancel? [U]: ").strip().lower()
            if ans.startswith("c"):
                sys.exit("[import] cancelled by user.")
            force_redownload = ans.startswith("r")
        if not force_redownload:
            print(f"[import] using cached {zip_path}")
            return zip_path

    os.makedirs(tag_dir, exist_ok=True)
    print(f"[import] downloading release '{tag}' from {repo}\n"
          f"[import]   into cache {tag_dir} (~720MB; set FIXS_MAP_CACHE to relocate) ...")
    rc = subprocess.call([gh, "release", "download", tag, "-R", repo,
                          "-p", "*.zip", "-D", tag_dir, "--clobber"])
    if rc != 0:
        sys.exit(f"[import] `gh release download {tag}` failed (rc={rc}).")
    zips = [f for f in os.listdir(tag_dir) if f.lower().endswith(".zip")]
    if not zips:
        sys.exit(f"[import] release '{tag}' has no .zip asset to import.")
    return os.path.join(tag_dir, zips[0])


def pick_and_import(repo, tag_prefix="", carla_root=None, ue4_root=None, force=False):
    """List `repo`'s map releases (or accept a local .zip), ask which to import,
    then download + cook it. If the chosen version is already cooked, skip the
    download (unless the user opts to re-import, or force=True)."""
    carla_root, ue4_root = _resolve_carla(carla_root, ue4_root)
    name, tag, local = choose_map(repo, tag_prefix, carla_root)

    if map_is_imported(carla_root, name) and not force:
        print(f"[import] '{name}' is already imported: {cooked_map_path(carla_root, name)}")
        if not sys.stdin.isatty():
            return 0
        if input("[import] re-import it (re-download + re-cook)? [y/N]: ").strip().lower() != "y":
            print("[import] keeping the existing map.")
            return 0
        force = True

    zip_path = local if local else download_release_zip(repo, tag, force_redownload=force)
    return ensure_map(name, carla_root=carla_root, ue4_root=ue4_root,
                      package_dir=zip_path, force=force)


DEFAULT_MAP_REPO = "ORNL-Real-Sim/FIXS_Applications"
DEFAULT_MAP_TAG_PREFIX = "map-"


def _app_root():
    """The application repo root that holds fixs_sources.txt: two levels up from
    this file (<repo>/FIXS/Carla/import_map.py -> <repo>)."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


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
    return ensure_map(package, args.carla_root, args.ue4_root,
                      url, args.package_dir, args.force,
                      prompt_if_exists=True, package_pick=args.package_pick)


if __name__ == "__main__":
    sys.exit(main())
