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


def open_bundle(src):
    """Split a map source into its CARLA package and its SUMO scenario.

    A Digital-Twin-Library map ships as one bundle - a zip (or folder) with a
    top-level `carla/` (the CARLA import package) and `sumo/` (the scenario).
    Returns `(carla_src, sumo_dir)`:
      - bundle  -> (`<unpacked>/carla`, `<unpacked>/sumo`); a zip is extracted
                   once into a sibling `<stem>_unpacked/` cache (re-extracted only
                   when the zip is newer), a folder is used in place.
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
            unpacked = os.path.join(
                os.path.dirname(os.path.abspath(src)),
                os.path.splitext(os.path.basename(src))[0] + "_unpacked")
            if not os.path.isdir(unpacked) or os.path.getmtime(src) > os.path.getmtime(unpacked):
                shutil.rmtree(unpacked, ignore_errors=True)
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
    return subprocess.call(cmd, cwd=carla_root, env=proc_env)


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


def choose_map(repo, tag_prefix=""):
    """Interactive chooser covering both cases: pick one of `repo`'s published
    map-* releases (auto-downloaded + cached), or select a local .zip / folder by
    hand. Returns (name, tag, local_path): for a release (name, tag, None); for a
    local pick (name, None, path). Exits if the session is non-interactive."""
    releases = list_map_releases(repo, tag_prefix)
    print("\n[import] Available map versions (newest first):")
    for i, r in enumerate(releases, 1):
        pkg = _package_from_tag(r["tag"], tag_prefix)
        flag = "  (pre-release)" if r["prerelease"] else ""
        print(f"   {i}) {pkg:<26} {r['date']}{flag}")
    if not releases:
        print("   (no published releases found)")
    print("   L) select a local .zip / folder instead")
    if not sys.stdin.isatty():
        sys.exit("[import] non-interactive session: cannot prompt. Pass --package "
                 "(+ --package-url/--package-dir), or --map, to choose non-interactively.")
    while True:
        hint = f"[1-{len(releases)} / L]" if releases else "[L]"
        default = ", Enter = 1 (newest)" if releases else ""
        ans = _prompt(f"[import] Which version? {hint}{default}: ").strip().lower()
        if ans == "" and releases:
            r = releases[0]
            return _package_from_tag(r["tag"], tag_prefix), r["tag"], None
        if ans == "l":
            path = _select_package("map", None)  # native file picker / typed path
            name = _infer_package_name(path)
            if not name:
                name = _prompt("[import] map name (matches <name>.json inside the package): ").strip()
            if not name:
                sys.exit("[import] no package name given; cannot import.")
            return name, None, path
        if ans.isdigit() and 1 <= int(ans) <= len(releases):
            r = releases[int(ans) - 1]
            return _package_from_tag(r["tag"], tag_prefix), r["tag"], None
        print("[import] invalid choice; enter a number, or L for a local file.")


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


def _map_cache_dir():
    """Local cache for downloaded map zips: ~/.fixs/maps (next to carla.json), or
    $FIXS_MAP_CACHE if set. Kept outside FIXS/ so `initialize` (which wipes FIXS/)
    never deletes it, and shared across app clones so a map is downloaded once."""
    d = os.environ.get("FIXS_MAP_CACHE") or os.path.join(os.path.dirname(env.CONFIG_PATH), "maps")
    os.makedirs(d, exist_ok=True)
    return d


def download_release_zip(repo, tag, force_redownload=False):
    """Return a local path to the release's .zip asset, downloading it via gh into
    the ~/.fixs/maps/<tag>/ cache. If a cached copy already exists, ask whether to
    reuse or re-download (default reuse); force_redownload skips the prompt and
    re-fetches. The zip stays in the cache (not deleted) so re-imports are free."""
    gh = _require_gh()
    tag_dir = os.path.join(_map_cache_dir(), tag)
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
    name, tag, local = choose_map(repo, tag_prefix)

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
