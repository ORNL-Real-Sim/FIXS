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
        _stage_from_path(src, import_dir)
    finally:
        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)

    if not os.path.isfile(descriptor):
        sys.exit(f"[import] after staging, descriptor still missing: {descriptor}\n"
                 f"         (the package must contain {name}.json + its asset folder)")
    return import_dir


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


def choose_release(releases, tag_prefix=""):
    """Print a numbered menu of releases and return the chosen tag. Empty input
    picks the newest (item 1). Exits if there is nothing to choose or the session
    is non-interactive (no way to prompt)."""
    if not releases:
        sys.exit("[import] no matching map releases found to choose from.")
    print("\n[import] Available map versions (newest first):")
    for i, r in enumerate(releases, 1):
        pkg = _package_from_tag(r["tag"], tag_prefix)
        flag = "  (pre-release)" if r["prerelease"] else ""
        print(f"   {i}) {pkg:<26} {r['date']}{flag}")
    if not sys.stdin.isatty():
        sys.exit("[import] non-interactive session: cannot prompt for a version. Pass "
                 "--package (+ --package-url / --package-dir) to import a specific one.")
    while True:
        ans = input(f"[import] Which version? [1-{len(releases)}], Enter = 1 (newest): ").strip()
        if ans == "":
            return releases[0]["tag"]
        if ans.isdigit() and 1 <= int(ans) <= len(releases):
            return releases[int(ans) - 1]["tag"]
        print("[import] invalid choice; enter a number from the list.")


def gh_download_release_zip(repo, tag, dest_dir):
    """Download the release's .zip asset into `dest_dir` via gh; return its path."""
    gh = _require_gh()
    os.makedirs(dest_dir, exist_ok=True)
    print(f"[import] downloading release '{tag}' from {repo} ...")
    rc = subprocess.call([gh, "release", "download", tag, "-R", repo,
                          "-p", "*.zip", "-D", dest_dir, "--clobber"])
    if rc != 0:
        sys.exit(f"[import] `gh release download {tag}` failed (rc={rc}).")
    zips = [f for f in os.listdir(dest_dir) if f.lower().endswith(".zip")]
    if not zips:
        sys.exit(f"[import] release '{tag}' has no .zip asset to import.")
    return os.path.join(dest_dir, zips[0])


def pick_and_import(repo, tag_prefix="", carla_root=None, ue4_root=None, force=False):
    """List `repo`'s map releases, ask which to import, then download + cook it.
    If the chosen version is already cooked, skip the download (unless the user
    opts to re-import, or force=True)."""
    carla_root, ue4_root = _resolve_carla(carla_root, ue4_root)
    tag = choose_release(list_map_releases(repo, tag_prefix), tag_prefix)
    name = _package_from_tag(tag, tag_prefix)

    if map_is_imported(carla_root, name) and not force:
        print(f"[import] '{name}' is already imported: {cooked_map_path(carla_root, name)}")
        if not sys.stdin.isatty():
            return 0
        if input("[import] re-import it (re-download + re-cook)? [y/N]: ").strip().lower() != "y":
            print("[import] keeping the existing map.")
            return 0
        force = True

    tmp = tempfile.mkdtemp(prefix="fixs-map-")
    try:
        zip_path = gh_download_release_zip(repo, tag, tmp)
        return ensure_map(name, carla_root=carla_root, ue4_root=ue4_root,
                          package_dir=zip_path, force=force)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


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

    if args.pick_release:
        repo = args.repo or mc.get("repo")
        if not repo:
            ap.error("--pick-release needs --repo or a 'repo=' line in --map-config")
        tag_prefix = args.tag_prefix if args.tag_prefix is not None else mc.get("tag_prefix", "")
        return pick_and_import(repo, tag_prefix, args.carla_root, args.ue4_root, args.force)

    package = args.package or mc.get("package")
    url = args.package_url or mc.get("url")
    if not package:
        ap.error("a map is required: pass --package, or --map-config with a package= line "
                 "(or use --pick-release to choose from the repo's releases)")

    # run standalone -> offer to re-import if the map already exists
    return ensure_map(package, args.carla_root, args.ue4_root,
                      url, args.package_dir, args.force,
                      prompt_if_exists=True, package_pick=args.package_pick)


if __name__ == "__main__":
    sys.exit(main())
