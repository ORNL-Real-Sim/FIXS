"""props.py - placement props and their manifest, shipped by the map library.

FIXS used to hardcode `/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLight`
and a `+300 cm` lift. That content path resolves to DIFFERENT assets on different
installs - a 3-lens head on a vertical pole on stock Windows CARLA 0.9.15, a bare
head on a box where someone stripped the pole by hand - so the same placer produced
poles floating 3 m in the air on one machine and correctly hung heads on another.
Nobody noticed, because the result is baked into the saved .umap.

The fix is to take the prop AND its placement numbers from the map library as
content, instead of resolving a stock path against whatever each machine happens to
have. This module is the reading half of that: it finds the manifest, installs the
props it names, and hands the numbers to the placer.

Two things it deliberately does NOT do:

  * supply defaults for anything the manifest declares. If a bundle ships a
    placement.yaml, every value the placer needs comes from it and a missing one is
    a hard error - substituting a stock asset is the bug this exists to remove. A
    bundle that ships NO manifest keeps the old hardcoded behaviour (see
    legacy_settings), because atlanta and mlk have no manifest yet and a flag day
    would strand them.
  * parse the manifest inside Unreal. UE's embedded Python is a different
    interpreter and has no pyyaml, so the manifest is read here (under the realsim
    env) and passed to the editor script as environment variables - the same way
    SUMO_TLS_TABLE_PATH already reaches it.

Ref: ORNL-Real-Sim/FIXS#223, ORNL-Real-Sim/Digital-Twin-Library#2
"""

import hashlib
import json
import os

MANIFEST_NAME = "placement.yaml"

# What the placer used before the manifest existed. Only for bundles that ship no
# placement.yaml; kept here rather than in the placer so there is exactly one copy
# of "the old numbers" and it is obvious what a manifest replaces.
LEGACY_TL_BLUEPRINT = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLight"
LEGACY_TL_GROUP_BLUEPRINT = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLightGroup"
LEGACY_TL_Z_OFFSET_CM = 300.0


class ManifestError(Exception):
    """The manifest is missing a value the placer needs, or contradicts the install."""


# ---------------------------------------------------------------- finding things

def find_manifest(*dirs):
    """First `placement.yaml` found in `dirs` or in a `props/` under them.

    Accepts both layouts so the tooler does not care which the library uses: the
    manifest next to `carla/`+`sumo/` in a map bundle, or inside a `props/` folder
    beside them. Returns None when no bundle in play ships one.
    """
    for d in dirs:
        if not d or not os.path.isdir(d):
            continue
        for candidate in (os.path.join(d, MANIFEST_NAME),
                          os.path.join(d, "props", MANIFEST_NAME)):
            if os.path.isfile(candidate):
                return candidate
    return None


def find_props_dir(manifest_path, explicit=None):
    """Where the .uasset files live: `explicit` if given, else the manifest's own
    directory, else a `props/` beside it. None when there are no assets to install
    (a bundle whose manifest only references props already installed)."""
    if explicit:
        return explicit if os.path.isdir(explicit) else None
    if not manifest_path:
        return None
    here = os.path.dirname(os.path.abspath(manifest_path))
    if _uassets_in(here):
        return here
    sibling = os.path.join(here, "props")
    return sibling if _uassets_in(sibling) else None


def _uassets_in(d):
    return bool(d and os.path.isdir(d)
                and [f for f in os.listdir(d) if f.lower().endswith(".uasset")])


# ---------------------------------------------------------------- reading it

def load(manifest_path):
    """Parse the manifest. Raises ManifestError if it cannot be read at all."""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment.yml declares pyyaml
        raise ManifestError(
            f"cannot read {manifest_path}: pyyaml is not installed ({exc}). "
            f"It is declared in FIXS/environment.yml - run under the realsim env.")
    try:
        with open(manifest_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except Exception as exc:
        raise ManifestError(f"cannot parse {manifest_path}: {exc}")
    if not isinstance(data, dict):
        raise ManifestError(f"{manifest_path} is not a mapping")
    return data


def _require(manifest, section, key, manifest_path):
    """A declared value, or a hard error naming what to add and where.

    Deliberately not `.get(key, default)`: the whole point of the manifest is that
    FIXS supplies no numbers of its own, so a missing key has to stop the run rather
    than quietly become the stock asset again."""
    block = manifest.get(section)
    if not isinstance(block, dict) or key not in block or block[key] is None:
        raise ManifestError(
            f"{manifest_path} declares no {section}.{key}. Every value the placer "
            f"needs comes from the manifest - add it there rather than letting FIXS "
            f"pick, which is how the same content path came to mean different assets "
            f"on different machines.")
    return block[key]


def tl_settings(manifest, manifest_path):
    """The traffic-light numbers the placer needs, all of them required.

    Deliberately no `table` key, though the draft manifest in
    Digital-Twin-Library#2 had one. Where each head goes is derived from the map's
    SUMO net by resolve_tl_table, which reads no committed table at all -- so there
    is no path for a manifest to name. Letting it name one would put the same fact
    in two places, free to drift apart, which is the failure this manifest exists
    to prevent.
    """
    return {
        "blueprint": str(_require(manifest, "traffic_lights", "blueprint", manifest_path)),
        "group_blueprint": str(_require(manifest, "traffic_lights", "group_blueprint",
                                        manifest_path)),
        "z_offset_cm": float(_require(manifest, "traffic_lights", "z_offset_cm",
                                      manifest_path)),
        "flip_yaw_180": bool(_require(manifest, "traffic_lights", "flip_yaw_180",
                                      manifest_path)),
    }


def assets_declared(manifest):
    """The asset basenames this manifest expects to be installed.

    Derived from the blueprint paths that live under `install_root`: one under it
    ships with the pack, one under /Game/Carla is stock CARLA content and must not
    be fetched or installed. So the manifest IS the index - there is no separate
    file list in the catalog, and so nothing that can drift out of sync with it.

    That also means this needs no directory listing, which matters for where this is
    going: once the map library is public these files come from
    raw.githubusercontent.com, which serves files but cannot enumerate a folder.
    """
    root = (manifest.get("install_root") or "").rstrip("/")
    if not root:
        return []
    names = set()
    for block in manifest.values():
        if not isinstance(block, dict):
            continue
        for key, value in block.items():
            if (key.endswith("blueprint") and isinstance(value, str)
                    and value.startswith(root + "/")):
                names.add(value.rsplit("/", 1)[-1])
    return sorted(names)


def legacy_settings():
    """The pre-manifest numbers, for bundles that ship no placement.yaml.

    flip_yaw_180 still honours a hand-set FLIP_SIGNAL_HEADS_180, because that
    environment variable WAS the only way to correct a back-to-front map before the
    manifest existed. Without this the values below would be pushed into the child
    environment and silently overwrite it - taking away the escape hatch from
    exactly the maps that have no manifest to replace it with.

    Where a manifest IS present, it wins outright and the variable is ignored: the
    map declares its own orientation, and a stray shell variable quietly overriding
    it would be a new version of the bug this all exists to fix.
    """
    return {
        "blueprint": LEGACY_TL_BLUEPRINT,
        "group_blueprint": LEGACY_TL_GROUP_BLUEPRINT,
        "z_offset_cm": LEGACY_TL_Z_OFFSET_CM,
        "flip_yaw_180": os.environ.get("FLIP_SIGNAL_HEADS_180", "").strip().lower()
        in {"1", "true", "yes", "on"},
    }


# ---------------------------------------------------------------- installing

def content_dir(carla_root, game_path):
    """`/Game/FIXS/Props` -> `<carla>/Unreal/CarlaUE4/Content/FIXS/Props`."""
    if not game_path.startswith("/Game/"):
        raise ManifestError(f"install_root must start with /Game/, got {game_path!r}")
    rel = game_path[len("/Game/"):].replace("/", os.sep)
    return os.path.join(carla_root, "Unreal", "CarlaUE4", "Content", rel)


def md5_of(path):
    h = hashlib.md5()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check_engine(manifest, ue4_root, manifest_path):
    """A .uasset is serialization-locked to the engine that saved it, so a manifest
    that declares one is checked against the engine actually installed. Mismatch is
    a hard error: the failure it prevents otherwise shows up as an editor that
    silently refuses to load the prop, long after the import looked fine."""
    want = (manifest.get("engine") or {}).get("unreal")
    if not want or not ue4_root:
        return None
    build = os.path.join(ue4_root, "Engine", "Build", "Build.version")
    if not os.path.isfile(build):
        return None
    try:
        with open(build, "r", encoding="utf-8-sig") as fh:
            info = json.load(fh)
        have = "{}.{}.{}".format(info.get("MajorVersion"), info.get("MinorVersion"),
                                 info.get("PatchVersion"))
    except Exception:
        return None
    # Compare on the declared precision: a manifest saying "4.26" accepts 4.26.2.
    want_parts = str(want).split(".")
    if have.split(".")[:len(want_parts)] != want_parts:
        raise ManifestError(
            f"{manifest_path} declares Unreal {want} but {ue4_root} is {have}. The "
            f"prop's .uasset is serialization-locked to the engine that saved it; a "
            f"different engine will not load it.")
    return have


def install(carla_root, props_dir, manifest, manifest_path, ue4_root=None):
    """Copy the pack's .uasset files to the manifest's install_root.

    Returns a list of (name, md5) for what is now installed, which the caller folds
    into the placement fingerprint so a changed prop forces re-placement.

    Notes on the two constraints that have already bitten someone:
      * the files are NOT renamed - a .uasset embeds its own package path, and a
        renamed file is rejected by the editor as "not a valid asset"
      * install_root is expected to sit outside /Game/Carla, because CARLA's own
        updaters own that tree and overwrite into it
    """
    import shutil

    if not props_dir:
        return []
    install_root = manifest.get("install_root")
    if not install_root:
        raise ManifestError(f"{manifest_path} declares no install_root")
    if install_root.startswith("/Game/Carla/"):
        print(f"[props] warning: install_root {install_root} is under /Game/Carla, "
              f"which CARLA's updaters overwrite; the prop will vanish on the next "
              f"CARLA update.")

    check_engine(manifest, ue4_root, manifest_path)

    dest_dir = content_dir(carla_root, install_root)
    os.makedirs(dest_dir, exist_ok=True)

    declared = _declared_md5s(props_dir)
    installed = []
    for name in sorted(os.listdir(props_dir)):
        if not name.lower().endswith(".uasset"):
            continue
        src = os.path.join(props_dir, name)
        digest = md5_of(src)
        expect = declared.get(os.path.splitext(name)[0])
        if expect and expect != digest:
            raise ManifestError(
                f"{name} is {digest} but its provenance.json says {expect}; the pack "
                f"is corrupt or was modified after export. Re-export rather than "
                f"hand-editing the asset.")
        dest = os.path.join(dest_dir, name)
        if not os.path.isfile(dest) or md5_of(dest) != digest:
            shutil.copy2(src, dest)
            print(f"[props] installed {name} -> {install_root}")
        installed.append((name, digest))

    if installed:
        _write_receipt(dest_dir, installed, manifest_path)
    return installed


def _declared_md5s(props_dir):
    """{asset name: md5} from a provenance.json beside the assets, if there is one."""
    path = os.path.join(props_dir, "provenance.json")
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as fh:
            prov = json.load(fh)
    except Exception:
        return {}
    asset = prov.get("asset")
    digest = (prov.get("exported") or {}).get("md5")
    return {asset: digest} if asset and digest else {}


def _write_receipt(dest_dir, installed, manifest_path):
    """Record what is installed, so a later run can tell whose prop is in there.

    Without this the install is anonymous: /Game/FIXS/Props sits OUTSIDE the map's
    content folder (on purpose - a re-import wipes that folder, and CARLA's updater
    owns /Game/Carla), so nothing else would ever notice a prop left behind by a
    different map bundle."""
    receipt = {
        "installed_from": os.path.abspath(manifest_path),
        "assets": {name: digest for name, digest in installed},
    }
    try:
        with open(os.path.join(dest_dir, ".fixs_props.json"), "w", encoding="utf-8") as fh:
            json.dump(receipt, fh, indent=2, sort_keys=True)
    except OSError as exc:
        print(f"[props] note: could not write install receipt ({exc}).")


# ---------------------------------------------------------------- fingerprint

def digests(props_dir):
    """[(name, md5)] for the .uasset files in `props_dir`, installing nothing.

    Same shape as what install() returns, so a fingerprint taken before installing
    matches the one taken after."""
    if not props_dir or not os.path.isdir(props_dir):
        return []
    return sorted((name, md5_of(os.path.join(props_dir, name)))
                  for name in os.listdir(props_dir) if name.lower().endswith(".uasset"))


def resolve_assets_dir(manifest_path, manifest, explicit=None, shared=None):
    """Where this manifest's .uasset files actually are.

    Three cases, in order: an explicit --props-dir; assets sitting beside the
    manifest (or in a props/ under it); otherwise the shared props cache.

    The third case is the one that is easy to miss. A map bundle may ship a
    placement.yaml purely to OVERRIDE a number - say flip_yaw_180 for a table whose
    headings run the other way - without shipping the assets, because the assets are
    shared. Resolving that manifest against the shared cache is what lets a map
    override the numbers without duplicating the prop.
    """
    if explicit:
        return explicit if os.path.isdir(explicit) else None
    beside = find_props_dir(manifest_path)
    if beside:
        return beside
    if shared and os.path.isdir(shared) and any(
            os.path.isfile(os.path.join(shared, name + ".uasset"))
            for name in assets_declared(manifest)):
        return shared
    return None


def bundle_fingerprint(*dirs, shared=None):
    """Fingerprint of the manifest + props in play, or None if there is no manifest.

    Lets a caller ask "would placing give the same result as last time?" without
    installing anything or launching the editor. `shared` must be the same props
    cache the placer will use, or this and place_tls would fingerprint different
    asset sets and the map would re-place on every single run."""
    manifest_path = find_manifest(*dirs)
    if not manifest_path:
        return None
    manifest = load(manifest_path)
    settings = tl_settings(manifest, manifest_path)
    assets = resolve_assets_dir(manifest_path, manifest, shared=shared)
    return fingerprint(settings, digests(assets))


def fingerprint(settings, installed):
    """A short digest of the values that determine placement, plus the props used.

    place_tls records this alongside its "already placed" marker. The marker used to
    say only THAT lights were placed, never WHICH ones - so correcting z_offset_cm in
    the manifest changed nothing until someone thought to re-import. That is the same
    defect as the original bug: state baked into a saved artifact with no record of
    the inputs that produced it.

    Taken over the RESOLVED settings rather than the manifest's bytes. Hashing the
    file would make re-editing a comment, or reformatting, indistinguishable from
    changing a number - and re-placement is a slow editor round-trip that rewrites
    the .umap, so it must be driven by what actually changes the result.
    """
    h = hashlib.md5()
    for key in ("blueprint", "group_blueprint", "z_offset_cm", "flip_yaw_180"):
        h.update("{}={!r};".format(key, settings[key]).encode("utf-8"))
    for name, digest in sorted(installed or []):
        h.update(name.encode("utf-8"))
        h.update(digest.encode("ascii"))
    return h.hexdigest()


# ---------------------------------------------------------------- to the editor

def placer_env(settings):
    """Manifest values as environment variables for the in-editor placer.

    UE's embedded Python cannot import pyyaml, so the numbers cross this boundary as
    strings. FLIP_SIGNAL_HEADS_180 keeps its existing name - the placer already read
    it - so a manifest and a hand-set shell variable mean the same thing."""
    return {
        "FIXS_TL_BLUEPRINT": str(settings["blueprint"]),
        "FIXS_TL_GROUP_BLUEPRINT": str(settings["group_blueprint"]),
        "FIXS_TL_Z_OFFSET_CM": repr(float(settings["z_offset_cm"])),
        "FLIP_SIGNAL_HEADS_180": "1" if settings["flip_yaw_180"] else "0",
    }


def describe(settings, manifest_path):
    """One line saying where each number came from - the thing whose absence made
    the original divergence invisible."""
    where = os.path.abspath(manifest_path) if manifest_path else "built-in defaults (no manifest)"
    return ("[props] traffic lights: blueprint={blueprint} z_offset_cm={z:g} "
            "flip_yaw_180={flip} (from {where})").format(
        blueprint=settings["blueprint"], z=settings["z_offset_cm"],
        flip=settings["flip_yaw_180"], where=where)
