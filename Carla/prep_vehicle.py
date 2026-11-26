"""
prep_vehicle.py - turn a vehicle bundle's source model into the rig import_vehicle.py installs.

  python Carla/prep_vehicle.py <bundle_dir> [--blender <blender.exe>] [--allow-mismatch]
  python Carla/prep_vehicle.py <bundle_dir> --no-prep --pack <id>.zip   zip it for a release

Reads <bundle_dir>/vehicle.yaml and the model its `source` names, then: converts to
metres and the CARLA vehicle frame (+X forward, +Z up, origin on the ground under the
centre), drops SketchUp's reversed twin faces when asked, finds the four wheels, sorts
material slots into paint / glass / light glass / opaque, checks the measured size,
wheelbase, track and wheel radius against the manifest, and has Blender rig the mesh to
CARLA's 4-wheel skeleton. Writes into <bundle_dir>/rig/: vehicle.fbx, materials.json,
textures/, prep_report.json. Those files are what gets published in the library bundle.

Collada (.dae) is read here with pycollada (Blender 5 has no Collada importer); FBX,
glTF and OBJ are read by Blender. For those, give source.forward/up as Blender shows the
model after import (+Z up). The model file is untrusted data: Blender runs with
--background --disable-autoexec --factory-startup, and this tool never opens a .blend.
"""
import argparse
import glob
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from vehicles import manifest as vmanifest  # noqa: E402
from vehicles import mesh  # noqa: E402

BLENDER_SCRIPT = os.path.join(HERE, "vehicles", "blender_rig.py")
BLENDER_FLAGS = ["--background", "--disable-autoexec", "--factory-startup"]


def find_blender(explicit=None):
    cands = [explicit, os.environ.get("BLENDER"), shutil.which("blender")]
    cands += sorted(glob.glob(r"C:\Program Files\Blender Foundation\Blender *\blender.exe"), reverse=True)
    for c in cands:
        if c and os.path.isfile(c):
            return c
    sys.exit("[prep] Blender not found; install Blender 4.2+ or pass --blender / set BLENDER.")


def run_blender(blender, args):
    cmd = [blender, *BLENDER_FLAGS, "--python", BLENDER_SCRIPT, "--", *args]
    print("[prep] " + " ".join(cmd), flush=True)
    rc = subprocess.call(cmd)
    if rc != 0:
        sys.exit("[prep] Blender exited {}".format(rc))


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_source(m, blender, work):
    src = m["source"]
    path = os.path.join(m["dir"], src["file"])
    if not os.path.isfile(path):
        sys.exit("[prep] source.file {} not found in the bundle".format(path))
    if src["format"] == "collada":
        chunks, materials = mesh.read_collada(path)
        tex_root = os.path.dirname(path)
    else:
        run_blender(blender, ["dump", path, src["format"], work])
        chunks, materials = mesh.read_npz_dump(os.path.join(work, "dump.npz"),
                                               os.path.join(work, "dump.json"))
        tex_root = os.path.join(work, "textures")
    return path, chunks, materials, tex_root


def copy_textures(slot_info, tex_root, dest, bundle=None):
    """Copy each slot's texture into rig/textures/: a bundle-relative path as given (the
    licence plate), else found by basename under `tex_root`."""
    os.makedirs(dest, exist_ok=True)
    index = {}
    for root, _, names in os.walk(tex_root):
        for n in names:
            index.setdefault(n.lower(), os.path.join(root, n))
    missing = []
    for s in slot_info:
        t = s.get("texture")
        if not t:
            continue
        own = os.path.join(bundle, t) if bundle else None
        found = own if own and os.path.isfile(own) else index.get(os.path.basename(t).lower())
        if found:
            shutil.copy2(found, os.path.join(dest, os.path.basename(found)))
            s["texture"] = os.path.basename(found)
        else:
            missing.append(t)
            s["texture"] = None
    return missing


def prep(bundle, blender=None, allow_mismatch=False, keep_work=False):
    mpath = vmanifest.find(bundle)
    if not mpath:
        sys.exit("[prep] {} has no vehicle.yaml".format(bundle))
    try:
        m = vmanifest.load(mpath)
    except vmanifest.ManifestError as exc:
        sys.exit("[prep] {}".format(exc))
    if m["source"] is None:
        sys.exit("[prep] {} declares no source; nothing to prepare.".format(mpath))
    src = m["source"]
    blender = find_blender(blender)
    work = tempfile.mkdtemp(prefix="fixs_vehprep_")
    try:
        src_path, chunks, materials, tex_root = read_source(m, blender, work)
        chunks = mesh.to_vehicle_frame(chunks, src["scale"], src["forward"], src["up"])
        try:
            arrays, meta = mesh.build_parts(chunks, materials, m)
        except mesh.MeshError as exc:
            sys.exit("[prep] {}".format(exc))
        measured = mesh.measure(meta)
        rows = vmanifest.check_dimensions(measured, m)
        print("[prep] {}: {} triangles before decimation, {} slots".format(
            m["id"], measured["triangles"], len(meta["slots"])))
        print(vmanifest.format_checks(rows))
        bad = [r[0] for r in rows if not r[4]]

        rig = os.path.join(m["dir"], vmanifest.RIG_DIR)
        os.makedirs(rig, exist_ok=True)
        report = {"id": m["id"], "source": {"file": src["file"], "sha256": sha256_of(src_path),
                                            "format": src["format"], "scale": src["scale"],
                                            "forward": src["forward"], "up": src["up"],
                                            "dedupe_twins": src["dedupe_twins"]},
                  "measured": measured, "checks": [list(r) for r in rows],
                  "wheels": meta["wheels"], "slots": meta["slot_info"]}
        if bad and not allow_mismatch:
            _write_json(os.path.join(m["dir"], vmanifest.RIG_REPORT), report)
            sys.exit("[prep] out of tolerance: {}. Fix source.scale / the model, correct the manifest, "
                     "or pass --allow-mismatch to accept it (recorded in prep_report.json).".format(bad))
        report["accepted_mismatch"] = bad

        import numpy as np
        np.savez_compressed(os.path.join(work, "parts.npz"), **arrays)
        _write_json(os.path.join(work, "parts.json"), meta)
        fbx = os.path.join(m["dir"], vmanifest.RIG_FBX)
        run_blender(blender, ["rig", work, fbx, repr(src["decimate"])])
        with open(os.path.join(work, "rig_stats.json"), "r", encoding="utf-8") as fh:
            report["rig"] = json.load(fh)
        report["rig"]["fbx"] = vmanifest.RIG_FBX
        report["rig"]["sha256"] = sha256_of(fbx)

        tex_dest = os.path.join(m["dir"], vmanifest.RIG_TEXTURES)
        if os.path.isdir(tex_dest):
            shutil.rmtree(tex_dest)
        missing = copy_textures(meta["slot_info"], tex_root, tex_dest, m["dir"])
        if missing:
            print("[prep] textures not found, slots left untextured: {}".format(missing))
        _write_json(os.path.join(m["dir"], vmanifest.RIG_MATERIALS), {"slots": meta["slot_info"]})
        report["slots"] = meta["slot_info"]
        _write_json(os.path.join(m["dir"], vmanifest.RIG_REPORT), report)
        print("[prep] wrote {}/ ({} triangles in the FBX)".format(rig, report["rig"]["triangles"]))
    finally:
        if keep_work:
            print("[prep] work dir kept: {}".format(work))
        else:
            shutil.rmtree(work, ignore_errors=True)
    return 0


def pack(bundle, out_zip, include_source=False):
    """Zip the publishable part of a prepared bundle: vehicle.yaml, README.md, rig/ (+ source/)."""
    import zipfile
    m = vmanifest.load(os.path.join(bundle, vmanifest.MANIFEST_NAME))
    if not os.path.isfile(os.path.join(bundle, vmanifest.RIG_FBX)):
        sys.exit("[prep] {} is not prepared yet (no {})".format(bundle, vmanifest.RIG_FBX))
    roots = [vmanifest.RIG_DIR] + (["source"] if include_source else [])
    plate = (m["materials"].get("plate") or {}).get("texture")
    files = [n for n in (vmanifest.MANIFEST_NAME, "README.md", plate)
             if n and os.path.isfile(os.path.join(bundle, n))]
    for r in roots:
        for root, _, names in os.walk(os.path.join(bundle, r)):
            files += [os.path.relpath(os.path.join(root, n), bundle) for n in names]
    with zipfile.ZipFile(out_zip, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(files):
            z.write(os.path.join(bundle, f), f.replace(os.sep, "/"))
    print("[prep] packed {} ({} files, {:.1f} MB) -> {}; sha256 {}".format(
        m["id"], len(files), os.path.getsize(out_zip) / 1e6, out_zip, sha256_of(out_zip)))
    return out_zip


def _write_json(path, obj):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bundle", help="bundle folder holding vehicle.yaml and the source model")
    ap.add_argument("--blender", help="blender executable (default: $BLENDER, PATH, Program Files)")
    ap.add_argument("--allow-mismatch", action="store_true",
                    help="write the rig even when measured dimensions are out of tolerance")
    ap.add_argument("--keep-work", action="store_true", help="keep the intermediate arrays")
    ap.add_argument("--pack", metavar="ZIP", help="then zip the bundle for a library release")
    ap.add_argument("--include-source", action="store_true", help="with --pack: also ship source/")
    ap.add_argument("--no-prep", action="store_true", help="with --pack: zip what rig/ already holds")
    args = ap.parse_args()
    bundle = os.path.abspath(args.bundle)
    if not args.no_prep:
        prep(bundle, args.blender, args.allow_mismatch, args.keep_work)
    if args.pack:
        pack(bundle, os.path.abspath(args.pack), args.include_source)
    return 0


if __name__ == "__main__":
    sys.exit(main())
