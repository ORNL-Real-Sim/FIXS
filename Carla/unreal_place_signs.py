"""
unreal_place_signs.py - place the road-sign meshes CARLA's import culled, and fix
their see-through materials, in a single editor pass.

Two problems, one editor session (editor startup is the expensive part):

1) PLACEMENT. CARLA's PrepareAssetsForCooking commandlet imports every mesh from
   the RoadRunner FBX but refuses to PLACE any whose name OR material contains
   "sign"/"light" (ValidateStaticMesh) - it expects to spawn its own signal actors
   from the OpenDRIVE. Result: custom RoadRunner signs are imported as orphan
   StaticMesh assets and never appear. We spawn every mesh CARLA culled *as a sign*
   (name/material contains "sign", none contains "light" - lights belong to
   place_tls). World position is baked into the vertices at import, so each is
   spawned at the origin and lands exactly where RoadRunner placed it.

2) MATERIALS. CARLA auto-generates one Material per sign from the RoadRunner PNGs;
   because those PNGs have alpha, Unreal imports them BLEND_TRANSLUCENT with alpha
   wired to Opacity, so the panel renders semi-transparent. The intended look is
   alpha-MASKED. We rebuild each translucent sign material as Masked (RGB->BaseColor,
   A->OpacityMask, clip 0.333). Set SIGNS_MATERIAL_MODE=opaque to ignore alpha
   entirely instead.

Run via the FULL editor (the commandlet has no viewport and spawn_actor crashes):
  UE4Editor <CarlaUE4.uproject> <map_path> -ExecutePythonScript="<this file>"

Required environment variables:
  SIGNS_MAP_PATH        level content path, e.g. /Game/<map>/Maps/<map>/<map>
  SIGNS_ASSET_ROOT      content root scanned recursively, e.g. /Game/<map>
  SIGNS_MATERIAL_MODE   optional: "masked" (default) or "opaque"
"""
import os

import unreal

MAP_PATH = os.environ.get("SIGNS_MAP_PATH")
ASSET_ROOT = os.environ.get("SIGNS_ASSET_ROOT")
MATERIAL_MODE = os.environ.get("SIGNS_MATERIAL_MODE", "masked").strip().lower()
if not MAP_PATH or not ASSET_ROOT:
    raise RuntimeError("SIGNS_MAP_PATH and SIGNS_ASSET_ROOT must be set, e.g. "
                       "/Game/M/Maps/M/M and /Game/M")

unreal.log_warning(f"place_signs: loading level {MAP_PATH}")
if not unreal.EditorLevelLibrary.load_level(MAP_PATH):
    raise RuntimeError(f"Failed to load level: {MAP_PATH}")

reg = unreal.AssetRegistryHelpers.get_asset_registry()
assets = reg.get_assets_by_path(unreal.Name(ASSET_ROOT), recursive=True)


# ============================================================
# 1) Place the culled sign meshes
# ============================================================
def cull_names(mesh):
    """Mesh name + all its material names, lowercased - the same strings CARLA's
    ValidateStaticMesh() inspects when deciding to cull."""
    names = [mesh.get_name()]
    for sm in mesh.get_editor_property("static_materials"):
        mi = sm.material_interface
        if mi:
            names.append(mi.get_name())
    return [n.lower() for n in names]


def is_culled_sign(mesh):
    """True for meshes CARLA culled *as signs*: some name/material contains 'sign',
    and none contains 'light' (lights belong to place_tls)."""
    names = cull_names(mesh)
    if any("light" in n for n in names):
        return False
    return any("sign" in n for n in names)


# idempotency: meshes already referenced by a placed actor
already = set()
for actor in unreal.EditorLevelLibrary.get_all_level_actors():
    if isinstance(actor, unreal.StaticMeshActor):
        comp = actor.static_mesh_component
        mesh = comp.static_mesh if comp else None
        if mesh:
            already.add(mesh.get_path_name())

spawned, skipped, scanned = 0, 0, 0
for data in assets:
    if str(data.asset_class) != "StaticMesh":
        continue
    scanned += 1
    mesh = unreal.EditorAssetLibrary.load_asset(str(data.package_name))
    if not mesh or not is_culled_sign(mesh):
        continue
    if mesh.get_path_name() in already:
        skipped += 1
        continue

    # signs get no auto-collision from the cook; match the commandlet's choice
    body_setup = mesh.get_editor_property("body_setup")
    if body_setup:
        body_setup.set_editor_property(
            "collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)

    # world position baked into the vertices -> spawn at origin, exactly like
    # CARLA's SpawnMeshesToWorld() does for the road/terrain meshes
    actor = unreal.EditorLevelLibrary.spawn_actor_from_object(
        mesh, unreal.Vector(0.0, 0.0, 0.0), unreal.Rotator(0.0, 0.0, 0.0))
    if not actor:
        unreal.log_warning(f"place_signs: failed to spawn {data.package_name}")
        continue
    actor.set_actor_label(f"RoadSign_{mesh.get_name()}", mark_dirty=True)
    spawned += 1

unreal.log_warning(f"place_signs: scanned {scanned} static meshes, spawned {spawned} "
                   f"signs, {skipped} already placed.")


# ============================================================
# 2) Fix the see-through sign materials (translucent -> masked)
# ============================================================
MEL = unreal.MaterialEditingLibrary
target_blend = "OPAQUE" if MATERIAL_MODE == "opaque" else "MASKED"
fixed, mat_skipped, mat_failed = 0, 0, 0
for d in assets:
    if str(d.asset_class) != "Material":
        continue
    if "sign" not in str(d.asset_name).lower():
        continue
    path = str(d.package_name)
    mat = unreal.EditorAssetLibrary.load_asset(path)
    if not mat:
        continue
    blend = str(mat.get_editor_property("blend_mode"))
    if target_blend in blend:
        mat_skipped += 1
        continue
    # masked mode only rebuilds translucent panels; opaque mode converts anything
    if MATERIAL_MODE != "opaque" and "TRANSLUCENT" not in blend:
        mat_skipped += 1
        continue
    textures = list(MEL.get_used_textures(mat))
    if not textures:
        unreal.log_warning(f"place_signs: material {d.asset_name} has no texture; skip")
        mat_failed += 1
        continue
    tex = textures[0]

    # clean rebuild: these are trivial 2-node auto-materials
    MEL.delete_all_material_expressions(mat)
    ts = MEL.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -400, 0)
    ts.set_editor_property("texture", tex)
    ts.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
    MEL.connect_material_property(ts, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
    if MATERIAL_MODE == "opaque":
        mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)
    else:
        MEL.connect_material_property(ts, "A", unreal.MaterialProperty.MP_OPACITY_MASK)
        mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
        mat.set_editor_property("opacity_mask_clip_value", 0.333)
    MEL.recompile_material(mat)   # rebuild the shader; the disk save is batched below
    fixed += 1

unreal.log_warning(f"place_signs: materials -> {MATERIAL_MODE}: fixed {fixed}, "
                   f"skipped {mat_skipped}, failed {mat_failed}.")


# ============================================================
# One batch save at the end: the placed actors dirty the level, the rebuilt
# materials dirty their content packages, and save_dirty_packages writes them all
# in a single pass (instead of a save per sign).
# ============================================================
if spawned or fixed:
    unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True, True)
    unreal.log_warning("place_signs: level + assets saved (batch).")
else:
    unreal.log_warning("place_signs: nothing to do (no signs found / already done).")
