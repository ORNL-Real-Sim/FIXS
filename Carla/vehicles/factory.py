"""Edits to CARLA's VehicleFactory `Vehicles` array, as plain dicts.

Stdlib only: the Unreal editor script imports this file from its own directory, so the
merge rules run (and are unit-tested) outside Unreal, and the editor only converts
`VehicleParameters` structs to and from these dicts.
"""

FIELDS = ("make", "model", "class_", "number_of_wheels", "generation", "object_type",
          "base_type", "special_type", "has_dynamic_doors", "has_lights",
          "recommended_colors", "supported_drivers")


class FactoryError(Exception):
    """The edit would touch an entry FIXS does not own."""


def blueprint_id(make, model):
    """The id CARLA derives for a factory entry (ActorBlueprintFunctionLibrary FillIdAndTags)."""
    return "vehicle.{}.{}".format(make, model).lower()


def entry_id(entry):
    return blueprint_id(entry.get("make", ""), entry.get("model", ""))


def find(entries, bp_id):
    """Indices of the entries that would produce `bp_id`."""
    return [i for i, e in enumerate(entries) if entry_id(e) == bp_id.lower()]


def merge(entries, new_entry, owned_ids=()):
    """`entries` with `new_entry` added, or replacing the entry with its id.

    Replacing is allowed only for an id in `owned_ids` (installed by FIXS before), so a
    library vehicle can never silently replace a stock CARLA car. Returns (entries, action)
    with action in {"added", "updated", "unchanged"}.
    """
    bp_id = entry_id(new_entry)
    hits = find(entries, bp_id)
    if hits and bp_id not in {o.lower() for o in owned_ids}:
        raise FactoryError(
            "VehicleFactory already has {} and FIXS did not install it; refusing to replace "
            "a stock or hand-added vehicle. Give the library vehicle another make/model."
            .format(bp_id))
    out = [e for i, e in enumerate(entries) if i not in hits]
    if len(hits) == 1 and same(entries[hits[0]], new_entry):
        return list(entries), "unchanged"
    out.append(dict(new_entry))
    return out, ("updated" if hits else "added")


def remove(entries, bp_id, owned_ids=()):
    """`entries` without `bp_id`. Returns (entries, removed_count)."""
    hits = find(entries, bp_id)
    if hits and bp_id.lower() not in {o.lower() for o in owned_ids}:
        raise FactoryError(
            "{} is in VehicleFactory but FIXS did not install it; not removing it.".format(bp_id))
    return [e for i, e in enumerate(entries) if i not in hits], len(hits)


def same(a, b):
    """Field-by-field equality, with colours compared as RGB(A) tuples."""
    for k in FIELDS:
        va, vb = a.get(k), b.get(k)
        if k == "recommended_colors":
            va = [tuple(c) for c in (va or [])]
            vb = [tuple(c) for c in (vb or [])]
        elif k == "supported_drivers":
            va, vb = list(va or []), list(vb or [])
        elif k == "class_":
            va, vb = _class_path(va), _class_path(vb)
        if va != vb:
            return False
    return True


def _class_path(v):
    """`/Game/X/BP_Y.BP_Y_C` for either spelling of a blueprint class path."""
    if not v:
        return ""
    v = str(v)
    return v if "." in v.rsplit("/", 1)[-1] else "{0}.{1}_C".format(v, v.rsplit("/", 1)[-1])
