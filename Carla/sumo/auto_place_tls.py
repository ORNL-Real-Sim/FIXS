"""
auto_place_tls.py - automatic traffic-light placement for a CARLA map.

Loads a map level, runs unreal_placing_tls.py to spawn traffic-light actors from
SUMO_TLS_TABLE_PATH, and saves the level. No manual editor clicking required.

IMPORTANT: run via the FULL editor with -ExecutePythonScript (a per-app launcher),
NOT the -Cmd / -run=pythonscript commandlet. The commandlet has no editor viewport,
and EditorLevelLibrary.spawn_actor_from_class crashes without one (it does
viewport-based placement). The full editor has a viewport, so placement must be done
synchronously here (the editor auto-quits right after this script returns).

  UE4Editor.exe <CarlaUE4.uproject> <map_path> -ExecutePythonScript="<this file>"

Required environment variables:
  SUMO_TLS_TABLE_PATH  path to traffic_light_table.csv (read by unreal_placing_tls.py)
  SUMO_TLS_MAP_PATH    content path of the level to place into, e.g.
                       /Game/RP_Ver0529/Maps/RP_Ver0529/RP_Ver0529
"""
import os
import unreal

MAP_PATH = os.environ.get("SUMO_TLS_MAP_PATH")
if not MAP_PATH:
    raise RuntimeError(
        "SUMO_TLS_MAP_PATH is not set. Set it to the level content path, e.g. "
        "/Game/RP_Ver0529/Maps/RP_Ver0529/RP_Ver0529"
    )

unreal.log_warning(f"auto_place_tls: loading level {MAP_PATH}")
if not unreal.EditorLevelLibrary.load_level(MAP_PATH):
    raise RuntimeError(f"Failed to load level: {MAP_PATH}")

# Reuse the proven placement script: it reads SUMO_TLS_TABLE_PATH, spawns the
# TrafficLightGroup/TrafficLight actors, and saves dirty packages at the end.
_here = os.path.dirname(os.path.abspath(__file__))
_placer = os.path.join(_here, "unreal_placing_tls.py")
unreal.log_warning(f"auto_place_tls: executing {_placer}")
with open(_placer) as fh:
    code = fh.read()
exec(compile(code, _placer, "exec"), {"__file__": _placer, "__name__": "__main__"})

unreal.log_warning("auto_place_tls: completed - traffic lights placed and level saved.")
