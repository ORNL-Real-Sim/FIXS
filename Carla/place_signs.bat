@echo off
REM Windows wrapper for the generic road-sign placer place_signs.py.
REM Places the RoadRunner sign meshes CARLA's import culled (+ fixes their
REM see-through materials) into a cooked map, e.g.:
REM   place_signs.bat --map <name>     (or --map-config, or pick from cooked maps)
python "%~dp0place_signs.py" %*
