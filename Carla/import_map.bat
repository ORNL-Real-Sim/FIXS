@echo off
REM Windows wrapper for the generic map importer import_map.py.
REM An app passes its map via --map-config <file> (package= and url=) or directly
REM with --package/--package-url, e.g.:
REM   import_map.bat --package-pick --map-config path\to\map.txt
python "%~dp0import_map.py" %*
