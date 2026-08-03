@echo off
REM Windows wrapper for the generic traffic-light placer place_tls.py.
REM An app passes its map + table, e.g.:
REM   place_tls.bat --map-config path\to\map.txt --tl-table path\to\traffic_light_table.csv
python "%~dp0place_tls.py" %*
