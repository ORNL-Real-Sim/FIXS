@echo off
REM ======================================================================
REM  WITH Waymo Waymax world model -- endless-loop demo (#189)
REM
REM  SUMO -> TrafficLayer -> WM (waymax, JAX). TrafficLayer auto-launches
REM  sumo-gui; Waymax's IDM route-following policy drives the surrounding bubble
REM  vehicles (car-following). Ego (red) loops via its SUMO route. Compare vs
REM  run_without_wm.bat and run_with_wm_alpasim.bat.
REM
REM  JAX is CPU-only on native Windows, so this is slower than the CATK (GPU)
REM  path -- expected. Uses the dedicated Waymax venv.
REM ======================================================================
setlocal
set RealSimPath=..\..\..
set configFilename=config_waymax.yaml
set WAYMAX_PY=C:\src_git\WM_Waymax\.venv-waymax\Scripts\python.exe

echo [waymax] Starting TrafficLayer.exe (auto-launches sumo-gui)...
start "TL-waymax(1357)" cmd /c %RealSimPath%\TrafficLayer\x64\Release\TrafficLayer.exe -f %configFilename%

echo [waymax] Waiting for TrafficLayer + SUMO to initialize...
timeout /t 4 /nobreak >nul

echo [waymax] Starting Waymax world-model translator (JAX venv, CPU)...
"%WAYMAX_PY%" ..\wm_translator.py -f %configFilename% --backend waymax

echo [waymax] translator exited.
