@echo off
REM ======================================================================
REM  WITH AlpaSim / CATK world model -- endless-loop demo (#189)
REM
REM  SUMO -> TrafficLayer -> WM (alpasim/CATK, PyTorch+CUDA). TrafficLayer
REM  auto-launches sumo-gui; CATK drives the surrounding bubble vehicles.
REM  Ego (red) loops via its SUMO route. Compare vs run_without_wm.bat and
REM  run_with_wm_waymax.bat.
REM
REM  Uses the dedicated CATK venv (torch 2.8 + PyG). Model weights/tokens are
REM  referenced by absolute path in config_alpasim.yaml.
REM ======================================================================
setlocal
set RealSimPath=..\..\..
set configFilename=config_alpasim.yaml
set CATK_PY=C:\src_git\WM_AlpaSim\.venv-catk\Scripts\python.exe
set ALPASIM_SRC=C:/src_git/WM_AlpaSim/src/trafficsim

echo [alpasim] Starting TrafficLayer.exe (auto-launches sumo-gui)...
start "TrafficLayer" cmd /c %RealSimPath%\TrafficLayer\x64\Release\TrafficLayer.exe -f %configFilename%

echo [alpasim] Waiting for TrafficLayer + SUMO to initialize...
timeout /t 4 /nobreak >nul

echo [alpasim] Starting CATK world-model translator (torch venv)...
"%CATK_PY%" ..\wm_translator.py -f %configFilename% --backend alpasim

echo [alpasim] translator exited.
