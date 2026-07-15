@echo off
REM ======================================================================
REM  WITH World Model -- WorldModels endless-loop demo (#189)
REM
REM  SUMO -> TrafficLayer -> WM (constant_velocity). TrafficLayer auto-launches
REM  sumo-gui; the WM owns the surrounding vehicles in the ego bubble and drives
REM  their poses via moveToXY. The ego (red) loops via its SUMO route.
REM
REM  Watch the bubble around the ego: those vehicles are now driven by the WM,
REM  not SUMO. With the constant_velocity baseline they do NOT brake for reds --
REM  that is the visual contrast vs run_without_wm.bat, and exactly what a real
REM  signal-aware WM (alpasim/waymax) must fix.
REM ======================================================================
setlocal
set RealSimPath=..\..\..
set configFilename=config.yaml

echo [WM] Starting TrafficLayer.exe (auto-launches sumo-gui)...
start "TrafficLayer" cmd /c %RealSimPath%\TrafficLayer\x64\Release\TrafficLayer.exe -f %configFilename%

echo [WM] Waiting for TrafficLayer + SUMO to initialize...
timeout /t 4 /nobreak >nul

echo [WM] Starting World Model translator (constant_velocity backend)...
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    call %USERPROFILE%\miniconda3\Scripts\activate.bat realsim
) else if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    call %USERPROFILE%\anaconda3\Scripts\activate.bat realsim
) else (
    call conda activate realsim
)

python ..\wm_translator.py -f %configFilename% --backend constant_velocity

echo [WM] translator exited.
