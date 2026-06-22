@echo off
REM ============================================================================
REM  FIXS #172 - (Re)generate the CarMaker road + ego TestRun for the demo.
REM ----------------------------------------------------------------------------
REM  Headless (no CarMaker GUI). Run ONCE (or after simple_traffic_light.xodr /
REM  .xosc change). It:
REM    1. Stages simple_traffic_light.xodr + .xosc + catalogs into the CM project.
REM    2. Runs osc2cm -> CarMaker road (simple_traffic_light.rd5, with the 56
REM       traffic signals imported from the .xodr as odrSignalId mounts) + the
REM       TestRun SimpleTL_VISSIM (ego on the corridor-loop trajectory + Path route).
REM    3. Runs build_ego.py -> swaps ego to McLaren F1 + IPGDriver longitudinal.
REM
REM  Traffic objects (RS_C) and the signal table are added by later steps.
REM  Outputs land in ProprietaryFiles/CM13_proj/Data/{Road,TestRun}.
REM ============================================================================

setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI

set CMPROJ=%RepoRoot%\ProprietaryFiles\CM13_proj
set OSC2CM=C:\IPG\carmaker\win64-13.1.3\bin\osc2cm.exe
set CM_CATALOGS=C:\IPG\carmaker\win64-13.1.3\Data\OpenSCENARIO\Catalogs
set OSCDIR=%CMPROJ%\Data\OpenSCENARIO
set PYTHON=C:\Users\yshao\miniconda3\envs\realsim_dev\python.exe

echo === FIXS #172 road/TestRun import ===
if not exist "%OSC2CM%" ( echo ERROR: osc2cm.exe not found: %OSC2CM% & pause & exit /b 1 )
if not exist "%HERE%simple_traffic_light.xodr" ( echo ERROR: simple_traffic_light.xodr missing & pause & exit /b 1 )

REM --- 1. stage xodr + xosc + catalogs ------------------------------------
if not exist "%OSCDIR%" mkdir "%OSCDIR%"
copy /Y "%HERE%simple_traffic_light.xodr" "%OSCDIR%\simple_traffic_light.xodr" >nul
copy /Y "%HERE%simple_traffic_light.xosc" "%OSCDIR%\simple_traffic_light.xosc" >nul
if not exist "%OSCDIR%\Catalogs" xcopy /E /I /Q /Y "%CM_CATALOGS%" "%OSCDIR%\Catalogs" >nul
echo [1/3] Staged xodr + xosc + catalogs

REM --- 2. osc2cm: OpenDRIVE -> rd5 + TestRun + ego ------------------------
echo [2/3] Running osc2cm...
"%OSC2CM%" -p "%CMPROJ%" -o "Data/OpenSCENARIO/simple_traffic_light.xosc" -r "simple_traffic_light" -t "SimpleTL_VISSIM" -e "Ego"
if not exist "%CMPROJ%\Data\Road\simple_traffic_light.rd5" ( echo ERROR: osc2cm did not produce simple_traffic_light.rd5 & pause & exit /b 2 )

REM --- 3. McLaren ego + IPGDriver longitudinal ---------------------------
echo [3/3] build_ego.py (McLaren + IPGDriver)...
"%PYTHON%" "%HERE%build_ego.py"

echo.
echo === Import complete ===
echo   Road:    %CMPROJ%\Data\Road\simple_traffic_light.rd5
echo   TestRun: %CMPROJ%\Data\TestRun\SimpleTL_VISSIM  (McLaren ego, corridor loop)
echo Now run:  run_cm_scene_only.bat   (open the scene in CarMaker, no VISSIM)
endlocal
