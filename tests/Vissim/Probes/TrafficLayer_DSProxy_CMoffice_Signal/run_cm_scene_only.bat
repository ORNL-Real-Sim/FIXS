@echo off
REM ============================================================================
REM  FIXS #172 - SimpleTrafficLight signal-STOP scene in CarMaker (NO VISSIM)
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this to drive the imported scene in CarMaker with the ego that
REM  STOPS at red lights:
REM    - corridor + 3 signalized intersections, looping ego
REM    - far-side signal heads (mounted at the start of the edge across the
REM      junction, facing back at the stop line)
REM    - a DrvStop per crossing -> the ego brakes at the stop line on red
REM
REM  It builds the signal-stop road/TestRun from the committed base road via
REM  add_signal_stops.py, then opens CarMaker + IPGMovie on the TestRun.
REM  Press START: standalone the 44 controllers cycle on their own (green/red);
REM  the ego stops at the reds. (In the full VISSIM co-sim the same controllers
REM  are driven live by VISSIM; the stopping mechanism is identical.)
REM
REM  Usage:
REM     run_cm_scene_only.bat            -> build + open CarMaker GUI + IPGMovie
REM     run_cm_scene_only.bat verify     -> build + run headless + stop-at-red report
REM
REM  Prereq: base road Data/Road/simple_traffic_light.rd5 (with the ego Route)
REM  must exist - it is committed; else run import_road.bat first + re-inject route.
REM ============================================================================

setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI

set CMPROJ=%RepoRoot%\ProprietaryFiles\CM13_proj
set CM_OFFICE=C:\IPG\carmaker\win64-13.1.3\bin\CM_Office.exe
set MOVIE=C:\IPG\carmaker\win64-13.1.3\GUI\Movie.exe
set TESTRUN=SimpleTL_SignalStop
if "%PYTHON%"=="" set PYTHON=python

echo ============================================================
echo  FIXS #172 - SignalStop scene in CarMaker (no VISSIM)
echo ============================================================

REM --- preflight ----------------------------------------------------------
if not exist "%CM_OFFICE%" ( echo ERROR: CM_Office.exe not found: %CM_OFFICE% & pause & exit /b 1 )
if not exist "%CMPROJ%\Data\Road\simple_traffic_light.rd5" (
    echo ERROR: base road simple_traffic_light.rd5 missing under %CMPROJ%\Data\Road
    echo   Run import_road.bat first ^(osc2cm^) + re-inject the route, then retry.
    pause & exit /b 1
)

REM --- build the signal-stop road + TestRun -------------------------------
echo Building signal-stop road ^(DrvStops + far-side heads^) + Car_Normal TestRun ...
"%PYTHON%" "%HERE%add_signal_stops.py"
if errorlevel 1 ( echo Build failed -- is Python on PATH? Set PYTHON=... & pause & exit /b 1 )

REM --- start fresh: close any stale CarMaker so it opens on THIS testrun ----
echo Closing any stale CarMaker windows...
taskkill /F /IM CM_Office.exe      >nul 2>&1
taskkill /F /IM HIL.exe            >nul 2>&1
taskkill /F /IM Movie.exe          >nul 2>&1

REM --- verify mode: headless run + stop-at-red report ---------------------
if /I "%~1"=="verify" (
    echo [verify] running %TESTRUN% headless ^(loops; runs the full TestRun^) ...
    "%CM_OFFICE%" -projectdir "%CMPROJ%" -cmd "SaveMode save" -run "%TESTRUN%"
    for /f "delims=" %%E in ('dir /b /s /o-d "%CMPROJ%\SimOutput\*%TESTRUN%_*.erg" 2^>nul') do ( set ERG=%%E& goto parsed )
    :parsed
    "%PYTHON%" "%HERE%verify_signalstop.py" "%ERG%" "%CMPROJ%\Data\Road\simple_traffic_light_signalstop.rd5"
    goto done
)

REM --- open the GUI on the project with the TestRun preloaded --------------
echo Opening CarMaker Office on TestRun %TESTRUN% ...
start "" "%CM_OFFICE%" -projectdir "%CMPROJ%" "%TESTRUN%"

REM --- open IPGMovie once the GUI (HIL.exe) is up, so it snaps to the scene -
if not exist "%MOVIE%" ( echo NOTE: Movie.exe not at %MOVIE% - open it manually if you want the 3D view & goto :done )
echo Waiting for the CarMaker GUI to come up before launching IPGMovie...
set /a _w=0
:waitgui
timeout /t 1 /nobreak >nul
tasklist /fi "imagename eq HIL.exe" 2>nul | find /i "HIL.exe" >nul && goto :guiup
set /a _w+=1
if %_w% lss 40 goto :waitgui
:guiup
timeout /t 3 /nobreak >nul
start "" "%MOVIE%"

:done
echo.
echo ============================================================
echo  Opened CarMaker on %TESTRUN%.
echo    - Press START: the ego loops the corridor and STOPS at red lights.
echo    - Far-side signal heads sit across each junction, facing the stop line.
echo  NOTE: standalone, signals cycle on their own; full sync needs the VISSIM co-sim.
echo ============================================================
echo.
pause
endlocal
