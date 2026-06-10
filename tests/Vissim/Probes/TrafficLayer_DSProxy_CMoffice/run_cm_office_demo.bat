@echo off
REM ============================================================================
REM  FIXS #168 - CarMaker (office) <-> VISSIM co-simulation demo  (SimpleEcho)
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. It:
REM    1. (via setup_gui.py) stages the DS-enabled network, writes
REM       config.runtime.yaml, and registers the custom CarMaker exe +
REM       "-f config.runtime.yaml" into the CM project GUI config.
REM    2. Launches TrafficLayer (DSProxy) -> it spawns VISSIM, then WAITS
REM       (blocking accept) for CarMaker to connect on port 2444.
REM    3. Opens the CarMaker Office GUI on the CM project.
REM
REM  THEN, in the CarMaker GUI (after TrafficLayer prints "VISSIM_Connect OK"):
REM    - Load TestRun "SimpleLoop_VISSIM_rs"
REM    - Press the green START button.
REM  That Start is what begins the lockstep co-simulation.
REM
REM  Stop order: Stop in CarMaker -> close VISSIM window -> Ctrl+C TrafficLayer.
REM ============================================================================

setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI

set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set CMEXE=%RepoRoot%\ProprietaryFiles\CM13_proj\src\CarMaker.win64.exe
set CMPROJ=%RepoRoot%\ProprietaryFiles\CM13_proj
set CM_OFFICE=C:\IPG\carmaker\win64-13.1.3\bin\CM_Office.exe
set TESTRUN=SimpleLoop_VISSIM_rs
set RUNCFG=%HERE%config.runtime.yaml
set PYTHON=C:\Users\yshao\miniconda3\envs\realsim_dev\python.exe

echo ============================================================
echo  FIXS #168 CarMaker-VISSIM co-simulation demo (SimpleEcho)
echo ============================================================

REM --- kill stale processes so the GUI launches FRESH with the right config -
REM CM_Office reads its command-line-options (CM.Args) only when it launches.
REM A leftover GUI window keeps using whatever config it was opened with -- if
REM that's a signal-sync config (SynchronizeTrafficSignal: true) the .lib opens
REM a 2nd socket the DSProxy TrafficLayer does not service, and the co-sim
REM deadlocks. Killing here guarantees a clean launch on the demo config.
echo Closing any stale CarMaker / TrafficLayer / VISSIM so the GUI starts fresh...
taskkill /F /IM CM_Office.exe        >nul 2>&1
taskkill /F /IM CarMaker.win64.exe   >nul 2>&1
taskkill /F /IM TrafficLayer.exe     >nul 2>&1
taskkill /F /IM VISSIM220.exe        >nul 2>&1

REM --- preflight -----------------------------------------------------------
if not exist "%TL%"        ( echo ERROR: TrafficLayer.exe missing: %TL%        & echo  build: scripts\dispatch\2_core_components.bat & pause & exit /b 1 )
if not exist "%CMEXE%"     ( echo ERROR: custom CarMaker.win64.exe missing: %CMEXE% & pause & exit /b 1 )
if not exist "%CM_OFFICE%" ( echo ERROR: CM_Office.exe not found: %CM_OFFICE%   & pause & exit /b 1 )
if not exist "%PYTHON%"    ( echo ERROR: python env not found: %PYTHON%         & pause & exit /b 1 )
if not exist "%CMPROJ%\Data\TestRun\%TESTRUN%" ( echo ERROR: TestRun %TESTRUN% missing. Run import_road.bat then build_testrun.py. & pause & exit /b 1 )

REM --- 1. stage network + write config.runtime.yaml + patch GUI config ------
echo [1/3] Preparing demo (stage network, write config, register exe)...
"%PYTHON%" "%HERE%setup_gui.py"
if errorlevel 1 ( echo ERROR: setup_gui.py failed. & pause & exit /b 1 )

REM --- 2. launch TrafficLayer (spawns VISSIM, then waits for CarMaker) ------
echo [2/3] Launching TrafficLayer (DSProxy). It spawns VISSIM, then waits on port 2444...
echo        (wait for it to print "VISSIM_Connect OK" before pressing Start)
start "FIXS TrafficLayer (DSProxy)" cmd /k "%TL% -f %RUNCFG%"

REM --- 3. open CarMaker Office GUI on the project ---------------------------
echo [3/3] Opening CarMaker Office GUI...
start "" "%CM_OFFICE%" -projectdir "%CMPROJ%"

echo.
echo ============================================================
echo  Launched. NOW in the CarMaker GUI:
echo    1) WAIT for the TrafficLayer window to print "VISSIM_Connect OK"
echo    2) Load TestRun:  %TESTRUN%
echo    3) Press  START.
echo  (TrafficLayer is already waiting; Start begins the lockstep
echo   CarMaker / VISSIM co-simulation.)
echo ============================================================
echo.
pause
endlocal
