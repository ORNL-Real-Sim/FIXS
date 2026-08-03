@echo off
REM ============================================================================
REM  FIXS #174 - SUMO <-> CarMaker (office) co-simulation demo  (SimpleLoop)
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. It:
REM    1. (via setup_gui.py) registers the custom CarMaker exe + "-f config.yaml"
REM       into the CM project GUI config (so Start runs the right exe + FIXS cfg).
REM    2. Launches SUMO on the shared SimpleLoop net (TraCI on port 1337).
REM    3. Launches TrafficLayer (SUMO path) -> connects to SUMO, then WAITS for
REM       CarMaker to connect on port 2444.
REM    4. Opens the CarMaker Office GUI on the CM project.
REM
REM  THEN, in the CarMaker GUI (after TrafficLayer prints "Waiting for all
REM  clients to connect"):
REM    - Load TestRun "SimpleLoop_rs"
REM    - Press the green START button.
REM  That Start begins the lockstep SUMO / CarMaker co-simulation.
REM
REM  Stop order: Stop in CarMaker -> close the SUMO window -> Ctrl+C TrafficLayer.
REM
REM  This is the SUMO sibling of ..\..\..\Vissim\Probes\TrafficLayer_DSProxy_CMoffice\
REM  run_cm_office_demo.bat -- same CarMaker side, SUMO traffic instead of VISSIM.
REM ============================================================================

setlocal
set HERE=%~dp0
REM probe is tests\Sumo\Probes\TrafficLayer_SUMO_CMoffice -> repo root is 4 up
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI

set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set CMEXE=%RepoRoot%\ProprietaryFiles\CM13_proj\src\CarMaker.win64.exe
set CMPROJ=%RepoRoot%\ProprietaryFiles\CM13_proj
set CM_OFFICE=C:\IPG\carmaker\win64-13.1.3\bin\CM_Office.exe
set TESTRUN=SimpleLoop_rs
set CONFIG=%HERE%config.yaml
set SUMOCFG=%RepoRoot%\tests\Sumo\networks\simple_loop\simple_loop.sumocfg

REM Python is used ONLY to patch the GUI config (setup_gui.py is pure stdlib) --
REM ANY Python 3 works, no packages. Honor a preset %PYTHON%; else try 'py', then
REM 'python'. Override with:  set PYTHON=C:\full\path\python.exe
if not defined PYTHON set "PYTHON=py"
"%PYTHON%" --version >nul 2>&1 || set "PYTHON=python"

echo ============================================================
echo  FIXS #174 SUMO-CarMaker co-simulation demo (SimpleLoop)
echo ============================================================

REM --- kill stale processes so the GUI launches FRESH with the right config ---
echo Closing any stale CarMaker / TrafficLayer / SUMO so the GUI starts fresh...
taskkill /F /IM CM_Office.exe        >nul 2>&1
taskkill /F /IM HIL.exe              >nul 2>&1
taskkill /F /IM Movie.exe            >nul 2>&1
taskkill /F /IM CarMaker.win64.exe   >nul 2>&1
taskkill /F /IM TrafficLayer.exe     >nul 2>&1
taskkill /F /IM sumo-gui.exe         >nul 2>&1
taskkill /F /IM sumo.exe             >nul 2>&1

REM --- preflight --------------------------------------------------------------
if not exist "%TL%"        ( echo ERROR: TrafficLayer.exe missing: %TL%        & echo  build: scripts\dispatch\2_core_components.bat & pause & exit /b 1 )
if not exist "%CMEXE%"     ( echo ERROR: custom CarMaker.win64.exe missing: %CMEXE% & pause & exit /b 1 )
if not exist "%CM_OFFICE%" ( echo ERROR: CM_Office.exe not found: %CM_OFFICE%   & pause & exit /b 1 )
if not exist "%SUMOCFG%"   ( echo ERROR: shared SUMO scenario missing: %SUMOCFG% & pause & exit /b 1 )
where sumo-gui >nul 2>&1   || ( echo ERROR: sumo-gui not on PATH ^(install SUMO, add %%SUMO_HOME%%\bin to PATH^) & pause & exit /b 1 )
"%PYTHON%" --version >nul 2>&1 || ( echo ERROR: no Python 3 found ^(install Python 3, or set PYTHON=full\path\python.exe^) & pause & exit /b 1 )
if not exist "%CMPROJ%\Data\TestRun\%TESTRUN%" ( echo ERROR: TestRun %TESTRUN% missing. Build it via the VISSIM probe's import_road.bat + build_testrun.py ^(shared CM road^). & pause & exit /b 1 )

REM --- 1. register custom exe + "-f config.yaml" into the CM GUI config --------
echo [1/3] Preparing demo (register custom exe + FIXS config into CM GUI)...
"%PYTHON%" "%HERE%setup_gui.py"
if errorlevel 1 ( echo ERROR: setup_gui.py failed. & pause & exit /b 1 )

REM --- 2. launch SUMO on the shared SimpleLoop net ----------------------------
echo [2/3] Launching SUMO (SimpleLoop, TraCI on port 1337)...
REM --seed 5: SUMO's UNSEEDED default realization halts the ego on the junction
REM curve and trips the stochastic IPGDriver off-road (~133 s). Seed 5 is a
REM verified-CLEAN realization (runs the full 1000 s), so the demo "just works".
REM (The off-road itself is a CarMaker IPGDriver junction defect, backend-agnostic;
REM  reproduce it with another seed, e.g. --seed 10, if you want to see it fail.)
start "FIXS SUMO" sumo-gui -c "%SUMOCFG%" --remote-port 1337 --step-length 0.1 --seed 5 --start

REM --- 2b. launch TrafficLayer (SUMO path; serves CarMaker on 2444) ------------
echo        Launching TrafficLayer (SUMO path). It connects to SUMO, then waits on port 2444...
echo        (wait for it to print "Waiting for all clients to connect" before pressing Start)
start "FIXS TrafficLayer (SUMO)" cmd /k "%TL% -f %CONFIG%"

REM --- 3. open CarMaker Office GUI on the project ------------------------------
echo [3/3] Opening CarMaker Office GUI...
start "" "%CM_OFFICE%" -projectdir "%CMPROJ%" "%TESTRUN%"

REM Open IPGMovie AFTER the GUI is up so it can connect/snap to it.
set MOVIE=C:\IPG\carmaker\win64-13.1.3\GUI\Movie.exe
if not exist "%MOVIE%" ( echo NOTE: Movie.exe not at %MOVIE% - open it manually & goto :skipmovie )
echo Waiting for the CarMaker GUI (HIL.exe) to come up before launching IPGMovie...
set /a _w=0
:waitgui
timeout /t 1 /nobreak >nul
tasklist /fi "imagename eq HIL.exe" 2>nul | find /i "HIL.exe" >nul && goto :guiup
set /a _w+=1
if %_w% lss 40 goto :waitgui
:guiup
timeout /t 3 /nobreak >nul
start "" "%MOVIE%"
:skipmovie

echo.
echo ============================================================
echo  Launched. NOW in the CarMaker GUI:
echo    1) WAIT for the TrafficLayer window to print "Waiting for all clients..."
echo    2) Load TestRun:  %TESTRUN%
echo    3) Press  START.
echo  (TrafficLayer is already connected to SUMO + waiting; Start begins the
echo   lockstep SUMO / CarMaker co-simulation.)
echo  Headless self-check instead:  %PYTHON% verify_sumo_cm.py
echo ============================================================
echo.
pause
endlocal
