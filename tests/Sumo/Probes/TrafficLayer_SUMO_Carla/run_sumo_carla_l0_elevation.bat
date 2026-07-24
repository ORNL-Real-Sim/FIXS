@echo off
REM ============================================================================
REM  FIXS #174 - SUMO ^<-^> Carla demo, EgoMode 1 (L0) on the ELEVATED SimpleLoop
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. Same L0 stack as run_sumo_carla_l0_tm.bat, but on a
REM  ROLLING-HILL loop: the ego is a REAL Carla vehicle (full PhysX) driven by
REM  Carla's native Traffic Manager over a 3D road, so you can watch it climb and
REM  descend the hills. Background traffic stays SUMO-driven.
REM
REM  ***  To tune the terrain, edit GRADE_AMP / N_HILLS below, then double-click  ***
REM  ***  again -- it regenerates the xodr AND the SUMO net from those two        ***
REM  ***  numbers (both stay consistent, one source), then runs.                  ***
REM
REM  The SUMO net is derived from the xodr by netconvert, so SUMO and the Carla
REM  world share ONE 3D geometry. The profile is closed (start elevation == end),
REM  so the loop joins seamlessly no matter how many hills.
REM
REM  If CARLA is ALREADY running with simple_loop_elevation.xodr loaded: set
REM  SKIP_CARLA=1 first (but note the terrain regen still runs, so only skip if
REM  you did NOT change GRADE_AMP / N_HILLS).
REM ============================================================================
setlocal

REM ====== TERRAIN KNOBS -- edit these two, then double-click to regenerate + run ======
set GRADE_AMP=3
set N_HILLS=3
REM  GRADE_AMP = hill amplitude in metres (peak grade ~= GRADE_AMP * 2*pi*N_HILLS / 774)
REM  N_HILLS   = number of up-and-down hills per lap (integer, so the loop stays closed)
REM ====================================================================================

set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI
set CARLADIR=%RepoRoot%\Carla
set ENVFILE=%CARLADIR%\carla.env

set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set VCE=%RepoRoot%\VirCarlaEnv\x64\Release\VirCarlaEnv.exe
if not exist "%VCE%" set VCE=%RepoRoot%\tests\SumoCarla\VirCarlaEnv.exe
set CONFIG=%HERE%config_l0_elevation.yaml
set TLS=%HERE%traffic_light_table.csv
set GEN=%RepoRoot%\tests\Vissim\SimpleEcho\gen_junction_loop_xodr.py
set XODR=%RepoRoot%\tests\Vissim\SimpleEcho\simple_loop_elevation.xodr
set NETDIR=%RepoRoot%\tests\Sumo\networks\simple_loop_elevation
set NET=%NETDIR%\simple_loop_elevation.net.xml
set SUMOCFG=%NETDIR%\simple_loop_elevation.sumocfg

REM --- load carla.env (for PY + carla host/port) ------------------------------
if not exist "%ENVFILE%" ( echo [ERROR] %ENVFILE% missing. Copy carla.env.example to carla.env and edit it. & pause & exit /b 1 )
for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%ENVFILE%") do set "%%A=%%B"
if not defined CARLA_PORT set CARLA_PORT=2000

echo ============================================================
echo  FIXS #174 SUMO-Carla L0 ELEVATION demo: %N_HILLS% hills, amp %GRADE_AMP% m
echo ============================================================

REM --- kill stale FIXS-side processes (leave CARLA alone) ----------------------
taskkill /F /IM TrafficLayer.exe  >nul 2>&1
taskkill /F /IM VirCarlaEnv.exe   >nul 2>&1
taskkill /F /IM sumo-gui.exe      >nul 2>&1
taskkill /F /IM sumo.exe          >nul 2>&1

REM --- preflight --------------------------------------------------------------
if not exist "%TL%"  ( echo ERROR: TrafficLayer.exe missing: %TL%   & echo  build: scripts\dispatch\2_core_components.bat & pause & exit /b 1 )
if not exist "%VCE%" ( echo ERROR: VirCarlaEnv.exe missing: %VCE% & pause & exit /b 1 )
if not exist "%GEN%" ( echo ERROR: generator missing: %GEN% & pause & exit /b 1 )
where sumo-gui >nul 2>&1 || ( echo ERROR: sumo-gui not on PATH ^(install SUMO, add %%SUMO_HOME%%\bin to PATH^) & pause & exit /b 1 )
where netconvert >nul 2>&1 || ( echo ERROR: netconvert not on PATH ^(SUMO bin^) & pause & exit /b 1 )

REM --- 0. regenerate the elevated xodr + SUMO net from the knobs (kept consistent) ----
echo [0/6] Regenerating terrain: %N_HILLS% hills x %GRADE_AMP% m ...
"%PY%" "%GEN%" 15 6.0 %GRADE_AMP% %N_HILLS%
if errorlevel 1 ( echo ERROR: xodr generation failed ^(is %%PY%% a working python?^). & pause & exit /b 1 )
if not exist "%NETDIR%" mkdir "%NETDIR%"
REM --geometry.max-segment-length 10: densify the net so SUMO's z tracks the xodr
REM elevation on the STRAIGHTS too. netconvert otherwise keeps only a straight's 2
REM endpoints, so SUMO's linear z misses mid-straight crests by metres and traffic
REM floats off the Carla road. 10 m spacing -> ~5 mm net-vs-xodr z error.
netconvert --opendrive-files "%XODR%" --geometry.max-segment-length 10 --output-file "%NET%"
if errorlevel 1 ( echo ERROR: netconvert xodr-^>net failed. & pause & exit /b 1 )
echo       xodr + net regenerated (SUMO and Carla share one 3D geometry).

REM --- 1-3. bring up CARLA + load the ELEVATED world (unless SKIP_CARLA) --------
if defined SKIP_CARLA (
    echo [1-3/6] SKIP_CARLA set -- assuming CARLA is up with simple_loop_elevation.xodr loaded.
) else (
    echo [1/6] Closing any running CARLA, then launching fresh...
    taskkill /F /IM UE4Editor.exe >nul 2>&1
    taskkill /F /IM CarlaUE4.exe  >nul 2>&1
    timeout /t 2 /nobreak >nul
    call "%CARLADIR%\launch_carla.bat"
    echo [2/6] Waiting for CARLA RPC on %CARLA_HOST%:%CARLA_PORT% ...
    powershell -NoProfile -ExecutionPolicy Bypass -File "%CARLADIR%\wait_for_rpc.ps1" -Port %CARLA_PORT%
    if errorlevel 1 ( echo ERROR: CARLA never came up on port %CARLA_PORT%. & pause & exit /b 1 )
    echo [3/6] Loading simple_loop_elevation.xodr as the CARLA world ^(the 3D mesh build can take a minute^)...
    "%PY%" "%CARLADIR%\load_opendrive_world.py" "%XODR%" --sync --delta 0.05 --timeout 300
    if errorlevel 1 ( echo ERROR: load_opendrive_world.py failed ^(is carla 0.9.15 in %%PY%%?^). & pause & exit /b 1 )
)

REM --- 4. SUMO (background traffic only; seed pinned for reproducibility) -------
echo [4/6] Launching SUMO (elevated SimpleLoop background-only, TraCI on port 1337)...
start "FIXS SUMO" sumo-gui -c "%SUMOCFG%" --remote-port 1337 --step-length 0.1 --seed 5 --start

REM --- 5. TrafficLayer --------------------------------------------------------
echo [5/6] Launching TrafficLayer (SUMO path)...
start "FIXS TrafficLayer (SUMO)" cmd /k "%TL% -f %CONFIG%"

REM --- 6. Carla bridge (EgoMode 1, TM on the 3D road) --------------------------
echo [6/6] Launching VirCarlaEnv (Carla bridge, EgoMode 1, elevated)...
start "FIXS VirCarlaEnv (L0 elevation)" cmd /k "%VCE% -f %CONFIG% -t %TLS%"

echo.
echo ============================================================
echo  Launched. After a one-time ~30 s "Pre-building TM InMemoryMap" pause, watch
echo  the ego climb and descend %N_HILLS% hills in the CARLA window (tire contact on
echo  the 3D road); in SUMO-gui a red 'ego' appears and background traffic reacts.
echo  Ego z + grade are logged to _datalog\l0_elevation.csv (plot to inspect).
echo  Stop: close VirCarlaEnv -^> close SUMO -^> Ctrl+C TrafficLayer -^> close CARLA.
echo ============================================================
echo.
pause
endlocal
