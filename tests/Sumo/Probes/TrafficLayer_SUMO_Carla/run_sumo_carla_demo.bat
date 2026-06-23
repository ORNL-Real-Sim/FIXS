@echo off
REM ============================================================================
REM  FIXS #174 - SUMO <-> Carla co-simulation demo  (SimpleLoop, no signals)
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. End-to-end one-click:
REM    1. launch CARLA from the source build (Carla\launch_carla.bat, -game)
REM    2. wait for the CARLA RPC port (Carla\wait_for_rpc.ps1)
REM    3. load simple_loop.xodr as the world (Carla\load_opendrive_world.py)
REM       -- coordinate-matched to the CarMaker xodr, no map package
REM    4. launch SUMO on the shared SimpleLoop net (TraCI port 1337)
REM    5. launch TrafficLayer (SUMO path) -> serves the Carla bridge on port 440
REM    6. launch VirCarlaEnv.exe -> spawns/poses the SUMO vehicles in Carla
REM
REM  If CARLA is ALREADY running with the world loaded, set SKIP_CARLA=1 to skip
REM  steps 1-3:   set SKIP_CARLA=1 & run_sumo_carla_demo.bat
REM
REM  One-time: copy Carla\carla.env.example -> Carla\carla.env and edit it.
REM  Stop order: close VirCarlaEnv -> close SUMO -> Ctrl+C TrafficLayer -> close CARLA.
REM ============================================================================
setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI
set CARLADIR=%RepoRoot%\Carla
set ENVFILE=%CARLADIR%\carla.env

set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set VCE=%RepoRoot%\VirCarlaEnv\x64\Release\VirCarlaEnv.exe
if not exist "%VCE%" set VCE=%RepoRoot%\tests\SumoCarla\VirCarlaEnv.exe
set CONFIG=%HERE%config.yaml
set TLS=%HERE%traffic_light_table.csv
set SUMOCFG=%RepoRoot%\tests\Sumo\network\simple_loop\simple_loop_ego.sumocfg
set XODR=%RepoRoot%\tests\Vissim\SimpleEcho\simple_loop.xodr

REM --- load carla.env (for PY + carla host/port) ------------------------------
if not exist "%ENVFILE%" ( echo [ERROR] %ENVFILE% missing. Copy carla.env.example to carla.env and edit it. & pause & exit /b 1 )
for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%ENVFILE%") do set "%%A=%%B"
if not defined CARLA_PORT set CARLA_PORT=2000

echo ============================================================
echo  FIXS #174 SUMO-Carla co-simulation demo (SimpleLoop)
echo ============================================================

REM --- kill stale FIXS-side processes (leave CARLA alone) ----------------------
taskkill /F /IM TrafficLayer.exe  >nul 2>&1
taskkill /F /IM VirCarlaEnv.exe   >nul 2>&1
taskkill /F /IM sumo-gui.exe      >nul 2>&1
taskkill /F /IM sumo.exe          >nul 2>&1

REM --- preflight --------------------------------------------------------------
if not exist "%TL%"      ( echo ERROR: TrafficLayer.exe missing: %TL%   & echo  build: scripts\dispatch\2_core_components.bat & pause & exit /b 1 )
if not exist "%VCE%"     ( echo ERROR: VirCarlaEnv.exe missing: %VCE% & pause & exit /b 1 )
if not exist "%SUMOCFG%" ( echo ERROR: shared SUMO scenario missing: %SUMOCFG% & pause & exit /b 1 )
if not exist "%XODR%"    ( echo ERROR: simple_loop.xodr missing: %XODR% & pause & exit /b 1 )
where sumo-gui >nul 2>&1 || ( echo ERROR: sumo-gui not on PATH ^(install SUMO, add %%SUMO_HOME%%\bin to PATH^) & pause & exit /b 1 )

REM --- 1-3. bring up CARLA + load the SimpleLoop world (unless SKIP_CARLA) -----
if defined SKIP_CARLA (
    echo [1-3/6] SKIP_CARLA set -- assuming CARLA is up with simple_loop.xodr loaded.
) else (
    echo [1/6] Launching CARLA source build...
    call "%CARLADIR%\launch_carla.bat"
    echo [2/6] Waiting for CARLA RPC on %CARLA_HOST%:%CARLA_PORT% ...
    powershell -NoProfile -ExecutionPolicy Bypass -File "%CARLADIR%\wait_for_rpc.ps1" -Port %CARLA_PORT%
    if errorlevel 1 ( echo ERROR: CARLA never came up on port %CARLA_PORT%. & pause & exit /b 1 )
    echo [3/6] Loading simple_loop.xodr as the CARLA world...
    "%PY%" "%CARLADIR%\load_opendrive_world.py" "%XODR%" --sync --delta 0.1
    if errorlevel 1 ( echo ERROR: load_opendrive_world.py failed ^(is carla 0.9.15 in %%PY%%?^). & pause & exit /b 1 )
)

REM --- 4. SUMO ----------------------------------------------------------------
echo [4/6] Launching SUMO (SimpleLoop, TraCI on port 1337)...
start "FIXS SUMO" sumo-gui -c "%SUMOCFG%" --remote-port 1337 --step-length 0.1 --start

REM --- 5. TrafficLayer --------------------------------------------------------
echo [5/6] Launching TrafficLayer (SUMO path)...
start "FIXS TrafficLayer (SUMO)" cmd /k "%TL% -f %CONFIG%"

REM --- 6. Carla bridge --------------------------------------------------------
echo [6/6] Launching VirCarlaEnv (Carla bridge)...
start "FIXS VirCarlaEnv" cmd /k "%VCE% -f %CONFIG% -t %TLS%"

echo.
echo ============================================================
echo  Launched. Watch the CARLA window: SUMO's SimpleLoop vehicles
echo  should spawn and follow their SUMO poses on the loop road.
echo  Stop order: close VirCarlaEnv -^> close SUMO -^> Ctrl+C TrafficLayer -^> close CARLA.
echo  Headless self-check instead:  %PY% verify_sumo_carla.py
echo ============================================================
echo.
pause
endlocal
