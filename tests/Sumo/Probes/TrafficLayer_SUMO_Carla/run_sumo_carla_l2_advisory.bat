@echo off
REM ============================================================================
REM  FIXS #174 - SUMO ^<-^> Carla demo, EgoMode 2 (L2) -- external speed advisory
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. Same stack as run_sumo_carla_l0_egodriver.bat, but the ego
REM  runs at L2: a real Carla vehicle (full PhysX), driven by the EgoDriver pursuit
REM  module, whose TARGET speed is supplied each 0.1 s by an ARTIFICIAL external
REM  speed controller (EgoSpeedAdvisor). The ego tracks a looping trapezoid speed
REM  profile (4 -^> 12 -^> 4 m/s) instead of a constant cruise.
REM
REM    1. launch CARLA (Carla\launch_carla.bat) + wait for RPC
REM    3. load simple_loop.xodr as the world
REM    4. launch SUMO on the BACKGROUND-ONLY SimpleLoop scenario
REM    5. launch TrafficLayer   6. launch VirCarlaEnv (config_l2_advisory.yaml)
REM
REM  Watch for: the ego repeatedly ACCELERATES to ~12 m/s, cruises, then SLOWS to
REM  ~4 m/s and repeats -- its measured speed chasing the commanded advisory.
REM  Verify after: _datalog\l2_advisory.csv has `speed` (measured) tracking
REM  `speedDesired` (commanded); plot with plot_l2_advisory.py.
REM
REM  If CARLA is ALREADY running with the world loaded:  set SKIP_CARLA=1 first.
REM ============================================================================
setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI
set CARLADIR=%RepoRoot%\Carla
set ENVFILE=%CARLADIR%\carla.env

set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set VCE=%RepoRoot%\VirCarlaEnv\x64\Release\VirCarlaEnv.exe
if not exist "%VCE%" set VCE=%RepoRoot%\tests\SumoCarla\VirCarlaEnv.exe
set CONFIG=%HERE%config_l2_advisory.yaml
set TLS=%HERE%traffic_light_table.csv
REM BACKGROUND-ONLY scenario: the ego is injected by TrafficLayer, not routed by SUMO.
set SUMOCFG=%RepoRoot%\tests\Sumo\networks\simple_loop\simple_loop.sumocfg
set XODR=%RepoRoot%\tests\Vissim\SimpleEcho\simple_loop.xodr

REM --- load carla.env (for PY + carla host/port) ------------------------------
if not exist "%ENVFILE%" ( echo [ERROR] %ENVFILE% missing. Copy carla.env.example to carla.env and edit it. & pause & exit /b 1 )
for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%ENVFILE%") do set "%%A=%%B"
if not defined CARLA_PORT set CARLA_PORT=2000

echo ============================================================
echo  FIXS #174 SUMO-Carla L2 demo: EXTERNAL speed advisory drives the ego target
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
    echo [1/6] Closing any running CARLA, then launching fresh...
    taskkill /F /IM UE4Editor.exe >nul 2>&1
    taskkill /F /IM CarlaUE4.exe  >nul 2>&1
    timeout /t 2 /nobreak >nul
    call "%CARLADIR%\launch_carla.bat"
    echo [2/6] Waiting for CARLA RPC on %CARLA_HOST%:%CARLA_PORT% ...
    powershell -NoProfile -ExecutionPolicy Bypass -File "%CARLADIR%\wait_for_rpc.ps1" -Port %CARLA_PORT%
    if errorlevel 1 ( echo ERROR: CARLA never came up on port %CARLA_PORT%. & pause & exit /b 1 )
    echo [3/6] Loading simple_loop.xodr as the CARLA world...
    "%PY%" "%CARLADIR%\load_opendrive_world.py" "%XODR%" --sync --delta 0.05 --timeout 300
    if errorlevel 1 ( echo ERROR: load_opendrive_world.py failed ^(is carla 0.9.15 in %%PY%%?^). & pause & exit /b 1 )
)

REM --- 4. SUMO (background traffic only; seed pinned for reproducibility) ------
echo [4/6] Launching SUMO (SimpleLoop background-only, TraCI on port 1337)...
start "FIXS SUMO" sumo-gui -c "%SUMOCFG%" --remote-port 1337 --step-length 0.1 --seed 5 --start

REM --- 5. TrafficLayer --------------------------------------------------------
echo [5/6] Launching TrafficLayer (SUMO path)...
start "FIXS TrafficLayer (SUMO)" cmd /k "%TL% -f %CONFIG%"

REM --- 6. Carla bridge (EgoMode 2) ---------------------------------------------
echo [6/6] Launching VirCarlaEnv (Carla bridge, EgoMode 2 Advisory)...
start "FIXS VirCarlaEnv (L2)" cmd /k "%VCE% -f %CONFIG% -t %TLS%"

echo.
echo ============================================================
echo  Launched. In the CARLA window the ego (tesla) repeatedly speeds
echo  up to ~12 m/s, cruises, then slows to ~4 m/s -- chasing the
echo  external advisory. In SUMO-gui a red 'ego' appears and the
echo  background traffic reacts.
echo  Stop order: close VirCarlaEnv -^> close SUMO -^> Ctrl+C TrafficLayer -^> close CARLA.
echo ============================================================
echo.
pause
endlocal
