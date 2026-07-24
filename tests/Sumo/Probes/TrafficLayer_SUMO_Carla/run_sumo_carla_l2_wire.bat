@echo off
REM ============================================================================
REM  FIXS #174 - SUMO ^<-^> Carla demo, EgoMode 2 (L2) -- EXTERNAL advisory via FIXS
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. The genuine "L2 via FIXS" path: the desired-speed advisory
REM  is produced by a SEPARATE controller process (py_ego_speed_advisor.py) that
REM  connects to TrafficLayer as a client and streams the ego's speedDesired over
REM  FIXS. TrafficLayer's SEQUENTIAL client path overlays it into the feed
REM  VirCarlaEnv receives, and VirCarlaEnv reads it off the wire -> the real Carla
REM  ego (full PhysX) tracks it.
REM
REM  Client ORDER matters (ascending port): the advisor is on 400 (processed
REM  first), VirCarlaEnv on 440 (sees the advisory in bucket B). Both from
REM  config_l2_wire.yaml.
REM
REM    1. launch CARLA + wait for RPC   3. load simple_loop.xodr
REM    4. SUMO (background)   5. TrafficLayer (opens clients 400 + 440)
REM    6. py_ego_speed_advisor.py (client 400)   7. VirCarlaEnv (client 440)
REM
REM  Watch: the ego repeatedly accelerates to ~12 m/s, cruises, slows to ~4 m/s --
REM  chasing the EXTERNAL controller's advisory. Verify: _datalog\l2_wire.csv has
REM  `speed` (measured) tracking `speedDesired` (the wire advisory); plot with
REM  plot_l2_advisory.py (point it at l2_wire.csv) or plot_l2_wire below.
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
set CONFIG=%HERE%config_l2_wire.yaml
set TLS=%HERE%traffic_light_table.csv
set ADVISOR=%HERE%py_ego_speed_advisor.py
set SUMOCFG=%RepoRoot%\tests\Sumo\networks\simple_loop\simple_loop.sumocfg
set XODR=%RepoRoot%\tests\Vissim\SimpleEcho\simple_loop.xodr

REM --- load carla.env (for PY + carla host/port) ------------------------------
if not exist "%ENVFILE%" ( echo [ERROR] %ENVFILE% missing. Copy carla.env.example to carla.env and edit it. & pause & exit /b 1 )
for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%ENVFILE%") do set "%%A=%%B"
if not defined CARLA_PORT set CARLA_PORT=2000

REM Advisor needs CommonLib Python deps (pyyaml). Uses the realsim conda env;
REM override PYADVISOR if your CommonLib deps live elsewhere.
if not defined PYADVISOR set PYADVISOR=%USERPROFILE%\miniconda3\envs\realsim\python.exe
if not exist "%PYADVISOR%" set PYADVISOR=%PY%

echo ============================================================
echo  FIXS #174 SUMO-Carla L2 demo: EXTERNAL controller advisory over FIXS
echo ============================================================

taskkill /F /IM TrafficLayer.exe  >nul 2>&1
taskkill /F /IM VirCarlaEnv.exe   >nul 2>&1
taskkill /F /IM sumo-gui.exe      >nul 2>&1
taskkill /F /IM sumo.exe          >nul 2>&1

if not exist "%TL%"      ( echo ERROR: TrafficLayer.exe missing: %TL%   & echo  build: scripts\dispatch\2_core_components.bat & pause & exit /b 1 )
if not exist "%VCE%"     ( echo ERROR: VirCarlaEnv.exe missing: %VCE% & pause & exit /b 1 )
if not exist "%SUMOCFG%" ( echo ERROR: shared SUMO scenario missing: %SUMOCFG% & pause & exit /b 1 )
if not exist "%XODR%"    ( echo ERROR: simple_loop.xodr missing: %XODR% & pause & exit /b 1 )
where sumo-gui >nul 2>&1 || ( echo ERROR: sumo-gui not on PATH ^(install SUMO, add %%SUMO_HOME%%\bin to PATH^) & pause & exit /b 1 )

if defined SKIP_CARLA (
    echo [1-3] SKIP_CARLA set -- assuming CARLA is up with simple_loop.xodr loaded.
) else (
    echo [1] Closing any running CARLA, then launching fresh...
    taskkill /F /IM UE4Editor.exe >nul 2>&1
    taskkill /F /IM CarlaUE4.exe  >nul 2>&1
    timeout /t 2 /nobreak >nul
    call "%CARLADIR%\launch_carla.bat"
    echo [2] Waiting for CARLA RPC on %CARLA_HOST%:%CARLA_PORT% ...
    powershell -NoProfile -ExecutionPolicy Bypass -File "%CARLADIR%\wait_for_rpc.ps1" -Port %CARLA_PORT%
    if errorlevel 1 ( echo ERROR: CARLA never came up on port %CARLA_PORT%. & pause & exit /b 1 )
    echo [3] Loading simple_loop.xodr as the CARLA world...
    "%PY%" "%CARLADIR%\load_opendrive_world.py" "%XODR%" --sync --delta 0.05 --timeout 300
    if errorlevel 1 ( echo ERROR: load_opendrive_world.py failed. & pause & exit /b 1 )
)

echo [4] Launching SUMO (SimpleLoop background-only, TraCI 1337, seed 5)...
start "FIXS SUMO" sumo-gui -c "%SUMOCFG%" --remote-port 1337 --step-length 0.1 --seed 5 --start

echo [5] Launching TrafficLayer (SUMO path; app clients 400 + 440)...
start "FIXS TrafficLayer (SUMO)" cmd /k "%TL% -f %CONFIG%"

echo [6] Launching py_ego_speed_advisor (external L2 controller, client 400)...
start "FIXS L2 advisor" cmd /k ""%PYADVISOR%" "%ADVISOR%" "%CONFIG%""

echo [7] Launching VirCarlaEnv (Carla bridge, EgoMode 2 wire, client 440)...
start "FIXS VirCarlaEnv (L2 wire)" cmd /k "%VCE% -f %CONFIG% -t %TLS%"

echo.
echo ============================================================
echo  Launched. The ego chases the EXTERNAL controller's advisory
echo  (accelerate to ~12, cruise, slow to ~4, repeat). Stop order:
echo  close VirCarlaEnv -^> advisor -^> SUMO -^> Ctrl+C TrafficLayer -^> CARLA.
echo ============================================================
echo.
pause
endlocal
