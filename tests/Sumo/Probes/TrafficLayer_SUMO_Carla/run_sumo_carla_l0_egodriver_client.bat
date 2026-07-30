@echo off
REM ============================================================================
REM  FIXS #174 - SUMO ^<-^> Carla demo, EgoMode 1 (L0) -- UNIFIED EgoDriver CLIENT
REM ----------------------------------------------------------------------------
REM  Same stack as run_sumo_carla_l0_egodriver.bat, but the driver is no longer
REM  in-process in VirCarlaEnv. ego_driver_client.py connects to TrafficLayer as a
REM  client, runs pure-pursuit + speed-hold, and streams an ACTUATION command
REM  (accelerator/brake pedals + steer angle) on the ego record. VirCarlaEnv
REM  (EgoL0Driver: Actuation) applies it via ApplyControl. This is the standalone
REM  unified-EgoDriver path that also serves L2 (add an advisor below it) and L4
REM  (swap the client for a real controller).
REM
REM  Launch order: CARLA + simple_loop.xodr, SUMO, TrafficLayer, ego_driver_client,
REM  VirCarlaEnv. TL serves the client on port 420 (before VirCarlaEnv on 440).
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
set CONFIG=%HERE%config_l0_egodriver_client.yaml
set CLIENT=%HERE%ego_driver_client.py
set TLS=%HERE%traffic_light_table.csv
set SUMOCFG=%RepoRoot%\tests\Sumo\networks\simple_loop\simple_loop.sumocfg
set XODR=%RepoRoot%\tests\Vissim\SimpleEcho\simple_loop.xodr

if not exist "%ENVFILE%" ( echo [ERROR] %ENVFILE% missing. & pause & exit /b 1 )
for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%ENVFILE%") do set "%%A=%%B"
if not defined CARLA_PORT set CARLA_PORT=2000

echo ============================================================
echo  FIXS #174 SUMO-Carla L0 demo: UNIFIED EgoDriver CLIENT drives the ego
echo ============================================================

taskkill /F /IM TrafficLayer.exe  >nul 2>&1
taskkill /F /IM VirCarlaEnv.exe   >nul 2>&1
taskkill /F /IM sumo-gui.exe      >nul 2>&1
taskkill /F /IM sumo.exe          >nul 2>&1

if not exist "%TL%"      ( echo ERROR: TrafficLayer.exe missing: %TL%   & pause & exit /b 1 )
if not exist "%VCE%"     ( echo ERROR: VirCarlaEnv.exe missing: %VCE% & pause & exit /b 1 )
if not exist "%CLIENT%"  ( echo ERROR: ego_driver_client.py missing: %CLIENT% & pause & exit /b 1 )
if not exist "%SUMOCFG%" ( echo ERROR: shared SUMO scenario missing: %SUMOCFG% & pause & exit /b 1 )
if not exist "%XODR%"    ( echo ERROR: simple_loop.xodr missing: %XODR% & pause & exit /b 1 )
where sumo-gui >nul 2>&1 || ( echo ERROR: sumo-gui not on PATH & pause & exit /b 1 )

if defined SKIP_CARLA (
    echo [1-3/7] SKIP_CARLA set -- assuming CARLA is up with simple_loop.xodr loaded.
) else (
    echo [1/7] Closing any running CARLA, then launching fresh...
    taskkill /F /IM UE4Editor.exe >nul 2>&1
    taskkill /F /IM CarlaUE4.exe  >nul 2>&1
    timeout /t 2 /nobreak >nul
    call "%CARLADIR%\launch_carla.bat"
    echo [2/7] Waiting for CARLA RPC on %CARLA_HOST%:%CARLA_PORT% ...
    powershell -NoProfile -ExecutionPolicy Bypass -File "%CARLADIR%\wait_for_rpc.ps1" -Port %CARLA_PORT%
    if errorlevel 1 ( echo ERROR: CARLA never came up on port %CARLA_PORT%. & pause & exit /b 1 )
    echo [3/7] Loading simple_loop.xodr as the CARLA world...
    "%PY%" "%CARLADIR%\load_opendrive_world.py" "%XODR%" --sync --delta 0.05 --timeout 300
    if errorlevel 1 ( echo ERROR: load_opendrive_world.py failed. & pause & exit /b 1 )
)

echo [4/7] Launching SUMO (SimpleLoop background-only, TraCI on port 1337)...
start "FIXS SUMO" sumo-gui -c "%SUMOCFG%" --remote-port 1337 --step-length 0.1 --seed 5 --start

echo [5/7] Launching TrafficLayer (SUMO path)...
start "FIXS TrafficLayer (SUMO)" cmd /k "%TL% -f %CONFIG%"

echo [6/7] Launching unified EgoDriver client (port 420, before VirCarlaEnv)...
start "FIXS EgoDriver client" cmd /k ""%PY%" "%CLIENT%" "%CONFIG%""

echo [7/7] Launching VirCarlaEnv (Carla bridge, EgoMode 1 Actuation)...
start "FIXS VirCarlaEnv (L0 Actuation)" cmd /k "%VCE% -f %CONFIG% -t %TLS%"

echo.
echo ============================================================
echo  Launched. The ego (tesla) should accelerate on its tires and lap the loop,
echo  driven by ego_driver_client.py's actuation over FIXS. In SUMO-gui a red
echo  'ego' appears and background traffic reacts.
echo  Stop order: close VirCarlaEnv -^> close EgoDriver client -^> close SUMO -^> Ctrl+C TrafficLayer -^> close CARLA.
echo ============================================================
echo.
pause
endlocal
