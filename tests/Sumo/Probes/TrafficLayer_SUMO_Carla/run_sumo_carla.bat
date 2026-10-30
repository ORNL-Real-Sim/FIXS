@echo off
REM ============================================================================
REM  SUMO <-> CARLA on SimpleLoop -- one launcher for every variant.
REM
REM    run_sumo_carla.bat [variant]          (double-click = demo)
REM
REM    demo                 SUMO drives every vehicle; CARLA shows them
REM    l0_tm                ego is a CARLA PhysX car on Traffic Manager
REM    l0_egodriver         ego on the bridge's built-in pursuit driver
REM    l0_egodriver_client  ego driven by ego_driver_client.py over FIXS
REM    l0_elevation         l0_tm on a rolling-hill loop (terrain regenerated)
REM    l2                   ego follows py_ego_speed_advisor.py's advisory
REM
REM  CARLA starts the one way FIXS starts it: run_cosim's launcher, reached
REM  through load_opendrive_world.py --launch, from ~/.fixs/carla.json. The bridge
REM  is the Python one run_cosim runs; RS_BRIDGE=cpp uses a VirCarlaEnv.exe BUILT
REM  from this tree instead, never the June copy under tests\SumoCarla.
REM  SKIP_CARLA=1 if CARLA is already up with the variant's road loaded.
REM  Headless PASS/FAIL check of the same stack: verify_sumo_carla.py
REM ============================================================================
setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI
set CARLADIR=%RepoRoot%\Carla
set NETS=%RepoRoot%\tests\Sumo\networks
set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set VCE=%RepoRoot%\VirCarlaEnv\x64\Release\VirCarlaEnv.exe
set PYBRIDGE=%CARLADIR%\VirEnv\mainVirCarla.py
set TLS=%HERE%traffic_light_table.csv
set XODR=%RepoRoot%\tests\Vissim\SimpleEcho\simple_loop.xodr
set CARLA_PORT=2000

set VARIANT=%~1
if "%VARIANT%"=="" set VARIANT=demo

REM --- what differs between variants -----------------------------------------
REM Everything but the plain demo takes the ego out of SUMO's demand, ticks CARLA
REM at 0.05 s under the 0.1 s feed, and seeds SUMO so runs are comparable.
set SUMOCFG=%NETS%\simple_loop\simple_loop.sumocfg
set DELTA=0.05
set SEED=--seed 5
set EXTRA=
if /i "%VARIANT%"=="demo" (
    set CONFIG=%HERE%config.yaml
    set SUMOCFG=%NETS%\simple_loop\simple_loop_ego.sumocfg
    set DELTA=0.1
    set SEED=
) else if /i "%VARIANT%"=="l0_tm" (
    set CONFIG=%HERE%config_l0_tm.yaml
) else if /i "%VARIANT%"=="l0_egodriver" (
    set CONFIG=%HERE%config_l0_egodriver.yaml
) else if /i "%VARIANT%"=="l0_egodriver_client" (
    set CONFIG=%HERE%config_l0_egodriver_client.yaml
    set EXTRA=%HERE%ego_driver_client.py
) else if /i "%VARIANT%"=="l0_elevation" (
    set CONFIG=%HERE%config_l0_elevation.yaml
    set XODR=%RepoRoot%\tests\Vissim\SimpleEcho\simple_loop_elevation.xodr
    set SUMOCFG=%NETS%\simple_loop_elevation\simple_loop_elevation.sumocfg
) else if /i "%VARIANT%"=="l2" (
    set CONFIG=%HERE%config_l2.yaml
    set EXTRA=%HERE%py_ego_speed_advisor.py
) else (
    echo ERROR: unknown variant "%VARIANT%". One of: demo l0_tm l0_egodriver l0_egodriver_client l0_elevation l2
    pause & exit /b 2
)

REM --- the interpreter FIXS is configured to use -------------------------------
set PY=
for /f "usebackq delims=" %%P in (`python -c "import sys; sys.path.insert(0, r'%CARLADIR%'); import carla_env_setup as e; print((e.load_config() or {}).get('python') or '')"`) do set "PY=%%P"
if not defined PY ( echo ERROR: no FIXS python configured. Run:  python "%CARLADIR%\carla_env_setup.py" & pause & exit /b 1 )

REM --- preflight ----------------------------------------------------------------
if not exist "%TL%"      ( echo ERROR: TrafficLayer.exe missing: %TL% & echo   build: scripts\dispatch\2_core_components.bat & pause & exit /b 1 )
if /i "%RS_BRIDGE%"=="cpp" if not exist "%VCE%" ( echo ERROR: RS_BRIDGE=cpp but no built VirCarlaEnv.exe: %VCE% & pause & exit /b 1 )
if not exist "%CONFIG%"  ( echo ERROR: config missing: %CONFIG% & pause & exit /b 1 )
where sumo-gui >nul 2>&1 || ( echo ERROR: sumo-gui not on PATH ^(add %%SUMO_HOME%%\bin^) & pause & exit /b 1 )

echo ============================================================
echo  SUMO-CARLA on SimpleLoop: %VARIANT%   (python: %PY%)
echo ============================================================

REM --- stale FIXS-side processes (a stale Python bridge dies with TrafficLayer) --
taskkill /F /IM TrafficLayer.exe >nul 2>&1
taskkill /F /IM VirCarlaEnv.exe  >nul 2>&1
taskkill /F /IM sumo-gui.exe     >nul 2>&1
taskkill /F /IM sumo.exe         >nul 2>&1

REM --- l0_elevation: regenerate the hilly road, so SUMO and CARLA share it ------
if /i "%VARIANT%"=="l0_elevation" (
    echo [0] Regenerating terrain: 3 hills x 3 m ...
    "%PY%" "%RepoRoot%\tests\Vissim\SimpleEcho\gen_junction_loop_xodr.py" 15 6.0 3 3
    if errorlevel 1 ( echo ERROR: terrain generation failed. & pause & exit /b 1 )
    if not exist "%NETS%\simple_loop_elevation" mkdir "%NETS%\simple_loop_elevation"
    netconvert --opendrive-files "%XODR%" --geometry.max-segment-length 10 --output-file "%NETS%\simple_loop_elevation\simple_loop_elevation.net.xml"
    if errorlevel 1 ( echo ERROR: netconvert xodr-^>net failed. & pause & exit /b 1 )
)
if not exist "%SUMOCFG%" ( echo ERROR: SUMO scenario missing: %SUMOCFG% & pause & exit /b 1 )
if not exist "%XODR%"    ( echo ERROR: road missing: %XODR% & pause & exit /b 1 )

REM --- CARLA + the road ---------------------------------------------------------
if defined SKIP_CARLA (
    echo [1] SKIP_CARLA set -- assuming CARLA is up with the road loaded.
) else (
    REM A server left in synchronous mode with nothing ticking keeps its port open
    REM and stops answering, so start from a clean one.
    echo [1] Closing any running CARLA, then starting the configured one...
    taskkill /F /IM CarlaUE4-Win64-Shipping.exe >nul 2>&1
    taskkill /F /IM CarlaUE4.exe  >nul 2>&1
    taskkill /F /IM UE4Editor.exe >nul 2>&1
    REM ping, not timeout: with Git's tools on PATH "timeout" is GNU coreutils.
    ping -n 3 127.0.0.1 >nul
    "%PY%" "%CARLADIR%\load_opendrive_world.py" "%XODR%" --launch --port %CARLA_PORT% --sync --delta %DELTA% --timeout 300
    if errorlevel 1 ( echo ERROR: CARLA did not come up with the road loaded. & pause & exit /b 1 )
)

REM --- SUMO, TrafficLayer, the variant's client, the bridge --------------------
echo [2] SUMO (TraCI on 1337)...
start "FIXS SUMO" sumo-gui -c "%SUMOCFG%" --remote-port 1337 --step-length 0.1 %SEED% --start
echo [3] TrafficLayer...
start "FIXS TrafficLayer" cmd /k ""%TL%" -f "%CONFIG%""
if defined EXTRA (
    echo [4] %EXTRA% ...
    start "FIXS client" cmd /k ""%PY%" "%EXTRA%" "%CONFIG%""
)
echo [5] Carla bridge...
if /i "%RS_BRIDGE%"=="cpp" (
    start "FIXS bridge (C++)" cmd /k ""%VCE%" -f "%CONFIG%" -t "%TLS%""
) else (
    start "FIXS bridge" cmd /k ""%PY%" -u "%PYBRIDGE%" -f "%CONFIG%" -t "%TLS%""
)

echo.
echo  Launched. Stop: close the bridge, client and SUMO windows, Ctrl+C TrafficLayer, close CARLA.
echo.
pause
