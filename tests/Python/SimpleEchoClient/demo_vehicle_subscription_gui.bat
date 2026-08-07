@echo off
REM ===========================================================================
REM  LIVE demo of the #176 `all` vehicle subscription.
REM  Opens three windows so you can watch the fix work:
REM    1. SUMO-GUI            -- ~30 cars driving the loop
REM    2. TrafficLayer        -- prints every vehicle it pulls from SUMO and the
REM                              count it sends to the client  ("send client veh: 30")
REM    3. Echo client         -- prints "Received 30 vehicles" + each vehicle, then
REM                              echoes them back (proves the round-trip / framing fix)
REM
REM  Contrast: with the OLD code (or the `all` flag dead) the client would print
REM  Received 0/1; with the fix it prints ~30 (every car SUMO is simulating).
REM
REM  Prereqs: SUMO on PATH, TrafficLayer.exe built (scripts\dispatch\2_core_components.bat).
REM  Set RS_ENV if your conda env is not 'realsim_dev'.  Close the windows when done.
REM ===========================================================================
setlocal
set "HERE=%~dp0"
if "%RS_ENV%"=="" set "RS_ENV=realsim_dev"
set "EXE=%HERE%..\..\..\TrafficLayer\x64\Release\TrafficLayer.exe"
set "LIBSUMO=%HERE%..\..\..\CommonLib\libsumo\bin"
set "RSPY=%USERPROFILE%\miniconda3\envs\%RS_ENV%\python.exe"
if not exist "%RSPY%" set "RSPY=%USERPROFILE%\anaconda3\envs\%RS_ENV%\python.exe"

if not exist "%EXE%" (
    echo ERROR: TrafficLayer.exe not found at %EXE%
    echo Build it first:  scripts\dispatch\2_core_components.bat
    pause & exit /b 1
)
if not exist "%RSPY%" (
    echo ERROR: python for env '%RS_ENV%' not found at %RSPY%
    echo Set RS_ENV to your conda env name and retry.
    pause & exit /b 1
)

REM clean any leftover processes from a previous run
taskkill /F /IM sumo-gui.exe >nul 2>&1
taskkill /F /IM sumo.exe >nul 2>&1
taskkill /F /IM TrafficLayer.exe >nul 2>&1

REM make the libsumo DLLs visible to TrafficLayer (child windows inherit this PATH)
set "PATH=%LIBSUMO%;%PATH%"

echo Launching SUMO-GUI...   (it loads, then waits for TrafficLayer to connect)
start "SUMO-GUI (#176 demo)" sumo-gui -c "%HERE%simple_loop_traffic.sumocfg" --remote-port 1337 --num-clients 1 --step-length 0.1 --start
timeout /t 3 /nobreak >nul

echo Launching TrafficLayer (config_demo_all.yaml, verbose)...
start "TrafficLayer (#176 ALL)" /D "%HERE%" cmd /k ""%EXE%" -f config_demo_all.yaml"
timeout /t 4 /nobreak >nul

echo Launching echo client...
start "Echo Client (#176 ALL)" /D "%HERE%" cmd /k ""%RSPY%" simple_echo_client.py --config config_demo_all.yaml"

echo.
echo ===========================================================================
echo  Three windows are starting. Watch:
echo    - SUMO-GUI:      ~30 cars driving the loop (blue) plus ego (red)
echo    - TrafficLayer:  "send client veh: 30"  + per-vehicle lines
echo    - Echo Client:   "Received 30 vehicles"  each step
echo.
echo  Close the three windows when you are done.
echo ===========================================================================
endlocal
