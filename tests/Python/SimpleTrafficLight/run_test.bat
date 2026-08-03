@echo off
REM Simple Traffic Light Test - TrafficLayer with Python Echo Client
REM Uses simple SUMO network with signalized intersections

set RealSimPath=..\..\..
set TestPath=.
set configFilename=config.yaml

REM Check if auto-launch is enabled in config
echo Checking config for auto-launch setting...
findstr /C:"EnableAutoLaunch: true" %configFilename% >nul
if %errorlevel%==0 (
    echo [CONFIG] Auto-launch ENABLED - TrafficLayer will start SUMO automatically
    echo [CONFIG] Skipping manual SUMO launch
    goto :skip_sumo
)

REM Start SUMO with the traffic light test scenario
echo [CONFIG] Auto-launch DISABLED - Starting SUMO manually...
start sumo-gui -c %TestPath%\simple_traffic_light.sumocfg --start --remote-port 1337 --step-length 0.1 --num-clients 1

echo Waiting for SUMO to initialize...
timeout /t 2 /nobreak

:skip_sumo
REM Start TrafficLayer.exe
echo Starting TrafficLayer.exe...
if exist "%RealSimPath%\TrafficLayer\x64\Release\TrafficLayer.exe" (
    start cmd /c "%RealSimPath%\TrafficLayer\x64\Release\TrafficLayer.exe" -f %configFilename%
) else if exist "%RealSimPath%\TrafficLayer\x64\Debug\TrafficLayer.exe" (
    start cmd /c "%RealSimPath%\TrafficLayer\x64\Debug\TrafficLayer.exe" -f %configFilename%
) else (
    echo [ERROR] Could not find TrafficLayer.exe in Release or Debug under %RealSimPath%\TrafficLayer\x64
    goto :end
)

echo Waiting for TrafficLayer to initialize...
timeout /t 3 /nobreak

REM Start Python Echo Client. Resolve an env python directly (no `conda activate`,
REM which is fragile); the client only needs PyYAML + repo CommonLib. Override with
REM   set PYTHON=C:\full\path\python.exe
echo Starting Python Echo Client...
if not defined PYTHON (
    for %%P in (
        "%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe"
        "%USERPROFILE%\anaconda3\envs\realsim_dev\python.exe"
        "%USERPROFILE%\miniconda3\envs\realsim\python.exe"
        "%USERPROFILE%\anaconda3\envs\realsim\python.exe"
    ) do if exist %%~P set "PYTHON=%%~P"
)
if not defined PYTHON set "PYTHON=python"
echo Using Python: %PYTHON%
"%PYTHON%" %TestPath%\echo_client.py

:end
pause
