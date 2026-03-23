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

REM Start Python Echo Client
echo Starting Python Echo Client...
REM Initialize conda if not already initialized
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    call %USERPROFILE%\miniconda3\Scripts\activate.bat realsim
) else if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    call %USERPROFILE%\anaconda3\Scripts\activate.bat realsim
) else (
    call conda activate realsim
)

python %TestPath%\echo_client.py

call conda deactivate

:end
pause
