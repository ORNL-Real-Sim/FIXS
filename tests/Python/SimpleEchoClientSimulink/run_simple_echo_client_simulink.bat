@echo off
REM Simple Echo Client with Simulink - Batch script to run SUMO simulation, Python client, and Simulink XIL

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

echo [CONFIG] Auto-launch DISABLED - Starting SUMO manually...
start sumo-gui -c %TestPath%\simple_loop.sumocfg --start --remote-port 1337 --step-length 0.1

echo Waiting for SUMO to initialize...
timeout /t 2 /nobreak

:skip_sumo
echo Starting TrafficLayer.exe...
start cmd /c %RealSimPath%\TrafficLayer\x64\Debug\TrafficLayer.exe -f %configFilename%

echo Waiting for TrafficLayer to initialize...
timeout /t 3 /nobreak

echo.
echo ========================================
echo MANUAL STEP REQUIRED:
echo Please start your Simulink XIL model now
echo The model should connect to port 2445
echo ========================================
echo.
echo Press any key once Simulink is running...
pause

echo Starting Python Echo Client...
REM Initialize conda if not already initialized
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    call %USERPROFILE%\miniconda3\Scripts\activate.bat realsim
) else if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    call %USERPROFILE%\anaconda3\Scripts\activate.bat realsim
) else (
    call conda activate realsim
)
python %TestPath%\simple_echo_client_simulink.py

pause
