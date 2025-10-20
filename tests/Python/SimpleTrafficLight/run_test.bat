@echo off
REM Simple Traffic Light Test - TrafficLayer with Python Echo Client
REM Uses simple SUMO network with signalized intersections

set TrafficLayerPath=C:\src_git\RS_FIXS\#51\TrafficLayer\x64\Debug
set configFilename=config.yaml

REM Start SUMO with the traffic light test scenario
echo Starting SUMO with traffic light test scenario...
start sumo-gui -c simple_traffic_light.sumocfg --remote-port 1337 --step-length 0.1 --start --num-clients 1

echo Waiting for SUMO to initialize...
timeout /t 2 /nobreak

REM Start TrafficLayer.exe
echo Starting TrafficLayer.exe...
start cmd /c "%TrafficLayerPath%\TrafficLayer.exe" -f %configFilename%

echo Waiting for TrafficLayer to initialize...
timeout /t 3 /nobreak

REM Start Python Echo Client
echo Starting Python Echo Client...
REM Initialize conda
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    call %USERPROFILE%\miniconda3\Scripts\activate.bat
    call conda activate realsim
) else if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    call %USERPROFILE%\anaconda3\Scripts\activate.bat
    call conda activate realsim
) else (
    call conda activate realsim
)

python echo_client.py

call conda deactivate

pause