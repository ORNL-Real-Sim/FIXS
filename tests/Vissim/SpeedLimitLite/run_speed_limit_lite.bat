@echo off
REM Vissim/SpeedLimitLite — Python-only twin of ../SpeedLimit/runSpeedLimit_test1.bat
REM
REM Stack: TrafficLayer + VISSIM (via Python COM) + Python client.
REM No MATLAB. No Simulink. No .mat regression check.

setlocal
set RealSimPath=..\..\..
set configFilename=config.yaml

echo [1/3] Starting TrafficLayer.exe ...
start "TrafficLayer" cmd /c %RealSimPath%\TrafficLayer\x64\Release\TrafficLayer.exe -f %configFilename%

echo Waiting 3s for TrafficLayer to bind ports ...
timeout /t 3 /nobreak >nul

echo [2/3] Starting VISSIM via Python COM (with ego injection at t=11.5s) ...
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    start "VISSIM-bootstrap" cmd /c ""%USERPROFILE%\miniconda3\Scripts\activate.bat" realsim_dev && python start_vissim.py"
) else if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    start "VISSIM-bootstrap" cmd /c ""%USERPROFILE%\anaconda3\Scripts\activate.bat" realsim_dev && python start_vissim.py"
) else (
    start "VISSIM-bootstrap" cmd /c "conda activate realsim_dev && python start_vissim.py"
)

echo Waiting 8s for VISSIM to open + load network ...
timeout /t 8 /nobreak >nul

echo [3/3] Starting Python speed-limit client (logs to speed_limit_lite_trace.csv) ...
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    call %USERPROFILE%\miniconda3\Scripts\activate.bat realsim_dev
) else if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    call %USERPROFILE%\anaconda3\Scripts\activate.bat realsim_dev
) else (
    call conda activate realsim_dev
)
python speed_limit_client.py

pause
endlocal
