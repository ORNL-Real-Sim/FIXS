@echo off
REM Vissim/SimpleEcho — orchestrate TrafficLayer + VISSIM (Python COM) + echo client
REM
REM Order:
REM   1. Launch TrafficLayer (waits for VISSIM driver model DLL to connect on 1337)
REM   2. Launch VISSIM via Python COM (loads network, triggers DLL load)
REM   3. Run Python echo client (subscribes to TrafficLayer on application port)

setlocal
set RealSimPath=..\..\..
set configFilename=config.yaml

echo [1/3] Starting TrafficLayer.exe ...
start "TrafficLayer" cmd /c %RealSimPath%\TrafficLayer\x64\Release\TrafficLayer.exe -f %configFilename%

echo Waiting 3s for TrafficLayer to bind ports ...
timeout /t 3 /nobreak >nul

echo [2/3] Starting VISSIM via Python COM ...
REM Activate conda env if available, then launch start_vissim.py in its own window
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    start "VISSIM-bootstrap" cmd /c ""%USERPROFILE%\miniconda3\Scripts\activate.bat" realsim_dev && python start_vissim.py"
) else if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    start "VISSIM-bootstrap" cmd /c ""%USERPROFILE%\anaconda3\Scripts\activate.bat" realsim_dev && python start_vissim.py"
) else (
    start "VISSIM-bootstrap" cmd /c "conda activate realsim_dev && python start_vissim.py"
)

echo Waiting 8s for VISSIM to open + load network ...
timeout /t 8 /nobreak >nul

echo [3/3] Starting Python echo client ...
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    call %USERPROFILE%\miniconda3\Scripts\activate.bat realsim_dev
) else if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    call %USERPROFILE%\anaconda3\Scripts\activate.bat realsim_dev
) else (
    call conda activate realsim_dev
)
python simple_echo_client.py

pause
endlocal
