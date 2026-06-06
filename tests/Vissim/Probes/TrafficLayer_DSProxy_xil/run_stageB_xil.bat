@echo off
REM Stage B end-to-end probe (issue #158).
REM
REM Three-process orchestration:
REM   1. Stage PTV's shipped DS example .inpx to a writable copy.
REM   2. Launch TrafficLayer.exe in DSProxy mode (config has VissimDSProxySetup
REM      AND ApplicationSetup.VehicleSubscription, so TrafficLayer opens its
REM      application server socket and waits for a client).
REM   3. Launch the fake-CarMaker Python client that connects, sends an ego
REM      pose per tick, receives VehFullData_t + TrafficLightData_t messages.
REM
REM Expected output:
REM   TrafficLayer prints "client connected on port 2444", then per-tick
REM   summary with growing vehicle count and egos=1.
REM   fake_carmaker prints "connected", then per-25-tick summary lines.

setlocal
set HERE=%~dp0
set RepoRoot=%HERE%..\..\..\..
set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set PTV_DIR=C:\Program Files\PTV Vision\PTV Vissim 2022\API\DrivingSimulator_DLL\example\DrivingSimulatorTextClient\data
set STAGE=%HERE%stage_network

if not exist "%TL%" (
    echo ERROR: TrafficLayer.exe not found at %TL%
    echo Run scripts\dispatch\2_core_components.bat first.
    exit /b 1
)

if not exist "%PTV_DIR%\driving_simulator_test.inpx" (
    echo ERROR: PTV example .inpx not found at %PTV_DIR%
    exit /b 2
)

if not exist "%STAGE%" mkdir "%STAGE%"
copy /Y "%PTV_DIR%\driving_simulator_test.inpx" "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\driving_simulator_test.layx" "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\driving_simulator_test.fzp"  "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\driving_simulator_test.pp"   "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\CARRE4E_RO_500_1.sig"        "%STAGE%\"  >nul

echo [1/2] Launching TrafficLayer in DSProxy mode (config: %HERE%config.yaml)
start "TrafficLayer (DSProxy mode)" cmd /c ""%TL%" -f "%HERE%config.yaml""

echo Waiting 4s for TrafficLayer to bind socket 2444 ...
timeout /t 4 /nobreak >nul

echo [2/2] Launching fake_carmaker.py
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    call "%USERPROFILE%\miniconda3\Scripts\activate.bat" realsim_dev
) else (
    call conda activate realsim_dev
)
python "%HERE%fake_carmaker.py"
exit /b %ERRORLEVEL%
