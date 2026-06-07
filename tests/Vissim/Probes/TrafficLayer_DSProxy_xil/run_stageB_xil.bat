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
REM Capture TL stdout to tl.log so the Python invariant check can
REM verify the 'ego registered: VISSIM VehicleID=...' line.
start "TrafficLayer (DSProxy mode)" /B cmd /c ""%TL%" -f "%HERE%config.yaml" > "%HERE%tl.log" 2>&1"

echo Waiting 18s for VISSIM startup + DSProxy handshake + socket bind ...
ping 127.0.0.1 -n 19 >nul

echo [2/2] Launching fake_carmaker.py
set "PYEXE="
if exist "%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe" set "PYEXE=%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe"
if "%PYEXE%"=="" ( echo ERROR: realsim_dev python.exe not found & exit /b 3 )
"%PYEXE%" "%HERE%fake_carmaker.py"
set RC=%ERRORLEVEL%

ping 127.0.0.1 -n 3 >nul
taskkill /F /IM TrafficLayer.exe >nul 2>&1
exit /b %RC%
