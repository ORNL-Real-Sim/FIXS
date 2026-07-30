@echo off
REM Stage A end-to-end probe (issue #158).
REM
REM Stages PTV's shipped DS example .inpx into a writable copy under
REM stage_network\, then launches TrafficLayer.exe in DSProxy mode against
REM that staged copy. TrafficLayer spawns VISSIM via DrivingSimulatorProxy.dll
REM and ticks through 30s of simulation at 10 Hz, printing a per-tick summary.
REM
REM No FIXS-side socket traffic — Stage A only proves the new code path
REM drives VISSIM and reads back state. CarMaker / DriverModel / Python
REM client integration come in Stages B-D of #158.

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
    echo Verify VISSIM 2022 install and the DrivingSimulator_DLL sample.
    exit /b 2
)

REM Stage PTV's shipped example .inpx + companions into a writable copy.
REM VISSIM writes lock files and temp data next to the .inpx; the install
REM tree is read-only without admin, so we mirror to %STAGE% first.
if not exist "%STAGE%" mkdir "%STAGE%"
copy /Y "%PTV_DIR%\driving_simulator_test.inpx" "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\driving_simulator_test.layx" "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\driving_simulator_test.fzp"  "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\driving_simulator_test.pp"   "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\CARRE4E_RO_500_1.sig"        "%STAGE%\"  >nul

echo Staged shipped example .inpx into %STAGE%
echo Launching TrafficLayer in DSProxy mode (config: %HERE%config.yaml)
"%TL%" -f "%HERE%config.yaml"
exit /b %ERRORLEVEL%
