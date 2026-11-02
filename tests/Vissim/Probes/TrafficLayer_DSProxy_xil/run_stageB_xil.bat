@echo off
REM Stage B end-to-end probe (issue #158).
REM
REM   1. Stage PTV's shipped DS example .inpx to stage_network\.
REM   2. Write config.runtime.yaml (gitignored) with NetworkFile resolved to this
REM      checkout's stage_network; the committed config.yaml holds a placeholder.
REM   3. Launch TrafficLayer.exe in DSProxy mode on the runtime config (tl.log).
REM   4. Launch the fake-CarMaker client: one ego pose out, vehicles + signals in, per tick.
REM
REM Pass: tl.log shows "=== DSProxy mode", "ego registered: VISSIM VehicleID=N"
REM and per-tick vehicles > 0 with egos=1.

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

set "PYEXE="
if exist "%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe" set "PYEXE=%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe"
if "%PYEXE%"=="" ( echo ERROR: realsim_dev python.exe not found & exit /b 3 )

if not exist "%STAGE%" mkdir "%STAGE%"
copy /Y "%PTV_DIR%\driving_simulator_test.inpx" "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\driving_simulator_test.layx" "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\driving_simulator_test.fzp"  "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\driving_simulator_test.pp"   "%STAGE%\"  >nul
copy /Y "%PTV_DIR%\CARRE4E_RO_500_1.sig"        "%STAGE%\"  >nul

echo [1/3] Resolve NetworkFile to this checkout
set CFG=%HERE%config.runtime.yaml
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$inpx = Join-Path '%STAGE%' 'driving_simulator_test.inpx';" ^
  "(Get-Content '%HERE%config.yaml') -replace \"^(\s*NetworkFile:\s*).*$\", \"`$1'$inpx'\" | Set-Content -Encoding ascii '%CFG%'"
if not exist "%CFG%" ( echo ERROR: failed to generate %CFG% & exit /b 4 )

echo [2/3] Launching TrafficLayer in DSProxy mode (log: %HERE%tl.log)
start "TrafficLayer (DSProxy mode)" /B cmd /c ""%TL%" -f "%CFG%" > "%HERE%tl.log" 2>&1"

echo Waiting 4s for TrafficLayer to bind socket 2444 ...
ping 127.0.0.1 -n 5 >nul

echo [3/3] Launching fake_carmaker.py
"%PYEXE%" "%HERE%fake_carmaker.py"
set RC=%ERRORLEVEL%

REM TrafficLayer exits on its own (end time or startup error) and closes VISSIM; never kill it mid-COM.
for /L %%i in (1,1,30) do (
    findstr /C:"=== DSProxy mode done" /C:"ERROR:" "%HERE%tl.log" >nul 2>&1 && goto :tl_done
    ping 127.0.0.1 -n 2 >nul
)
echo WARNING: TrafficLayer has not finished; see %HERE%tl.log
:tl_done
findstr /C:"ego registered: VISSIM VehicleID=" "%HERE%tl.log" >nul 2>&1 || (
    echo FAIL: ego never registered in VISSIM; see %HERE%tl.log
    if %RC%==0 set RC=5
)
exit /b %RC%
