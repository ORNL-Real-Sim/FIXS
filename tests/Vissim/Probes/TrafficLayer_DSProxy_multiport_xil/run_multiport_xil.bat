@echo off
REM Stage B multi-port routing probe (issue #158, PR #165).
REM
REM   1. Stage PTV's shipped DS example .inpx to stage_network/.
REM   2. Resolve config.yaml's NetworkFile to THIS checkout's stage_network
REM      and write config.runtime.yaml (gitignored). VissimSetup.NetworkFile
REM      must be absolute, so the committed config cannot be checkout-portable
REM      on its own — the generated copy is what TrafficLayer actually reads.
REM   3. Launch TrafficLayer in DSProxy mode with TWO VehicleSubscription
REM      entries (ports 2444 + 2445).
REM   4. Launch multiport_clients.py — opens both client sockets in
REM      threads and validates per-port filtering.

setlocal
set HERE=%~dp0
set RepoRoot=%HERE%..\..\..\..
set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set PTV_DIR=C:\Program Files\PTV Vision\PTV Vissim 2022\API\DrivingSimulator_DLL\example\DrivingSimulatorTextClient\data
set STAGE=%HERE%stage_network

if not exist "%TL%" ( echo ERROR: TrafficLayer.exe missing & exit /b 1 )
if not exist "%PTV_DIR%\driving_simulator_test.inpx" ( echo ERROR: PTV example .inpx missing & exit /b 2 )

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

echo [2/3] Launch TrafficLayer in DSProxy mode (multi-port)
start "TrafficLayer (multi-port)" /B cmd /c ""%TL%" -f "%CFG%" > "%HERE%tl.log" 2>&1"

echo Waiting 18s for VISSIM startup + ports 2444+2445 bind ...
ping 127.0.0.1 -n 19 >nul

echo [3/3] Launch multiport_clients.py
"%PYEXE%" "%HERE%multiport_clients.py"
set RC=%ERRORLEVEL%

ping 127.0.0.1 -n 3 >nul
taskkill /F /IM TrafficLayer.exe >nul 2>&1
exit /b %RC%
