@echo off
REM Stage B+ probe: Python ego + Python CAV + signal validation
REM through TrafficLayer with FIXS DriverModel relay.
setlocal
set HERE=%~dp0
set RepoRoot=%HERE%..\..\..\..
set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set DM=%RepoRoot%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim.dll

if not exist "%TL%" ( echo ERROR: TrafficLayer.exe missing & exit /b 1 )
if not exist "%DM%" ( echo ERROR: DriverModel DLL missing & exit /b 2 )

set "PYEXE="
if exist "%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe" set "PYEXE=%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe"
if "%PYEXE%"=="" ( echo ERROR: realsim_dev python.exe not found & exit /b 3 )

echo [1/4] Patch .inpx (hook FIXS DriverModel on Car type, EnableRealSim: true)
"%PYEXE%" "%HERE%patch_inpx.py" || exit /b 4

echo [2/4] Resolve NetworkFile to this checkout
set CFG=%HERE%config.runtime.yaml
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$inpx = Join-Path '%HERE%stage_network' 'driving_simulator_test.inpx';" ^
  "(Get-Content '%HERE%config.yaml') -replace \"^(\s*NetworkFile:\s*).*$\", \"`$1'$inpx'\" | Set-Content -Encoding ascii '%CFG%'"
if not exist "%CFG%" ( echo ERROR: failed to generate %CFG% & exit /b 5 )

echo [3/4] Launch TrafficLayer in DSProxy mode + DriverModel relay
start "TrafficLayer (DSProxy + DM relay)" /B cmd /c ""%TL%" -f "%CFG%" > "%HERE%tl.log" 2>&1"

echo Waiting 20s for VISSIM startup + DM connect + app socket bind ...
ping 127.0.0.1 -n 21 >nul

echo [4/4] Launch python_ego_cav.py
"%PYEXE%" "%HERE%python_ego_cav.py"
set RC=%ERRORLEVEL%

REM TrafficLayer exits on its own (end time or startup error) and closes VISSIM; never kill it mid-COM.
for /L %%i in (1,1,30) do (
    findstr /C:"=== DSProxy mode done" /C:"ERROR:" "%HERE%tl.log" >nul 2>&1 && goto :tl_done
    ping 127.0.0.1 -n 2 >nul
)
echo WARNING: TrafficLayer has not finished; see %HERE%tl.log
:tl_done
exit /b %RC%
