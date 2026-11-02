@echo off
REM Integrated regression test for FIXS#158 Stages B + C.
REM
REM Orchestrates:
REM   1. Patch the staged .inpx to attach FIXS DriverModel to Car (type 100)
REM      with EnableRealSim: false (coexist_par.yaml).
REM   2. Write config.runtime.yaml (gitignored) with NetworkFile resolved to this
REM      checkout's stage_network, and launch TrafficLayer.exe in DSProxy mode on it.
REM   3. Launch python_ego.py — connects to TL, runs an egoctrl-style
REM      deterministic trajectory, captures invariants, writes summary.json.

setlocal
set HERE=%~dp0
set RepoRoot=%HERE%..\..\..\..
set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set DM=%RepoRoot%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim.dll

if not exist "%TL%" (
    echo ERROR: TrafficLayer.exe not found at %TL%
    echo Run: scripts\dispatch\2_core_components.bat
    exit /b 1
)
if not exist "%DM%" (
    echo ERROR: FIXS DriverModel_RealSim.dll not found at %DM%
    echo Run: scripts\dispatch\3_vissim_components.bat
    exit /b 2
)

REM Resolve Python.
set "PYEXE="
if exist "%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe" set "PYEXE=%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe"
if "%PYEXE%"=="" (
    echo ERROR: realsim_dev python.exe not found
    exit /b 3
)

echo [1/4] Patching .inpx to attach FIXS DriverModel on Car ^(type 100^)
"%PYEXE%" "%HERE%patch_inpx.py"
if errorlevel 1 exit /b 4

echo [2/4] Resolve NetworkFile to this checkout
set CFG=%HERE%config.runtime.yaml
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$inpx = Join-Path '%HERE%stage_network' 'driving_simulator_test.inpx';" ^
  "(Get-Content '%HERE%config.yaml') -replace \"^(\s*NetworkFile:\s*).*$\", \"`$1'$inpx'\" | Set-Content -Encoding ascii '%CFG%'"
if not exist "%CFG%" ( echo ERROR: failed to generate %CFG% & exit /b 5 )

echo [3/4] Launching TrafficLayer in DSProxy mode
REM Redirect TL's stdout/stderr to a file so we can debug post-mortem.
start "TrafficLayer (DSProxy + DM coexist)" /B cmd /c ""%TL%" -f "%CFG%" > "%HERE%tl.log" 2>&1"

echo Waiting 18s for VISSIM startup + DSProxy handshake + socket bind ...
ping 127.0.0.1 -n 19 >nul

echo [4/4] Launching python_ego.py
"%PYEXE%" "%HERE%python_ego.py"
set RC=%ERRORLEVEL%

REM TrafficLayer exits on its own (end time or startup error) and closes VISSIM; never kill it mid-COM.
for /L %%i in (1,1,30) do (
    findstr /C:"=== DSProxy mode done" /C:"ERROR:" "%HERE%tl.log" >nul 2>&1 && goto :tl_done
    ping 127.0.0.1 -n 2 >nul
)
echo WARNING: TrafficLayer has not finished; see %HERE%tl.log
:tl_done
exit /b %RC%
