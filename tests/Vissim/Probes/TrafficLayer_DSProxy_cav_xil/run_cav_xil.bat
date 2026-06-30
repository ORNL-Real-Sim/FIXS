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

echo [1/3] Patch .inpx (hook FIXS DriverModel on Car type, EnableRealSim: true)
"%PYEXE%" "%HERE%patch_inpx.py" || exit /b 4

echo [2/3] Launch TrafficLayer in DSProxy mode + DriverModel relay
start "TrafficLayer (DSProxy + DM relay)" /B cmd /c ""%TL%" -f "%HERE%config.yaml" > "%HERE%tl.log" 2>&1"

echo Waiting 20s for VISSIM startup + DM connect + app socket bind ...
ping 127.0.0.1 -n 21 >nul

echo [3/3] Launch python_ego_cav.py
"%PYEXE%" "%HERE%python_ego_cav.py"
set RC=%ERRORLEVEL%

ping 127.0.0.1 -n 3 >nul
taskkill /F /IM TrafficLayer.exe >nul 2>&1
exit /b %RC%
