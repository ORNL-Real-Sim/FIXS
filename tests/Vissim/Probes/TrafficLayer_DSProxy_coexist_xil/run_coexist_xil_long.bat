@echo off
REM Long-duration variant of the integrated B+C regression test (#158).
REM Runs the same Stage B+C pipeline as run_coexist_xil.bat but for 5 min
REM (3000 ticks @ 10 Hz). Used to validate long-haul connection +
REM protocol stability.

setlocal
set HERE=%~dp0
set RepoRoot=%HERE%..\..\..\..
set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set DM=%RepoRoot%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim.dll

if not exist "%TL%" ( echo ERROR: TrafficLayer.exe missing & exit /b 1 )
if not exist "%DM%" ( echo ERROR: FIXS DriverModel DLL missing & exit /b 2 )

set "PYEXE="
if exist "%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe" set "PYEXE=%USERPROFILE%\miniconda3\envs\realsim_dev\python.exe"
if "%PYEXE%"=="" ( echo ERROR: realsim_dev python.exe not found & exit /b 3 )

echo [1/3] Patching .inpx to attach FIXS DriverModel on Car (type 100)
"%PYEXE%" "%HERE%patch_inpx.py" || exit /b 4

echo [2/3] Launch TrafficLayer in DSProxy mode (5-min run)
start "TrafficLayer (DSProxy + DM coexist, long)" /B cmd /c ""%TL%" -f "%HERE%config_long.yaml" > "%HERE%tl_long.log" 2>&1"

echo Waiting 18s for VISSIM startup + DSProxy handshake + socket bind ...
ping 127.0.0.1 -n 19 >nul

echo [3/3] Launch python_ego_long.py (3000 tick target = 5 min)
"%PYEXE%" "%HERE%python_ego_long.py"
set RC=%ERRORLEVEL%

ping 127.0.0.1 -n 3 >nul
taskkill /F /IM TrafficLayer.exe >nul 2>&1
exit /b %RC%
