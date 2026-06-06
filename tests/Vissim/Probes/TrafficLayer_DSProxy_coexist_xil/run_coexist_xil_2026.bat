@echo off
REM VISSIM 2026 variant of the integrated B+C regression test (#158).
REM Same orchestration as run_coexist_xil.bat, but stages from the 2026
REM install dir and runs TrafficLayer against config_2026.yaml.

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

echo [1/3] Patching .inpx ^(VISSIM 2026 install^) with FIXS DriverModel on Car
"%PYEXE%" "%HERE%patch_inpx.py" --vissim-version 2026
if errorlevel 1 exit /b 4

echo [2/3] Launching TrafficLayer in DSProxy mode against config_2026.yaml
start "TrafficLayer (DSProxy 2026 + DM coexist)" /B cmd /c ""%TL%" -f "%HERE%config_2026.yaml" > "%HERE%tl_2026.log" 2>&1"

echo Waiting 22s for VISSIM 2026 startup ...
ping 127.0.0.1 -n 23 >nul

echo [3/3] Launching python_ego.py against config_2026.yaml
"%PYEXE%" "%HERE%python_ego.py" --config config_2026.yaml
set RC=%ERRORLEVEL%

ping 127.0.0.1 -n 3 >nul
taskkill /F /IM TrafficLayer.exe >nul 2>&1
exit /b %RC%
