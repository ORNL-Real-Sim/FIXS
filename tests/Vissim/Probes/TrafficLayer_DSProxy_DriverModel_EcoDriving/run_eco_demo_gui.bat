@echo off
REM ============================================================================
REM  FIXS #173 - Eco-driving CAV demo (GUI).
REM  CarMaker (office) <-> VISSIM via DSProxy, WITH the FIXS DriverModel co-loaded
REM  (Stage C co-load) on the signalized corridor (simple_traffic_light).
REM
REM  A fraction of VISSIM background Cars are DriverModel-hooked CAVs whose desired
REM  speed is set every tick by a GLOSA eco-driving controller; the CarMaker ego is
REM  the XIL vehicle you watch. Four processes:
REM    VISSIM (spawned by TrafficLayer) + TrafficLayer (DSProxy + DM relay)
REM    + cav_controller.py (the eco brain) + CarMaker Office GUI.
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. It:
REM    1. builds the SHARED Demo 2 assets (rd5 + signal table + Cosim TestRun),
REM    2. generates signals.json (fixed green windows from the plan),
REM    3. (setup_eco.py) stages the network, HOOKS the FIXS DriverModel on Car
REM       type 100, writes config.runtime.yaml, registers the CM exe + args,
REM    4. launches TrafficLayer (DSProxy) -> spawns VISSIM, serves 2444/2445/1337/1338,
REM    5. launches the eco-driving CAV controller (connects on 1338),
REM    6. opens the CarMaker Office GUI.
REM
REM  THEN in the CarMaker GUI (after TrafficLayer prints "VISSIM_Connect OK",
REM  "signal listener bound on port 2445", "CAV-controller listener bound on port 1338",
REM  and "DriverModel connected"):
REM    - Load TestRun "SimpleTrafficLight_Cosim"
REM    - Press the green START button.
REM
REM  PREREQUISITES (see README):
REM    - DriverModel_RealSim.dll built (scripts\dispatch\3_vissim_components.bat)
REM    - The relay actuation smoke test has passed once (cav_xil, SubEgoOnly:false)
REM    - signals.json positions calibrated (otherwise CAVs free-flow, no eco shaping)
REM
REM  Stop order: Stop in CarMaker -> close the controller window -> close VISSIM ->
REM  Ctrl+C TrafficLayer. (Never taskkill VISSIM while it holds the .inpx.)
REM ============================================================================

setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI
set DEMO2=%RepoRoot%\tests\Vissim\Probes\TrafficLayer_DSProxy_CMoffice_Signal

REM Optional arg 1: eco penetration rate (0.0 .. 1.0). Default 1.0 (full eco).
REM Run once with 1.0 (eco) and once with 0.0 (baseline), then verify_eco.py
REM compares out/summary_eco.json vs out/summary_baseline.json.
set PEN=%1
if "%PEN%"=="" set PEN=1.0

set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set DM=%RepoRoot%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim.dll
set CMEXE=%RepoRoot%\ProprietaryFiles\CM13_proj\src\CarMaker.win64.exe
set CMPROJ=%RepoRoot%\ProprietaryFiles\CM13_proj
set CM_OFFICE=C:\IPG\carmaker\win64-13.1.3\bin\CM_Office.exe
set TESTRUN=SimpleTrafficLight_Cosim
set RUNCFG=%HERE%config.runtime.yaml
set PYTHON=C:\Users\yshao\miniconda3\envs\realsim_dev\python.exe
if not exist "%PYTHON%" set PYTHON=python

echo ============================================================
echo  FIXS #173 Eco-driving CAV demo (DSProxy + DriverModel co-load)
echo ============================================================

echo Closing any stale CarMaker / TrafficLayer / controller so the GUI starts fresh...
taskkill /F /IM CM_Office.exe        >nul 2>&1
taskkill /F /IM HIL.exe              >nul 2>&1
taskkill /F /IM Movie.exe            >nul 2>&1
taskkill /F /IM CarMaker.win64.exe   >nul 2>&1
taskkill /F /IM TrafficLayer.exe     >nul 2>&1

REM --- preflight -----------------------------------------------------------
if not exist "%TL%"        ( echo ERROR: TrafficLayer.exe missing: %TL%  ^(build: scripts\dispatch\2_core_components.bat^) & pause & exit /b 1 )
if not exist "%DM%"        ( echo ERROR: DriverModel_RealSim.dll missing: %DM%  ^(build: scripts\dispatch\3_vissim_components.bat^) & pause & exit /b 1 )
if not exist "%CMEXE%"     ( echo ERROR: custom CarMaker.win64.exe missing: %CMEXE% & pause & exit /b 1 )
if not exist "%CM_OFFICE%" ( echo ERROR: CM_Office.exe not found: %CM_OFFICE% & pause & exit /b 1 )

REM --- 1. build SHARED Demo 2 assets (rd5 + signal table + Cosim TestRun) ----
echo [1/6] Building shared Demo 2 assets (rd5 + signal table + SimpleTrafficLight_Cosim)...
"%PYTHON%" "%DEMO2%\add_signal_stops.py"    || ( echo add_signal_stops FAILED & pause & exit /b 1 )
"%PYTHON%" "%DEMO2%\build_signal_table.py"  || ( echo build_signal_table FAILED & pause & exit /b 1 )
"%PYTHON%" "%DEMO2%\build_cosim_testrun.py" || ( echo build_cosim_testrun FAILED & pause & exit /b 1 )

REM --- 2. generate signals.json (fixed green windows from the plan) ---------
echo [2/6] Generating signals.json (fixed signal timing)...
"%PYTHON%" "%HERE%build_eco_signals.py" || ( echo build_eco_signals FAILED & pause & exit /b 1 )

REM --- 3. stage network + HOOK DriverModel + write config + register exe ----
echo [3/6] Preparing GUI (stage network, hook DriverModel on Car type 100, write config)...
"%PYTHON%" "%HERE%setup_eco.py"
if errorlevel 1 ( echo ERROR: setup_eco.py failed. & pause & exit /b 1 )

REM --- 3b. VISSIM COM self-heal preflight (Win11 24H2) ----------------------
echo [preflight] Checking VISSIM COM dispatch health...
set _VTRY=0
:vchk
"%PYTHON%" "%DEMO2%\vissim_dispatch_probe.py" >nul 2>&1
if not errorlevel 1 goto vok
set /a _VTRY+=1
if %_VTRY% geq 3 goto vbad
echo [preflight] VISSIM dispatch not responding (try %_VTRY%/3) -- clearing zombie VISSIM + stale locks...
taskkill /F /IM VISSIM220.exe            >nul 2>&1
del /Q "%TEMP%\VISSIM\*.lock"            >nul 2>&1
del /Q "%TEMP%\VISSIM\vissim_msgs*.txt"  >nul 2>&1
del /Q "%TEMP%\VISSIM\CommonDialogs.log" >nul 2>&1
timeout /t 2 /nobreak >nul
goto vchk
:vbad
echo [preflight] WARNING: VISSIM COM still unhealthy after cleanup. See CLAUDE.md soft reset.
goto vdone
:vok
echo [preflight] VISSIM dispatch OK.
:vdone

REM --- 4. launch TrafficLayer (spawns VISSIM, serves 2444/2445/1337/1338) ---
echo [4/6] Launching TrafficLayer (DSProxy + DriverModel relay)...
echo        (wait for "VISSIM_Connect OK", "signal listener bound on port 2445",
echo         "CAV-controller listener bound on port 1338", "DriverModel connected")
start "FIXS TrafficLayer (DSProxy + DM relay)" cmd /k "%TL% -f %RUNCFG%"

REM --- 5. launch the eco-driving CAV controller (connects on 1338) ----------
echo [5/6] Launching eco-driving CAV controller (GLOSA, port 1338, penetration %PEN%)...
start "FIXS eco CAV controller" cmd /k "%PYTHON% %HERE%cav_controller.py --config %HERE%config.yaml --signals %HERE%signals.json --penetration %PEN%"

REM --- 6. open CarMaker Office GUI on the project ---------------------------
echo [6/6] Opening CarMaker Office GUI...
start "" "%CM_OFFICE%" -projectdir "%CMPROJ%" "%TESTRUN%"

set MOVIE=C:\IPG\carmaker\win64-13.1.3\GUI\Movie.exe
if not exist "%MOVIE%" ( echo NOTE: Movie.exe not at %MOVIE% - open it manually & goto :skipmovie )
echo Waiting for the CarMaker GUI (HIL.exe) before launching IPGMovie...
set /a _w=0
:waitgui
timeout /t 1 /nobreak >nul
tasklist /fi "imagename eq HIL.exe" 2>nul | find /i "HIL.exe" >nul && goto :guiup
set /a _w+=1
if %_w% lss 40 goto :waitgui
:guiup
timeout /t 3 /nobreak >nul
start "" "%MOVIE%"
:skipmovie

echo.
echo ============================================================
echo  Launched. NOW in the CarMaker GUI:
echo    1) WAIT for TrafficLayer: "VISSIM_Connect OK" + "CAV-controller listener
echo       bound on port 1338" + "DriverModel connected"
echo    2) Load TestRun:  %TESTRUN%
echo    3) Press  START.
echo  The ego drives the corridor; DriverModel-hooked CAVs (RS_C cars) ease toward
echo  green per the eco-driving controller. Watch the controller window for
echo  "eco_active" counts.
echo ============================================================
echo.
pause
endlocal
