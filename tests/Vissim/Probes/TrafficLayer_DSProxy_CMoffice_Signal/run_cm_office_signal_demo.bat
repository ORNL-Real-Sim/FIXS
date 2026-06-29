@echo off
REM ============================================================================
REM  FIXS #172 - CarMaker (office) <-> VISSIM co-simulation demo WITH SIGNAL SYNC
REM  Signalized corridor (simple_traffic_light). Vehicles AND traffic signals.
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. It:
REM    1. builds the demo assets (rd5 + signal table + SimpleTrafficLight_Cosim TestRun),
REM    2. (via setup_gui.py) stages the signalized DS network, writes
REM       config.runtime.yaml, and registers the custom CarMaker exe +
REM       "-f config.runtime.yaml -s <RSsignalTable.csv>" into the CM GUI config,
REM    3. launches TrafficLayer (DSProxy) -> it spawns VISSIM, serves the vehicle
REM       port 2444 AND the signal port 2445 (Plan A), then WAITS for CarMaker,
REM    4. opens the CarMaker Office GUI on the project + IPGMovie.
REM
REM  THEN, in the CarMaker GUI (after TrafficLayer prints "VISSIM_Connect OK" and
REM  "signal listener bound on port 2445"):
REM    - Load TestRun "SimpleTrafficLight_Cosim"
REM    - Press the green START button.
REM  You'll see VISSIM background traffic as RS_C cars and the ego braking when
REM  its light (driven by VISSIM) turns red.
REM
REM  Stop order: Stop in CarMaker -> close VISSIM window -> Ctrl+C TrafficLayer.
REM  (Never taskkill VISSIM while it holds the .inpx -- leaks CodeMeter sessions.)
REM ============================================================================

setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI

set TL=%RepoRoot%\TrafficLayer\x64\Release\TrafficLayer.exe
set CMEXE=%RepoRoot%\ProprietaryFiles\CM13_proj\src\CarMaker.win64.exe
set CMPROJ=%RepoRoot%\ProprietaryFiles\CM13_proj
set CM_OFFICE=C:\IPG\carmaker\win64-13.1.3\bin\CM_Office.exe
set TESTRUN=SimpleTrafficLight_Cosim
set RUNCFG=%HERE%config.runtime.yaml
set PYTHON=C:\Users\yshao\miniconda3\envs\realsim_dev\python.exe
if not exist "%PYTHON%" set PYTHON=python

echo ============================================================
echo  FIXS #172 CarMaker-VISSIM co-simulation demo + SIGNAL SYNC
echo ============================================================

REM --- kill stale processes so the GUI launches FRESH with the right config ---
echo Closing any stale CarMaker / TrafficLayer / VISSIM so the GUI starts fresh...
taskkill /F /IM CM_Office.exe        >nul 2>&1
taskkill /F /IM HIL.exe              >nul 2>&1
taskkill /F /IM Movie.exe            >nul 2>&1
taskkill /F /IM CarMaker.win64.exe   >nul 2>&1
taskkill /F /IM TrafficLayer.exe     >nul 2>&1
REM taskkill /F /IM VISSIM220.exe        >nul 2>&1

REM --- preflight -----------------------------------------------------------
if not exist "%TL%"        ( echo ERROR: TrafficLayer.exe missing: %TL%  ^(build: scripts\dispatch\2_core_components.bat^) & pause & exit /b 1 )
if not exist "%CMEXE%"     ( echo ERROR: custom CarMaker.win64.exe missing: %CMEXE% & pause & exit /b 1 )
if not exist "%CM_OFFICE%" ( echo ERROR: CM_Office.exe not found: %CM_OFFICE% & pause & exit /b 1 )

REM --- 1. build demo assets (idempotent) -----------------------------------
echo [1/4] Building demo assets (rd5 + signal table + SimpleTrafficLight_Cosim)...
"%PYTHON%" "%HERE%add_signal_stops.py"     || ( echo add_signal_stops FAILED & pause & exit /b 1 )
"%PYTHON%" "%HERE%build_signal_table.py"   || ( echo build_signal_table FAILED & pause & exit /b 1 )
"%PYTHON%" "%HERE%build_cosim_testrun.py"  || ( echo build_cosim_testrun FAILED & pause & exit /b 1 )

REM --- 2. stage network + write config + register exe (-f config -s table) --
echo [2/4] Preparing GUI (stage network, write config, register exe + signal table)...
"%PYTHON%" "%HERE%setup_gui.py"
if errorlevel 1 ( echo ERROR: setup_gui.py failed. & pause & exit /b 1 )

REM --- 2b. VISSIM COM self-heal preflight (Win11 24H2) ----------------------
REM  On 24H2, VISSIM crashes a fraction of COM dispatches at startup and leaves
REM  a zombie VISSIM220.exe + stale FlexNet token + lock files; the NEXT launch
REM  then fails with "Server execution failed" (0x80080005) and STAYS broken.
REM  So: probe the dispatch; ONLY if it is already broken, reap the zombie and
REM  clean locks, then retry. Reaping only a corpse (no live co-sim yet) avoids
REM  leaking CodeMeter sessions -- which is why the blanket VISSIM kill above
REM  stays commented out. This is the same check-and-clean the headless harness
REM  (verify_demo.py loop) does, which is why it never wedges.
echo [preflight] Checking VISSIM COM dispatch health...
set _VTRY=0
:vchk
"%PYTHON%" "%HERE%vissim_dispatch_probe.py" >nul 2>&1
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
echo [preflight] WARNING: VISSIM COM still unhealthy after cleanup.
echo             Try the admin FlexNet bounce or a reboot (CLAUDE.md soft reset), then retry.
echo             Continuing anyway; TrafficLayer may fail to spawn VISSIM.
goto vdone
:vok
echo [preflight] VISSIM dispatch OK.
:vdone

REM --- 3. launch TrafficLayer (spawns VISSIM, serves 2444 + 2445, waits) ----
echo [3/4] Launching TrafficLayer (DSProxy). It spawns VISSIM, then waits on ports 2444 + 2445...
echo        (wait for "VISSIM_Connect OK" and "signal listener bound on port 2445" before Start)
start "FIXS TrafficLayer (DSProxy + signals)" cmd /k "%TL% -f %RUNCFG%"

REM --- 4. open CarMaker Office GUI on the project ---------------------------
echo [4/4] Opening CarMaker Office GUI...
start "" "%CM_OFFICE%" -projectdir "%CMPROJ%" "%TESTRUN%"

REM Open IPGMovie AFTER the GUI is up so it can connect to it.
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
echo    1) WAIT for the TrafficLayer window: "VISSIM_Connect OK"
echo                                  + "signal listener bound on port 2445"
echo    2) Load TestRun:  %TESTRUN%
echo    3) Press  START.
echo  The ego drives the corridor and brakes at the VISSIM-driven red;
echo  VISSIM background traffic renders as RS_C cars.
echo ============================================================
echo.
pause
endlocal
