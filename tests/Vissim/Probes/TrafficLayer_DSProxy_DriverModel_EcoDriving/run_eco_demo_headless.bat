@echo off
REM ============================================================================
REM  FIXS #173 - Eco-driving CAV demo, ONE-CLICK HEADLESS (PASS/FAIL).
REM  CarMaker <-> VISSIM <-> FIXS with DSProxy AND the FIXS DriverModel co-loaded
REM  (Stage C). Runs the whole pipeline and prints PASS/FAIL -- no GUI, no Start.
REM ----------------------------------------------------------------------------
REM  Build steps (idempotent), via Demo 2's shared builders:
REM    1. add_signal_stops.py     -> simple_traffic_light.rd5 + SimpleTrafficLight_ego
REM    2. build_signal_table.py   -> RSsignalTable.csv
REM    3. build_cosim_testrun.py  -> SimpleTrafficLight_Cosim
REM    4. build_eco_signals.py    -> signals.json (kept if already calibrated)
REM  Then verify_eco_demo.py:
REM    5. stages the network, HOOKS the FIXS DriverModel on Car type 100, launches
REM       TrafficLayer (DSProxy + relay) + cav_controller.py + headless CarMaker,
REM       and asserts: ego in, signals synced, DriverModel connected, CAVs relayed,
REM       and the relay did NOT deadlock (ticks advanced past tick 1).
REM
REM  Prereqs: VISSIM 2022 licensed + healthy; TrafficLayer.exe,
REM           DriverModel_RealSim.dll, and the headless CarMaker exe built.
REM ============================================================================

setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI
set DEMO2=%RepoRoot%\tests\Vissim\Probes\TrafficLayer_DSProxy_CMoffice_Signal
set PYTHON=C:\Users\yshao\miniconda3\envs\realsim_dev\python.exe
if not exist "%PYTHON%" set PYTHON=python

echo ============================================================
echo  FIXS #173 building demo assets (shared Demo 2 + signals.json) ...
echo ============================================================
"%PYTHON%" "%DEMO2%\add_signal_stops.py"    || (echo add_signal_stops FAILED & pause & exit /b 1)
"%PYTHON%" "%DEMO2%\build_signal_table.py"  || (echo build_signal_table FAILED & pause & exit /b 1)
"%PYTHON%" "%DEMO2%\build_cosim_testrun.py" || (echo build_cosim_testrun FAILED & pause & exit /b 1)
"%PYTHON%" "%HERE%build_eco_signals.py"     || (echo build_eco_signals FAILED & pause & exit /b 1)

echo.
echo ============================================================
echo  FIXS #173 headless eco-driving co-sim (DSProxy + DriverModel co-load)
echo ============================================================
"%PYTHON%" "%HERE%verify_eco_demo.py"
set RC=%ERRORLEVEL%

echo.
if "%RC%"=="0" (
    echo ============================================================
    echo  DEMO PASSED. CarMaker drove the ego into VISSIM, signals synced,
    echo  and DriverModel-hooked CAVs were relayed eco-driving speed commands
    echo  alongside DSProxy -- the Stage C co-load works.
    echo ============================================================
) else (
    echo ============================================================
    echo  DEMO did not pass (rc=%RC%). See the evidence above.
    echo  If DriverModel connected but ticks stalled, the SubEgoOnly/per-Car
    echo  relay path is the issue (README "Status").
    echo ============================================================
)
echo.
pause
endlocal
exit /b %RC%
