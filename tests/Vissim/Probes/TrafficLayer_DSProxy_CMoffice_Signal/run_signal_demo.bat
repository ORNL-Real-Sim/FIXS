@echo off
REM ============================================================================
REM  FIXS #172 - CarMaker <-> VISSIM <-> FIXS co-simulation, ONE-CLICK HEADLESS
REM  WITH traffic-signal sync (vehicles transmitted AND signals synced).
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. It builds the demo assets, then runs the whole pipeline
REM  and prints PASS/FAIL with evidence -- no GUI, no manual Start.
REM
REM  Build steps (idempotent):
REM    1. add_signal_stops.py     -> simple_traffic_light_signalstop.rd5 + SimpleTL_SignalStop
REM    2. build_signal_table.py   -> RSsignalTable.csv + VISSIM-convention names
REM    3. build_cosim_testrun.py  -> SimpleTL_Cosim (ego + 50 RS_C traffic slots)
REM  Then verify_signal_demo.py:
REM    4. stages the signalized DS network, launches TrafficLayer (DSProxy -> VISSIM),
REM       launches the headless CarMaker exe with -f config + -s RSsignalTable.csv,
REM       and asserts the round-trip (vehicles>0 AND signals>0, ego=1, SIM_END).
REM
REM  Prereqs: VISSIM 2022 licensed + healthy (CodeMeter); TrafficLayer.exe and the
REM           custom CarMaker exe built (verify builds the headless harness exe).
REM ============================================================================

setlocal
set HERE=%~dp0
set PYTHON=C:\Users\yshao\miniconda3\envs\realsim_dev\python.exe
if not exist "%PYTHON%" set PYTHON=python

echo ============================================================
echo  FIXS #172 building demo assets ...
echo ============================================================
"%PYTHON%" "%HERE%add_signal_stops.py"        || (echo add_signal_stops FAILED & pause & exit /b 1)
"%PYTHON%" "%HERE%build_signal_table.py"      || (echo build_signal_table FAILED & pause & exit /b 1)
"%PYTHON%" "%HERE%build_cosim_testrun.py"     || (echo build_cosim_testrun FAILED & pause & exit /b 1)

echo.
echo ============================================================
echo  FIXS #172 headless CarMaker-VISSIM-FIXS co-sim + signal sync
echo ============================================================
"%PYTHON%" "%HERE%verify_signal_demo.py"
set RC=%ERRORLEVEL%

echo.
if "%RC%"=="0" (
    echo ============================================================
    echo  DEMO PASSED. CarMaker drove the ego into VISSIM, rendered VISSIM
    echo  background traffic, AND synced its traffic lights to VISSIM signal
    echo  states -- the ego brakes at the VISSIM-driven red.
    echo ============================================================
) else (
    echo ============================================================
    echo  DEMO did not pass (rc=%RC%). See the evidence above.
    echo  Common cause: VISSIM 2022 license (CodeMeter) - see README.
    echo ============================================================
)
echo.
pause
endlocal
exit /b %RC%
