@echo off
REM ============================================================================
REM  FIXS #168 - CarMaker <-> VISSIM <-> FIXS co-simulation, ONE-CLICK HEADLESS
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this. It runs the WHOLE pipeline automatically and prints a
REM  PASS/FAIL with evidence -- no GUI, no manual Start.
REM
REM  Under the hood it calls verify_demo.py, which:
REM    1. stages the DS-enabled SimpleEcho network + writes config.runtime.yaml,
REM    2. launches TrafficLayer (DSProxy) -> spawns VISSIM 2022, waits for the
REM       VISSIM_Connect handshake,
REM    3. launches the REAL custom CarMaker.win64.exe (links VirtualEnvironment.lib)
REM       headless against TrafficLayer,
REM    4. captures both sides and asserts the round-trip:
REM         CarMaker "RealSim Initialized" + SIM_END, and TrafficLayer per-tick
REM         "vehicles=N ... egos=1" (ego -> VISSIM, VISSIM traffic -> CarMaker).
REM
REM  This is the same exe and the same data exchange you get from the CarMaker
REM  GUI (run_cm_office_demo.bat) -- the GUI just adds the live 3D view.
REM
REM  Prereqs: VISSIM 2022 licensed + healthy (CodeMeter); binaries built;
REM           import_road.bat + build_testrun.py already run (the committed CM
REM           project is in that state).
REM ============================================================================

setlocal
set HERE=%~dp0
set PYTHON=C:\Users\yshao\miniconda3\envs\realsim_dev\python.exe

if not exist "%PYTHON%" (
    echo ERROR: python env not found: %PYTHON%
    echo Edit PYTHON in this .bat to your realsim_dev python, or run verify_demo.py manually.
    pause & exit /b 1
)

echo ============================================================
echo  FIXS #168 headless CarMaker-VISSIM-FIXS co-simulation demo
echo ============================================================
"%PYTHON%" "%HERE%verify_demo.py"
set RC=%ERRORLEVEL%

echo.
if "%RC%"=="0" (
    echo ============================================================
    echo  DEMO PASSED. CarMaker drove the ego into VISSIM and rendered
    echo  VISSIM background traffic, via TrafficLayer + DSProxy.
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
