@echo off
REM ============================================================================
REM  FIXS #172 -- CarMaker signal-stop verification (Q3 + Q1)
REM
REM  Q3: a traffic-light HEAD is mounted PAST the intersection (far side, s=130 m)
REM      while a DrvStop stop marker sits at the stop line (s=100 m). With the light
REM      red, the ego must stop at the STOP LINE (~100 m), NOT at the head.
REM  Q1: two heads on the SAME lane with DIFFERENT states (one red, one green).
REM
REM  Usage:
REM     run_signal_stop_test.bat            -> build Q3 + open CarMaker GUI to watch
REM     run_signal_stop_test.bat q1         -> build Q1 + open CarMaker GUI to watch
REM     run_signal_stop_test.bat verify     -> build Q3, run headless, print PASS/FAIL
REM     run_signal_stop_test.bat verify q1  -> build Q1, run headless, print profile
REM
REM  Requires: CarMaker 13.1.3 (CM_Office.exe), a Python with no extra deps.
REM  Edit CM_EXE / PYTHON below if your install paths differ.
REM ============================================================================
setlocal
set HERE=%~dp0
set CM_EXE=C:\IPG\carmaker\win64-13.1.3\bin\CM_Office.exe
set CMPROJ=%HERE%..\..\..\ProprietaryFiles\CM13_proj

REM --- resolve a python interpreter (override by setting PYTHON env var) ---
if "%PYTHON%"=="" set PYTHON=python

set MODE=%1
set VAR=%2
if /I "%MODE%"=="q1" ( set VARIANT=q1& set MODE=gui& goto build )
if /I "%MODE%"=="verify" ( if /I "%VAR%"=="q1" ( set VARIANT=q1 ) else ( set VARIANT=q3 )& goto build )
set VARIANT=q3
set MODE=gui

:build
echo [build] generating %VARIANT% road + testrun ...
"%PYTHON%" "%HERE%build_signal_stop_test.py" %VARIANT%
if errorlevel 1 ( echo Build failed -- is Python on PATH? Set PYTHON=... & exit /b 1 )

if /I "%VARIANT%"=="q1" ( set TR=SignalStopTest_Q1 ) else ( set TR=SignalStopTest )

if /I "%MODE%"=="verify" goto verify

echo [gui] opening CarMaker -- press Start, watch the ego stop at the stop line.
"%CM_EXE%" -projectdir "%CMPROJ%" "%TR%"
goto end

:verify
echo [verify] running %TR% headless ...
"%CM_EXE%" -projectdir "%CMPROJ%" -cmd "SaveMode save" -run "%TR%"
for /f "delims=" %%E in ('dir /b /s /o-d "%CMPROJ%\SimOutput\*%TR%_*.erg" 2^>nul') do ( set ERG=%%E& goto parsed )
:parsed
echo [verify] parsing %ERG%
"%PYTHON%" "%HERE%parse_erg.py" "%ERG%" 100 130

:end
endlocal
