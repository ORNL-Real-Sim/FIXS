@echo off
REM ============================================================================
REM  FIXS #172 - Open the SimpleTrafficLight scene in CarMaker Office (NO VISSIM)
REM ----------------------------------------------------------------------------
REM  DOUBLE-CLICK this to just LOOK AT / drive the imported scene in CarMaker:
REM    - the corridor + 3 signalized intersections + cross-streets + U-turns
REM    - the 56 traffic-signal objects osc2cm imported from the .xodr
REM
REM  It does NOT launch VISSIM or TrafficLayer (no co-simulation, no signal sync
REM  yet). Pressing START drives the ego with CarMaker's own IPGDriver only.
REM  For the full VISSIM signal-sync co-sim, a separate run_*.bat will follow.
REM
REM  Prereq: run import_road first (osc2cm) so SimpleTL_VISSIM exists. Already
REM  done if Data/TestRun/SimpleTL_VISSIM is present.
REM ============================================================================

setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI

set CMPROJ=%RepoRoot%\ProprietaryFiles\CM13_proj
set CM_OFFICE=C:\IPG\carmaker\win64-13.1.3\bin\CM_Office.exe
set MOVIE=C:\IPG\carmaker\win64-13.1.3\GUI\Movie.exe
set TESTRUN=SimpleTL_VISSIM

echo ============================================================
echo  FIXS #172 - SimpleTrafficLight scene in CarMaker (no VISSIM)
echo ============================================================

REM --- preflight ----------------------------------------------------------
if not exist "%CM_OFFICE%" ( echo ERROR: CM_Office.exe not found: %CM_OFFICE% & pause & exit /b 1 )
if not exist "%CMPROJ%\Data\TestRun\%TESTRUN%" (
    echo ERROR: TestRun %TESTRUN% missing under %CMPROJ%\Data\TestRun
    echo   Run import_road first ^(osc2cm^) to generate the road + TestRun.
    pause & exit /b 1
)

REM --- start fresh: close any stale CarMaker so it opens on THIS testrun ----
echo Closing any stale CarMaker windows...
taskkill /F /IM CM_Office.exe      >nul 2>&1
taskkill /F /IM HIL.exe            >nul 2>&1
taskkill /F /IM Movie.exe          >nul 2>&1

REM --- open the GUI on the project with the TestRun preloaded --------------
REM CarMaker "CM [opts] [testrun]" form: pass the TestRun as the trailing arg.
echo Opening CarMaker Office on TestRun %TESTRUN% ...
start "" "%CM_OFFICE%" -projectdir "%CMPROJ%" "%TESTRUN%"

REM --- open IPGMovie once the GUI (HIL.exe) is up, so it snaps to the scene -
if not exist "%MOVIE%" ( echo NOTE: Movie.exe not at %MOVIE% - open it manually if you want the 3D view & goto :done )
echo Waiting for the CarMaker GUI to come up before launching IPGMovie...
set /a _w=0
:waitgui
timeout /t 1 /nobreak >nul
tasklist /fi "imagename eq HIL.exe" 2>nul | find /i "HIL.exe" >nul && goto :guiup
set /a _w+=1
if %_w% lss 40 goto :waitgui
:guiup
timeout /t 3 /nobreak >nul
start "" "%MOVIE%"

:done
echo.
echo ============================================================
echo  Opened CarMaker on %TESTRUN%.
echo    - Inspect the road, the 3 intersections, and the signal objects.
echo    - Press START to drive the ego (IPGDriver only; no VISSIM signal sync).
echo  NOTE: signals will NOT change yet - that needs the VISSIM co-sim run.
echo ============================================================
echo.
pause
endlocal
