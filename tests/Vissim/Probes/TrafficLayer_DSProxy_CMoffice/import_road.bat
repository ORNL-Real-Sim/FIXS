@echo off
REM ============================================================================
REM  FIXS #168 - (Re)generate the CarMaker road + TestRun for the demo.
REM ----------------------------------------------------------------------------
REM  Run this ONCE (or after the SimpleEcho network changes). It is headless:
REM  no CarMaker GUI needed. It:
REM    1. Copies SimpleEcho's simple_loop.xodr into the CM project + catalogs.
REM    2. Runs osc2cm.exe to convert OpenDRIVE -> CarMaker road (.rd5) + a
REM       TestRun (SimpleLoop) + ego vehicle/driver.
REM    3. Runs RealSimCarMakerSetup.py to add 20 RS_Cxxx traffic objects,
REM       producing the demo TestRun SimpleLoop_rs.
REM
REM  Outputs land in ProprietaryFiles/CM13_proj/Data/{Road,TestRun,Vehicle,...}.
REM ============================================================================

setlocal EnableDelayedExpansion
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI

set CMPROJ=%RepoRoot%\ProprietaryFiles\CM13_proj
set OSC2CM=C:\IPG\carmaker\win64-13.1.3\bin\osc2cm.exe
set CM_CATALOGS=C:\IPG\carmaker\win64-13.1.3\Data\OpenSCENARIO\Catalogs
set SRC_XODR=%RepoRoot%\tests\Vissim\SimpleEcho\simple_loop.xodr
set OSCDIR=%CMPROJ%\Data\OpenSCENARIO
set PYTHON=C:\Users\yshao\miniconda3\envs\realsim_dev\python.exe

echo === FIXS #168 road/TestRun import ===

if not exist "%OSC2CM%"  ( echo ERROR: osc2cm.exe not found: %OSC2CM% & pause & exit /b 1 )
if not exist "%SRC_XODR%" ( echo ERROR: simple_loop.xodr not found: %SRC_XODR% & pause & exit /b 1 )

REM --- 1. stage xodr + the bundled .xosc + catalogs ------------------------
if not exist "%OSCDIR%" mkdir "%OSCDIR%"
copy /Y "%SRC_XODR%" "%OSCDIR%\simple_loop.xodr" >nul
copy /Y "%HERE%simple_loop.xosc" "%OSCDIR%\simple_loop.xosc" >nul
if not exist "%OSCDIR%\Catalogs" (
    echo Copying OpenSCENARIO catalogs from CM data pool...
    xcopy /E /I /Q /Y "%CM_CATALOGS%" "%OSCDIR%\Catalogs" >nul
)
echo [1/3] Staged xodr + xosc + catalogs into %OSCDIR%

REM --- 2. osc2cm: OpenDRIVE -> rd5 + TestRun + ego ------------------------
echo [2/3] Running osc2cm (OpenDRIVE -^> CarMaker road + TestRun)...
"%OSC2CM%" -p "%CMPROJ%" -o "Data/OpenSCENARIO/simple_loop.xosc" -r "simple_loop" -t "SimpleLoop" -e "Ego"
if not exist "%CMPROJ%\Data\Road\simple_loop.rd5" ( echo ERROR: osc2cm did not produce simple_loop.rd5 & pause & exit /b 2 )

REM --- 3. add 20 RS_Cxxx traffic objects ---------------------------------
echo [3/3] Adding 20 traffic objects (RealSimCarMakerSetup.py)...
if not exist "%PYTHON%" (
    echo WARNING: python env not found at %PYTHON%
    echo   The base TestRun SimpleLoop exists but has NO traffic objects;
    echo   VISSIM vehicles will arrive on the socket but not render in CarMaker.
    echo   Run RealSimCarMakerSetup.py manually with your python to add them.
    pause & exit /b 0
)
"%PYTHON%" "%RepoRoot%\CarMaker\RealSimCarMakerSetup.py" --cm-project-path "%CMPROJ%" --testrun "SimpleLoop" --cm-install-path "C:\IPG" --output-testrun "SimpleLoop_rs" --car 20 --truck 0 --no-random-traffic

echo.
echo === Import complete ===
echo   Road:    %CMPROJ%\Data\Road\simple_loop.rd5
echo   TestRun: %CMPROJ%\Data\TestRun\SimpleLoop_rs  (ego + 20 traffic objects)
echo Now run:  run_cm_office_demo.bat
endlocal
