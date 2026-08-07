@echo off
REM ============================================================================
REM  FIXS #172 -- minimal junction signal-stop test: open a variant in CarMaker+Movie.
REM
REM  Variants (head location for the ego's straight movement; controller forced RED):
REM    approach : head on the APPROACH edge  (x=-7, junction entry)   -> ego stops
REM    farside  : head on the DEPARTURE edge ACROSS the junction (x=+5) -> ego stops
REM    nohead   : no head referencing the controller                  -> ego RUNS the red
REM
REM  Usage:
REM    run_junction_test.bat farside    :: build + open CarMaker + IPGMovie on the farside variant
REM    run_junction_test.bat approach   :: (default)
REM    run_junction_test.bat <v> verify :: build + run headless + print whether the ego stopped
REM ============================================================================
setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..") do set RepoRoot=%%~fI
set CMPROJ=%RepoRoot%\ProprietaryFiles\CM13_proj
set CM_OFFICE=C:\IPG\carmaker\win64-13.1.3\bin\CM_Office.exe
set MOVIE=C:\IPG\carmaker\win64-13.1.3\GUI\Movie.exe
if "%PYTHON%"=="" set PYTHON=python
set V=%1
if "%V%"=="" set V=approach
if /I "%V%"=="verify" set V=approach

echo [build] %V% variant ...
"%PYTHON%" "%HERE%build_junction_test.py" %V%
if errorlevel 1 ( echo Build failed -- Python on PATH? Set PYTHON=... & pause & exit /b 1 )

taskkill /F /IM CM_Office.exe >nul 2>&1
taskkill /F /IM HIL.exe       >nul 2>&1
taskkill /F /IM Movie.exe     >nul 2>&1

if /I "%2"=="verify" (
  echo [verify] running junction_%V% headless ...
  "%CM_OFFICE%" -projectdir "%CMPROJ%" -cmd "SaveMode save" -run junction_%V%
  for /f "delims=" %%E in ('dir /b /s /o-d "%CMPROJ%\SimOutput\*junction_%V%_*.erg" 2^>nul') do ( set ERG=%%E& goto p )
  :p
  "%PYTHON%" "%HERE%parse_erg.py" "%ERG%" 2>nul | findstr /i "stop v="
  goto end
)

echo Opening CarMaker on junction_%V% ...
start "" "%CM_OFFICE%" -projectdir "%CMPROJ%" "junction_%V%"
if not exist "%MOVIE%" ( echo NOTE: open IPGMovie manually for the 3D view & goto end )
set /a _w=0
:waitgui
timeout /t 1 /nobreak >nul
tasklist /fi "imagename eq HIL.exe" 2>nul | find /i "HIL.exe" >nul && goto guiup
set /a _w+=1
if %_w% lss 40 goto waitgui
:guiup
timeout /t 3 /nobreak >nul
start "" "%MOVIE%"

:end
echo.
echo Opened junction_%V%. Press START: controller is RED; watch whether the ego stops.
echo   approach head x=-7 (stops) ^| farside head x=+5 across junction (stops) ^| nohead (runs red)
echo.
pause
endlocal
