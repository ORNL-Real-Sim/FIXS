@echo off
REM ============================================================================
REM  Build CarMaker_headless.win64.exe -- the HEADLESS self-test harness used by
REM  verify_demo.py. It is NOT the shipped exe.
REM
REM  Why it exists: the office exe gates the VirtualEnvironment.lib connect on
REM  SCState_StartWait (correct for the GUI). Headless (-screen, no GUI) never
REM  reaches that state at the hook, so a -screen co-sim never connects. The
REM  RS_HEADLESS build flag (CarMaker.props) makes User.c gate on SCState_Start
REM  instead AND renames the binary, so this validation exe can never be confused
REM  with / clobber the shipped GUI CarMaker.win64.exe.
REM
REM  Reproducible: rebuilds VirtualEnvironment.lib (so co-sim fixes in
REM  VirEnv_Wrapper.cpp / SocketHelper.cpp / VirEnvHelper.cpp are linked in) then
REM  the headless exe. Re-run after touching any of those or User.c. The shipped
REM  GUI exe is unaffected -- build it the normal way (dispatch / office).
REM ============================================================================
setlocal
set HERE=%~dp0
for %%I in ("%HERE%..\..\..\..") do set RepoRoot=%%~fI
set CMSLN=%RepoRoot%\ProprietaryFiles\CM13_proj\src\CarMaker.sln
set VELN=%RepoRoot%\VirtualEnvironment\VirtualEnvironment.sln

REM --- locate MSBuild via vswhere (machine-independent) ---
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" ( echo ERROR: vswhere not found under Program Files ^(x86^) & exit /b 1 )
set MSBUILD=
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe`) do set MSBUILD=%%i
if "%MSBUILD%"=="" ( echo ERROR: MSBuild not found via vswhere & exit /b 1 )

echo [1/2] Rebuilding VirtualEnvironment.lib (Release x64)...
"%MSBUILD%" "%VELN%" -p:Configuration=Release -p:Platform=x64 -v:minimal -nologo
if errorlevel 1 ( echo ERROR: VirtualEnvironment.lib build failed & exit /b 1 )

echo [2/2] Building CarMaker_headless.win64.exe (RS_HEADLESS)...
"%MSBUILD%" "%CMSLN%" -target:CarMaker -p:Configuration=Release -p:RS_HEADLESS=1 -v:minimal -nologo
if errorlevel 1 ( echo ERROR: headless CarMaker build failed & exit /b 1 )

echo.
echo Done: %RepoRoot%\ProprietaryFiles\CM13_proj\src\CarMaker_headless.win64.exe
endlocal
