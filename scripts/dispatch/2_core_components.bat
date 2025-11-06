@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ====================================
REM Build Core RealSim Components
REM Builds: TrafficLayer, VirtualEnvironment
REM ====================================

set "SCRIPT_DIR=%~dp0"
set "RUN_MODE=%~1"

for %%I in ("%SCRIPT_DIR%..\..") do set "REPO_ROOT=%%~fI"
set "DISPATCH_DIR=%SCRIPT_DIR%"

REM Use shared log files if set by dispatch, otherwise create local ones
if not defined RS_BUILD_LOG set "RS_BUILD_LOG=%DISPATCH_DIR%build.log"
if not defined RS_BUILD_SUMMARY set "RS_BUILD_SUMMARY=%DISPATCH_DIR%build_summary.log"
set "LOG_OUTPUT=%RS_BUILD_LOG%"
set "LOG_SUMMARY=%RS_BUILD_SUMMARY%"

REM Handle optional window request
if /I "%RUN_MODE%"=="window" (
    start "RS FIXS Core Build" cmd /k "%~f0" inline
    exit /b 0
)

REM Determine run mode (standalone vs inline)
if /I "%RUN_MODE%"=="inline" (
    set "RUN_MODE=inline"
) else (
    if /I "%RUN_MODE%"=="standalone" (
        set "RUN_MODE=standalone"
    ) else (
        if defined RS_FIXS_AUTOMATION (
            set "RUN_MODE=inline"
        ) else (
            set "RUN_MODE=standalone"
        )
    )
)

set "STACK_CHANGED=0"
set "BUILD_RESULT=0"
set "FAILED_BUILDS="

REM Configuration detection
REM When double-clicked: builds both Debug and Release (change STANDALONE_CONFIGS to build only what you need)
REM When called from dispatch.bat: RS_BUILD_CONFIG will be set to Release
if not defined RS_BUILD_CONFIG (
    set "STANDALONE_CONFIGS=Debug Release"
    REM To build Debug only when double-clicked, change above line to: set "STANDALONE_CONFIGS=Debug"
) else (
    set "STANDALONE_CONFIGS=%RS_BUILD_CONFIG%"
)

if not defined REPO_ROOT (
    echo ERROR: Failed to resolve repository root from script directory.
    set "BUILD_RESULT=1"
    goto :cleanup
)

if not exist "%REPO_ROOT%" (
    echo ERROR: Repository root not found: %REPO_ROOT%
    set "BUILD_RESULT=1"
    goto :cleanup
)

pushd "%REPO_ROOT%" >nul
if errorlevel 1 (
    echo ERROR: Failed to change directory to %REPO_ROOT%
    set "BUILD_RESULT=1"
    goto :cleanup
)
set "STACK_CHANGED=1"

REM Only initialize logs in standalone mode
if /I "%RUN_MODE%"=="standalone" (
    >"%LOG_SUMMARY%" echo Core Components Build Results
    >>"%LOG_SUMMARY%" echo ================================
    >>"%LOG_SUMMARY%" echo.
    if exist "%LOG_OUTPUT%" del "%LOG_OUTPUT%" >nul 2>&1
) else (
    >>"%LOG_SUMMARY%" echo.
    >>"%LOG_SUMMARY%" echo Core Components Build
    >>"%LOG_SUMMARY%" echo ----------------------
)

REM Build TrafficLayer
for %%C in (%STANDALONE_CONFIGS%) do (
    call :BuildSolution "TrafficLayer (%%C)" ".\TrafficLayer\TrafficLayer.sln" "/p:Configuration=%%C"
    if errorlevel 1 (
        call :TrackFailure "TrafficLayer (%%C)"
        set "BUILD_RESULT=1"
    )
)

REM Build VirtualEnvironment
for %%C in (%STANDALONE_CONFIGS%) do (
    call :BuildSolution "VirtualEnvironment (%%C)" ".\VirtualEnvironment\VirtualEnvironment.sln" "/p:Configuration=%%C"
    if errorlevel 1 (
        call :TrackFailure "VirtualEnvironment (%%C)"
        set "BUILD_RESULT=1"
    )
)

echo.
echo ==============================
if defined FAILED_BUILDS (
    echo Core build completed with failures!
    echo Failed builds: %FAILED_BUILDS%
) else (
    echo All core components built successfully!
)
echo Check %LOG_SUMMARY% for summary and %LOG_OUTPUT% for details.
echo ==============================

:cleanup
if "%STACK_CHANGED%"=="1" popd >nul

if /I "%RUN_MODE%"=="standalone" (
    echo.
    pause
)
exit /b %BUILD_RESULT%

:BuildSolution
set "TARGET_NAME=%~1" & set "SOLUTION_PATH=%~2" & set "MSBUILD_ARGS=%~3"
if not exist "%SOLUTION_PATH%" echo ===^> %TARGET_NAME% skipped (solution not found)>>"%LOG_SUMMARY%" & exit /b 0
echo Building %TARGET_NAME%...
echo ===^> %TARGET_NAME% build started>>"%LOG_SUMMARY%"
(call msbuild "%SOLUTION_PATH%" %MSBUILD_ARGS%) >>"%LOG_OUTPUT%" 2>&1
if errorlevel 1 echo ===^> %TARGET_NAME% built failed>>"%LOG_SUMMARY%" & exit /b 1
echo ===^> %TARGET_NAME% built success>>"%LOG_SUMMARY%"
exit /b 0

:TrackFailure
if defined FAILED_BUILDS (
    set "FAILED_BUILDS=%FAILED_BUILDS% %~1"
) else (
    set "FAILED_BUILDS=%~1"
)
exit /b 0
