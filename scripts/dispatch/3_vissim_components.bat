@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ====================================
REM Build VISSIM Components
REM Builds: DriverModel_RealSim and DriverModel_RealSim_v2021
REM ====================================

set "SCRIPT_DIR=%~dp0"
set "RUN_MODE=%~1"

for %%I in ("%SCRIPT_DIR%..\..") do set "REPO_ROOT=%%~fI"
for %%I in ("%SCRIPT_DIR%.") do set "DISPATCH_DIR=%%~fI"

REM Use shared log files if set by dispatch, otherwise create local ones
if not defined RS_BUILD_LOG set "RS_BUILD_LOG=%DISPATCH_DIR%build.log"
if not defined RS_BUILD_SUMMARY set "RS_BUILD_SUMMARY=%DISPATCH_DIR%build_summary.log"
set "LOG_OUTPUT=%RS_BUILD_LOG%"
set "LOG_SUMMARY=%RS_BUILD_SUMMARY%"

REM Handle optional window request
if /I "%RUN_MODE%"=="window" (
    start "RS FIXS VISSIM Build" cmd /k "%~f0" inline
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
    >"%LOG_SUMMARY%" echo VISSIM Build Results
    >>"%LOG_SUMMARY%" echo ====================
    >>"%LOG_SUMMARY%" echo.
    if exist "%LOG_OUTPUT%" del "%LOG_OUTPUT%" >nul 2>&1
) else (
    >>"%LOG_SUMMARY%" echo.
    >>"%LOG_SUMMARY%" echo VISSIM Components Build
    >>"%LOG_SUMMARY%" echo -----------------------
)

REM Build VISSIM server components if present
if exist ".\ProprietaryFiles\VISSIMserver" (
    call :BuildSolution "DriverModel_RealSim" ".\ProprietaryFiles\VISSIMserver\VISSIMserver.sln" "/target:DriverModel_RealSim /p:Configuration=Release"
    if errorlevel 1 (
        call :TrackFailure "DriverModel_RealSim"
        set "BUILD_RESULT=1"
    )

    call :BuildSolution "DriverModel_RealSim_v2021" ".\ProprietaryFiles\VISSIMserver\VISSIMserver.sln" "/target:DriverModel_RealSim_v2021 /p:Configuration=Release"
    if errorlevel 1 (
        call :TrackFailure "DriverModel_RealSim_v2021"
        set "BUILD_RESULT=1"
    )
) else (
    echo VISSIMserver folder not found, skipping VISSIM builds
    >>"%LOG_SUMMARY%" echo VISSIMserver folder not found, skipping VISSIM builds
    goto :cleanup
)

echo.
echo ==============================
if defined FAILED_BUILDS (
    echo VISSIM build completed with failures!
    echo Failed builds: %FAILED_BUILDS%
) else (
    echo All VISSIM components built successfully!
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
