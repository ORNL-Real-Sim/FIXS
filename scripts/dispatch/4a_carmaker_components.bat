@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ====================================
REM Build CarMaker Components
REM Builds: CarMaker desktop and Simulink variants
REM ====================================

set "SCRIPT_DIR=%~dp0"
set "RUN_MODE=%~1"
set "YAML_HELPER=%SCRIPT_DIR%yaml_helper.ps1"

for %%I in ("%SCRIPT_DIR%..\..") do set "REPO_ROOT=%%~fI"
for %%I in ("%SCRIPT_DIR%.") do set "DISPATCH_DIR=%%~fI"
set "DEPS_FILE=%REPO_ROOT%\dependencies.yaml"

REM Use shared log files if set by dispatch, otherwise create local ones
if not defined RS_BUILD_LOG set "RS_BUILD_LOG=%DISPATCH_DIR%build.log"
if not defined RS_BUILD_SUMMARY set "RS_BUILD_SUMMARY=%DISPATCH_DIR%build_summary.log"
set "LOG_OUTPUT=%RS_BUILD_LOG%"
set "LOG_SUMMARY=%RS_BUILD_SUMMARY%"

REM Handle optional window request
if /I "%RUN_MODE%"=="window" (
    start "RS FIXS CarMaker Build" cmd /k "%~f0" inline
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
set "CARMAKER_VERSIONS="

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

if not exist "%DEPS_FILE%" (
    echo ERROR: dependencies.yaml not found at %DEPS_FILE%
    set "BUILD_RESULT=1"
    goto :cleanup
)

echo Parsing dependencies from: %DEPS_FILE%
call :ReadYamlList "%DEPS_FILE%" "carmaker" "versions" CARMAKER_VERSIONS
if not defined CARMAKER_VERSIONS (
    echo ERROR: CarMaker versions not found in dependencies.yaml
    set "BUILD_RESULT=1"
    goto :cleanup
)

echo.
echo Using versions from dependencies.yaml:
echo   CarMaker versions: %CARMAKER_VERSIONS%
echo.

REM Only initialize logs in standalone mode
if /I "%RUN_MODE%"=="standalone" (
    >"%LOG_SUMMARY%" echo CarMaker Build Results
    >>"%LOG_SUMMARY%" echo ======================
    >>"%LOG_SUMMARY%" echo.
    if exist "%LOG_OUTPUT%" del "%LOG_OUTPUT%" >nul 2>&1
) else (
    >>"%LOG_SUMMARY%" echo.
    >>"%LOG_SUMMARY%" echo CarMaker Components Build
    >>"%LOG_SUMMARY%" echo -------------------------
)

REM Build CarMaker desktop and Simulink variants
for %%v in (%CARMAKER_VERSIONS%) do (
    set "CM_VERSION=%%v"
    REM Extract major version (e.g., 13.1.3 -> 13, 11.1.2 -> 11)
    for /f "tokens=1 delims=." %%m in ("!CM_VERSION!") do set "CM_MAJOR=%%m"

    if exist ".\ProprietaryFiles\CM!CM_MAJOR!_proj" (
        call :BuildSolution "CarMaker!CM_MAJOR!" ".\ProprietaryFiles\CM!CM_MAJOR!_proj\src\CarMaker.sln" "/target:CarMaker /p:Configuration=Release"
        if errorlevel 1 (
            call :TrackFailure "CarMaker!CM_MAJOR!"
            set "BUILD_RESULT=1"
        )

        call :BuildSolution "CarMaker!CM_MAJOR! Simulink" ".\ProprietaryFiles\CM!CM_MAJOR!_proj\src_cm4sl\CarMaker for Simulink.sln" "/p:Configuration=Release"
        if errorlevel 1 (
            call :TrackFailure "CarMaker!CM_MAJOR! Simulink"
            set "BUILD_RESULT=1"
        )
    ) else (
        >>"%LOG_SUMMARY%" echo CM!CM_MAJOR!_proj folder not found, skipping CarMaker !CM_MAJOR! builds
    )
)

echo.
echo ==============================
if defined FAILED_BUILDS (
    echo CarMaker build completed with failures!
    echo Failed builds: %FAILED_BUILDS%
) else (
    echo All CarMaker components built successfully!
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

:ReadYamlList
setlocal EnableExtensions EnableDelayedExpansion
set "FILE=%~1"
set "SECTION=%~2"
set "LIST_KEY=%~3"
set "OUT_VAR=%~4"
set "RESULT="

if exist "%YAML_HELPER%" (
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%FILE%" -Section "%SECTION%" -ListKey "%LIST_KEY%" -ReturnList`) do (
        if not defined RESULT set "RESULT=%%~I"
    )
)

endlocal & set "%OUT_VAR%=%RESULT%"
exit /b 0

:ReadYamlVersion
setlocal EnableExtensions EnableDelayedExpansion
set "FILE=%~1"
set "SECTION=%~2"
set "OUT_VAR=%~3"
set "RESULT="

if exist "%YAML_HELPER%" (
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%FILE%" -Section "%SECTION%"`) do (
        if not defined RESULT set "RESULT=%%~I"
    )
)

endlocal & set "%OUT_VAR%=%RESULT%"
exit /b 0
