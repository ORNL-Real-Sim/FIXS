@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ====================================
REM Build RealSimSocket MEX (mexw64)
REM Supports double-click (standalone) and inline invocation via dispatch.
REM ====================================

set "SCRIPT_DIR=%~dp0"
set "RUN_MODE=%~1"
set "YAML_HELPER=%SCRIPT_DIR%yaml_helper.ps1"

if /I "%RUN_MODE%"=="window" (
    start "RealSimSocket MEX Build" cmd /k "%~f0" inline
    exit /b 0
)

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

set "BUILD_RESULT=0"
set "MATLAB_VERSION="
set "MATLAB_INSTALL="
set "VS_VERSION="
set "DEPS_FILE=%SCRIPT_DIR%..\..\dependencies.yaml"
set "STACK_CHANGED=0"

echo Parsing dependencies from: %DEPS_FILE%

if not exist "%DEPS_FILE%" (
    echo ERROR: dependencies.yaml not found at %DEPS_FILE%
    set "BUILD_RESULT=1"
    goto :end
)

call :ReadYamlVersion "%DEPS_FILE%" "matlab" MATLAB_VERSION
call :ReadYamlVersion "%DEPS_FILE%" "visual_studio" VS_VERSION

if not defined MATLAB_VERSION (
    echo ERROR: MATLAB version not found in dependencies.yaml
    set "BUILD_RESULT=1"
    goto :end
)

if not defined VS_VERSION (
    echo WARNING: Visual Studio version not found in dependencies.yaml, MEX will auto-detect compiler
)

if defined MATLAB_ROOT (
    set "MATLAB_INSTALL=%MATLAB_ROOT%"
) else if exist "%ProgramFiles%\MATLAB\R%MATLAB_VERSION%" (
    set "MATLAB_INSTALL=%ProgramFiles%\MATLAB\R%MATLAB_VERSION%"
) else (
    echo MATLAB not found at standard location, attempting auto-detection...
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%SCRIPT_DIR%detect_tool_paths.ps1" -Tool "matlab"`) do set "MATLAB_INSTALL=%%~I"
)

if not defined MATLAB_INSTALL (
    echo ERROR: Could not locate MATLAB installation
    echo Please set MATLAB_ROOT environment variable or add install_path to dependencies.yaml
    set "BUILD_RESULT=1"
    goto :end
)

echo.
echo Building RealSimSocket MEX using MATLAB version: %MATLAB_VERSION%
echo MATLAB root: %MATLAB_INSTALL%

if not exist "%MATLAB_INSTALL%\bin\mex.bat" (
    echo ERROR: Unable to locate mex.bat at: %MATLAB_INSTALL%\bin\mex.bat
    set "BUILD_RESULT=1"
    goto :end
)

set "SOURCE_DIR=%SCRIPT_DIR%..\..\CommonLib"
set "SOURCE_FILE=%SOURCE_DIR%\RealSimSocket.cpp"

if not exist "%SOURCE_FILE%" (
    echo ERROR: Source file not found: %SOURCE_FILE%
    set "BUILD_RESULT=1"
    goto :end
)

pushd "%SOURCE_DIR%" >nul
if %ERRORLEVEL% EQU 0 (
    set "STACK_CHANGED=1"
) else (
    echo ERROR: Failed to change directory to %SOURCE_DIR%
    set "BUILD_RESULT=1"
    goto :end
)

REM Set up compiler if Visual Studio version is specified
if defined VS_VERSION (
    echo Configuring MEX to use Visual Studio %VS_VERSION%...
    call "%MATLAB_INSTALL%\bin\mex.bat" -setup C++ -client engine COMPILER=msvc%VS_VERSION% >nul 2>&1
    if %ERRORLEVEL% NEQ 0 (
        echo WARNING: Failed to configure Visual Studio %VS_VERSION%, MEX will use default compiler
    )
)

echo Invoking mex...
call "%MATLAB_INSTALL%\bin\mex.bat" -largeArrayDims -outdir "%SOURCE_DIR%" "%SOURCE_FILE%"
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ==============================
    echo RealSimSocket MEX build FAILED!
    echo ==============================
    set "BUILD_RESULT=1"
) else (
    echo.
    echo ==============================
    echo RealSimSocket MEX built successfully.
    echo Output: %SOURCE_DIR%\RealSimSocket.mexw64
    echo ==============================
    set "BUILD_RESULT=0"
)

:end
if "%STACK_CHANGED%"=="1" popd >nul
if /I "%RUN_MODE%"=="standalone" (
    echo.
    pause
)
exit /b %BUILD_RESULT%

:ReadYamlVersion
set "FILE=%~1"
set "SECTION=%~2"
set "OUT_VAR=%~3"
set "RESULT="

if exist "%YAML_HELPER%" (
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%FILE%" -Section "%SECTION%"`) do (
        if not defined RESULT set "RESULT=%%~I"
    )
)

set "%OUT_VAR%=%RESULT%"
exit /b 0
