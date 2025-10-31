@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ====================================
REM Build dSPACE Library for CarMaker
REM Supports double-click (standalone) and inline invocation via dispatch.
REM ====================================

set "SCRIPT_DIR=%~dp0"
set "RUN_MODE=%~1"
set "YAML_HELPER=%SCRIPT_DIR%yaml_helper.ps1"

REM Handle optional window request
if /I "%RUN_MODE%"=="window" (
    start "dSPACE Build" cmd /k "%~f0" inline
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

set "BUILD_RESULT=0"
set "DSPACE_VERSION="
set "DSPACE_INSTALL="
set "DEPS_FILE=%SCRIPT_DIR%..\..\dependencies.yaml"
set "CARMAKER_PRIMARY_VERSION="
set "CARMAKER_VERSIONS="
set "CARMAKER_BASE=C:\IPG\carmaker"
set "STACK_CHANGED=0"

if defined RS_FIXS_CARMAKER_BASE set "CARMAKER_BASE=%RS_FIXS_CARMAKER_BASE%"
if defined CARMAKER_ROOT set "CARMAKER_BASE=%CARMAKER_ROOT%"

echo Parsing dependencies from: %DEPS_FILE%

if not exist "%DEPS_FILE%" (
    echo ERROR: dependencies.yaml not found at %DEPS_FILE%
    set "BUILD_RESULT=1"
    goto :end
)

call :ReadYamlVersion "%DEPS_FILE%" "dspace" DSPACE_VERSION
call :ReadYamlVersion "%DEPS_FILE%" "carmaker" CARMAKER_PRIMARY_VERSION
call :ReadYamlList "%DEPS_FILE%" "carmaker" "versions" CARMAKER_VERSIONS

if not defined DSPACE_VERSION (
    echo Falling back to default dSPACE version 2024a...
    set "DSPACE_VERSION=2024a"
)

if not defined CARMAKER_VERSIONS (
    if defined CARMAKER_PRIMARY_VERSION (
        set "CARMAKER_VERSIONS=%CARMAKER_PRIMARY_VERSION%"
    )
)

if not defined CARMAKER_VERSIONS (
    echo WARNING: CarMaker versions not found in dependencies.yaml. Defaulting to 13.1.3
    set "CARMAKER_VERSIONS=13.1.3"
)

if /I "%DSPACE_VERSION%"=="2024a" (
    set "DSPACE_INSTALL=C:\Program Files\dSPACE ConfigurationDesk 2024-A (24.1)"
)

set "DSPACE_VERSION_SAFE=%DSPACE_VERSION%"
set "DSPACE_VERSION_SAFE=%DSPACE_VERSION_SAFE:.=_%"
if not defined DSPACE_INSTALL (
    echo ERROR: Could not determine dSPACE install path for version: %DSPACE_VERSION%
    set "BUILD_RESULT=1"
    goto :end
)

echo.
echo Building dSPACE library version: %DSPACE_VERSION%
echo Install path: %DSPACE_INSTALL%
echo CarMaker install root: %CARMAKER_BASE%
echo Target CarMaker versions: %CARMAKER_VERSIONS%

set "HAS_CFD=0"
if exist "%DSPACE_INSTALL%\CFD_vars.bat" set "HAS_CFD=1"
if "%HAS_CFD%"=="0" goto :missing_cfd

for %%I in ("%DSPACE_INSTALL%\SCALEXIO\DsBuildLibrary.mk") do set "DSPACE_MAKEFILE=%%~fI"
if not defined DSPACE_MAKEFILE goto :missing_makefile
if not exist "%DSPACE_MAKEFILE%" goto :missing_makefile

pushd "%SCRIPT_DIR%..\..\CommonLib" >nul
if %ERRORLEVEL% EQU 0 (
    set "STACK_CHANGED=1"
) else (
    echo ERROR: Failed to change directory to CommonLib
    set "BUILD_RESULT=1"
    goto :end
)

call "%DSPACE_INSTALL%\CFD_vars.bat"

set "CARMAKER_SUCCESS_COUNT=0"
set "CARMAKER_FAIL_COUNT=0"

for %%V in (%CARMAKER_VERSIONS%) do (
    call :BuildCarMakerVariant "%%~V"
    if errorlevel 1 (
        set /a CARMAKER_FAIL_COUNT+=1
        set "BUILD_RESULT=1"
    ) else (
        set /a CARMAKER_SUCCESS_COUNT+=1
    )
)

echo.
echo CarMaker build summary: !CARMAKER_SUCCESS_COUNT! succeeded, !CARMAKER_FAIL_COUNT! failed

if not "!CARMAKER_FAIL_COUNT!"=="0" (
    echo ERROR: One or more CarMaker library builds failed.
) else (
    echo All requested CarMaker library builds completed successfully.
)

goto :end

:BuildCarMakerVariant
setlocal EnableDelayedExpansion
set "CM_VERSION=%~1"
if not defined CM_VERSION (
    echo ERROR: Received empty CarMaker version.
    endlocal & exit /b 1
)

set "CARMAKER_INCLUDE=%CARMAKER_BASE%\win64-!CM_VERSION!\include"
if not exist "!CARMAKER_INCLUDE!" (
    echo ERROR: CarMaker include path not found for version !CM_VERSION! at !CARMAKER_INCLUDE!
    endlocal & exit /b 1
)

set "CM_VERSION_SAFE=!CM_VERSION!"
set "CM_VERSION_SAFE=!CM_VERSION_SAFE:.=_!"
set "OUTPUT_NAME=RealSimDsLib_%DSPACE_VERSION_SAFE%_CM!CM_VERSION_SAFE!"
set "CPP_OPTS=-std=c++11 -I!CARMAKER_INCLUDE! -DDSRTLX -DRS_DSPACE -DRS_CAVE -DRS_DEBUG"

echo.
echo ----------------------------------
echo Building dSPACE library for CarMaker !CM_VERSION! ...
echo Include path: !CARMAKER_INCLUDE!
echo Output name : !OUTPUT_NAME! (CarMaker !CM_VERSION!)
echo ----------------------------------

call dsmake -f "%DSPACE_MAKEFILE%" output_filename=!OUTPUT_NAME! source_files="SocketHelper.cpp MsgHelper.cpp VirEnvHelper.cpp VirEnv_Wrapper.cpp" custom_cpp_options="!CPP_OPTS!" target=Dsx86_32
set "DSM_RESULT=!ERRORLEVEL!"

if not "!DSM_RESULT!"=="0" (
    echo.
    echo ==============================
    echo dSPACE library build FAILED for CarMaker !CM_VERSION! (exit code !DSM_RESULT!)
    echo ==============================
    endlocal & exit /b !DSM_RESULT!
)

echo.
echo ==============================
echo dSPACE library built successfully: lib!OUTPUT_NAME!.a
echo ==============================
endlocal & exit /b 0

:missing_makefile
echo ERROR: dSPACE makefile not found: %DSPACE_INSTALL%\SCALEXIO\DsBuildLibrary.mk
set "BUILD_RESULT=1"
goto :end

:missing_cfd
echo ERROR: dSPACE environment file not found: %DSPACE_INSTALL%\CFD_vars.bat
set "BUILD_RESULT=1"

:end
if "%STACK_CHANGED%"=="1" popd >nul
if /I "%RUN_MODE%"=="standalone" (
    echo.
    pause
)
exit /b %BUILD_RESULT%

:ReadYamlList
REM %1 file path, %2 subsection name, %3 list key, %4 output var
setlocal EnableExtensions EnableDelayedExpansion
set "FILE=%~1"
set "TARGET=%~2"
set "LIST_KEY=%~3"
set "OUT_VAR=%~4"
set "RESULT="

if exist "%YAML_HELPER%" (
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%FILE%" -Section "%TARGET%" -ListKey "%LIST_KEY%" -ReturnList`) do (
        if not defined RESULT set "RESULT=%%~I"
    )
)

endlocal & set "%OUT_VAR%=%RESULT%"
exit /b 0

:ReadYamlVersion
REM %1 file path, %2 subsection name, %3 output var
setlocal EnableExtensions EnableDelayedExpansion
set "FILE=%~1"
set "TARGET=%~2"
set "OUT_VAR=%~3"
set "RESULT="

if exist "%YAML_HELPER%" (
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%FILE%" -Section "%TARGET%"`) do (
        if not defined RESULT set "RESULT=%%~I"
    )
)

endlocal & set "%OUT_VAR%=%RESULT%"
exit /b 0



