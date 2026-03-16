@echo off
setlocal enabledelayedexpansion

echo ==============================
echo RealSim FIXS Release Builder
echo ==============================
echo.

REM Set paths
set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..\..") do set "SOURCE_PATH=%%~fI"
set "RELEASE_PATH=%SOURCE_PATH%\build"
set "DEPS_FILE=%SOURCE_PATH%\dependencies.yaml"
set "YAML_HELPER=%SCRIPT_DIR%yaml_helper.ps1"

REM Set build configuration for dispatch (component scripts will use this)
set "RS_BUILD_CONFIG=Release"

REM Set up shared log files
set "RS_BUILD_LOG=%SCRIPT_DIR%build.log"
set "RS_BUILD_SUMMARY=%SCRIPT_DIR%build_summary.log"

REM Initialize log files
>"%RS_BUILD_LOG%" echo RealSim FIXS Build Log
>>"%RS_BUILD_LOG%" echo =====================
>>"%RS_BUILD_LOG%" echo Build started: %date% %time%
>>"%RS_BUILD_LOG%" echo.
>"%RS_BUILD_SUMMARY%" echo RealSim FIXS Build Summary
>>"%RS_BUILD_SUMMARY%" echo ==========================
>>"%RS_BUILD_SUMMARY%" echo Build started: %date% %time%
>>"%RS_BUILD_SUMMARY%" echo.

REM Parse versions from dependencies.yaml
set "MATLAB_VERSION="
set "CARMAKER_VERSIONS="
set "YAML_MATLAB_VERSION="
set "YAML_MATLAB_DIR="
if exist "%YAML_HELPER%" (
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%DEPS_FILE%" -Section "matlab"`) do set "MATLAB_VERSION=%%~I"
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%DEPS_FILE%" -Section "carmaker" -ListKey "versions" -ReturnList`) do set "CARMAKER_VERSIONS=%%~I"
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%DEPS_FILE%" -Section "yaml_matlab"`) do set "YAML_MATLAB_VERSION=%%~I"
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%DEPS_FILE%" -Section "yaml_matlab" -Key "location"`) do set "YAML_MATLAB_DIR=%%~I"
)
if not defined MATLAB_VERSION (
    echo ERROR: MATLAB version not found in dependencies.yaml
    exit /b 1
)

echo Using versions from dependencies.yaml:
echo   MATLAB: %MATLAB_VERSION%
if defined CARMAKER_VERSIONS (
    echo   CarMaker: %CARMAKER_VERSIONS%
) else (
    echo   CarMaker: [not configured - CarMaker steps may be skipped]
)
echo.

REM Clean and create build directory
echo Cleaning build directory...
if exist "%RELEASE_PATH%" (
    rmdir /S /Q "%RELEASE_PATH%"
)
mkdir "%RELEASE_PATH%"

REM Capture build start time
set "BUILD_START=%date% %time%"
set "BUILD_START_SECONDS=%time:~0,2%%time:~3,2%%time:~6,2%"

REM ====================================
REM Step 1: Compile External Libraries
REM ====================================
echo [1/8] Checking external libraries...
if not exist "%SOURCE_PATH%\CommonLib\yaml-cpp\build" (
    echo External libraries not found. Compiling...
    call "%~dp0\1_external_libraries.bat"
    if %ERRORLEVEL% neq 0 (
        echo ERROR: Failed to compile external libraries!
        goto :failed
    )
) else (
    echo External libraries already compiled.
)
echo.

REM ====================================
REM Step 2: Compile Core Components
REM ====================================
echo [2/8] Compiling core components (TrafficLayer)...
call "%~dp0\2_core_components.bat" inline
if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to compile core components!
    goto :failed
)
echo.

REM ====================================
REM Step 3: Compile VISSIM Components
REM ====================================
echo [3/8] Compiling VISSIM components...
call "%~dp0\3_vissim_components.bat" inline
if %ERRORLEVEL% neq 0 (
    echo WARNING: VISSIM components build failed or skipped
)
echo.

REM ====================================
REM Step 4: Compile VirtualEnvironment
REM ====================================
echo [4/8] Compiling VirtualEnvironment...
call "%~dp0\4_virtual_environment.bat" inline
if %ERRORLEVEL% neq 0 (
    echo WARNING: VirtualEnvironment build failed or skipped
)
echo.

REM ====================================
REM Step 5a: Compile CarMaker Components
REM ====================================
echo [5a/8] Compiling CarMaker components...
call powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0\5a_carmaker_components.ps1" -RunMode inline
if %ERRORLEVEL% neq 0 (
    echo WARNING: CarMaker components build failed or skipped
)
echo.

REM ====================================
REM Step 5b: Compile dSPACE Libraries for CarMaker
REM ====================================
echo [5b/8] Compiling dSPACE libraries for CarMaker...
call powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0\5b_carmaker_dspace.ps1" -RunMode inline
if %ERRORLEVEL% neq 0 (
    echo WARNING: dSPACE library build failed or skipped
)
echo.

REM ====================================
REM Step 6: Build RealSimSocket MEX
REM ====================================
echo [6/8] Building RealSimSocket MEX...
call powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0\6_mex_realsim_socket.ps1" -RunMode inline
if %ERRORLEVEL% neq 0 (
    echo WARNING: RealSimSocket MEX build failed or skipped
)
echo.

REM ====================================
REM Step 7: Copy Files to Build Directory
REM ====================================

echo [7/8] Copying files to build directory...

REM Release executables
echo Copying executables...
if exist "%SOURCE_PATH%\TrafficLayer\x64\Release\TrafficLayer.exe" (
    copy /Y "%SOURCE_PATH%\TrafficLayer\x64\Release\TrafficLayer.exe" "%RELEASE_PATH%\TrafficLayer.exe" >nul
) else (
    echo WARNING: TrafficLayer.exe not found
)

REM Release VirtualEnvironment library
echo Copying VirtualEnvironment library...
if exist "%SOURCE_PATH%\VirtualEnvironment\x64\Release\VirtualEnvironment.lib" (
    copy /Y "%SOURCE_PATH%\VirtualEnvironment\x64\Release\VirtualEnvironment.lib" "%RELEASE_PATH%\VirtualEnvironment.lib" >nul
) else (
    echo WARNING: VirtualEnvironment.lib not found
)

REM Release CommonLib - libraries only
echo Copying CommonLib libraries...
if exist "%SOURCE_PATH%\CommonLib\libsumo" (
    xcopy /Y /E /I "%SOURCE_PATH%\CommonLib\libsumo" "%RELEASE_PATH%\CommonLib\libsumo" >nul
)
if defined YAML_MATLAB_DIR (
    if exist "%SOURCE_PATH%\%YAML_MATLAB_DIR%" (
        for %%d in ("%SOURCE_PATH%\%YAML_MATLAB_DIR%") do set "YAML_MATLAB_FOLDER=%%~nxd"
        xcopy /Y /E /I "%SOURCE_PATH%\%YAML_MATLAB_DIR%" "%RELEASE_PATH%\CommonLib\!YAML_MATLAB_FOLDER!" >nul
    )
)

REM Release CommonLib - MATLAB source files (exclude test/utility scripts)
echo Copying MATLAB source files...
if not exist "%RELEASE_PATH%\CommonLib" mkdir "%RELEASE_PATH%\CommonLib"
set "EXCLUDE_FILES=autoRealSimBatchCall.m main_autoRealSim.m startVissim.m"
for %%f in ("%SOURCE_PATH%\CommonLib\*.m") do (
    set "SKIP="
    for %%e in (%EXCLUDE_FILES%) do if "%%~nxf"=="%%e" set "SKIP=1"
    if not defined SKIP copy /Y "%%f" "%RELEASE_PATH%\CommonLib\" >nul 2>&1
)
if exist "%SOURCE_PATH%\CommonLib\RealSimSocket.mexw64" (
    copy /Y "%SOURCE_PATH%\CommonLib\RealSimSocket.mexw64" "%RELEASE_PATH%\CommonLib\" >nul
)

REM Release CarMaker files
echo Copying CarMaker files...
if defined CARMAKER_VERSIONS (
    for %%v in (%CARMAKER_VERSIONS%) do (
        for /f "tokens=1 delims=." %%m in ("%%v") do (
            set "CM_VERSION_SAFE=%%v"
            set "CM_VERSION_SAFE=!CM_VERSION_SAFE:.=_!"

            if exist "%SOURCE_PATH%\ProprietaryFiles\CM%%m_proj\src\CarMaker.win64.exe" (
                if not exist "%RELEASE_PATH%\CarMaker\CM%%m" mkdir "%RELEASE_PATH%\CarMaker\CM%%m"
                copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM%%m_proj\src\CarMaker.win64.exe" "%RELEASE_PATH%\CarMaker\CM%%m\" >nul
                copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM%%m_proj\src_cm4sl\libcarmaker4sl.mexw64" "%RELEASE_PATH%\CarMaker\CM%%m\" >nul

                REM Copy corresponding dSPACE library to CarMaker folder
                for %%f in ("%SOURCE_PATH%\CommonLib\libRealSimDsLib_*_CM!CM_VERSION_SAFE!.a") do (
                    copy /Y "%%f" "%RELEASE_PATH%\CarMaker\CM%%m\" >nul 2>&1
                )
            )
        )
    )
) else (
    echo WARNING: CarMaker versions not configured, skipping CarMaker artifact copy
)

REM Copy CarMaker utility files
if exist "%SOURCE_PATH%\CarMaker" (
    xcopy /Y /E /I "%SOURCE_PATH%\CarMaker" "%RELEASE_PATH%\CarMaker" >nul
)

REM Release VISSIM files
echo Copying VISSIM files...
if exist "%SOURCE_PATH%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim.dll" (
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim.dll" "%RELEASE_PATH%\" >nul
)
if exist "%SOURCE_PATH%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim_v2021.dll" (
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim_v2021.dll" "%RELEASE_PATH%\" >nul
)

echo.

REM ====================================
REM Step 8: Generate BUILD_INFO.txt
REM ====================================

REM Calculate build duration
set "BUILD_END=%date% %time%"
set "BUILD_END_SECONDS=%time:~0,2%%time:~3,2%%time:~6,2%"
set /a "DURATION_SECONDS=BUILD_END_SECONDS-BUILD_START_SECONDS"
if %DURATION_SECONDS% lss 0 set /a "DURATION_SECONDS+=86400"
set /a "DURATION_MIN=DURATION_SECONDS/60"
set /a "DURATION_SEC=DURATION_SECONDS%%60"
set "BUILD_DURATION=%DURATION_MIN%m %DURATION_SEC%s"

echo [8/8] Generating BUILD_INFO.txt...
call powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0\7_build_info.ps1" -RunMode inline -BuildStartTime "%BUILD_START%" -BuildDuration "%BUILD_DURATION%"
echo.
echo ==============================
echo RealSim: Release Complete!
echo ==============================
echo Files copied to: %RELEASE_PATH%
echo Build info: %RELEASE_PATH%\BUILD_INFO.txt
echo Build logs: %RS_BUILD_LOG%
echo Build summary: %RS_BUILD_SUMMARY%
echo.
pause
exit /b 0

:failed
echo.
echo ==============================
echo RealSim: Release FAILED!
echo ==============================
echo Check logs for details:
echo   Summary: %RS_BUILD_SUMMARY%
echo   Details: %RS_BUILD_LOG%
echo.
pause
exit /b 1
