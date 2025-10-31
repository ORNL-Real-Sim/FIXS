@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ====================================
REM Generate BUILD_INFO.txt
REM Creates comprehensive build metadata file in build/ directory
REM ====================================

set "SCRIPT_DIR=%~dp0"
set "RUN_MODE=%~1"

for %%I in ("%SCRIPT_DIR%..\..") do set "REPO_ROOT=%%~fI"
set "BUILD_DIR=%REPO_ROOT%\build"
set "DEPS_FILE=%REPO_ROOT%\dependencies.yaml"
set "OUTPUT_FILE=%BUILD_DIR%\BUILD_INFO.txt"
set "YAML_HELPER=%SCRIPT_DIR%yaml_helper.ps1"
set "CARMAKER_VERSIONS="
set "YAML_MATLAB_DIR="

REM Get build start time from command line or use current time
set "BUILD_START_TIME=%~2"
if not defined BUILD_START_TIME set "BUILD_START_TIME=%date% %time%"

REM Calculate build duration if end time provided
set "BUILD_DURATION=%~3"
if not defined BUILD_DURATION set "BUILD_DURATION=N/A"

if not exist "%BUILD_DIR%" (
    echo ERROR: Build directory not found: %BUILD_DIR%
    exit /b 1
)

REM Read CarMaker versions and YAMLMatlab location from dependencies.yaml
if exist "%YAML_HELPER%" (
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%DEPS_FILE%" -Section "carmaker" -ListKey "versions" -ReturnList`) do set "CARMAKER_VERSIONS=%%~I"
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%DEPS_FILE%" -Section "yaml_matlab" -Key "location"`) do set "YAML_MATLAB_DIR=%%~I"
)

echo Generating BUILD_INFO.txt...

REM Get system information
for /f "tokens=*" %%i in ('ver') do set "OS_VERSION=%%i"
for /f "tokens=*" %%i in ('hostname') do set "HOSTNAME=%%i"

REM Get git information
for /f "tokens=*" %%i in ('git rev-parse --abbrev-ref HEAD 2^>nul') do set "GIT_BRANCH=%%i"
for /f "tokens=*" %%i in ('git rev-parse --short HEAD 2^>nul') do set "GIT_COMMIT=%%i"
for /f "tokens=*" %%i in ('git log -1 --pretty^=%%s 2^>nul') do set "GIT_MSG=%%i"
for /f "tokens=*" %%i in ('git describe --tags --abbrev^=0 2^>nul') do set "GIT_TAG=%%i"
if not defined GIT_TAG set "GIT_TAG=No tag"

REM Get git status (check for uncommitted changes)
set "GIT_STATUS_CLEAN=1"
git diff-index --quiet HEAD -- 2>nul
if errorlevel 1 set "GIT_STATUS_CLEAN=0"

REM Count modified/staged files
set "GIT_MODIFIED=0"
for /f %%i in ('git status --porcelain 2^>nul ^| find /c /v ""') do set "GIT_MODIFIED=%%i"

REM Get dependency versions from dependencies.yaml
call :ReadYamlVersion "sumo" SUMO_VER
call :ReadYamlVersion "carla" CARLA_VER
call :ReadYamlVersion "carmaker" CARMAKER_VER
call :ReadYamlVersion "matlab" MATLAB_VER
call :ReadYamlVersion "dspace" DSPACE_VER

REM Get Visual Studio / MSBuild version
for /f "tokens=*" %%i in ('msbuild -version 2^>nul ^| findstr /r "^[0-9]"') do set "MSBUILD_VER=%%i"

REM Get tool paths
for /f "tokens=*" %%i in ('where msbuild 2^>nul') do set "MSBUILD_PATH=%%i"
for /f "tokens=*" %%i in ('where matlab 2^>nul') do set "MATLAB_PATH=%%i"

REM Define symbols for output
set "CHECK=✓"
set "CROSS=✗"

REM Scan build directory for files
set "COUNT_CORE=0"
set "COUNT_VISSIM=0"
set "COUNT_CM=0"
set "COUNT_MATLAB=0"

if exist "%BUILD_DIR%\TrafficLayer.exe" set /a COUNT_CORE+=1
if exist "%BUILD_DIR%\VirtualEnvironment.lib" set /a COUNT_CORE+=1
if exist "%BUILD_DIR%\DriverModel_RealSim.dll" set /a COUNT_VISSIM+=1
if exist "%BUILD_DIR%\DriverModel_RealSim_v2021.dll" set /a COUNT_VISSIM+=1

REM Count CarMaker items
for %%v in (%CARMAKER_VERSIONS%) do (
    for /f "tokens=1 delims=." %%m in ("%%v") do (
        if exist "%BUILD_DIR%\CarMaker\CM%%m" set /a COUNT_CM+=1
    )
)

REM Count dSPACE libraries
set "COUNT_DSLIB=0"
for %%f in ("%BUILD_DIR%\CarMaker\libRealSimDsLib_*.a") do set /a COUNT_DSLIB+=1

REM Count MATLAB files
if exist "%BUILD_DIR%\CommonLib\RealSimSocket.mexw64" set /a COUNT_MATLAB+=1
for %%f in ("%BUILD_DIR%\CommonLib\*.m") do set /a COUNT_MATLAB+=1

REM Build statistics
REM Count total possible components: 3 core + 2 VISSIM + N CarMaker versions
set /a "TOTAL_COMPONENTS=3+2"
for %%v in (%CARMAKER_VERSIONS%) do set /a "TOTAL_COMPONENTS+=1"
set /a "BUILT_COMPONENTS=%COUNT_CORE%+%COUNT_VISSIM%+%COUNT_CM%"
set /a "SKIPPED_COMPONENTS=%TOTAL_COMPONENTS%-%BUILT_COMPONENTS%"

REM ====================================
REM Generate BUILD_INFO.txt
REM ====================================

>"%OUTPUT_FILE%" echo ================================================================================
>>"%OUTPUT_FILE%" echo RealSim FIXS Build Information
>>"%OUTPUT_FILE%" echo ================================================================================
>>"%OUTPUT_FILE%" echo.
>>"%OUTPUT_FILE%" echo BUILD METADATA
>>"%OUTPUT_FILE%" echo --------------
>>"%OUTPUT_FILE%" echo Build Date:           %BUILD_START_TIME%
>>"%OUTPUT_FILE%" echo Build Duration:       %BUILD_DURATION%
>>"%OUTPUT_FILE%" echo Build Configuration:  Release
>>"%OUTPUT_FILE%" echo Build Platform:       x64
>>"%OUTPUT_FILE%" echo.
>>"%OUTPUT_FILE%" echo SOURCE INFORMATION
>>"%OUTPUT_FILE%" echo ------------------
>>"%OUTPUT_FILE%" echo Git Branch:           %GIT_BRANCH%
>>"%OUTPUT_FILE%" echo Git Commit:           %GIT_COMMIT% (%GIT_MSG%)
>>"%OUTPUT_FILE%" echo Git Tag:              %GIT_TAG%
if %GIT_MODIFIED% gtr 0 (
    >>"%OUTPUT_FILE%" echo Git Status:           Modified ^(%GIT_MODIFIED% file^(s^)^)
) else (
    >>"%OUTPUT_FILE%" echo Git Status:           Clean
)
>>"%OUTPUT_FILE%" echo.
>>"%OUTPUT_FILE%" echo DEPENDENCY VERSIONS
>>"%OUTPUT_FILE%" echo -------------------
>>"%OUTPUT_FILE%" echo Simulators:
if defined SUMO_VER >>"%OUTPUT_FILE%" echo   SUMO:               %SUMO_VER%
if defined CARLA_VER >>"%OUTPUT_FILE%" echo   CARLA:              %CARLA_VER%
if defined CARMAKER_VER >>"%OUTPUT_FILE%" echo   CarMaker:           %CARMAKER_VER%
>>"%OUTPUT_FILE%" echo   VISSIM:             2022
>>"%OUTPUT_FILE%" echo.
>>"%OUTPUT_FILE%" echo Development Tools:
if defined MATLAB_VER >>"%OUTPUT_FILE%" echo   MATLAB:             %MATLAB_VER%
if defined DSPACE_VER >>"%OUTPUT_FILE%" echo   dSPACE:             %DSPACE_VER%
if defined MSBUILD_VER >>"%OUTPUT_FILE%" echo   MSBuild:            %MSBUILD_VER%
>>"%OUTPUT_FILE%" echo.
>>"%OUTPUT_FILE%" echo Libraries:
>>"%OUTPUT_FILE%" echo   yaml-cpp:           ^(from CommonLib/yaml-cpp^)
if defined SUMO_VER >>"%OUTPUT_FILE%" echo   libsumo:            ^(from SUMO %SUMO_VER%^)
>>"%OUTPUT_FILE%" echo.
>>"%OUTPUT_FILE%" echo BUILD RESULTS
>>"%OUTPUT_FILE%" echo -------------

REM Core Components
>>"%OUTPUT_FILE%" echo Core Components:
call :FileStatus "%BUILD_DIR%\TrafficLayer.exe" "TrafficLayer.exe"
call :FileStatus "%BUILD_DIR%\VirtualEnvironment.lib" "VirtualEnvironment.lib"
>>"%OUTPUT_FILE%" echo.

REM VISSIM Interface
>>"%OUTPUT_FILE%" echo VISSIM Interface:
call :FileStatus "%BUILD_DIR%\DriverModel_RealSim.dll" "DriverModel_RealSim.dll" "Not built - VISSIMserver not found"
call :FileStatus "%BUILD_DIR%\DriverModel_RealSim_v2021.dll" "DriverModel_RealSim_v2021.dll" "Not built - VISSIMserver not found"
>>"%OUTPUT_FILE%" echo.

REM CarMaker Integration
>>"%OUTPUT_FILE%" echo CarMaker Integration:
for %%v in (%CARMAKER_VERSIONS%) do (
    for /f "tokens=1 delims=." %%m in ("%%v") do (
        call :FileStatus "%BUILD_DIR%\CarMaker\CM%%m" "CarMaker %%v ^(CM%%m^)" "Skipped - CM%%m_proj not found"
    )
)

REM List dSPACE libraries
for %%f in ("%BUILD_DIR%\CarMaker\libRealSimDsLib_*.a") do (
    call :FileStatus "%%f" "%%~nxf"
)

REM List CarMaker Python utilities
for %%f in ("%BUILD_DIR%\CarMaker\*.py") do (
    call :FileStatus "%%f" "%%~nxf"
)
>>"%OUTPUT_FILE%" echo.

REM MATLAB/Simulink
>>"%OUTPUT_FILE%" echo MATLAB/Simulink:
call :FileStatus "%BUILD_DIR%\CommonLib\RealSimSocket.mexw64" "RealSimSocket.mexw64"
if exist "%BUILD_DIR%\CommonLib\*.m" (
    >>"%OUTPUT_FILE%" echo   %CHECK% CommonLib MATLAB files                  [%COUNT_MATLAB% files]
)
if defined YAML_MATLAB_DIR (
    for %%d in ("%REPO_ROOT%\%YAML_MATLAB_DIR%") do set "YAML_MATLAB_FOLDER=%%~nxd"
    if exist "%BUILD_DIR%\CommonLib\!YAML_MATLAB_FOLDER!" (
        >>"%OUTPUT_FILE%" echo   %CHECK% !YAML_MATLAB_FOLDER!/                       [Library]
    )
)
if exist "%BUILD_DIR%\CommonLib\libsumo" (
    >>"%OUTPUT_FILE%" echo   %CHECK% libsumo/                                [Library]
)
>>"%OUTPUT_FILE%" echo.

REM Build warnings (parse from build_results.log if available)
if exist "%SCRIPT_DIR%build_results.log" (
    >>"%OUTPUT_FILE%" echo BUILD WARNINGS
    >>"%OUTPUT_FILE%" echo --------------
    >>"%OUTPUT_FILE%" echo See build logs for detailed warning information
    >>"%OUTPUT_FILE%" echo.
)

REM Build environment
>>"%OUTPUT_FILE%" echo BUILD ENVIRONMENT
>>"%OUTPUT_FILE%" echo -----------------
>>"%OUTPUT_FILE%" echo Host OS:              %OS_VERSION%
>>"%OUTPUT_FILE%" echo Build Tool Paths:
if defined MSBUILD_PATH >>"%OUTPUT_FILE%" echo   msbuild:            %MSBUILD_PATH%
if defined MATLAB_PATH >>"%OUTPUT_FILE%" echo   MATLAB:             %MATLAB_PATH%
if defined DSPACE_VER >>"%OUTPUT_FILE%" echo   dSPACE:             C:\Program Files\dSPACE ConfigurationDesk %DSPACE_VER%
>>"%OUTPUT_FILE%" echo.

REM Package contents
>>"%OUTPUT_FILE%" echo PACKAGE CONTENTS
>>"%OUTPUT_FILE%" echo ----------------
>>"%OUTPUT_FILE%" echo build/
if exist "%BUILD_DIR%\TrafficLayer.exe" >>"%OUTPUT_FILE%" echo   ├── TrafficLayer.exe
if exist "%BUILD_DIR%\VirtualEnvironment.lib" >>"%OUTPUT_FILE%" echo   ├── VirtualEnvironment.lib
if exist "%BUILD_DIR%\DriverModel_RealSim.dll" >>"%OUTPUT_FILE%" echo   ├── DriverModel_RealSim.dll
if exist "%BUILD_DIR%\DriverModel_RealSim_v2021.dll" >>"%OUTPUT_FILE%" echo   ├── DriverModel_RealSim_v2021.dll
if exist "%BUILD_DIR%\CommonLib" (
    >>"%OUTPUT_FILE%" echo   ├── CommonLib/
    if exist "%BUILD_DIR%\CommonLib\RealSimSocket.mexw64" >>"%OUTPUT_FILE%" echo   │   ├── RealSimSocket.mexw64
    if exist "%BUILD_DIR%\CommonLib\*.m" >>"%OUTPUT_FILE%" echo   │   ├── *.m files ^(%COUNT_MATLAB% files^)
    if exist "%BUILD_DIR%\CommonLib\libsumo" >>"%OUTPUT_FILE%" echo   │   ├── libsumo/
    if defined YAML_MATLAB_DIR (
        for %%d in ("%REPO_ROOT%\%YAML_MATLAB_DIR%") do set "YAML_MATLAB_FOLDER=%%~nxd"
        if exist "%BUILD_DIR%\CommonLib\!YAML_MATLAB_FOLDER!" >>"%OUTPUT_FILE%" echo   │   └── !YAML_MATLAB_FOLDER!/
    )
)
if exist "%BUILD_DIR%\CarMaker" (
    >>"%OUTPUT_FILE%" echo   └── CarMaker/
    for %%v in (%CARMAKER_VERSIONS%) do (
        for /f "tokens=1 delims=." %%m in ("%%v") do (
            if exist "%BUILD_DIR%\CarMaker\CM%%m" >>"%OUTPUT_FILE%" echo       ├── CM%%m/
        )
    )
    for %%f in ("%BUILD_DIR%\CarMaker\libRealSimDsLib_*.a") do (
        >>"%OUTPUT_FILE%" echo       ├── %%~nxf
    )
    for %%f in ("%BUILD_DIR%\CarMaker\*.py") do (
        >>"%OUTPUT_FILE%" echo       └── %%~nxf
    )
)
>>"%OUTPUT_FILE%" echo.

REM Notes
>>"%OUTPUT_FILE%" echo NOTES
>>"%OUTPUT_FILE%" echo -----
if %BUILT_COMPONENTS% lss %TOTAL_COMPONENTS% (
    >>"%OUTPUT_FILE%" echo - Partial build: %BUILT_COMPONENTS% of %TOTAL_COMPONENTS% components built successfully
    >>"%OUTPUT_FILE%" echo - %SKIPPED_COMPONENTS% components skipped due to missing source directories
) else (
    >>"%OUTPUT_FILE%" echo - Full build: All %TOTAL_COMPONENTS% components built successfully
)
if %COUNT_DSLIB% gtr 0 (
    >>"%OUTPUT_FILE%" echo - dSPACE libraries available for CarMaker 13.1.3 ^(11.1.2 also supported^)
)
if %GIT_MODIFIED% gtr 0 (
    >>"%OUTPUT_FILE%" echo - Build contains uncommitted changes
)
>>"%OUTPUT_FILE%" echo - See dependencies.yaml for required external tool versions
>>"%OUTPUT_FILE%" echo.
>>"%OUTPUT_FILE%" echo ================================================================================
>>"%OUTPUT_FILE%" echo Generated by dispatch.bat on %date% %time%
>>"%OUTPUT_FILE%" echo ================================================================================

echo BUILD_INFO.txt generated: %OUTPUT_FILE%

if /I "%RUN_MODE%"=="standalone" (
    echo.
    pause
)
exit /b 0

REM ====================================
REM Helper Functions
REM ====================================

:FileStatus
set "FILEPATH=%~1"
set "FILENAME=%~2"
set "MISSING_MSG=%~3"

if exist "%FILEPATH%" (
    for %%A in ("%FILEPATH%") do (
        set "SIZE=%%~zA"
        set "TIMESTAMP=%%~tA"
    )
    set /a "SIZE_KB=!SIZE! / 1024"
    >>"%OUTPUT_FILE%" echo   %CHECK% %FILENAME%
) else (
    if defined MISSING_MSG (
        >>"%OUTPUT_FILE%" echo   %CROSS% %FILENAME%                            [%MISSING_MSG%]
    )
)
exit /b 0

:ReadYamlVersion
set "SECTION=%~1"
set "OUT_VAR=%~2"
set "RESULT="

if exist "%YAML_HELPER%" (
    for /f "usebackq tokens=* delims=" %%I in (`powershell -NoProfile -File "%YAML_HELPER%" -File "%DEPS_FILE%" -Section "%SECTION%" 2^>nul`) do (
        if not defined RESULT set "RESULT=%%~I"
    )
)

set "%OUT_VAR%=%RESULT%"
exit /b 0