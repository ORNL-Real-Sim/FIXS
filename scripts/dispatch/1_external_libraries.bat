@echo off
REM ====================================
REM Build External Libraries
REM Builds yaml-cpp (and libevent if needed)
REM ====================================

REM Resolve repo root relative to this script's location
set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..\..") do set "REPO_ROOT=%%~fI"

REM Read Visual Studio version from dependencies.yaml
for /f "delims=" %%a in ('powershell -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%yaml_helper.ps1" -File "%REPO_ROOT%\dependencies.yaml" -Section visual_studio') do set VS_VERSION=%%a

if "%VS_VERSION%"=="" (
    echo ERROR: Could not read Visual Studio version from dependencies.yaml
    pause
    exit /b 1
)

REM Map version to CMake generator (auto-parse VS year to CMake version)
set /a "VS_CMAKE_VER=%VS_VERSION%-2005"
set "CMAKE_GENERATOR=Visual Studio %VS_CMAKE_VER% %VS_VERSION%"

echo Using CMake Generator: %CMAKE_GENERATOR%

REM Note: libevent is not used at this moment
REM cd .\CommonLib\libevent
REM if not exist build md build
REM cd build
REM cmake -G "%CMAKE_GENERATOR%" -DEVENT__DISABLE_MBEDTLS=ON ..
REM cmake --build . --config Release
REM cmake --build . --config Debug
REM cd ..\..\..\

set "YAMLCPP_DIR=%REPO_ROOT%\CommonLib\yaml-cpp"
set "YAMLCPP_BUILD=%YAMLCPP_DIR%\build"

if not exist "%YAMLCPP_BUILD%" mkdir "%YAMLCPP_BUILD%"
pushd "%YAMLCPP_BUILD%"
cmake -G "%CMAKE_GENERATOR%" "%YAMLCPP_DIR%"
cmake --build . --config Release
cmake --build . --config Debug
popd

REM Only pause if not called from dispatch
if not "%1"=="inline" pause