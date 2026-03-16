@echo off
REM ====================================
REM Build External Libraries
REM Builds yaml-cpp (and libevent if needed)
REM ====================================

cd ..\..

REM Read Visual Studio version from dependencies.yaml
for /f "delims=" %%a in ('powershell -ExecutionPolicy Bypass -File scripts\dispatch\yaml_helper.ps1 -File dependencies.yaml -Section visual_studio') do set VS_VERSION=%%a

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

cd .\CommonLib\yaml-cpp
if not exist build md build
cd build
cmake -G "%CMAKE_GENERATOR%" ..
cmake --build . --config Release
cmake --build . --config Debug

REM Only pause if not called from dispatch
if not "%1"=="inline" pause