@echo off
REM ============================================================
REM Build External Libraries (yaml-cpp)
REM CI-safe version: no pauses, proper error handling
REM ============================================================

REM Move to repository root (this script expected at scripts\dispatch)
cd ..\..

REM ------------------------------------------------------------
REM Read Visual Studio version from dependencies.yaml
REM ------------------------------------------------------------
for /f "delims=" %%a in ('powershell -ExecutionPolicy Bypass -File scripts\dispatch\yaml_helper.ps1 -File dependencies.yaml -Section visual_studio') do set VS_VERSION=%%a

if "%VS_VERSION%"=="" (
    echo ERROR: Could not read Visual Studio version from dependencies.yaml
    exit /b 1
)

REM ------------------------------------------------------------
REM Convert VS version to a CMake generator string
REM Example: VS_VERSION=2019 -> "Visual Studio 14 2019"
REM (The math is: 2019 - 2005 = 14)
REM ------------------------------------------------------------
set /a "VS_CMAKE_VER=%VS_VERSION%-2005"
set "CMAKE_GENERATOR=Visual Studio %VS_CMAKE_VER% %VS_VERSION%"

echo Using CMake Generator: %CMAKE_GENERATOR%

REM ------------------------------------------------------------
REM Build yaml-cpp
REM ------------------------------------------------------------
cd CommonLib\yaml-cpp

if not exist build (
    mkdir build
)

cd build

echo Configuring yaml-cpp...
cmake -G "%CMAKE_GENERATOR%" ..
if errorlevel 1 exit /b 1

echo Building yaml-cpp (Release)...
cmake --build . --config Release
if errorlevel 1 exit /b 1

echo Building yaml-cpp (Debug)...
cmake --build . --config Debug
if errorlevel 1 exit /b 1

echo yaml-cpp build completed successfully.
exit /b 0
