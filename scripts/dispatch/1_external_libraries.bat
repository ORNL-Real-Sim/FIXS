@echo off
REM ====================================
REM Build External Libraries
REM Builds yaml-cpp (and libevent if needed)
REM ====================================

cd ..\..

REM Note: libevent is not used at this moment
REM cd .\CommonLib\libevent
REM if not exist build md build
REM cd build
REM cmake -G "Visual Studio 17 2022" -DEVENT__DISABLE_MBEDTLS=ON ..
REM cmake --build . --config Release
REM cmake --build . --config Debug
REM cd ..\..\..\

cd .\CommonLib\yaml-cpp
if not exist build md build
cd build
cmake -G "Visual Studio 17 2022" ..
cmake --build . --config Release
cmake --build . --config Debug

pause