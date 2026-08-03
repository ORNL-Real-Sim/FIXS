@echo off
REM Build + run the #174 simulator-free guard (no CarMaker, no Carla, no server).
setlocal
cd /d "%~dp0"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" ( echo ERROR: vswhere not found; install VS 2022. & exit /b 1 )
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -property installationPath`) do set "VSPATH=%%i"
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul

set "CLIB=..\..\CommonLib"
set "INC=/I%CLIB% /I%CLIB%\yaml-cpp\include /I%CLIB%\yaml-cpp\build\include"
set "YAML=%CLIB%\yaml-cpp\build\Release\yaml-cpp.lib"

echo === smoke_interface (interface only, SDK-free) ===
cl /std:c++17 /EHsc /MD /nologo /W3 /DWIN32 /D_CRT_SECURE_NO_WARNINGS smoke_interface.cpp /Fe:smoke_interface.exe
if errorlevel 1 ( echo SMOKE BUILD_FAILED & exit /b 1 )
.\smoke_interface.exe
if errorlevel 1 ( echo SMOKE RUN_FAILED & exit /b 1 )

echo.
echo === replay_core (VirEnvCore + mock, SDK-free) ===
cl /std:c++17 /EHsc /MD /nologo /W3 /DWIN32 /D_CRT_SECURE_NO_WARNINGS %INC% replay_core.cpp "%CLIB%\VirEnvCore.cpp" "%CLIB%\MsgHelper.cpp" "%CLIB%\SocketHelper.cpp" "%CLIB%\ConfigHelper.cpp" /Fe:replay_core.exe /link "%YAML%" ws2_32.lib
if errorlevel 1 ( echo REPLAY BUILD_FAILED & exit /b 1 )
.\replay_core.exe
if errorlevel 1 ( echo REPLAY RUN_FAILED & exit /b 1 )
echo.
echo ALL VIRENVCORE GUARD TESTS PASSED
endlocal
