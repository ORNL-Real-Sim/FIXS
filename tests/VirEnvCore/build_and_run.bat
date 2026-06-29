@echo off
REM Build + run the #174 simulator-free guard (IVirEnvBackend smoke). No CarMaker,
REM no Carla, no server -- proves the core verb interface compiles + works SDK-free.
setlocal
cd /d "%~dp0"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" ( echo ERROR: vswhere not found; install VS 2022. & exit /b 1 )
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -property installationPath`) do set "VSPATH=%%i"
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /std:c++17 /EHsc /nologo /W3 smoke_interface.cpp /Fe:smoke_interface.exe
if errorlevel 1 ( echo BUILD_FAILED & exit /b 1 )
echo --- running ---
smoke_interface.exe
endlocal
