@echo off
REM ============================================================================
REM  Launch CARLA as a server from the source build, no editor GUI / no Play
REM  press. Uses the editor binary in -game mode (loads uncooked content
REM  directly). Generalized from the ANL launch_carla_roosevelt.bat -- map and
REM  paths come from carla.env, nothing Roosevelt-specific.
REM
REM  Usage:  launch_carla.bat [map_content_path]
REM    map_content_path  optional; overrides DEFAULT_MAP from carla.env.
REM                      Ignored in spirit for OpenDRIVE-standalone scenarios
REM                      (load_opendrive_world.py replaces the world after connect).
REM ============================================================================
setlocal
set HERE=%~dp0

REM --- load carla.env (KEY=VALUE lines; '#' comments) -------------------------
set "ENVFILE=%HERE%carla.env"
if not exist "%ENVFILE%" (
    echo [ERROR] %ENVFILE% not found. Copy carla.env.example to carla.env and edit it.
    pause & exit /b 1
)
for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%ENVFILE%") do set "%%A=%%B"

if not defined UE4_ROOT ( echo [ERROR] UE4_ROOT not set ^(carla.env or system env^). & pause & exit /b 1 )
if not exist "%CARLA_UPROJECT%" ( echo [ERROR] CARLA_UPROJECT not found: %CARLA_UPROJECT% & pause & exit /b 1 )

set "MAP=%~1"
if "%MAP%"=="" set "MAP=%DEFAULT_MAP%"

set "UE4EDITOR=%UE4_ROOT%\Engine\Binaries\Win64\UE4Editor.exe"
if not exist "%UE4EDITOR%" ( echo [ERROR] UE4Editor.exe not found: %UE4EDITOR% & pause & exit /b 1 )

echo Launching CARLA (-game) on %MAP% ...
echo   uproject: %CARLA_UPROJECT%
REM For render-bound viz you can append -quality-level=Low below (cheap render: no
REM motion blur, simpler shadows/AA -> higher frame rate). Left off by default.
start "CARLA" "%UE4EDITOR%" "%CARLA_UPROJECT%" "%MAP%" -game -windowed -ResX=1280 -ResY=720
endlocal
