@echo off
REM FIXS_FRONTDOOR: 1
REM ==========================================================================
REM  FIXS co-simulation - the front door (Windows). Double-click to run.
REM
REM  Download this one file into a repo, run it, and it installs FIXS and runs a
REM  co-sim. Nothing else is required to start.
REM
REM  WHAT THIS FILE IS ALLOWED TO KNOW is deliberately almost nothing: where FIXS
REM  comes from, whether it is installed, and which python to hand over to. Every
REM  option, every menu and every error message belongs to the engine, and this
REM  forwards its whole command line untouched - so there is no flag list here to
REM  drift from the engine's, which is exactly how the old per-repo wrappers
REM  accumulated bugs (see FIXS#313).
REM
REM  It is authored in FIXS (scripts\frontdoor\) and published as a release
REM  asset. Do not edit your copy: an update will tell you when the contract
REM  version above has moved, and the answer is to re-download, not to patch.
REM
REM      FIXS.bat --help              every option, from the engine
REM      FIXS.bat --update-fixs       fetch or refresh the FIXS build
REM ==========================================================================
setlocal enabledelayedexpansion
set "ROOT=%~dp0"
set "ROOTARG=%ROOT:~0,-1%"
set "FRONTDOOR_CONTRACT=1"
set "DEFAULT_REPO=ORNL-Real-Sim/FIXS"
set "MANIFEST=%ROOT%fixs.json"
set "LEGACY=%ROOT%fixs_sources.txt"

REM The two values needed before any engine code exists on disk: where FIXS comes
REM from, and which release this repo pins. They live in fixs.json, the same file
REM that declares the applications. Read with a regex and ONLY these keys - that
REM is the whole contract, so no schema growth reaches this file. Batch cannot do
REM this itself, hence the powershell one-liners; fixs_sources.txt is still
REM honoured for repos integrated before fixs.json existed.
call :json_field FIXS_REPO    fixs repo
call :json_field FIXS_VERSION fixs version
call :json_field MANIFEST_ENV fixs env
if not defined FIXS_REPO    call :txt_field FIXS_REPO    fixs_repo
if not defined FIXS_VERSION call :txt_field FIXS_VERSION fixs_default_version
if not defined FIXS_REPO set "FIXS_REPO=%DEFAULT_REPO%"

REM The env applications run in. FIXS defaults to 'realsim', the name its own
REM environment.yml carries; a repo that wants its apps' extra packages kept out
REM of the engine's env names its own here. Already set? that wins.
if not defined FIXS_ENV_NAME if defined MANIFEST_ENV set "FIXS_ENV_NAME=%MANIFEST_ENV%"

REM --update-fixs is the ONE name this file answers, because it is the one action
REM that must work before there is an engine to answer it. Everything else falls
REM through to run_cosim, whose --help is the reference.
if /I "%~1"=="--update-fixs" (
    call :fetch_fixs "%~2"
    if errorlevel 1 exit /b 1
    call :seed_manifest
    exit /b 0
)

REM The gate is FIXS_VERSION.txt, not any .py: the updater writes that marker LAST
REM and only on a complete install, whereas the python ships inside the build zip
REM and exists the moment it is unpacked - before the native runtime is fetched. A
REM fetch that died at the runtime step used to leave a headers-only bundle that
REM still satisfied the old gate, and the co-sim then failed with "Unable to
REM locate SUMO library directory" instead of anything about the failed update.
if not exist "%ROOT%FIXS\FIXS_VERSION.txt" (
    if exist "%ROOT%FIXS" (
        echo [FIXS] the FIXS build is incomplete ^(no FIXS_VERSION.txt^) - refetching ...
    ) else (
        echo [FIXS] FIXS is not installed here - fetching it first ...
    )
    call :fetch_fixs ""
    if errorlevel 1 (
        echo [FIXS] setup failed - see above. Not continuing.
        pause
        exit /b 1
    )
    call :seed_manifest
)

call :check_contract
call :findpy || exit /b 1

"%PY%" "%ROOT%FIXS\cosim\run_cosim.py" %*
REM Captured BEFORE pause: pause resets ERRORLEVEL to 0, so exiting after it would
REM report every failed run as a success to whatever called this.
set "RC=%ERRORLEVEL%"
pause
exit /b %RC%

REM ------------------------------------------------------------------ helpers

:json_field
REM %1 = variable to set, %2 = block, %3 = key
set "%~1="
if not exist "%MANIFEST%" exit /b 0
for /f "usebackq delims=" %%V in (`powershell -NoProfile -Command ^
  "$ErrorActionPreference='SilentlyContinue';" ^
  "$d = Get-Content -Raw '%MANIFEST%' | ConvertFrom-Json;" ^
  "$v = $d.'%~2'.'%~3'; if ($v) { $v }"`) do set "%~1=%%V"
exit /b 0

:txt_field
REM %1 = variable to set, %2 = key in the legacy fixs_sources.txt
set "%~1="
if not exist "%LEGACY%" exit /b 0
for /f "usebackq tokens=1,* delims==" %%A in (`findstr /b /i /c:"%~2" "%LEGACY%"`) do (
    for /f "tokens=* delims= " %%V in ("%%B") do set "%~1=%%V"
)
exit /b 0

REM Bootstrap. The updater lives in FIXS and is fetched from the release being
REM installed, so the unpacker always matches the bundle it unpacks (#272). What
REM stays here is only which repo, which ref, and run it against our root - a
REM contract that does not change when the release format does, which is what
REM makes it safe for this file to sit in every application repo.
:fetch_fixs
set "WANT=%~1"
set "REF=%WANT%"
if not defined REF set "REF=%FIXS_VERSION%"
REM 'main' is the last resort, not a version: a script taken from main can still
REM list the releases and hand off to whichever one is chosen.
if not defined REF set "REF=main"
set "UPD=%TEMP%\update_fixs_%RANDOM%%RANDOM%.ps1"
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "$repo='%FIXS_REPO%'; $ref='%REF%'; $out='%UPD%';" ^
  "function Get-It($r){ Invoke-WebRequest -UseBasicParsing -TimeoutSec 30 -Uri \"https://raw.githubusercontent.com/$repo/$r/scripts/update_fixs.ps1\" -OutFile $out }" ^
  "try { Get-It $ref } catch { if ($ref -ne 'main') { Write-Host \"[FIXS] no updater at '$ref'; falling back to 'main'.\"; Get-It 'main' } else { throw } }"
if errorlevel 1 (
    echo [FIXS] Could not download the FIXS updater from %FIXS_REPO%.
    echo [FIXS] Check your network, or that the repo is reachable.
    exit /b 1
)
set "UPDARGS=-Root "%ROOTARG%" -Repo "%FIXS_REPO%" -SelfRef "%REF%""
if not "%WANT%"=="" set "UPDARGS=!UPDARGS! -Version "%WANT%""
if defined FIXS_VERSION set "UPDARGS=!UPDARGS! -DefaultVersion "%FIXS_VERSION%""
powershell -NoProfile -ExecutionPolicy Bypass -File "%UPD%" !UPDARGS!
set "RC=!ERRORLEVEL!"
del "%UPD%" >nul 2>nul
exit /b !RC!

REM Record what was installed, so a fresh clone of this repo bootstraps the same
REM engine without anyone hand-writing config. Only ever CREATED, never edited:
REM once the file exists it is the repo's, and it is where apps get declared.
:seed_manifest
if exist "%MANIFEST%" exit /b 0
if exist "%LEGACY%"   exit /b 0
if not exist "%ROOT%FIXS\FIXS_VERSION.txt" exit /b 0
for /f "usebackq tokens=1" %%V in ("%ROOT%FIXS\FIXS_VERSION.txt") do (
    if not defined INSTALLED set "INSTALLED=%%V"
)
if not defined INSTALLED exit /b 0
> "%MANIFEST%" (
    echo {
    echo   "schema": 2,
    echo   "fixs": { "repo": "%FIXS_REPO%", "version": "%INSTALLED%" },
    echo   "apps": []
    echo }
)
echo [FIXS] wrote %MANIFEST% - commit it; it pins the engine this repo runs.
echo [FIXS] Declare your applications in its "apps" list when you have some.
exit /b 0

REM Say so when this file is older than the engine it just installed. It is never
REM overwritten in place: cmd re-reads a running .bat by byte offset, and this
REM file is also the repo's committed entry point. Re-copying is the fix.
:check_contract
set "SHIPPED=%ROOT%FIXS\frontdoor\FIXS.bat"
if not exist "%SHIPPED%" exit /b 0
set "WANTC="
for /f "usebackq tokens=3" %%C in (`findstr /b /c:"REM FIXS_FRONTDOOR:" "%SHIPPED%"`) do set "WANTC=%%C"
if not defined WANTC exit /b 0
if "%WANTC%"=="%FRONTDOOR_CONTRACT%" exit /b 0
echo [FIXS] this FIXS.bat is contract v%FRONTDOOR_CONTRACT%; the installed
echo [FIXS] build expects v%WANTC%. Copy FIXS\frontdoor\FIXS.bat over it.
exit /b 0

REM Any python 3 is enough: run_cosim re-execs under the interpreter carla.json
REM names, so conda never needs to be active and this need not know the env.
:findpy
set "PY=python"
where python >nul 2>nul || set "PY=py"
where %PY% >nul 2>nul || (
    echo [FIXS] No python 3 found on PATH. Install one, then run FIXS.bat --setup
    pause
    exit /b 1
)
exit /b 0
