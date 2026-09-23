@echo off
REM FIXS_FRONTDOOR: 1
REM ==========================================================================
REM  FIXS co-simulation - the front door (Windows). Double-click to run.
REM
REM  Download this one file into a repo, run it, and it installs FIXS and runs a
REM  co-sim. Nothing else is required to start.
REM
REM  WHAT THIS FILE IS ALLOWED TO KNOW is deliberately little: where FIXS comes
REM  from, whether it is installed, which python to hand over to, and how to be
REM  pleasant to someone who double-clicked it. Every option, every menu and
REM  every error message belongs to the engine, and this forwards its command
REM  line untouched - there is no flag translation here to drift from the
REM  engine's, which is exactly how the old per-repo wrappers accumulated bugs
REM  (see FIXS#313). The option list below is a short guide to engine names, not
REM  a second parser.
REM
REM  The double-click prompt, the pause-only-when-double-clicked rule and the
REM  python search come from FIXS_Applications' run_cosim.bat, where each one
REM  answered a real support question; they move here so that repo loses nothing
REM  by switching to this file.
REM
REM  It is authored in FIXS (scripts\frontdoor\) and published as a release
REM  asset. Do not edit your copy: an update will tell you when the contract
REM  version above has moved, and the answer is to re-download, not to patch.
REM
REM      FIXS.bat                     run (double-click: asks for options first)
REM      FIXS.bat --gui               the FIXS window
REM      FIXS.bat --help              the common options
REM      FIXS.bat --update-fixs       fetch or refresh the FIXS build
REM ==========================================================================
setlocal enabledelayedexpansion
set "ROOT=%~dp0"
set "ROOTARG=%ROOT:~0,-1%"
set "FRONTDOOR_CONTRACT=1"
set "DEFAULT_REPO=ORNL-Real-Sim/FIXS"
set "MANIFEST=%ROOT%fixs.json"
set "LEGACY=%ROOT%fixs_sources.txt"

REM Double-clicked, or run from a console? Explorer starts us as `cmd /c <path>`,
REM so our own filename is in the PARENT command line; a console session's is
REM not. Two things depend on it: whether to offer somewhere to type options (a
REM double-click has no command line to put them on), and whether output that
REM would otherwise vanish with the window gets a pause - a pause a scripted
REM caller must never get, or it sits on "Press any key" with nobody there.
REM Only asked with NO arguments: a scripted `cmd /c "...\FIXS.bat --map x"`
REM matches this test too, and a double-click never has arguments. The options
REM prompt re-invokes us WITH them, so it says so through FIXS_FROM_PROMPT.
REM findstr by full path: a PATH with unix tools ahead of System32 resolves a bare
REM `find`/`findstr` to something else, and the test then silently says "console".
set "DBLCLICK="
if defined FIXS_FROM_PROMPT goto :dbl_yes
if not "%~1"=="" goto :dbl_done
echo %cmdcmdline% | "%SystemRoot%\System32\findstr.exe" /i /c:"%~nx0" >nul && set "DBLCLICK=1"
goto :dbl_done
:dbl_yes
set "DBLCLICK=1"
:dbl_done

REM The values needed before any engine code exists on disk: where FIXS comes
REM from, which release this repo pins, and which env its apps run in. They live
REM in fixs.json, the same file that declares the applications, and ONLY these
REM keys are read here - so no schema growth reaches this file. fixs_sources.txt
REM is still honoured for repos integrated before fixs.json existed.
call :json_field FIXS_REPO    fixs repo
call :json_field FIXS_VERSION fixs version
call :json_field MANIFEST_ENV fixs env
if not defined FIXS_REPO    call :txt_field FIXS_REPO    fixs_repo
if not defined FIXS_VERSION call :txt_field FIXS_VERSION fixs_default_version
if not defined MANIFEST_ENV call :txt_field MANIFEST_ENV fixs_env
if not defined FIXS_REPO set "FIXS_REPO=%DEFAULT_REPO%"

REM The env applications run in. FIXS defaults to 'realsim', the name its own
REM environment.yml carries; a repo that wants its apps' extra packages kept out
REM of the engine's env names its own here. Already set? that wins.
if not defined FIXS_ENV_NAME if defined MANIFEST_ENV set "FIXS_ENV_NAME=%MANIFEST_ENV%"

REM A double-click has nowhere to type options, so everything but a plain run
REM would be reachable only from a terminal. Ask once. Whatever is typed is handed
REM back to this same file as an ordinary command line, so the answer means
REM exactly what the command line means and there is no second grammar here.
REM 'gui' alone is the one shorthand: it is the answer most people double-clicking
REM want, and it is short to type.
if not defined DBLCLICK goto :no_prompt
if not "%~1"=="" goto :no_prompt
echo.
echo FIXS - co-simulation
echo.
call :print_options
echo.
echo Press Enter to run, type  gui  for the FIXS window, or type any options above.
set "OPTS="
set /p "OPTS=  options: "
REM Empty - or EOF, which is what something scripted gets - falls through to the
REM ordinary run. Never recurse on nothing.
if not defined OPTS goto :no_prompt
if /I "!OPTS!"=="gui" set "OPTS=--gui"
set "FIXS_FROM_PROMPT=1"
call "%~f0" !OPTS!
exit /b !ERRORLEVEL!
:no_prompt

REM The only names this file answers itself: help, because it must work before
REM there is an engine to ask; and --update-fixs, the one action that must work
REM before there is an engine at all. goto, not an inline block: cmd expands %~2
REM into a parenthesized block's text while scanning for its closing paren, so a
REM value holding ')' would end the block early.
if /I "%~1"=="--help" goto :usage
if /I "%~1"=="-h"     goto :usage
if /I "%~1"=="/?"     goto :usage
if /I "%~1"=="--update-fixs" goto :do_update

REM Python BEFORE the fetch. Both are prerequisites, and this is the one that is
REM answered in a second - a machine with no python 3 used to sit through the whole
REM FIXS download and only then be told to go and install python.
call :findpy || exit /b 1

REM The gate is FIXS_VERSION.txt, not any .py: the updater writes that marker LAST
REM and only on a complete install, whereas the python ships inside the build zip
REM and exists the moment it is unpacked - before the native runtime is fetched. A
REM fetch that died at the runtime step used to leave a headers-only bundle that
REM still satisfied the old gate, and the co-sim then failed with "Unable to
REM locate SUMO library directory" instead of anything about the failed update.
if exist "%ROOT%FIXS\FIXS_VERSION.txt" goto :installed
if exist "%ROOT%FIXS" (
    echo [FIXS] the FIXS build is incomplete ^(no FIXS_VERSION.txt^) - refetching ...
) else (
    echo [FIXS] FIXS is not installed here - fetching it first ...
)
REM INSTALL THE DECLARED PIN, not "whatever the picker defaults to". A repo that
REM says which engine it runs has already made the choice, and an automatic
REM bootstrap is not the moment to reopen it. Passing the pin also makes this ONE
REM lookup of one release by tag, which still works when the releases INDEX is
REM down but the release itself is fine (seen live: GitHub 504 on the index while
REM /releases/tags/<tag> served normally). No pin -> the picker, correctly.
call :fetch_fixs "%FIXS_VERSION%"
if not errorlevel 1 goto :installed
echo [FIXS] setup failed - see above. Not continuing.
if defined DBLCLICK pause
exit /b 1
:installed

REM Every run, not only after a fetch: a repo can arrive at an installed FIXS\ some
REM other way - migrating off run_cosim.bat, or a colleague's copy - and it should
REM still end up with its pin recorded. Returns at once when a manifest (or a
REM legacy fixs_sources.txt) exists, so this costs one stat on every later run.
call :seed_manifest
call :check_contract

"%PY%" "%ROOT%FIXS\cosim\run_cosim.py" %*
REM Captured BEFORE pause: pause resets ERRORLEVEL to 0, so exiting after it would
REM report every failed run as a success to whatever called this.
set "RC=%ERRORLEVEL%"
if defined DBLCLICK pause
exit /b %RC%

REM ------------------------------------------------------------------ actions

:do_update
REM An optional version follows: --update-fixs v0.10.0 skips the picker.
call :fetch_fixs "%~2"
set "RC=!ERRORLEVEL!"
if "!RC!"=="0" call :seed_manifest
if defined DBLCLICK pause
exit /b !RC!

:usage
echo FIXS - co-simulation
echo.
echo USAGE
echo   FIXS.bat                      run it. Asks what to run, remembers, replays.
echo                                 Everything is changed from that menu.
echo.
call :print_options
echo.
echo These are the engine's own option names; this file passes them through. The
echo full list, for scripts and developers:
echo   python FIXS\cosim\run_cosim.py --help
if defined DBLCLICK pause
exit /b 0

REM The option list, in one place: --help and the double-click prompt both print
REM it, and a second copy is how the two come to disagree. Three groups, each one
REM question: what to do instead of a run, where CARLA is, what to run.
:print_options
echo   --gui                         the FIXS window: pick, run, stop, watch the log
echo   --setup [carla]               configure a simulator here ^(default: carla^)
echo   --update-python               rebind the python env, keeping the CARLA setup
echo   --import-map [MAP]            install a map ^(no MAP: list the published ones^)
echo   --update-fixs [VERSION]       fetch or refresh the FIXS build
echo                                 ^(no VERSION: pick from a menu^)
echo   --version                     what is installed here
echo   --doctor                      check this machine can run a co-sim
echo   --cleanup                     stop what a crashed run left behind
echo.
echo   --peer HOST[:PORT]            CARLA runs there; this machine runs the traffic half
echo   --serve                       CARLA runs here; wait for the traffic machine to call
echo   --sumo-only                   traffic only, no CARLA, nothing rendered
echo.
echo   --map NAME                    the map, instead of the menu asking
echo   --sumocfg PATH                the SUMO scenario, instead of the menu asking
echo   --app-args "ARGS"             extra arguments for the app's own controller
goto :eof

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

REM Any python 3 is enough as a bootstrap: run_cosim re-execs under the
REM interpreter carla.json names, so conda never needs to be active.
REM
REM It has to BE a python 3, though, and `where python` does not establish that.
REM With an old python 2 first on PATH the engine got handed to it, and what came
REM back named a line of run_cosim.py and PEP 263 - nothing pointing at the
REM interpreter. So each candidate is ASKED what it is. The py launcher goes
REM first: it ships with every python.org install and cannot be an alias.
:findpy
set "PY="
REM An explicit pin wins over the search: the escape hatch for a machine whose
REM system python must stay a 2.x, or that has several python 3s and needs one in
REM particular - including one that is not on PATH. It names only the BOOTSTRAP
REM interpreter; the co-sim's env is still carla.json's answer.
REM goto, not an inline block: a path holding ')' would end the block early.
if defined FIXS_BOOTSTRAP_PYTHON goto :findpy_pinned
REM -3.10 before -3: `py -3` means the HIGHEST 3.x installed, and on a machine
REM without conda, setup pip-installs into whatever interpreter it runs under -
REM and there is no carla wheel for 3.11+.
call :try_py3 py -3.10
if not defined PY call :try_py3 py -3
if not defined PY call :try_py3 python
if not defined PY call :try_py3 python3
if not defined PY call :findpy_conda
if defined PY exit /b 0
echo [FIXS] No Python 3 found.
echo.
echo        Install Python 3.10, then run this file again - it does the rest,
echo        including building the FIXS python env.
echo          Miniconda      https://www.anaconda.com/docs/getting-started/miniconda/install
echo          or python.org  https://www.python.org/downloads/release/python-3109/
echo.
echo        3.10 specifically: the CARLA client wheel is published only for
echo        CPython 3.7-3.10, so pip finds nothing to install on 3.11+.
echo        Already have one somewhere unusual? set FIXS_BOOTSTRAP_PYTHON to it.
if defined DBLCLICK pause
exit /b 1

REM A pin that is not a python 3 is an error, not a reason to search: the user
REM said which interpreter to use, and quietly using another one is how you end
REM up debugging the wrong python.
:findpy_pinned
call :try_py3_path "%FIXS_BOOTSTRAP_PYTHON%"
if defined PY exit /b 0
echo [FIXS] FIXS_BOOTSTRAP_PYTHON is set, but that is not a usable python 3:
echo          %FIXS_BOOTSTRAP_PYTHON%
echo        Point it at a python 3.10 executable, or clear it to search PATH.
if defined DBLCLICK pause
exit /b 1

REM A conda install is the python 3 most likely to exist and NOT be on PATH: the
REM Miniconda installer recommends against adding itself and does not register
REM with the py launcher. Same roots as env_setup._conda_roots, kept in step by
REM hand because this runs before any python does. conda's own registry comes
REM first: it finds a conda on another drive that no guessed root would.
:findpy_conda
if defined CONDA_PREFIX call :try_py3_path "%CONDA_PREFIX%\python.exe"
if defined PY exit /b 0
if exist "%USERPROFILE%\.conda\environments.txt" for /f "usebackq delims=" %%L in ("%USERPROFILE%\.conda\environments.txt") do if not defined PY call :try_py3_path "%%L\python.exe"
if defined PY exit /b 0
for %%R in ("%USERPROFILE%\miniconda3" "%USERPROFILE%\anaconda3" "%USERPROFILE%\miniforge3" "%USERPROFILE%\mambaforge" "%LOCALAPPDATA%\miniconda3" "%LOCALAPPDATA%\anaconda3" "%ProgramData%\miniconda3" "%ProgramData%\anaconda3") do if not defined PY call :try_py3_path "%%~R\python.exe"
exit /b 0

REM A candidate given as a full PATH must PRINT its own sys.executable to count.
REM Invoked directly rather than through `for /f`, whose cmd /c eats the leading
REM quote of a quoted absolute path. A temp file because an exit code proves
REM nothing: pointed at cmd.exe, `cmd.exe -c ...` opens a shell and exits 0.
REM Stdin from nul so a candidate that wants a console cannot sit there waiting.
:try_py3_path
if not exist %1 exit /b 0
set "PROBEFILE=%TEMP%\fixs_bootstrap_py_%RANDOM%.txt"
%1 -c "import sys; sys.stdout.write(sys.executable if sys.version_info[0] == 3 else '')" <nul >"%PROBEFILE%" 2>nul
set "CAND="
if exist "%PROBEFILE%" set /p CAND=<"%PROBEFILE%"
del "%PROBEFILE%" >nul 2>nul
if not defined CAND exit /b 0
if not exist "%CAND%" exit /b 0
set "PY=%CAND%"
exit /b 0

REM Sets PY to the candidate's own sys.executable when it is a python 3. Resolving
REM to the path keeps PY one quotable token, which `py -3` is not. A python 2
REM prints nothing; `if exist` also rejects a stub that prints prose rather than a
REM path - the Microsoft Store's python3.exe is one.
:try_py3
where %1 >nul 2>nul || exit /b 0
for /f "usebackq delims=" %%P in (`%* -c "import sys; sys.stdout.write(sys.executable if sys.version_info[0] == 3 else '')" ^<nul 2^>nul`) do if exist "%%P" set "PY=%%P"
exit /b 0
