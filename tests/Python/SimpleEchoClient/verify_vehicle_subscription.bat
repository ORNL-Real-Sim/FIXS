@echo off
REM ===========================================================================
REM  One-click verification for issue #176
REM    (1) message-framing regression test  -- no simulator, fast
REM    (2) end-to-end subscription comparison -- SUMO + TrafficLayer + Python client
REM        ego_only (1)  <  radius 120 m (~13)  <  all (~34 = whole network)
REM
REM  Prereqs: SUMO on PATH, and TrafficLayer.exe already built
REM           (scripts\dispatch\2_core_components.bat).
REM  Conda env: set RS_ENV below if yours is not 'realsim_dev'.
REM ===========================================================================
setlocal
set "HERE=%~dp0"
if "%RS_ENV%"=="" set "RS_ENV=realsim_dev"
set "EXE=%HERE%..\..\..\TrafficLayer\x64\Release\TrafficLayer.exe"

REM --- activate conda env so 'python' is the realsim interpreter ---
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    call "%USERPROFILE%\miniconda3\Scripts\activate.bat" %RS_ENV%
) else if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    call "%USERPROFILE%\anaconda3\Scripts\activate.bat" %RS_ENV%
) else (
    call conda activate %RS_ENV%
)

echo ===========================================================================
echo  #176 verification   (conda env: %RS_ENV%)
echo ===========================================================================

echo.
echo [1/2] Message-framing regression test (no simulator)...
python "%HERE%test_msg_framing.py"
if errorlevel 1 goto :fail

echo.
echo [2/2] End-to-end subscription comparison (SUMO + TrafficLayer)...
if not exist "%EXE%" (
    echo   TrafficLayer.exe not found at %EXE%
    echo   Build it first:  scripts\dispatch\2_core_components.bat
    goto :fail
)
python "%HERE%run_subscription_compare.py" --steps 400 --warmup 320
if errorlevel 1 goto :fail

echo.
echo ===========================================================================
echo  ALL CHECKS PASSED  (#176)
echo ===========================================================================
endlocal
exit /b 0

:fail
echo.
echo ###########################################################################
echo  VERIFICATION FAILED  (see output above)
echo ###########################################################################
endlocal
exit /b 1
