@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ====================================
REM Build RealSimSocket MEX (mexw64)
REM Supports double-click (standalone) and inline invocation via dispatch.
REM ====================================

set "SCRIPT_DIR=%~dp0"
set "RUN_MODE=%~1"

if /I "%RUN_MODE%"=="window" (
    start "RealSimSocket MEX Build" cmd /k "%~f0" inline
    exit /b 0
)

if /I "%RUN_MODE%"=="inline" (
    set "RUN_MODE=inline"
) else (
    if /I "%RUN_MODE%"=="standalone" (
        set "RUN_MODE=standalone"
    ) else (
        if defined RS_FIXS_AUTOMATION (
            set "RUN_MODE=inline"
        ) else (
            set "RUN_MODE=standalone"
        )
    )
)

set "BUILD_RESULT=0"
set "MATLAB_VERSION="
set "MATLAB_INSTALL="
set "DEPS_FILE=%SCRIPT_DIR%..\..\dependencies.yaml"
set "STACK_CHANGED=0"

echo Parsing dependencies from: %DEPS_FILE%

if not exist "%DEPS_FILE%" (
    echo ERROR: dependencies.yaml not found at %DEPS_FILE%
    set "BUILD_RESULT=1"
    goto :end
)

call :ReadYamlVersion "%DEPS_FILE%" "matlab" MATLAB_VERSION

if not defined MATLAB_VERSION (
    echo Falling back to default MATLAB version 2024a...
    set "MATLAB_VERSION=2024a"
)

if defined MATLAB_ROOT (
    set "MATLAB_INSTALL=%MATLAB_ROOT%"
) else if exist "%ProgramFiles%\MATLAB\R%MATLAB_VERSION%" (
    set "MATLAB_INSTALL=%ProgramFiles%\MATLAB\R%MATLAB_VERSION%"
) else (
    set "MATLAB_INSTALL=C:\Program Files\MATLAB\R%MATLAB_VERSION%"
)

echo.
echo Building RealSimSocket MEX using MATLAB version: %MATLAB_VERSION%
echo MATLAB root: %MATLAB_INSTALL%

if not exist "%MATLAB_INSTALL%\bin\mex.bat" (
    echo ERROR: Unable to locate mex.bat at: %MATLAB_INSTALL%\bin\mex.bat
    echo Ensure MATLAB is installed or set MATLAB_ROOT environment variable.
    set "BUILD_RESULT=1"
    goto :end
)

set "SOURCE_DIR=%SCRIPT_DIR%..\..\CommonLib"
set "SOURCE_FILE=%SOURCE_DIR%\RealSimSocket.cpp"

if not exist "%SOURCE_FILE%" (
    echo ERROR: Source file not found: %SOURCE_FILE%
    set "BUILD_RESULT=1"
    goto :end
)

pushd "%SOURCE_DIR%" >nul
if %ERRORLEVEL% EQU 0 (
    set "STACK_CHANGED=1"
) else (
    echo ERROR: Failed to change directory to %SOURCE_DIR%
    set "BUILD_RESULT=1"
    goto :end
)

echo Invoking mex...
call "%MATLAB_INSTALL%\bin\mex.bat" -v -largeArrayDims -outdir "%SOURCE_DIR%" "%SOURCE_FILE%"
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ==============================
    echo RealSimSocket MEX build FAILED!
    echo ==============================
    set "BUILD_RESULT=1"
) else (
    echo.
    echo ==============================
    echo RealSimSocket MEX built successfully.
    echo Output: %SOURCE_DIR%\RealSimSocket.mexw64
    echo ==============================
    set "BUILD_RESULT=0"
)

:end
if "%STACK_CHANGED%"=="1" popd >nul
if /I "%RUN_MODE%"=="standalone" (
    echo.
    pause
)
exit /b %BUILD_RESULT%

:ReadYamlVersion
REM %1 file path, %2 subsection name under development_tools, %3 output var
setlocal EnableDelayedExpansion
set "FILE=%~1"
set "TARGET=%~2"
set "OUT_VAR=%~3"
set "TMP_FILE=%TEMP%\rs_fixs_%RANDOM%%RANDOM%.txt"

powershell -NoProfile -Command "try { $target = '%TARGET%'; $file = '%FILE%'; $lines = Get-Content -Path $file -ErrorAction Stop; $pattern = '^[\s]*' + [regex]::Escape($target) + ':'; $match = $lines | Select-String -Pattern $pattern | Select-Object -First 1; $value = ''; if ($match) { $lineIndex = $match.LineNumber - 1; $indent = $lines[$lineIndex].Length - $lines[$lineIndex].TrimStart().Length; for ($i = $lineIndex + 1; $i -lt $lines.Count; $i++) { $current = $lines[$i]; if ([string]::IsNullOrWhiteSpace($current)) { continue }; $currIndent = $current.Length - $current.TrimStart().Length; if ($currIndent -le $indent) { break }; $verMatch = [regex]::Match($current, '^[\s]*version:[\s]*\"(.+?)\"'); if ($verMatch.Success) { $value = $verMatch.Groups[1].Value; break } } }; Set-Content -Path '%TMP_FILE%' -Value $value -Encoding ASCII } catch { Set-Content -Path '%TMP_FILE%' -Value '' -Encoding ASCII }" 2>nul

set "RESULT="
if exist "%TMP_FILE%" (
    set /p RESULT=<"%TMP_FILE%"
    del "%TMP_FILE%" >nul 2>&1
)

endlocal & set "%OUT_VAR%=%RESULT%"
exit /b 0
