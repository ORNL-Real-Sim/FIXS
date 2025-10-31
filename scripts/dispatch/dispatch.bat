@echo off
setlocal enabledelayedexpansion

echo ==============================
echo RealSim FIXS Release Builder
echo ==============================
echo.

REM Version Configuration (update these to match dependencies.yaml)
set DSPACE_VERSION=2024a
set MATLAB_VERSION=2024a

REM Set paths (script runs from scripts/dispatch/)
set SOURCE_PATH=%CD%\..\..\
set RELEASE_PATH=%SOURCE_PATH%build

REM Clean and create build directory
echo Cleaning build directory...
if exist "%RELEASE_PATH%" (
    rmdir /S /Q "%RELEASE_PATH%"
)
mkdir "%RELEASE_PATH%"

REM ====================================
REM Step 1: Compile External Libraries
REM ====================================
echo [1/5] Checking external libraries...
if not exist "%SOURCE_PATH%\CommonLib\yaml-cpp\build" (
    echo External libraries not found. Compiling...
    call "%~dp0\1_external_libraries.bat"
    if %ERRORLEVEL% neq 0 (
        echo ERROR: Failed to compile external libraries!
        goto :failed
    )
) else (
    echo External libraries already compiled.
)
echo.

REM ====================================
REM Step 2: Compile All Components
REM ====================================
echo [2/5] Compiling all components...
call "%~dp0\2_all_components.bat"
if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to compile components!
    goto :failed
)
echo.

REM ====================================
REM Step 3: Build RS dSPACE Library
REM ====================================
echo [3/5] Building RealSim dSPACE library...
call "%~dp0\3_dspace_library.bat" inline
if %ERRORLEVEL% neq 0 (
    echo WARNING: dSPACE library build failed or skipped
)
echo.

REM ====================================
REM Step 4: Build RealSimSocket MEX
REM ====================================
echo [4/5] Building RealSimSocket MEX...
call "%~dp0\4_mex_realsimsocket.bat" inline
if %ERRORLEVEL% neq 0 (
    echo WARNING: RealSimSocket MEX build failed or skipped
)
echo.

REM ====================================
REM Step 5: Release Files to build\
REM ====================================
echo [5/5] Copying files to build directory...

REM Release executables
echo Copying executables...
copy /Y "%SOURCE_PATH%\TrafficLayer\x64\Release\TrafficLayer.exe" "%RELEASE_PATH%\TrafficLayer.exe" >nul

REM Release CommonLib folder with complete structure (includes libsumo)
echo Copying CommonLib...
xcopy /Y /E /I "%SOURCE_PATH%\CommonLib" "%RELEASE_PATH%\CommonLib" >nul

REM Release CarMaker files
echo Copying CarMaker files...
if exist "%SOURCE_PATH%\ProprietaryFiles\CM11_proj\src\CarMaker.win64.exe" (
    if not exist "%RELEASE_PATH%\CarMaker\CM11" mkdir "%RELEASE_PATH%\CarMaker\CM11"
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM11_proj\src\CarMaker.win64.exe" "%RELEASE_PATH%\CarMaker\CM11\" >nul
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM11_proj\src_cm4sl\libcarmaker4sl.mexw64" "%RELEASE_PATH%\CarMaker\CM11\" >nul
)
if exist "%SOURCE_PATH%\ProprietaryFiles\CM10_proj\src\CarMaker.win64.exe" (
    if not exist "%RELEASE_PATH%\CarMaker\CM10" mkdir "%RELEASE_PATH%\CarMaker\CM10"
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM10_proj\src\CarMaker.win64.exe" "%RELEASE_PATH%\CarMaker\CM10\" >nul
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM10_proj\src_cm4sl\libcarmaker4sl.mexw64" "%RELEASE_PATH%\CarMaker\CM10\" >nul
)
if exist "%SOURCE_PATH%\ProprietaryFiles\CM9_proj\src\CarMaker.win64.exe" (
    if not exist "%RELEASE_PATH%\CarMaker\CM9" mkdir "%RELEASE_PATH%\CarMaker\CM9"
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM9_proj\src\CarMaker.win64.exe" "%RELEASE_PATH%\CarMaker\CM9\" >nul
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM9_proj\src_cm4sl\libcarmaker4sl.mexw64" "%RELEASE_PATH%\CarMaker\CM9\" >nul
)
if exist "%SOURCE_PATH%\ProprietaryFiles\CM13_proj\src\CarMaker.win64.exe" (
    if not exist "%RELEASE_PATH%\CarMaker\CM13" mkdir "%RELEASE_PATH%\CarMaker\CM13"
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM13_proj\src\CarMaker.win64.exe" "%RELEASE_PATH%\CarMaker\CM13\" >nul
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\CM13_proj\src_cm4sl\libcarmaker4sl.mexw64" "%RELEASE_PATH%\CarMaker\CM13\" >nul
)

REM Copy CarMaker utility files
if exist "%SOURCE_PATH%\CarMaker" (
    xcopy /Y /E /I "%SOURCE_PATH%\CarMaker" "%RELEASE_PATH%\CarMaker" >nul
)

REM Copy dSPACE library (if present)
echo Copying dSPACE library...
copy /Y "%SOURCE_PATH%\CommonLib\libRealSimDsLib_%DSPACE_VERSION%.a" "%RELEASE_PATH%\CarMaker\" 2>nul

REM Release VISSIM files
echo Copying VISSIM files...
if exist "%SOURCE_PATH%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim.dll" (
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim.dll" "%RELEASE_PATH%\" >nul
)
if exist "%SOURCE_PATH%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim_v2021.dll" (
    copy /Y "%SOURCE_PATH%\ProprietaryFiles\VISSIMserver\x64\Release\DriverModel_RealSim_v2021.dll" "%RELEASE_PATH%\" >nul
)

echo.
echo ==============================
echo RealSim: Release Complete!
echo ==============================
echo Files copied to: %RELEASE_PATH%
echo.
pause
exit /b 0

:failed
echo.
echo ==============================
echo RealSim: Release FAILED!
echo ==============================
pause
exit /b 1
