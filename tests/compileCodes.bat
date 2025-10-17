@echo off
setlocal enabledelayedexpansion

REM Clear previous test results
echo Build Results > testsResults.log
echo ============== >> testsResults.log
echo. >> testsResults.log

REM Build all projects - output shows in console and saves to build_output.log
REM Concise summary goes to testsResults.log

REM Build TrafficLayer
echo Building TrafficLayer...
msbuild ..\TrafficLayer\TrafficLayer.sln /p:Configuration=Release > build_output.log 2>&1
set BUILD_STATUS=%ERRORLEVEL%
if %BUILD_STATUS%==0 (
	echo ===^> TrafficLayer built success>> testsResults.log
	copy /Y "..\CommonLib\libsumo\libsumocpp.dll" "..\TrafficLayer\x64\Release\"
	copy /Y "..\CommonLib\libsumo\libtracicpp.dll" "..\TrafficLayer\x64\Release\"
)else (
	echo ===^> TrafficLayer built failed>> testsResults.log
	goto :build_failed
)

REM Build VISSIMserver projects (if available in ProprietaryFiles)
if exist "..\ProprietaryFiles\VISSIMserver" (
	echo Building DriverModel_RealSim...
	msbuild ..\ProprietaryFiles\VISSIMserver\VISSIMserver.sln /target:DriverModel_RealSim /p:Configuration=Release >> build_output.log 2>&1
	set BUILD_STATUS=%ERRORLEVEL%
	if %BUILD_STATUS%==0 (
		echo ===^> DriverModel_RealSim built success>> testsResults.log
	)else (
		echo ===^> DriverModel_RealSim built failed>> testsResults.log
		goto :build_failed
	)

	echo Building DriverModel_RealSim_v2021...
	msbuild ..\ProprietaryFiles\VISSIMserver\VISSIMserver.sln /target:DriverModel_RealSim_v2021 /p:Configuration=Release >> build_output.log 2>&1
	set BUILD_STATUS=%ERRORLEVEL%
	if %BUILD_STATUS%==0 (
		echo ===^> DriverModel_RealSim_v2021 built success>> testsResults.log
	)else (
		echo ===^> DriverModel_RealSim_v2021 built failed>> testsResults.log
		goto :build_failed
	)
) else (
	echo VISSIMserver folder not found, skipping VISSIM builds>> testsResults.log
)

REM Build VirtualEnvironment
echo Building VirtualEnvironment...
msbuild ..\VirtualEnvironment\VirtualEnvironment.sln /p:Configuration=Release >> build_output.log 2>&1
set BUILD_STATUS=%ERRORLEVEL%
if %BUILD_STATUS%==0 (
	echo ===^> VirtualEnvironment built success>> testsResults.log
)else (
	echo ===^> VirtualEnvironment built failed>> testsResults.log
	goto :build_failed
)

:: Build CarMaker versions (9, 10, 11, 13)
for %%v in (9 10 11 13) do (
	if exist "..\ProprietaryFiles\CM%%v_proj" (
		echo Building CarMaker%%v...
		msbuild ..\ProprietaryFiles\CM%%v_proj\src\CarMaker.sln /target:CarMaker /p:Configuration=Release >> build_output.log 2>&1
		set BUILD_STATUS=!ERRORLEVEL!
		if !BUILD_STATUS!==0 (
			echo ===^> CarMaker%%v built success>> testsResults.log
		)else (
			echo ===^> CarMaker%%v built failed>> testsResults.log
			goto :build_failed
		)

		echo Building CarMaker%%v Simulink...
		msbuild "..\ProprietaryFiles\CM%%v_proj\src_cm4sl\CarMaker for Simulink.sln" /target:"CarMaker for Simulink" /p:Configuration=Release >> build_output.log 2>&1
		set BUILD_STATUS=!ERRORLEVEL!
		if !BUILD_STATUS!==0 (
			echo ===^> CarMaker%%v Simulink built success>> testsResults.log
		)else (
			echo ===^> CarMaker%%v Simulink built failed>> testsResults.log
			goto :build_failed
		)
	) else (
		echo CM%%v_proj folder not found, skipping CarMaker %%v builds>> testsResults.log
	)
)

echo.
echo All builds completed successfully!
echo Check testsResults.log for summary and build_output.log for details.
pause
exit 0

:build_failed
echo.
echo Build failed! Check testsResults.log for summary and build_output.log for details.
pause
exit -1