@echo off
REM ======================================================================
REM  BASELINE (no World Model) -- WorldModels endless-loop demo (#189)
REM
REM  Pure SUMO: the ego circulates the loop and ALL surrounding traffic is
REM  driven natively by SUMO -- so it obeys the traffic signals (stops at red).
REM  Compare visually against run_with_wm.bat.
REM ======================================================================
setlocal
echo [no-WM] Launching sumo-gui (native SUMO, follows signals)...
start "SUMO (no WM)" sumo-gui -c simple_traffic_light.sumocfg --start --delay 40 --window-size 1400,900

echo.
echo [no-WM] SUMO-GUI launched. Press Play; the ego (red) loops and background
echo         traffic stops at the 3 red signals. This is the baseline to compare
echo         against run_with_wm.bat.
