@echo off
REM Windows wrapper for the cross-platform run_cosim.py orchestrator.
REM Pass any run_cosim.py args, e.g.:
REM   run_cosim.bat --sumocfg fixtures\grid_tls.sumocfg --map Town01
python "%~dp0run_cosim.py" %*
