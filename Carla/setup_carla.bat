@echo off
REM Windows wrapper for carla_env_setup.py.
REM Run this any time to choose / change which CARLA run_cosim uses
REM (packaged build vs source build, or a different install). The choice is
REM saved to %USERPROFILE%\.fixs\carla.json and reused on every run.
python "%~dp0carla_env_setup.py" %*
