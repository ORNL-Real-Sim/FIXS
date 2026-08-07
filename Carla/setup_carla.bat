@echo off
REM Windows wrapper for carla_env_setup.py.
REM Run this any time to choose / change which CARLA run_cosim uses
REM (packaged build vs source build, or a different install). The choice is
REM saved to %USERPROFILE%\.fixs\carla.json and reused on every run - by every
REM entry point, not just run_cosim.
REM   setup_carla.bat                  pick the CARLA (and resolve the python env)
REM   setup_carla.bat --update-python  rebind ONLY the python env, keep the CARLA
REM   setup_carla.bat --show           print the saved config
python "%~dp0carla_env_setup.py" %*
