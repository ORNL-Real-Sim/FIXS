set RealSimPath=..\..\
set RealSimAppPath=.\
set configFilename=config_RS_noSimulinkRS.yaml
set CmProjPath=C:\src_git\RS_FIXS_full\ProprietaryFiles\CM13_proj

start sumo-gui -c %RealSimAppPath%\coordMerge.sumocfg --remote-port 1337 --step-length 0.1 --start

start cmd /c C:\src_git\RS_FIXS_full\TrafficLayer\x64\Release\TrafficLayer.exe -f %configFilename%

call conda activate realsimdev
python %RealSimPath%\CarMaker\RealSimSetCarMakerConfig.py --cm-project-path %CmProjPath% --configFile %configFilename%
call conda deactivate

pause

