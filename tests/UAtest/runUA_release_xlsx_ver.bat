
set RealSimPath=..\..\
set configFilename=RS_config_release.yaml
set CmProjPath=..\..\CM13_proj

start sumo-gui -c .\runmine.sumocfg --remote-port 1337 --step-length 0.1 --start 

start cmd /c %RealSimPath%\TrafficLayer.exe -f %configFilename%

call conda activate realsimdev
python %RealSimPath%\CarMaker\RealSimSetCarMakerConfig.py --cm-project-path %CmProjPath% --configFile %configFilename% --signal-table-path %CmProjPath%\Data\Road\UA_Feb2024_TFlight_RSsignalTable.xlsx
call conda deactivate

pause