
set RealSimPath=..\..\
set configFilename=RS_config_release.yaml
set CmProjPath=..\..\CM13_proj


call conda activate realsimdev
python %RealSimPath%\CarMaker\RealSimCarMakerSetup.py --cm-project-path %CmProjPath% --testrun RS_UA_sumo_test_0208_v3 --cm-install-path C:\IPG --output-testrun RS_UA_sumo_test_0208_v3_with_RS --car 50 --truck 50 --sumo-file-path .\simulation\network\net.net.xml
call conda deactivate

pause