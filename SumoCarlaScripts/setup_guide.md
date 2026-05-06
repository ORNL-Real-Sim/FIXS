# Traffic Light Sync Script Environment
Both `helper_scripts/sumo_carla_tl_sync.py` and `helper_scripts/run_synchronization/run_synchronization.py` implement the same table based SUMO->CARLA traffic light synchronization and use the same setup. Both scripts expect a `traffic_light_table.csv` which maps (junction_id, link_id) pairs to traffic light actor coordinates in Carla, and for those traffic light actors to exist in Carla. Instructions below explain how to generate the traffic light table and place corresponding traffic light actors in Carla using the Unreal editor. 

# Generate `traffic_light_table.csv` using `utils/extract_sumo_tls_as_table.py`
Set line 283 to path to SUMO .net.xml file for intended scenario
Set line 284 to desired output path for `traffic_light_table.csv`
ex:
```
sumo_net_file = 'test_scenarios\MLK\MLK_final_elevation.net.xml' # path to SUMO net xml
output_path = 'test_scenarios\MLK' # output path for traffic_light_table.csv
```

If you are having an issue where signal heads are oriented opposite of the expected direction, setting `FLIP_SIGNAL_HEADS_180` to `True` on line 29 will flip each signal head 180 degrees.

# Generate Traffic Lights in Unreal from `traffic_light_table.csv`

In the Unreal editor:
Open map:

File → Open Level  
Content/[map_package_name]/Maps/[map_name]

Set an environment variable called `SUMO_TLS_TABLE_PATH` to the full path location of your traffic light table in the UnrealEditor console:
Change the input type of the Output Log to 'python', then enter:
```bash
import os
os.environ["SUMO_TLS_TABLE_PATH"] = "/home/[user_name]/path/to/traffic_light_table.csv"
print(os.environ["SUMO_TLS_TABLE_PATH"])
```
Ensure lines 24 and 25 of `/helper_scripts/unreal_placing_tls.py`
```
TRAFFICLIGHT_GROUP_BLUEPRINT_PATH = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLightGroup"
TRAFFICLIGHT_HEAD_ONLY_BLUEPRINT_PATH = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLight"
```
Point to correct asset locations for TrafficLightGroup and TrafficLightHeadOnly blueprints.

Run script:

File → Execute Python Script...

Select:

/helper_scripts/unreal_placing_tls.py

# Run SUMO -> CARLA traffic light synchronization

Before running either synchronization script:
- Start CARLA and load the target map.
- Ensure `traffic_light_table.csv` has been generated and the corresponding traffic light actors have been placed in CARLA.
- Ensure `SUMO_HOME` is set and SUMO's `tools` directory is available on `PYTHONPATH` so `traci` and `sumolib` can be imported.

## Option 1: `helper_scripts/sumo_carla_tl_sync.py`

This script directly mirrors SUMO traffic light states into CARLA traffic light actors. 
- `--passive` is an option intended to work with other co-simulation programs, when set it prompts the script not to tick CARLA and wait for external ticks.

Required argument:
- `--tl-table`: path to `traffic_light_table.csv`

SUMO connection options:
- `--sumocfg`: path to a SUMO `.sumocfg` file if this script should launch SUMO itself
- `--sumo-gui`: optional, launches `sumo-gui` instead of headless `sumo`
- `--sumo-port`: use this instead of `--sumocfg` to connect to an already running SUMO TraCI server
- `--sumo-step-length`: optional SUMO step length; if omitted, it uses `--fixed-dt`

CARLA and matching options:
- `--carla-host`: CARLA host, default `localhost`
- `--carla-port`: CARLA port, default `2000`
- `--fixed-dt`: CARLA fixed delta seconds, default `0.05`
- `--offset-x` and `--offset-y`: optional offsets applied when matching CARLA traffic lights to SUMO coordinates
- `--max-match-dist`: optional maximum allowed matching distance, default `50.0`
- `--xodr`: optional `.xodr` file can be used to generate a standalone OpenDRIVE CARLA world, this option should only be used if the `.xodr` file already contains expected traffic light actors. For convience, a `.xodr` under `/helper_scripts/MLK_scenario` with traffic lights has been provided that corresponds with this scenario: https://github.com/ORNL-Real-Sim/FIXS/tree/feature/120/tests/Applications/SUMO_CARLA_EcoDriving/MLK_Sumo_Scenario 

Example launching SUMO from the script:
```bash
python helper_scripts/sumo_carla_tl_sync.py \
  --sumocfg test_scenarios/MLK/your_scenario.sumocfg \
  --tl-table test_scenarios/MLK/traffic_light_table.csv \
  --fixed-dt 0.05 \
  --sumo-gui
```

Example connecting to an already running SUMO TraCI server:
```bash
python helper_scripts/sumo_carla_tl_sync.py \
  --sumo-port 8813 \
  --tl-table test_scenarios/MLK/traffic_light_table.csv \
  --fixed-dt 0.05
```

## Option 2: `helper_scripts/run_synchronization/run_synchronization.py`

This script runs the broader CARLA <-> SUMO synchronization workflow with the same traffic light table based synchronization logic used by `helper_scripts/sumo_carla_tl_sync.py`.

Required argument:
- `sumo_cfg_file`: path to the SUMO `.sumocfg` file

To synchronize traffic lights from SUMO to CARLA:
- Set `--tls-manager sumo`
- Provide `--tl-table` with the path to `traffic_light_table.csv`

Useful options:
- `--carla-host`: CARLA host, default `127.0.0.1`
- `--carla-port`: CARLA port, default `2000`
- `--sumo-host`: SUMO host if needed
- `--sumo-port`: SUMO TraCI port if needed
- `--sumo-gui`: optional, runs SUMO with GUI
- `--step-length`: synchronization step length, default `0.05`
- `--client-order`: TraCI client order, default `1`
- `--offset-x` and `--offset-y`: optional offsets for traffic light matching
- `--max-match-dist`: optional maximum matching distance, default `50.0`
- `--sync-vehicle-lights`, `--sync-vehicle-color`, or `--sync-vehicle-all`: optional vehicle property synchronization
- `--debug`: enable debug logging

Example:
```bash
python helper_scripts/run_synchronization/run_synchronization.py \
  test_scenarios/MLK/your_scenario.sumocfg \
  --tls-manager sumo \
  --tl-table test_scenarios/MLK/traffic_light_table.csv \
  --step-length 0.05 \
  --sumo-gui
```

If traffic lights do not match correctly, first verify the `traffic_light_table.csv` coordinates, then adjust `--offset-x`, `--offset-y`, or `--max-match-dist` as needed.
