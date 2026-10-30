# RealSim SUMO documentation

## Contacts
Yunli Shao\
realsimxil@gmail.com

## Table of Contents
* [Simulation Setups](#simulation-setups)
* [Specific Features for SUMO Co-simulation](#specific-features-for-sumo-co-simulation)
    * [Speed Limit](#speed-limit)
    * [Preceding Vehicle](#preceding-vehicle)
    * [Multiple Clients](#multiple-clients)
* [SumoSetup in config.yaml](#sumosetup-in-configyaml)
* [A TraCI script as a FIXS client: fixs.sumo.traci](#a-traci-script-as-a-fixs-client-fixssumotraci)
* [Performance & TraCI transport (libtraci vs libsumo)](#performance--traci-transport-libtraci-vs-libsumo)

## Simulation Setups
1. Select the message data fields, ip address and port of the TrafficLayer, vehicle id of interest in the config.yaml. In this example, vehicle id 'vehicle_0' is selected, ip address is '127.0.0.1', port is 420.  
```yaml
# Global Simulation setup
SimulationSetup:
    
    # Master Switch to turn on/off RealSim interface
    # if turned off, VISSIM will just run without RealSim
    # SUMO needs to run without traci
    EnableRealSim: true
    
    # Whether or not to save verbose log during the simulation. skip log can potentially speed up
    EnableVerboseLog: false
    
    # Simulation end time
    # if NOT specificed, SimulationEndTime will be set to a large value (90000 seconds)
    #--------------------------------------------------
    #SimulationEndTime: 1800
    
    # specify which traffic simulator
    #--------------------------------------------------
    SelectedTrafficSimulator: 'SUMO'

    # default will send all
    VehicleMessageField: [id, type, speed, acceleration, positionX, positionY, positionZ, color, linkId, laneId, distanceTravel, speedDesired]

    # by default it is false 
    EnableExternalDynamics: true 


# ip and port are server ip and port

# setup Application Layer
ApplicationSetup:
    # turn on/off application layer
    EnableApplicationLayer: false
    
    #--------------------------------------------------
    VehicleSubscription: 

        
# subscription in XilSetup must be subset of ApplicationSetup
XilSetup:
    # enable/disable XIL
    EnableXil: true
    
    #--------------------------------------------------
    VehicleSubscription: 
    -   type: ego
        attribute: {id: ['vehicle_0'], radius: [0]}
        ip: ["127.0.0.1"]
        port: [420]
```
1. Create a batch file to run. Launch of the TrafficLayer.exe can be within this batch file or within matlab script, see next step. 
```bat
start sumo -c .\osm.sumocfg --remote-port 1337 --step-length 0.1 --start --netstate-dump osm_out.xml --netstate-dump.precision 5 --num-clients 1

:: can start TrafficLayer.exe here or in matlab script
::start cmd /k ..\..\realsimrelease\TrafficLayer.exe -f config_SUMOdriver.yaml 

```

1. Create a runSumo.m matlab script to specify the RealSim library folder, yaml configuration file, and simulink model. Then call the batch script to launch SUMO and start TrafficLayer.exe. Then initialize simulink using RealSimInitSimulink and start Simulink. 
```matlab
% initialization of vehicle data subscription setup before runnning any
% simulink files. A config.yaml needs to present to read the vehicle data
% subscription setup.

close all;clear all;clc;format compact;

%% Initializations
RealSimPath = '..\..\CommonLib';
configFilename = '.\config_SUMOdriver.yaml';
stopTime = 250; % simulation stop time in seconds. co-simulation will automatically stop after this seconds
simModelName = 'sumoDriverClient';

%% add path of RealSim tools
addpath(genpath(RealSimPath))

%% Run Batch Scripts
system("runSUMOdriver.bat")
system(['start cmd /c ..\..\TrafficLayer\x64\Release\TrafficLayer.exe -f ', sprintf('%s', configFilename)])

%% Initialize RealSim for Simulink, Read yaml file
[VehicleMessageFieldDefInputVec, VehDataBus, TrafficLayerIP, TrafficLayerPort] = RealSimInitSimulink(configFilename);
RealSimPara = struct;
RealSimPara.speedInit = 0; % initial speed of the ego vehicle when entering SUMO network
RealSimPara.tLookahead = 0.1; % use 0.1 for exteranl control, recommend to use tLookahead >= 0.2 for SUMO driver
RealSimPara.smoothWindow = 1; % number of moving average data point, 1 essentially mean no moving average

RealSimPara.speedSource = 2; % select sine wave for tracking

%% RealSim Start Procedure
tic

% start simulink model
% !!! specify the followings: 
%   1) simulink model name
%   2) stopTime of the simulink model
load_system(simModelName)
set_param(simModelName,'StopTime',num2str(stopTime));
% set_param(simModelName, 'SimulationCommand', 'start');
VehicleOut = sim(simModelName); % alternatively can use 'sim' command

sim_time = toc
```


## Specific Features for SUMO Co-simulation
### Speed Limit
- In SUMO, speed limit can be specified on the entire edge or on a lane of an edge. Real-Sim supports retrieving speed limit of both implementations.
- Junctions in SUMO can be assigned a different speed limit than edges connecting to them. Currently Real-Sim will neglect any speed limit specified on junctions. 
- In SUMO, different vehicle types can have different speed limit, which is not currently supported by Real-Sim. 
- For next speed limit, if multiple lanes have different speed limits, Real-Sim will return any of the speed limits since it does not know which lane the ego vehicle will drive ahead of time. 
- If route of the ego is changed during simulation, then the speed limit information from Real-Sim may not reflect the latest route.

### Preceding Vehicle
- Real-Sim will only consider any vehicle within 1000 meters radius of the ego vehicle as the lead vehicle/preceding vehicle. In the future, this radius will be a parameter that can be changed in config.yaml

### Multiple Clients
SUMO natively supports multiple TraCI clients. So Real-Sim can be used by another application, e.g., a python script, together with SUMO. The following shows an example to run a python script together with Real-Sim

- in the runSumo.bat, specify to have total of **2 clients**. Both the python script and Real-Sim will connect to SUMO through port **1337**
```bat
start sumo-gui -c .\speedLimit.sumocfg --remote-port 1337 --step-length 0.1 --start --num-clients 2
```
- inside python script, connects to the SUMO, assuming it is running at localhost IP address 127.0.0.1. Note since we started SUMO from the runSumo.bat, here we call ```traci.init``` rather than ```traci.start```. Also, note ```traci.setOrder``` is mandatory for multiple TraCI clients. By default, Real-Sim will always be order 1, so python script have to be in order >= 2
```python
PORT = int(1337)
traci.init(PORT,host="localhost")
traci.setOrder(2)

#....
# more traci commands
#....
```

There should be no need to modify anything on the Real-Sim side. In the future, the setOrder of Real-Sim can be one argument user can change in the yaml. For more information, check the following screenshots from SUMO documentation

https://sumo.dlr.de/docs/TraCI.html
![](img/SUMOtraciMultiDoc1.png)
https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html
![](img/SUMOtraciMultiDoc2.png)


## SumoSetup in config.yaml
In the config.yaml, there is a section that can define specific settings for SUMO simulation. For example:
```yaml
SumoSetup:
    # set the speed mode, in integer. default value is 0
    # check Sumo documentation https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#speed_mode_0xb3

    SpeedMode: 32
```
The SpeedMode is an integer defines behavior of SetSpeed command of SUMO TraCI API. More parameters can be included in SumoSetup for future releases.

## A TraCI script as a FIXS client: fixs.sumo.traci

A script that already drives SUMO through TraCI joins a FIXS co-simulation by
changing its import:

```diff
-import traci
+import fixs.sumo.traci as traci
```

Everything else stays: `traci.start(...)`, the `traci.simulationStep()` loop, the
`traci.vehicle.*` / `traci.lane.*` / `traci.trafficlight.*` calls, `traci.close()`.
Unlike the *Multiple Clients* setup above, this is not a second TraCI client: there
is still one connection to SUMO, TrafficLayer's, and each call runs on it.

Enable it in the scenario's yaml. It is experimental, so it is off by default:

```yaml
SumoSetup:
    EnableTraciRelay: true
```

What the script's own calls mean under FIXS:

| call | under FIXS |
| --- | --- |
| `traci.start(cmd)` | Reports `cmd`'s `-c` file as this run's scenario (the same handoff `fixs.sumo.scenario()` uses), then connects. SUMO is started by FIXS, so the rest of `cmd` does not reach it; put run options in apps.json `sumo_args`. |
| `traci.simulationStep()` | Hands the current tick back to FIXS and waits for the next one. FIXS's warm-up runs before the first step returns. |
| `traci.close()` | `fixs.close()`. |
| `traci.load()`, `setOrder()` | Refused: FIXS owns the session. |
| `*.subscribe()`, `subscribeLeader()`, `getSubscriptionResults()` | As in TraCI: TrafficLayer places the subscription in SUMO, and each `traci.simulationStep()` brings every result back in one call. A variable whose answer libtraci cannot carry (a list of compounds, such as `getNextTLS`) is refused at `subscribe()`; read it with its getter. |
| `*.subscribeContext()`, subscription filters | Refused: subscribe each object, or read the FIXS feed. |
| `*.unsubscribe()` | Stops the results reaching the script. The SUMO subscription stays, since TrafficLayer may subscribe the same object itself. |

Calls are answered only while the script holds a tick; TrafficLayer serves them
while it waits for this client. A call made before the first `traci.simulationStep()`
first waits for FIXS's first tick.
Every getter is a round trip inside the lockstep, as it is with plain TraCI, and the
relay makes each one cost about twice as much, so read per-step state by
subscription. TrafficLayer warns once when a
client's relayed calls take more than 10% of the tick, and
`SimulationSetup.EnableTimingReport: true` prints where each tick's time goes, per client.
Values come back as TraCI's own
doubles; the FIXS feed carries float32. Under VISSIM, or a TrafficLayer built with
`ENABLE_LIBSUMO`, there is no TraCI connection to relay onto and TrafficLayer
refuses the setting at startup.

### Porting a TraCI script, step by step

1. **Change the import** to `import fixs.sumo.traci as traci`. Keep `traci.start()`,
   the step loop and the calls as they are.
2. **Read per-step state by subscription.** Variable subscriptions work as in TraCI;
   context subscriptions do not, so subscribe each object instead. A script that reads
   with getters runs unchanged, but each getter is a relayed round trip.
3. **Write the scenario yaml.** Start from a working SUMO config and add
   `SumoSetup.EnableTraciRelay: true`. The script is still a FIXS client, so keep an
   `ApplicationSetup.VehicleSubscription` with its port: that is how TrafficLayer
   knows to wait for it each tick. Subscribe only the ego by id: the script reads SUMO
   through the relay, and an `all` subscription makes TrafficLayer collect and send
   every vehicle each tick for a feed the script never reads.
4. **Move SUMO's options out of `traci.start()`.** FIXS starts SUMO and only the `-c`
   file is taken from `cmd`, so flags such as `--seed` or `--time-to-teleport` belong
   where the launcher reads them (FIXS_Applications: apps.json `sumo_args`).
5. **Keep the script fed through warm-up** if it counts ticks or tracks state from
   the first step, as a plain TraCI script sees every step from SUMO's begin time: list
   its port in `SimulationSetup.WarmUpServePorts`. Otherwise its first step is the first
   tick after warm-up.
6. **Let FIXS end the run.** When TrafficLayer stops, the next call raises
   `traci.exceptions.FatalTraCIError`, as native TraCI does when SUMO closes.

The script then reads SUMO's own values, so its results equal the same script on
plain TraCI. A controller written against the FIXS feed sees float32 instead and can
differ from the TraCI version by that rounding. Worked example:
[FIXS_Applications `apps/mlk_eco_driving`](https://github.com/ORNL-Real-Sim/FIXS_Applications/tree/dev/apps/mlk_eco_driving),
app `mlk_eco_driving_traci`: `eco_driving_mpr_SUMO_MLK_traci.py` (plain TraCI) and
`eco_driving_mpr_SUMO_MLK_traci_fixs.py` (the same file, import changed).

## Performance & TraCI transport (libtraci vs libsumo)

> Notes from the #177 investigation (measured, June 2026). Detailed data + reusable
> benchmark harnesses live in
> [`tests/Sumo/Probes/TrafficLayer_SUMO_CMoffice/PERF_177_TRACI.md`](../tests/Sumo/Probes/TrafficLayer_SUMO_CMoffice/PERF_177_TRACI.md).

### Which transport the build uses
`CommonLib/TrafficHelper.h` leaves `ENABLE_LIBSUMO` **commented out**, so the build
compiles the **`libtraci`** namespace — the *socket* TraCI client. `TrafficLayer`
therefore talks to a **separate `sumo`/`sumo-gui` process over TCP** (see
`run_sumo_cm_demo.bat`: `sumo-gui --remote-port ...`). The alternative, `libsumo`,
embeds SUMO **in-process** (no socket). The `#ifdef` is already in the header, so it is
a per-build choice.

Measured cost difference (SUMO 1.21, simple_loop, 36 veh):

| | per single getter call | bulk subscription (1 call/step, all veh) |
|---|---:|---:|
| libtraci (socket) | ~78 µs (TCP round-trip) | ~150 µs/step transport penalty |
| libsumo (in-process) | ~0.3 µs | baseline |

Implication: **libsumo only wins big against per-call patterns** (one TraCI getter per
vehicle per step). For a single bulk context subscription the socket penalty is only
~150 µs/step. So the fix for slow SUMO co-sim is to **stop making per-vehicle
individual TraCI calls** (fold data into the subscription / cache it), *not* to switch
transport. After that, libtraci ≈ libsumo and you keep the GUI.

### #177 root cause (slow SUMO↔CarMaker co-sim)
The dominant per-step cost was **`Vehicle::getNextTLS(vehID)` called per traffic
vehicle per step** in `TrafficHelper::parserSumoSubscription` (≈ 2.5 ms/veh →
~90 ms/step at 36 veh, RTF ≈ 1.1). It is **not** the context-subscription variable
list (that is ~free). `getNextTLS` is O(upcoming-route length); the SimpleLoop demand
uses `repeat="100000"` (~400 k-edge route) so it walks the whole thing. The data it
produces (`signalLight*`) is not even sent unless a `signalLight*` field is in
`VehicleMessageField`. Same for `getLeader`/`getSpeed` (→ `precedingVehicle*`). VISSIM
is immune because the DSProxy path makes **zero** per-vehicle TraCI calls.

### Why you cannot have libsumo + sumo-gui on Windows
`libsumo` cannot drive `sumo-gui` on Windows — confirmed at the SUMO source level
(`src/libsumo/GUI.cpp`, a hard `#ifdef WIN32 { WRITE_WARNING("Libsumo on Windows does
not work with GUI, falling back to plain libsumo."); return false; }`), present on the
latest `main` and unchanged through 1.27. Reason: libsumo drives the GUI **incrementally
from the simulation/client thread** instead of FOX's blocking main-thread event loop,
and **FOX windows are thread-affine** ("FOX really likes to work in the main thread") —
Win32's window/message model doesn't tolerate that, so it is hard-disabled (X11/Linux is
"highly experimental" but works). A separately-launched `sumo-gui` cannot attach to an
in-process libsumo sim (no server socket; it would be a *different* simulation). SUMO's
planned **FOX→Qt** migration (issue eclipse-sumo/sumo#311) has been backlogged since
2010, so do not count on it.

**Practical guidance:** keep **libtraci + `sumo-gui`** for interactive/demo runs (GUI,
real-time once the per-vehicle calls are fixed); use **libsumo headless** only for
max-throughput batch runs, where CarMaker IPGMovie / Carla already provide the visual.
