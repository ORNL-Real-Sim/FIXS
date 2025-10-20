# Configuration Reference

The Real-Sim interface is configured through YAML files (for example `config.yaml`). Each top-level section maps directly to structures parsed in `CommonLib/ConfigHelper`. This document describes the supported keys, their defaults, and any special behaviour such as derived paths.

> **Path handling.** Unless stated otherwise, relative paths are resolved against the directory that contains `TrafficLayer.exe`. When running from Visual Studio the executable typically lives under `TrafficLayer\x64\<Config>`, while standalone builds place it under `build`. The interface also accepts absolute paths.

## SimulationSetup

| Key | Type | Default | Notes |
| --- | --- | --- | --- |
| EnableRealSim | bool | true | Master switch for the interface. |
| EnableVerboseLog | bool | false | Enables detailed logging in TrafficLayer and companion tools. |
| SimulationEndTime | double | 90000 | End time (seconds). Large default keeps the session open until external shutdown. |
| EnableExternalDynamics | bool | false | When true, SUMO speed updates use `setPreviousSpeed` so acceleration limits are respected. |
| VehicleMessageField | string list | All supported fields | Controls which vehicle fields are exchanged. Provide a subset if bandwidth is a concern. |
| SelectedTrafficSimulator | string | `"SUMO"` | Use `"SUMO"`, `"VISSIM"`, etc. |
| TrafficSimulatorIP | string | 127.0.0.1 | Host for TraCI/VISSIM connections. |
| TrafficSimulatorPort | int | 1337 | Port for the selected simulator. |
| SimulationMode | int | 0 | Bitmask mode as documented in the README. |
| SimulationModeParameter | double | 0 | Auxiliary parameter consumed by selected modes. |
| TrafficLayerIP | string | inferred | Defaults to the first vehicle subscription IP if not provided. |
| TrafficLayerPort | int | inferred | Defaults to the first vehicle subscription port if not provided. |

## SumoSetup

| Key | Type | Default | Notes |
| --- | --- | --- | --- |
| SpeedMode | int | 0 | Passed to `SUMO_TRACI_NAMESPACE::Vehicle::setSpeedMode`. |
| ExecutionOrder | int | 1 | Order passed to `Simulation::setOrder`. |
| EnableAutoLaunch | bool | false | When true, TrafficLayer starts SUMO automatically; otherwise it expects a running instance. |
| SumoConfigFile | string | `""` | Path to `.sumocfg` used when auto-launching. |
| NumClients | int | 1 | Number of SUMO clients negotiated during auto-launch. |
| RuntimeLibraryPath | string | empty | Optional override for the SUMO runtime directory (DLL/.so path). |

When `RuntimeLibraryPath` is omitted the executable searches for the libsumo runtime in the following order:

1. `libsumo\bin` located next to `TrafficLayer.exe`.
2. `CommonLib\libsumo\bin` relative to the executable, walking up to four parent directories (covers `TrafficLayer/x64/<Config>` and `build`).
3. `CommonLib\libsumo\bin` relative to the current working directory.

On Windows the resolved folder is supplied to `SetDllDirectory`. On Linux the folder is prepended to `LD_LIBRARY_PATH`. Leave the field commented out unless you intentionally repackage the SUMO binaries elsewhere.

## ApplicationSetup

| Key | Type | Default | Notes |
| --- | --- | --- | --- |
| EnableApplicationLayer | bool | false | Enables the application message bus. |
| NumberOfApplications | int | 0 | Count of downstream application sockets (required when the layer is enabled). |
| ApplicationPort | int list | [] | Ports that receive messages. |
| VehicleSubscription / DetectorSubscription / SignalSubscription | list of subscription blocks | [] | Each block has `type`, an `attribute` map, `ip`, and `port`. Allowed `type` values include `ego`, `point`, `edge`, and `pattern`. |

Each subscription block extends the base message set delivered to the application layer. Attributes are expressed as YAML maps; for example:

```yaml
VehicleSubscription:
- type: ego
  attribute: {id: ['ego']}
  ip: ['127.0.0.1']
  port: [2444]
```

## XilSetup

| Key | Type | Default | Notes |
| --- | --- | --- | --- |
| EnableXil | bool | false | Enables the co-simulation bridge. |
| AsServer | bool | false | Determines whether TrafficLayer listens for XIL connections. |
| VehicleSubscription / DetectorSubscription / SignalSubscription | list | [] | Same schema as the application-layer subscriptions. |

When the application layer is disabled but XIL is enabled, the interface automatically reuses the XIL vehicle subscriptions to seed outbound traffic.

## CarMakerSetup

| Key | Type | Default | Notes |
| --- | --- | --- | --- |
| EnableCosimulation | bool | false | Turn on integration with IPG CarMaker. |
| EnableEgoSimulink | bool | false | Enable Simulink-controlled ego vehicle. |
| CarMakerIP / CarMakerPort | string / int | 127.0.0.1 / 16666 | Connection details for CarMaker. |
| TrafficRefreshRate | double | 0.123 | Update rate used when publishing to CarMaker. |
| EgoId / EgoType | string | `"ego"` / `"passenger"` | Identifier and SUMO type for the simulated ego. |
| SynchronizeTrafficSignal | bool | false | If enabled, the interface subscribes to signal controllers for synchronization. |
| TrafficSignalPort | int | derived | Port used when signals are synchronized. |

## CarlaSetup

| Key | Type | Default | Notes |
| --- | --- | --- | --- |
| EnableVerboseLog | bool | false | Verbose logging for the CARLA bridge. |
| EnableCosimulation | bool | false | Enables CARLA co-simulation. |
| EnableExternalControl | bool | false | Allows external controllers to override CARLA vehicles. |
| UseVehicleTypeAsBlueprint | bool | false | Interpret SUMO vehicle types as CARLA blueprints. |
| CarlaServerIP / CarlaServerPort | string / int | 127.0.0.1 / 2000 | CARLA server endpoint. |
| CarlaClientIP / CarlaClientPort | string / int | 127.0.0.1 / 0 | Client binding for CARLA streaming. |
| CarlaMapName | string | `"Town01"` | Desired CARLA map. |
| CenteredViewId | string | `""` | Actor id used for camera centering. |
| TrafficRefreshRate | double | 0.1 | CARLA update period. |
| InterestedIds | string list | [] | Vehicles that receive mirrored CARLA updates. |

## Additional Sections

Other sections follow the same pattern:

- Detector and signal subscriptions use `pattern` or `id` filters to pull edge-based data from the simulator.
- Legacy documentation for specific application workflows has been moved to `tests/Applications/Eco_Fixed_Timming/ConfigSetup.md`.

## Updating the YAML

1. Copy an existing configuration (for example `tests/Python/SimpleEchoClient/config.yaml`).
2. Adjust the sections described above.
3. If SUMO auto-launch is enabled, ensure the `.sumocfg` path is valid and that either `RuntimeLibraryPath` is set or the default `CommonLib/libsumo/bin` directory remains beside the executable.
4. Keep indentation spaces (YAML does not support tabs).

For quick reference, see the inline comments in `tests/Python/SimpleEchoClient/config.yaml`, which mirror the default options discussed here.
