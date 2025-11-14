# FAQ

## Why does `TrafficLayer.exe` fail to start?

- Confirm the YAML path in the launch command is correct.
- Check that the configured port is not already in use (e.g., via `netstat -ano`).
- Make sure the simulator you selected in the YAML matches the one you intend to run.

## SUMO/VISSIM cannot connect to the Real-Sim interface. What should I check?

1. IP address and port inside the simulator configuration must match the `TrafficSimulatorIP` and `TrafficSimulatorPort` values in the YAML.
2. Firewalls can block the loopback connection—try temporarily disabling or adding a rule for the relevant ports.

## How do I add another ego vehicle?

- Update the `VehicleSubscription` block in the YAML to include the additional vehicle id/type.
- For VISSIM and CarMaker, follow the dedicated sections (`doc/VISSIMdoc.md`, `doc/CarMakerDoc.md`) that explain how to create unique vehicle classes or IDs.

## Where do I find example configs?

- `tests/` contains ready-to-run SUMO/VISSIM examples.
- `doc/ConfigSetup.md` documents every field if you need to build a config from scratch.

## How can I debug Real-Sim + CarMaker builds?

- The “Debugging” section in `doc/VISSIMdoc.md` covers Visual Studio attachment; CarMaker uses similar steps—build the executable in Debug and attach to the process before launching the simulation.
- Enable verbose logging in the YAML and inspect the generated log files to see the message flow.
