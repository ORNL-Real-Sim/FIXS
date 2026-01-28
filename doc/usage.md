# Usage

This page highlights the common workflows once your environment is installed. For detailed simulator steps, keep the SUMO/VISSIM/CarMaker docs nearby.

## Launch Order

1. **TrafficLayer** – start from a terminal: `TrafficLayer.exe -f path\to\config.yaml`.
2. **Application layer** – run your MATLAB/Simulink model or other control logic if enabled in the config.
3. **Traffic simulator** – start SUMO, VISSIM, or CarMaker with the matching test case.

## Editing Configurations

- `SimulationSetup`: choose the simulator (`SUMO`, `VISSIM`, `CarMaker`) and set `TrafficSimulatorIP/Port`.
- `ApplicationSetup`: subscribe to vehicles (by id or type) and ports for Real-Sim clients.
- `XilSetup`: mirror the relevant subscriptions when doing HIL/dSPACE runs.

Refer to `doc/ConfigSetup.md` for the full schema and field descriptions.

## Running Provided Scenarios

1. Inspect the `tests/` directory for ready-made batch scripts (e.g., `runCoordMergeSUMO.bat`).
2. Update paths inside the script to point at your simulator binaries and config file.
3. Execute the script, then monitor logs under `TrafficLayerLogs/` for handshake status.

## Collecting Data

- Enable verbose logging in the YAML (`EnableVerboseLog: true`) to capture full message streams.
- Use the provided Python utilities in `CommonLib` to post-process sensor/vehicle data.

## Troubleshooting

See `doc/setupGuide.md` for general issues (missing packages, Conda prompts) and each simulator doc for platform-specific debugging steps.
