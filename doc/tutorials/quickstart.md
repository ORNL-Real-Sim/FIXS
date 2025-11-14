# Quickstart Tutorial

This walkthrough guides you through a minimalist SUMO-based co-simulation to confirm Real-Sim is wired correctly.

## 1. Prep the Environment

- Complete the steps in `doc/getting-started.md`.
- Ensure SUMO is installed and on `PATH`.
- Copy `tests\coordMerge\config_SUMO.yaml` into a working folder.

## 2. Launch TrafficLayer

```powershell
TrafficLayer.exe -f coordMerge\config_SUMO.yaml
```

Verify the console prints the “Succesful Environment Check” summary (from `env.check.py`).

## 3. Start SUMO

```powershell
sumo-gui -c tests\coordMerge\coordMerge.sumocfg --remote-port 1337 --step-length 0.1 --start
```

SUMO should connect to the Real-Sim port configured in the YAML.

## 4. Observe Data Flow

- Watch `TrafficLayer.exe` logs to ensure vehicle updates stream in.
- Optionally, start the Simulink template model `CarMaker\RealSimGeneric.mdl` to subscribe to the same vehicles.

At this point you have a working single-client setup. Continue to `tutorials/examples.md` for variations (multiple vehicles, dSPACE, etc.).
