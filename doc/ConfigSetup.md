# Configuration Setup Guide 

#ecodrivingConfig_SUMO.yaml — Configuration Documentation

This page explains the structure and purpose of the `ecodrivingConfig_SUMO.yaml` file used in the Real-Sim environment. Each section of the YAML controls a different aspect of the simulation (metadata, experiment setup, traffic simulator, external dynamics, etc.).

```yaml
# =========================
# Metadata
# =========================
Metadata:
  name:             "Eco-Driving Test Scenario"
  version:          "1.0.0"
  description:      >
    Runs a traffic simulation on SUMO, with
    RealSim linking to a Python controller and Simulink for dynamics.
  simulation_end:   33000

# =========================
# Base Directory
# =========================
root_dir: "./tests/Applications/Eco_Fixed_Timming"

# =========================
# Experiments
# =========================
Experiments:
  config_file: "ecodrivingConfig.yaml"   # RealSim scenario YAML to modify and run
  eco_driving: true                      # Enable eco-driving logic in Python controller
  vehicle_dynamics: true                 # Delegate dynamics to Simulink if true

  SUMO:
    port: 1337                           # TCP port for SUMO TraCI
    src_path: "sumo_files"               # Path where SUMO network/config files are stored

  Simulink:
    port: 420                            # TCP port for Simulink (XIL client)
    model_name: "EV_longitude"           # Name of MATLAB/Simulink model to launch

  FIXS:
    port: 430                            # TCP port for RealSim’s Traffic ↔ Application link

# =========================
# Simulation Setup
# =========================
SimulationSetup:
  EnableRealSim: true
  EnableVerboseLog: true
  SimulationEndTime: 33000
  SelectedTrafficSimulator: 'SUMO'

  VehicleMessageField:        # Vehicle data fields sent each timestep
    - id
    - type
    - speed
    - acceleration
    - positionX
    - positionY
    - positionZ
    - color
    - linkId
    - laneId
    - distanceTravel
    - speedDesired

  EnableExternalDynamics: true
  TrafficSimulatorPort: 1337   # Overwritten at runtime from Experiments.SUMO.port

# =========================
# SUMO Setup
# =========================
SumoSetup:
  SpeedMode: 32                # TraCI bitmask: 31 = safety on, 32 = free-run

# =========================
# Application Setup
# =========================
ApplicationSetup:
  EnableApplicationLayer: true
  VehicleSubscription:
    - type: ego
      attribute:
        id:     ['ego']        # Vehicle ID filter
        radius: [0]            # Radius 0 = exact match
      ip:   ['127.0.0.1']      # Controller IP (localhost)
      port: [430]              # Overwritten from Experiments.FIXS.port

# =========================
# XIL Setup
# =========================
XilSetup:
  EnableXil: true              # Master switch for XIL interface
  VehicleSubscription:
    - type: ego
      attribute:
        id:     ['ego']        # Vehicle ID filter
        radius: [0]            # Radius 0 = exact match
      ip:   ['127.0.0.1']      # Simulink/hardware IP (localhost)
      port: [420]              # Overwritten from Experiments.Simulink.port
