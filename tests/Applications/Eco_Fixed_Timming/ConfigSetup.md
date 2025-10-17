# ecodrivingConfig_SUMO.yaml Configuration Documentation 

#MetaData
Metadata:
  name:             "Eco-Driving Test Scenario"
  version:          "1.0.0"
  description:      >
    Runs a  traffic simulation on SUMO, with
    RealSim linking to a Python controller and Simulink for dynamics.
  simulation_end:   33000


# Base directory containing all experiment data
root_dir: "./tests/Applications/Eco_Fixed_Timming"

Experiments:
    
    # Name of the RealSim scenario YAML to modify and run
    config_file: "ecodrivingConfig.yaml"
    # Enable eco-driving logic in the Python controller
    eco_driving: true
    # Delegate vehicle dynamics (accel/brake) to Simulink if true
    vehicle_dynamics: true

    SUMO:
      # TCP port for SUMO’s TraCI interface
      port: 1337
      # Relative path under source_dir where SUMO files live
      src_path: "sumo_files"

    Simulink:
      # TCP port where Simulink (XIL client) listens
      port: 420
      # Name of the MATLAB/Simulink model to launch
      model_name: "EV_longitude"

    FIXS:
      # TCP port for RealSim’s TrafficLayer ↔ ApplicationLayer link
      port: 430


# RealSim Scenario File: ecodrivingConfig.yaml
# The Python script will load & modify this in each experiment’s output_dir.

SimulationSetup:
  # Turns on RealSim interface (traffic ↔ controller ↔ XIL)
  EnableRealSim: true
  # Write detailed logs (useful for debugging; may be slower simulation)
  EnableVerboseLog: true
  # Stop simulation after this many seconds (33 000 s =~ 9 hours)
  SimulationEndTime: 33000
  # Choose traffic engine: 'SUMO' or 'VISSIM'
  SelectedTrafficSimulator: 'SUMO'
  # Vehicle data fields sent each timestep
  Below is all the data that is interracted with from vehicle
  VehicleMessageField:
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
  # If true, external dynamics (Simulink) compute acceleration/brake; else SUMO will handle it
  EnableExternalDynamics: true
  # Overwritten at runtime to Experiments.Exp1.SUMO.port (e.g., 1337)
  TrafficSimulatorPort: 1337

SumoSetup:
  # SUMO TraCI bitmask: 31 = safety checks on; 32 = all checks off (free-run)
  SpeedMode: 32

ApplicationSetup:
  # Enable forwarding vehicle data to your Python controller
  EnableApplicationLayer: true
  VehicleSubscription:
    - type: ego               # Only subscribe to the “ego” vehicle
      attribute:
        id:     ['ego']       # Filter by vehicle ID = “ego”
        radius: [0]           # Radius 0 = exact ID match (no spatial filter)
      ip:   ['127.0.0.1']     # Controller IP address (localhost)
      # Overwritten at runtime to Experiments.Exp1.FIXS.port (e.g., 430)
      port: [430]

XilSetup:
  # Enable forwarding vehicle data to Simulink or hardware

  #Master switch for the XIL interface
  EnableXil: true             #RealSim will open a network connection and send out vehicle updates each simulation step
  VehicleSubscription:        #this test scenario only has one entry 
    - type: ego               # Only subscribe to the “ego” vehicle
      attribute:
        id:     ['ego']       # Filter by vehicle ID = “ego”
        radius: [0]           # Spatial Radius 0 = exact ID match
      ip:   ['127.0.0.1']     # Simulink/hardware IP address (localhost)
      # Overwritten at runtime to Experiments.Exp1.Simulink.port (e.g., 420)
      port: [420]
