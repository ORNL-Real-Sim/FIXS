# Tutorial Examples

Use these scenarios to explore different combinations of simulators and Real-Sim components. Each builds on the Quickstart.

## 1. SUMO + CarMaker Visualization

1. Follow the Quickstart to run SUMO + TrafficLayer.
2. Open `CarMaker\CM11_proj` and load the `coordMerge_rs` testrun.
3. Launch CarMaker Office or Simulink per `doc/CarMakerDoc.md` and confirm the SUMO vehicles appear inside CarMaker.

## 2. VISSIM Ego Vehicle Control

1. Copy `doc/VISSIMdoc.md`’s YAML example and adjust the vehicle id to match your network.
2. Enable the “Control Initial States of an Ego Vehicle” steps to spawn the vehicle via COM.
3. Observe the enforced “Go Straight” behavior by issuing commands from your controller.

## 3. dSPACE / HIL Scenario

1. Build the dSPACE library as described in `doc/CarMakerDoc.md`.
2. Open the ConfigurationDesk project `RS_DS_CM11_noSimulinkRS_useManeuver`.
3. Run `tests\SumoIpg\runSumoIpg_release.bat` and verify the traffic signals synchronize.

## 4. Multi-Client Experiment

1. Duplicate the YAML config and assign unique `TrafficSimulatorPort` / application ports.
2. Launch two copies of SUMO (or SUMO + VISSIM) each pointing to its own config.
3. Confirm TrafficLayer routes each simulator’s data to the correct subscribers.

These examples are meant to be customized—swap in your own networks or automation scripts once the baseline runs are stable.
