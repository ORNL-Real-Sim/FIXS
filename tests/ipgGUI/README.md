# GUI for Real-Sim FIXS
Real-Sim GUI for non-developer users to run co-simulation involving IPG CarMaker/TruckMaker. The GUI allows users to select IPG testrun file, Sumo simulation configuration, Simulink model, and directory to store simulation output data.

## 🚀 Features
 - Currently support: **CarMaker-Sumo** and **CarMaker-Simulink-Sumo** co-simulations
 - Can be run using Python or Matlab (Simulink users) scripts

## 🛠️ Installation
Can be installed with conda command: `conda env create -f environment.yml`

## 📁 Project Structure
#### As a good practice, a Real-Sim project (when involving IPG) can be organized following the structure:
Project_root \
├── tests/ \
│ ├── **GUI scripts** \
│ ├── SUMO_test_name/ \
│ │ ├── .sumocfg \
│ │ └── RS_config_release.yaml \
│ │ └── SUMO simulation files/ \
│ │ │ ├── Routes/ \
│ │ │ └── Network/ \
│ │ │ └── Additional files/ \
├── CM_proj/ \
│ ├── src_cm4sl/ \
│ │ ├── Simulink model (.slx or .mdl) \
│ │ └── cmenv.m \
│ │ └── ... other matlab/simulink files \
│ ├── Data/ \
│ │ ├── TestRun/ \
│ │ └── ... other folders for CM runs (roads, vehicles, tires, etc.) \
│ ├── ... other folders for this CM project \
├── CarMaker/ \
├── CommonLib/ \
├── TrafficLayer.exe

### ⚙️ Launching the GUI
**Matlab user:** run `RS_init.m` to initialize the workspace and Real-Sim config, then run `run_sim_gui.m` to launch the GUI. \
**Python user:** depending on your co-sim combination, run the corresponding `.py` file. Note that if you wanna involve Simulink, you can put the codes of `RS_init.m` in the `init` callback function of the Simulink file (can be open via model explorer).