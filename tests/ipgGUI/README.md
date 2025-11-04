# GUI for Real-Sim FIXS
Real-Sim GUI for non-developer users to run co-simulation involving IPG CarMaker/TruckMaker.

## 🚀 Features
 - Currently support: **CarMaker-Sumo** and **CarMaker-Simulink-Sumo** co-simulations
 - Can be run using Python or Matlab (Simulink users) scripts

## 🛠️ Installation
Can be installed with conda command: `conda env create -f environment.yml`

## 📁 Project Structure
#### As a good practice, a Real-Sim project (when involving IPG) follows the following structure:
Project_root \
├── tests/ \
│ ├── SUMO_test_name/ \
│ │ ├── .sumocfg \
│ │ └── RS_config_release.yaml \
│ │ └── SUMO simulation files/ \
│ │ │ ├── Routes/ \
│ │ │ └── Network/ \
│ │ │ └── Additional files/ \
├── CM_proj/ \
│ ├── src_cm4sl/ \
│ │ ├── Simulink file \
│ │ └── **Matlab-based GUI** \
│ │ └── ... other matlab/simulink files \
│ ├── Data/ \
│ │ ├── TestRun/ \
│ │ └── ... other folders for CM runs (roads, vehicles, tires, etc.) \
│ ├── ... other folders for this CM project \
├── CarMaker/ \
├── CommonLib/ \
├── TrafficLayer.exe
