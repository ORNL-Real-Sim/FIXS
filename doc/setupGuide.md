# Setup Guide

> Guide for installing, configuring, and running the Real-Sim Environment

---

## Prerequisites

**System & Hardware**

- **OS:** Windows 10+
- **Tools (Versions):**
  - Python 3.8+
  - SUMO 1.13.0
  - MATLAB 9.7
  - CarMaker 10.0.1
  - VISSIM 11.0

**Python Packages**

- easydict  
- numpy  
- pyyaml  
- ruamel.yaml  
- pandas  
- matplotlib  
- traci  
- sumolib  

---

## Installation

1. Clone the repository.
2. Check `requirements.txt`.
3. Run the `env.check.py` file to install required frameworks and packages.  
   - This script checks for each required tool, prompts to auto-install, and installs any missing Python packages.
4. Run the program in your IDE.

Once installation is complete, you should see an output similar to:

```text
# Successful output message
Succesful Environment Check:

Environment Setup Script (Windows + Conda)

Checking if Conda is available...
 Conda is available.

 Checking if Python is installed in Conda...
 # packages in environment at C:\Users\YourComputer\miniconda3:
 #
 # Name                     Version          Build               Channel
 brotli-python              1.0.9            py313h5da7b33_9
 python                     3.13.2           hadb2040_100_cp313
 python_abi                 3.13              0_cp313

 Checking if SUMO is installed...
 SUMO is installed at: C:\Users\YourComputer\sumo-1.21.0\bin\sumo.EXE
 easydict is already installed.
 numpy is already installed.
 yaml is already installed.
 ruamel.yaml is already installed.
 pandas is already installed.
 matplotlib is already installed.
 traci is already installed.
 sumolib is already installed.

Environment Setup check completed.

```

## Running Test Cases

- **Test script:** `run_experiments.py`  
- **Config file:** `ecodrivingConfig_SUMO.yaml`  

See the [Config Setup](ConfigSetup.md) page for test case setup details.

---

## Troubleshooting

### Missing packages
1. Check `requirements.txt` and run the `env.check.py` script.  
2. You should get a confirmation message if all dependencies are installed.  
3. If not, manually install missing requirements and rerun the script.  
4. Ensure your system meets all prerequisites.

### Miniconda prompt
- When running `env.check.py`, you may be prompted to download Miniconda.  
- Type **Y** and confirm.  
- This will install Miniconda and allow Python and SUMO to be set up automatically.

### Cannot run the script
1. If the script fails, manually install the correct Python version first.  
2. Verify all frameworks listed in `requirements.txt` are installed.  
3. Rerun `env.check.py` to finalize environment setup.
