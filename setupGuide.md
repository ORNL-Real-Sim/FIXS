
# {{Real-Sim}} Setup Guide

> Guide for installing, configuring, and running the Real-Sim Environment

---

## Table of Contents

1. [Prerequisites]  
2. [Installation]  
3. [Running Test Cases] 
4. [Troubleshooting]  

---

## 1. Prerequisites

System & Hardware Prerequisites:
- **OS:** Windows 10+
- **Tools(Versions):**
  -Python 3.8+  
  -SUMO 1.13.0
  -Matlab 9.7
  -carmaker 10.0.1
  -vissim 11.0
- **Packages:**
  easydict
  numpy
  pyyaml
  ruamel.yaml
  pandas
  matplotlib
  traci
  sumolib



## 2. Installation
1.clone repo
2.check requirements.txt 
3.run the env.check.py file for installation of required frameworks and packages
  -env.check.py will check for each required tool, prompts to auto-install and install any missing Python packages
4.output after checks have passed:"Environment Setup check completed."
5.Run the program on your IDE

1.clone repository 
2.check requirements.txt
3. run the environment setup python env.check.py
This should handle all setup for you.
Once it has successfully completed you will see the message below:

#successful output message
----------------------------------
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
python_abi                 3.13             0_cp313

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
----------------------------------
## 3.Running Test Cases
1. test script: run_experiments.py
config file: ecodrivingConfig_SUMO.yaml

Navigate to the ConfigSetup.md file for test case setup

## 4.Troubleshooting

--Missing packages
1.Check requirements.txt and run the env.check.py script to ensure that you have all nescessary dependencies on your device.You should get a confirmation message indicating you meet all requirements.If you do not receive this message you can also manually install these requirements yourself and then rerun the script.Ensure your system meets all specifications as well.

--Miniconda prompt
1.When you run the env.check.py script you will get a prompt that asks you to download miniconda. Type "Y" and then select yes when prompted to download miniconda. This will then allow manual installation of Python and SUMO into your computer as well.

--Cannot run the script
1.If the script is not running on your device you can manually download python version first. Check requirements.txt for all the required frameworks. Then run env.check.py to ensure environment is set up. 




