# Simple Echo Client

A simple Python client that receives VehicleData from TrafficLayer.exe and echoes it back.

## Setup

1. Create conda environment (first time only):
```bash
conda create -n realsim python=3.9
conda activate realsim
pip install -r ../../../requirements.txt
```

2. Ensure SUMO is installed and accessible in your PATH

## Running

Simply run the batch script:
```bash
run_simple_echo_client.bat
```

This will:
1. Start SUMO-GUI with a simple loop scenario (one ego vehicle driving in a square)
2. Start TrafficLayer.exe
3. Start the Python echo client

## What it does

- The ego vehicle (red car) drives continuously in a loop
- TrafficLayer receives vehicle data from SUMO and sends it to the Python client
- The Python client receives the data, prints it, and echoes it back
- You should see vehicle position and speed updating in the console

## Files

- `config.yaml` - Configuration for TrafficLayer connection and message fields
- `simple_echo_client.py` - Python client script
- `simple_loop.sumocfg` - SUMO configuration
- `simple_loop.net.xml` - SUMO network (square loop)
- `simple_loop.rou.xml` - SUMO routes (one ego vehicle)
- `run_simple_echo_client.bat` - Launch script
