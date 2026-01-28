# Getting Started

Use this page as the on-ramp into the Real-Sim toolchain. It pulls together the minimum set of steps from the existing guides so you can validate your environment quickly before diving into any of the traffic-simulator-specific docs.

## 1. Clone and Initialize

1. Clone the repository and update submodules (see `README.md` for any project-specific flags).
2. Review `requirements.txt` together with `doc/setupGuide.md` so you know which external tools (SUMO, CarMaker, MATLAB, etc.) must already be on your workstation.

## 2. Verify Prerequisites

1. Run `env.check.py` from the repo root to check Conda, Python, and the required Python packages.  
2. Install any missing traffic simulator binaries (CarMaker, VISSIM, SUMO) following the instructions in their respective docs under `doc/`.

## 3. Configure Your First Scenario

1. Copy one of the provided configs, e.g., `tests\coordMerge\config_SUMO.yaml`.
2. Update IP/port information to match your machine (both Real-Sim’s `TrafficLayer.exe` and the simulator it talks to).
3. If you are targeting SUMO, walk through `doc/SUMOdoc.md`; for VISSIM or CarMaker, follow their corresponding guides.

## 4. Run a Smoke Test

1. Launch `TrafficLayer.exe -f <your-config.yaml>`.
2. Start the simulator specified in the config.
3. For MATLAB/Simulink workflows, open the provided example model under `CarMaker` or `CommonLib` and confirm the interface connects.

Once you can send messages end-to-end, proceed to the Tutorial pages for deeper walkthroughs.
