# Installation

This page collects the cross-cutting install steps referenced throughout the simulator-specific guides. Follow it in sequence, then jump to the tutorial or platform doc that matches your workflow.

## 1. System Packages

- **Windows 10/11** with administrator privileges.
- Visual Studio 2022 (Desktop development with C++) for building CarMaker integrations.
- MATLAB/Simulink (version aligned with your CarMaker release; see `doc/CarMakerDoc.md`).
- SUMO, VISSIM, and CarMaker binaries when needed (see their respective docs for supported versions).

## 2. Python Environment

```powershell
# from the repo root
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

Alternatively, run `env.check.py` to let the project script verify/repair your Conda-based toolchain.

## 3. Build Helper Libraries

Some workflows (especially CarMaker) require compiled helper libraries:

1. Open `VirtualEnvironment/VirtualEnvironment.sln` in Visual Studio.
2. Build Release (and Debug if needed) targets for the `VirtualEnvironment` project.
3. If you need dSPACE support, follow the “Compile dSPACE library” section of `doc/CarMakerDoc.md`.

## 4. TrafficLayer and Configs

1. Review the sample configs under `tests/` or `doc/ConfigSetup.md`.
2. Copy a config file and adjust IP/port bindings plus vehicle subscription blocks for your simulator.
3. Keep `TrafficLayer.exe` accessible (either build it yourself or use the shipped binary) and test it with `TrafficLayer.exe -f <config.yaml>` before launching a full simulation.

After completing these steps, you are ready to move on to the Tutorial pages (`tutorials/quickstart.md` and `tutorials/examples.md`) for simulator-specific walkthroughs.
