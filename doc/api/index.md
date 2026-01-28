# API Reference

The Real-Sim repositories primarily expose executable interfaces (TrafficLayer, MATLAB/Simulink helpers, simulator plugins). A formal API reference is forthcoming; for now, use the pointers below:

- **Python helpers** – explore the modules under `CommonLib/` and `scripts/` for utilities such as `RealSimCarMakerSetup.py`.
- **MATLAB helpers** – see `CarMaker/src_cm4sl` for templates like `RealSimGeneric.mdl` and helper scripts (`RealSimInitSimulink.m`).
- **C/C++ interfaces** – review the headers in `VirtualEnvironment/` and the snippets embedded in `doc/CarMakerDoc.md` showing how to integrate `VirEnv_Wrapper`.

When the APIs stabilize, this section will include autodoc-generated references for the Python modules plus call signatures for the MATLAB/C++ helpers.
