# Getting Started

The path from nothing to a first co-simulation, then on to the guide for your
simulator.

## 1. Get FIXS

Either use a published build or build from source.

- **Published build:** download `fixs-build-<channel>.zip` from
  [GitHub Releases](https://github.com/ORNL-Real-Sim/FIXS/releases). The
  [Releases](changelog.md) page explains the channels; `stable` is built from `main`.
- **From source:** clone the repository and run `scripts\dispatch\dispatch.bat`. Its
  first step initializes a fresh clone (submodules, native dependencies, yaml-cpp).
  [BUILD.md](BUILD.md) covers prerequisites, what gets built, and building single
  components.

The supported versions of SUMO, CARLA, CarMaker, MATLAB and dSPACE are pinned in
[`dependencies.yaml`](../dependencies.yaml).

## 2. Set up Python

The Python helpers and example clients use the conda environment defined in
[`environment.yml`](../environment.yml):

```batch
conda env create -f environment.yml
conda activate realsim
```

`python env.check.py` checks for conda, SUMO and the required Python packages. If
conda is missing it offers to download and install Miniconda.

## 3. Run a first example

[`tests/Python/SimpleEchoClient`](../tests/Python/SimpleEchoClient/) is the smallest
end-to-end setup: SUMO drives a vehicle around a loop, TrafficLayer relays its state,
and a Python client receives it and echoes it back. `run_simple_echo_client.bat`
starts all three in order: SUMO, then TrafficLayer, then the client. It expects
TrafficLayer from a source build (`TrafficLayer\x64\Release`); with a published build,
point it at your `TrafficLayer.exe`. The folder's README describes what you should
see.

## 4. Next steps

- [Configuration reference](ConfigSetup.md): every `config.yaml` key.
- Simulator guides: [VISSIM](VISSIMdoc.md), [SUMO](SUMOdoc.md),
  [CarMaker](CarMakerDoc.md), [CARLA](CARLAdoc.md).
- When something fails, TrafficLayer writes its errors to `TrafficLayer.err`, and
  `EnableVerboseLog: true` in `SimulationSetup` logs the full message flow. See also
  the [FAQ](faq.md).
