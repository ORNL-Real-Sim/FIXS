# tests/Vissim/networks/

Shared VISSIM network files used by **more than one** scenario.

## Convention

- A scenario folder (e.g. `tests/Vissim/SpeedLimit/`) owns its network if
  the network is used **only** by that scenario.
- If two or more scenarios reuse the same VISSIM network, the `.inpx`,
  `.inp0`, `.layx`, and `.sig` files are promoted here, into a subfolder
  named after the network.
- Scenario scripts load the shared network by relative path, e.g.
  `../networks/speedLimit/speedLimit.inpx`.

## Current networks

| Folder | Used by | Notes |
|---|---|---|
| `speedLimit/` | `SpeedLimit/`, `SpeedLimitLite/`, `SimpleEcho/` (placeholder) | Dummy network with three speed-limit zones and a signal head. |
