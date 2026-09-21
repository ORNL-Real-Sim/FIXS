# Relayed TraCI, end to end (#356)

A live co-simulation in which one Python client reads its tick from the FIXS feed
**and** calls `traci.*` through `import fixs.traci as traci`. Both answers come from
the same SUMO over the same single connection, and the test asserts they agree.

```
powershell -ExecutionPolicy Bypass -File run_relay_test.ps1
```

Headless and self-checking: it starts SUMO and TrafficLayer, runs `relay_client.py`,
kills both, and exits non-zero if any assertion failed. Needs
`TrafficLayer\x64\Release\TrafficLayer.exe` (`scripts\dispatch\2_core_components.bat`)
and `SUMO_HOME`. `-Python <path>` if `realsim_dev` is not where it expects.

## What it asserts

| | |
| --- | --- |
| **the cross-check** | `traci.vehicle.getSpeed('ego')` equals the feed's `ego.speed` exactly. Two independent paths to the same number: the relay is talking to the run FIXS is driving, not to something else. |
| beyond the feed | `lane.getIDList()`, `lane.getLinks()`, `vehicle.getRoute()` — none of which the ~30 `VehDataMsgDefs` fields can answer. That is the point of the relay. |
| a setter lands | `setParameter` then `getParameter`, and the value is still there one tick later. |
| chunking | a 20 KB parameter round trip. `MAX_RECORD_SIZE` is 8192 and a wire contract shared with dSPACE, so this splits across records in **both** directions and is reassembled. |
| the error path | `getSpeed('nosuchveh')` raises `traci.TraCIException` carrying SUMO's own message. |
| refusals | `simulationStep()`, `subscribe()`, and a raw `CMD_CLOSE` at the seam. |
| `traci.start()` | returns the version when it names the config FIXS is running, raises when it names another. |
| the co-sim survives | the feed still arrives after all of the above. |

The deterministic network is the probe's (`tests/Sumo/Probes/TraciRelay/net/`):
`sigma=0`, no speed deviation, so "equals exactly" is a fair assertion rather than a
flaky one.

## The other two

- `tests/Python/unit/test_traci_relay.py` — the same seam against a fake socket. No
  simulator, so it runs in `python -m pytest tests/Python/unit/ -v`.
- `tests/Sumo/Probes/TraciRelay/` — the original #356 probe: does this work at all,
  measured against a real TraCI connection before any FIXS code existed.
