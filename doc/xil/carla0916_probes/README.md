# CARLA 0.9.16 telemetry probes

Scripts behind the measurements in [`../Carla0916Investigation.md`](../Carla0916Investigation.md)
§4.2 and §5. Investigation only — not part of the FIXS build.

## Running them

Start a 0.9.16 server (see `Carla0916Investigation.md` §4.1 — `make package` does not
work, launch through the engine's editor binary):

```
C:\src_ext\CarlaUnreal\Engine\Binaries\Win64\UE4Editor.exe ^
    C:\src_ext\CarlaSrc_0_9_16\Unreal\CarlaUE4\CarlaUE4.uproject ^
    -game -carla-server -carla-rpc-port=2000 -RenderOffScreen -quality-level=Low -nosound
```

Then, against a Python 3.10 with the 0.9.16 wheel installed:

```
python -u 01_launch_traffic_telemetry.py
```

Use `-u`. These scripts print as they go and a buffered stdout loses everything if the
client library faults on teardown.

## What each one answers

| script | question | §  |
|---|---|---|
| `01_launch_traffic_telemetry.py` | does the server run, spawn traffic, and return telemetry at all? | 4.2 |
| `02_telemetry_conventions.py` | which index is which corner, and what are the signs? | 5.1–5.3 |
| `03_torque_identity.py` | is `torque` anything other than `-long_force * radius`? | 5.1 |

`03` also contains a second experiment (H2) that tries to recover the wheel inertia from
`J_w * dw/dt = torque` during a coast. **It is inconclusive by construction** and no
conclusion is drawn from it in the write-up: the driveline stays engaged through a coast,
so engine braking and the differential contribute torques that the telemetry does not
report, and the implied `J_w` comes out negative. Recovering `J_w` needs the disengaged
test in `CarlaDynoCoupling.md` §8, not this one.
| `04_drag_and_balance.py` | what is `Cd*A`, and does `m*a = sum(Fx) - drag`? | 5.4–5.5 |
| `05_balance_by_speed.py` | where does that balance break down? | 5.4 |

## Method notes

Every script that measures forces spawns a `sensor.other.collision` and reports the hit
count per phase. An earlier round of this work on 0.9.15 produced alarming cornering
numbers that turned out to be collision contamination, so a phase with a nonzero count
is not evidence of anything.

Scripts 02–05 search the map for the longest straight run of waypoints and spawn there,
rather than at an arbitrary spawn point, to keep the vehicle out of junctions and off
the curb for the length of the test.
