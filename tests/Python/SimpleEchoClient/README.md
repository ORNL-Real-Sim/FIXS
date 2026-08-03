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

## Vehicle-subscription verification (issue #176)

### Watch it live (recommended first look)

```
demo_vehicle_subscription_gui.bat
```
Opens three windows so you can *see* the fix: **SUMO-GUI** (~30 cars driving the loop),
**TrafficLayer** (prints each vehicle and `send client veh: 30`), and the **echo client**
(`Received 30 vehicles` each step). With the bug, the client would show `Received 0`/`1`.
Close the windows when done.

### One-click pass/fail

```
verify_vehicle_subscription.bat
```
Runs both checks below and prints `ALL CHECKS PASSED` / `VERIFICATION FAILED`. Needs SUMO
on PATH and `TrafficLayer.exe` already built (`scripts\dispatch\2_core_components.bat`).
Set `RS_ENV` if your conda env is not `realsim_dev`.

### What it checks

**(1) `test_msg_framing.py` — message-framing regression (no simulator).** Packs N
vehicles with the real `SocketHelper.sendData` and parses them back with the real
`recv_data`, asserting the header `total_msg_size` equals the bytes actually sent and that
two back-to-back messages don't desync. This locks the #176 framing fix
(`MsgHelper.pack_*_data` must return the full record size); it FAILS for every N if the
old body-only size is restored. Runs in well under a second.

**(2) `run_subscription_compare.py` — end-to-end subscription comparison.** Runs the same
multi-vehicle network under three subscription variants and reports the peak vehicle count
each delivers to the client:

| variant | `VehicleSubscription` attribute | expected count |
|---------|--------------------------------|----------------|
| `ego_only` | `{id: ['ego'], radius: [0]}`   | 1 (ego only) |
| `radius`   | `{id: ['ego'], radius: [120]}` | a strict subset (~13 of ~30) |
| `all`      | `{all: ['true']}`              | every vehicle in the network (~30) |

Before the #176 fix, `all` was dead code on the SUMO path (`subscribeAllVehicle` parsed
but never consumed) so it delivered **0** vehicles. After the fix it delivers the whole
network, giving the stable ordering `all > radius > ego_only`. (The cap is
`N_MAX_VEH = 100` in `TrafficHelper.h`; this loop holds ~30 on its lanes.)

Run directly (resolve the realsim python first):
```bash
<realsim_python> run_subscription_compare.py --steps 400 --warmup 320   # all three
<realsim_python> run_subscription_compare.py --only all                 # one variant
```

Supporting files:
- `simple_loop_traffic.{rou.xml,sumocfg}` - the loop with **distributed background
  traffic** (4 rotated routes so vehicles spread around the loop, ~30 present). Needed so a
  position-based `radius` subscription is a strict subset rather than catching everyone.
- `config_ego_only.yaml`, `config_radius.yaml`, `config_all.yaml` - the three variants,
  differing only in `VehicleSubscription`.

### Notes on the harness

- **Headless SUMO, runner-owned.** The variant configs set `EnableAutoLaunch: false`; the
  runner launches headless `sumo` itself (no GUI window) and tears it down between variants,
  so ports 1337/2444 are deterministically freed.
- **`--max-echo` (client).** The echo client sends every received vehicle back to
  TrafficLayer by default (`--max-echo 0`), which exercises TrafficLayer's *receive-many*
  path the same way a centralized CAV controller would. `--max-echo N` caps the echo to N
  vehicles (e.g. `1` to mirror XIL, where CarMaker returns only the ego pose). This path
  used to deadlock above ~1 echoed vehicle because of a frame-size bug (#176, see below);
  it now round-trips tens of vehicles per step.

### Frame-size bug fixed under #176 (why the round-trip used to fail)

While building this test, the multi-vehicle echo exposed a latent framing bug independent
of the `all` subscription: `MsgHelper.pack_veh_data` (and the traffic-light/detector
equivalents) returned the **body-only** size, so `SocketHelper.sendData` under-counted the
message's `total_msg_size` header by `msg_each_header_size` (3 bytes) per record. A message
with one record self-corrected by luck; with two or more, the declared total was shorter
than the bytes actually sent, so the receiver (`SocketHelper::recvData` in TrafficLayer)
stopped early, left bytes in the socket, and desynced the next message — eventually reading
a garbage record size and disconnecting. Any client returning ≥2 vehicles (a centralized
controller) hit this. Fixed by returning the full record size from the pack functions;
`recvData` was also hardened to drain partial TCP reads and to reject out-of-range record
sizes instead of overrunning its buffer.
