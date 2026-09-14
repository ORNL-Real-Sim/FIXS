# Dyno / CARLA speed-coupling study

Companion to `FIXS_Applications#24`. No CARLA server, no dyno, no hardware.

## The question

The bench holds a real vehicle against a road resistance. CARLA meanwhile runs
its own vehicle, with its own mass and its own resistance. We tie the two
together and ask **how much that papers over, and what it costs**.

## What is real here and what is not

The bench is [`CommonLib/xil`](../../../../CommonLib/xil/) — the shipped
simulator, driver and link, not a copy:

```
v_ref ─▶ [RobotDriver] ─▶ pedals ─▶ [DynoSimulator] ─▶ v_measured
                                      road resistance
         and the reference crosses [InProcessPair], so swapping to UDP
         against a real cell is a transport change, not a code change
```

The CARLA side is a **surrogate**: a longitudinal ODE written in this file,
parameterised from constants measured off a live 0.9.16 server. No server is
contacted. When one is, the surrogate is what the link points away from.

The surrogate shares the bench's powertrain envelope on purpose, so the two
differ in exactly three separable places: **resistance curve, mass, and the
bench's torque delivery lag**.

## Running it

Use the `realsim` conda env. Everything needs only the standard library;
matplotlib is optional and draws the figure, so use `fixs_applications` for that.

```bash
PY=~/miniconda3/envs/realsim/python.exe

$PY -m pytest tests/Python/unit/test_xil_dyno.py tests/Python/unit/test_xil_link.py -q
$PY -m pytest tests/Xil/Probes/dyno_speed_sync/ -q
$PY tests/Xil/Probes/dyno_speed_sync/dyno_sync_sim.py --sync-sweep 0.002,0.02,0.05
```

## The four couplings

| coupling | what CARLA is doing | why it is here |
|---|---|---|
| `free` | its own driver, never synced | how far the two drift apart on their own |
| `track` | its own driver, closing the loop on the bench speed | **what ORNL's rig actually does** |
| `forced_coast` | coasting, speed overwritten | pessimistic bound: the sync supplies the whole tractive effort |
| `forced_driven` | its own driver **and** speed overwritten | the alternative to tracking |

`track` is the deployed architecture — `carla_standalone_drive.py` computes
`a = 0.55·(v_dyno − v_ego)`, maps it to a pedal and applies it, and never
overwrites CARLA's speed. The `forced_*` couplings are the comparison.

## What it found

**1. The model disagreement changes sign at about 20 m/s.**

```
   v [m/s]     road resistance - CARLA resistance [N]
         0                   -106.1
        10                    -74.8
        20                     -0.6
        30                    116.5
        40                    276.5
```

CARLA's rolling resistance dominates low down, the bench's `C` dominates high up.
Below the crossing the sync pushes CARLA forward; above it, holds CARLA back. No
single bias correction removes that.

**2. Syncing faster buys accuracy, not quietness.**

```
  sync [s]   speed err rms [m/s]   F_sync rms [N]
     0.002              0.0007              879.2
     0.010              0.0028              867.4
     0.020              0.0053              841.6
     0.050              0.0116              742.0
```

A 25x range of sync interval moves the speed error 17x and the injected force
16%. `F_sync = m*dv/T` with `dv` growing in `T`, so the two largely cancel. **The
disturbance is set by the model mismatch, not the sync rate.**

**3. Whether CARLA drives itself dominates everything.**

```
  coupling        err rms [m/s]   F_sync rms [N]   F_sync max [N]
  free                   0.0516              n/a              n/a
  track                  0.1599              n/a              n/a
  forced_coast           0.0087           2681.7           4759.0
  forced_driven          0.0028            867.4           1722.8
```

Coasting makes the overwrite supply the whole tractive effort. With CARLA's own
driver, the sync supplies only the residual — 867 N rms, and 83 kJ against
4235 kJ of tractive work, **1.95 %**.

## Parameters, and which are trustworthy

| value | source |
|---|---|
| CARLA `CdA = 0.377 m^2`, `rho = 1.25` | CARLA source, **confirmed in motion**: `drag/v^2` constant to 3.2 % over 941 samples |
| CARLA `mass = 1845 kg` | `get_physics_control()` on the stock Tesla Model 3 |
| CARLA `roll_coeff = 0.012` | **not measured.** It lives inside CARLA's tire model and only appears lumped into `sum(long_force)`, and it decides where the sign change in finding (1) sits |
| bench road resistance A/B/C | representative EV6 AWD values, not measured from our dyno |
| bench powertrain peaks, `max_speed` | manufacturer-derived, **not measured**. The instrumented cycles reached 41 kW at the wheels against a rated 239 kW, so that data bounds demand and cannot confirm capability |
| bench torque bandwidth 5 Hz, roller inertia 40 kg m^2 | placeholders for the real bench |
| `RobotDriver` PI gains | a placeholder for the real vehicle's controller, and it will flatter one — no rate limits, no acceleration envelope, no regen blending |

**The first thing to replace with measurement is `roll_coeff`**, because it
decides where the sign change lands and that crossing is the whole character of
the mismatch.

## A note on this machine

The full suite and the study both crash intermittently here with a Windows fatal
exception unrelated to any assertion — roughly two runs in three. The same code
passes when it completes, and no crash has reproduced under isolation. Re-run
rather than debug.

## Feeding it a real cycle

```bash
python dyno_sync_sim.py --cycle-csv mycycle.csv --sync-dt 0.01
```

Two columns, `time [s], speed [m/s]`, header optional, resampled onto `--dt`.
