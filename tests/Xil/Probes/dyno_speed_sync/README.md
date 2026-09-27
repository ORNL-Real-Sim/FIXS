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

## The other script: `tracking_scenarios.py`

`dyno_sync_sim.py` prices the couplings. `tracking_scenarios.py` answers a
narrower question — **how should CARLA be told to follow the bench?** — and
writes a plotly page, `out/tracking_scenarios.html`.

Two scenarios side by side. **Free driving** isolates the tracking law: nothing
is in front, so any disagreement is the law and the plant with no planner
reacting to it. **Following a braking leader** gives that disagreement a
consequence, because an IDM sized the reference on the ego achieving it.

Two things a controller can write through `ego.set`, which is the same choice
`COMMAND_SHAPE` makes in `ego_agent_controller.py`:

| command | who closes the speed loop | gains |
|---|---|---|
| `speed` | CARLA's own Ackermann controller | kp 50, ki 5 — the MLK app's |
| `pedals` | we do, writing throttle and brake | kp 0.55, ki 0.5, pedal = a/3.2 |

`kp = 0.55` and the `a/3.2` map come from ORNL's `carla_standalone_drive.py`.
**The integral term does not** — their law is proportional only, and without it
CARLA sits 0.44 m/s slow and drifts 40 m back over 90 s. Set `PEDAL_KI = 0` to
reproduce theirs.

```
  scenario command   err_rms  err_mean  x_diverge[m]  T_bench_pk  T_carla_pk  ratio  %capac
  free     speed      0.0173   -0.0004         -0.03        2169        1712   0.79     58%
  free     pedals     0.3770   -0.0213         -1.92        2169        2349   1.08     57%
  leader   speed      0.0095    0.0006          0.05        1491        1191   0.80     21%
  leader   pedals     0.1216   -0.0010         -0.09        1491        1422   0.95     22%
```

### Why the torque columns are there

Tracking well is not the same as tracking honestly. The two plants are **33 %
apart in effective inertia** (2452 kg against 1845) and their resistance curves
cross — 166 N against 241 N at 10 m/s, 546 N against 429 N at 30 m/s. For CARLA
to hold the bench's speed it must therefore produce a *different* torque than the
bench did, and a stiff loop will deliver that difference without complaint while
the speed trace looks perfect.

So the figure plots **torque on the wheel axis** for both sides on one axis, and
the table reports the peak ratio and how much of CARLA's capability it took.
Under a speed command CARLA peaks at **0.79x** the bench's torque — which is what
the mass ratio predicts, `1845 / 2452 = 0.75` — and never exceeds **58 %** of
what its powertrain can deliver. The tracking is bought with plausible torque,
not by driving CARLA to something no vehicle would do.

That check is the point. Had the ratio come back at 3x, or the capability
fraction at 100 %, the speed command would be producing a correct-looking
trajectory out of a fictional vehicle.

### Which torque, exactly

The bench publishes three per wheel and they are not interchangeable:

| field | what it is |
|---|---|
| `drive_torque_Nm` | what the powertrain delivered, after the lag |
| `brake_torque_Nm` | the friction brake share that actually acted |
| `axle_torque_Nm` | `drive − brake − J_wheel · dω/dt` — **on the wheel axis** |

`axle_torque_Nm` is the measured one. On a chassis dyno it is the contact force
the roller senses times the radius; on an axle dyno it is the shaft torque the
hub transducer reads. The others are upstream of the wheel inertia and are what
the powertrain *produced*, not what the bench *measures*.

An earlier revision of this table plotted `drive − brake`. The difference is the
wheel-inertia term, 38 Nm peak against a 2207 Nm peak, **1.7 %** — so no
conclusion moved — but it was the wrong quantity, and on an axle dyno the wheel
inertia is precisely what the hub unit senses.

The CARLA surrogate is a point mass with no wheel, so its propulsive force times
the radius is the same quantity with no inertia term to subtract. One caveat: in
the `speed` branch there is no powertrain on the CARLA side — `F = m · a_cmd`
straight from the controller, clipped to the envelope — so `T_carla` there is a
torque *implied* by the demanded acceleration rather than one a powertrain
computed. In the `pedals` branch it does go through the envelope.

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
