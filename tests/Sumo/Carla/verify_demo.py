"""
verify_demo.py - gated end-to-end SUMO <-> CARLA smoke test.

Headless self-check (cross-platform, FIXS test convention). SKIPS cleanly
(exit 0) unless CARLA_ROOT is set and CARLA's bundled Town01 co-sim net is
present - so it is safe to run anywhere. When CARLA is available it:

  1. launches CARLA headless (resolved from CARLA_ROOT, OS-aware),
  2. loads the stock Town01 map (no custom asset needed),
  3. runs the SUMO <-> CARLA co-sim on CARLA's bundled Town01 SUMO net for
     VERIFY_STEPS ticks,
  4. asserts SUMO vehicles transferred into CARLA,
  5. tears CARLA down.

Run: python verify_demo.py        (env: VERIFY_STEPS overrides the step count)
"""
import os
import platform
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# the co-sim runtime lives at the repo root: FIXS_root/Carla
CARLA = os.path.normpath(os.path.join(HERE, "..", "..", "..", "Carla"))
sys.path.insert(0, CARLA)  # for run_cosim
sys.path.insert(0, os.path.join(CARLA, "helper_scripts"))
sys.path.insert(0, os.path.join(CARLA, "helper_scripts", "run_synchronization"))

CARLA_ROOT = os.environ.get("CARLA_ROOT")
STEPS = int(os.environ.get("VERIFY_STEPS", "200"))


def main():
    if not CARLA_ROOT:
        print("SKIP: CARLA_ROOT not set - skipping the CARLA end-to-end smoke test.")
        return 0

    town_cfg = os.path.join(CARLA_ROOT, "Co-Simulation", "Sumo", "examples", "Town01.sumocfg")
    if not os.path.isfile(town_cfg):
        print(f"SKIP: CARLA Town01 co-sim net not found ({town_cfg}).")
        return 0

    import run_cosim
    headless = platform.system() != "Windows"
    proc = run_cosim.launch_carla(CARLA_ROOT, 2000, render_offscreen=headless)
    try:
        if not run_cosim.wait_for_port("localhost", 2000):
            print("FAIL: CARLA RPC port did not open.")
            return 1

        import carla
        client = carla.Client("localhost", 2000)
        client.set_timeout(60.0)
        client.load_world("Town01")
        time.sleep(2.0)

        from sumo_integration.sumo_simulation import SumoSimulation
        from sumo_integration.carla_simulation import CarlaSimulation
        from run_synchronization import SimulationSynchronization

        sumo = SumoSimulation(town_cfg, 0.05, None, None, False, 1, use_landmark_tls=True)
        carla_sim = CarlaSimulation("localhost", 2000, 0.05)
        sync = SimulationSynchronization(sumo, carla_sim, tls_manager="none")
        try:
            for _ in range(STEPS):
                sync.tick()
            vehicles = len(carla_sim.world.get_actors().filter("vehicle.*"))
        finally:
            sync.close()

        print(f"[VERIFY] after {STEPS} steps: {vehicles} SUMO vehicles in CARLA")
        if vehicles <= 0:
            print("FAIL: no vehicles transferred SUMO -> CARLA.")
            return 1
        print("PASS: SUMO <-> CARLA co-sim smoke test")
        return 0
    finally:
        run_cosim.kill_carla(proc)


if __name__ == "__main__":
    sys.exit(main())
