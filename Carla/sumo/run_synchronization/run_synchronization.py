#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to integrate CARLA and SUMO simulations
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import time

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

#try:
#    sys.path.append(
#        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
#                  (sys.version_info.major, sys.version_info.minor,
#                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
#except IndexError:
#    pass

import carla  # pylint: disable=import-error

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from sumo_integration.sumo_carla_tl_table_manager import SumoCarlaTLTableManager  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================


def _lerp_heading_deg(prev_deg, next_deg, f):
    """Interpolate a SUMO heading (degrees, north = 0, CLOCKWISE) on the SHORTEST arc.

    The wrap matters: a vehicle pointing due north oscillates across the 360/0 seam
    (359.8 -> 0.2), where a plain lerp would sweep the long way round - a full spin
    inside one step. The +540 offset makes the dividend of the modulo non-negative,
    which python's % already guarantees but C's fmod does not; the formula is kept
    identical to the C++ one (lerpHeadingDeg in CommonLib/VirEnvCore.cpp) so both
    bridges move a vehicle the same way.
    """
    p = prev_deg % 360.0
    n = next_deg % 360.0
    d = (n - p + 540.0) % 360.0 - 180.0
    return (p + d * f) % 360.0


def _lerp_sumo_transform(a, b, f):
    """Blend two SUMO-frame transforms (as sumo_simulation.get_actor builds them:
    location in SUMO coordinates, rotation.yaw = SUMO angle, rotation.pitch = slope).

    Interpolating HERE, before BridgeHelper converts to the CARLA frame, is what
    keeps this bridge equivalent to the native one: VirEnvCore likewise interpolates
    the raw wire pose and leaves the frame conversion to the backend."""
    if f >= 1.0:
        return b                        # exact sample, no arithmetic on the endpoint
    la, lb = a.location, b.location
    ra, rb = a.rotation, b.rotation
    return carla.Transform(
        carla.Location(la.x + (lb.x - la.x) * f,
                       la.y + (lb.y - la.y) * f,
                       la.z + (lb.z - la.z) * f),
        carla.Rotation(ra.pitch + (rb.pitch - ra.pitch) * f,   # slope: no wrap
                       _lerp_heading_deg(ra.yaw, rb.yaw, f),
                       ra.roll + (rb.roll - ra.roll) * f))


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """
    def __init__(self,
                 sumo_simulation,
                 carla_simulation,
                 tls_manager='none',
                 tl_table_manager=None,
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False,
                 net_offset=None,
                 feed_length=0.1,
                 realtime=True):

        self.sumo = sumo_simulation
        self.carla = carla_simulation

        self.tls_manager = tls_manager
        self.tl_table_manager = tl_table_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        # One SUMO step per FIXS feed period; CARLA may tick finer and interpolate
        # across it - the same arrangement the FIXS-native stack and the CarMaker
        # host use (CarMaker runs 0.001 s against the same 0.1 s feed). carla.step_length
        # IS the CARLA tick, so this is how many ticks fall in one SUMO step.
        self.sub_steps = max(1, int(round(feed_length / self.carla.step_length)))
        self.realtime = realtime
        self._sim_time = 0.0
        self._wall_start = time.time()
        # k and k+1 SUMO samples per mapped vehicle, so a sub-step can interpolate
        # between two RECEIVED states rather than between where it last drew and the
        # new target (which lags half a step at coarse sub-stepping).
        self._prev = {}
        self._next = {}

        if tls_manager == 'carla':
            self.sumo.switch_off_traffic_lights()
        elif tls_manager == 'sumo':
            self.carla.switch_off_traffic_lights()

        # Clear vehicles left behind by a crashed or killed earlier run. This bridge
        # owns every vehicle in the world (visualisation co-sim), so any actor already
        # present is an orphan: spawning a fresh fleet on top of one collides with it,
        # doubles the traffic, and on a heavy map has been seen to take the server down.
        # VirCarlaEnv has always done this; the two bridges should not differ here.
        try:
            stale = self.carla.world.get_actors().filter('vehicle.*')
            if len(stale):
                logging.info('clearing %d stale vehicle actor(s) from a prior run',
                             len(stale))
                for actor in stale:
                    actor.destroy()
        except RuntimeError as exc:
            logging.warning('could not clear stale vehicles: %s', exc)

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.
        self.carla2sumo_ids = {}  # Contains only actors controlled by carla.

        # A/B probe, the mirror of VirCarlaEnv's RS_POSE_LOG: per applied pose, the
        # SUMO sample that produced it and the CARLA transform that went out, plus
        # both candidate anchor half-lengths (SUMO length/2, which this bridge uses
        # for the front-bumper->centre shift, and the spawned blueprint's real
        # bbox.extent.x, which the native bridge uses). Set RS_POSE_LOG_PY=<path.csv>.
        self._probe = None
        self._step = 0
        probe_path = os.environ.get("RS_POSE_LOG_PY")
        if probe_path:
            # line-buffered: a Ctrl-C / kill mid-run still leaves a usable log
            self._probe = open(probe_path, "w", buffering=1, encoding="utf-8")
            self._probe.write("step,sub,f,id,bp,sumo_x,sumo_y,sumo_z,sumo_angle,"
                              "sumo_slope,sumo_extent_x,bbox_extent_x,carla_x,carla_y,"
                              "carla_z,carla_yaw,carla_pitch\n")
            print("[probe] py pose log -> " + probe_path)

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        # For a CARLA map generated FROM the SUMO net, the SUMO net offset aligns
        # the two coordinate frames. For a map imported independently (e.g. from
        # RoadRunner) that already sits in the SUMO-local frame, pass
        # net_offset=(0, 0) so vehicles land on the roads instead of being shifted
        # by the (large UTM) net offset and ending up far off the map.
        BridgeHelper.offset = net_offset if net_offset is not None else self.sumo.get_net_offset()

        # Configuring carla simulation in sync mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.carla.step_length
        self.carla.world.apply_settings(settings)

        traffic_manager = self.carla.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

    def _pace(self):
        """Sleep so each CARLA sub-tick lands at its own wall-clock sim time.

        Per SUB-TICK, not per SUMO step, so the ticks spread evenly instead of
        bursting and then waiting - which is what makes a sub-stepped render actually
        look smooth. Never over-throttles: if we fell behind, the sleep is skipped,
        and past a quarter second the reference resyncs so a hitch does not become a
        permanent debt. Same shape as VirCarlaEnv's realtime pacing."""
        self._sim_time += self.carla.step_length
        if not self.realtime:
            return
        target = self._wall_start + self._sim_time
        now = time.time()
        if now < target:
            time.sleep(target - now)
        elif now - target > 0.25:
            self._wall_start = now - self._sim_time

    def tick(self):
        """One FIXS feed step: advance SUMO once, then render `sub_steps` CARLA ticks
        across it, interpolating the traffic between the two SUMO samples.
        """
        # -----------------
        # sumo-->carla sync
        # -----------------
        self.sumo.tick()
        self._step += 1

        # Spawning new sumo actors in carla (i.e, not controlled by carla).
        sumo_spawned_actors = self.sumo.spawned_actors - set(self.carla2sumo_ids.values())
        for sumo_actor_id in sumo_spawned_actors:
            self.sumo.subscribe(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.sync_vehicle_color)
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                   sumo_actor.extent)

                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
            else:
                self.sumo.unsubscribe(sumo_actor_id)

        # Destroying sumo arrived actors in carla.
        for sumo_actor_id in self.sumo.destroyed_actors:
            if sumo_actor_id in self.sumo2carla_ids:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

        # Stage the k -> k+1 pair for every mapped vehicle: the sample just read is
        # k+1, the one staged last time is k. First sight has no k, so it holds still
        # for one step rather than sliding in from nowhere.
        for sumo_actor_id in self.sumo2carla_ids:
            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            self._prev[sumo_actor_id] = self._next.get(sumo_actor_id, sumo_actor)
            self._next[sumo_actor_id] = sumo_actor
        for gone in set(self._next) - set(self.sumo2carla_ids):
            self._prev.pop(gone, None)
            self._next.pop(gone, None)

        # Updates traffic lights in carla based on sumo information. Once per SUMO
        # step: signal states only change on that grid.
        if self.tls_manager == 'sumo':
            if self.tl_table_manager is not None:
                self.tl_table_manager.apply_sumo_states_to_carla()
            else:
                common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
                for landmark_id in common_landmarks:
                    sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
                    carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)

                    self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

        # Updating sumo actors in carla, once per CARLA tick. sub_steps == 1 applies
        # the received pose directly (f = 1.0 short-circuits the blend), so the 1:1
        # case is bit-for-bit what it always was.
        for sub in range(1, self.sub_steps + 1):
            f = sub / float(self.sub_steps)
            for sumo_actor_id in list(self.sumo2carla_ids):
                carla_actor_id = self.sumo2carla_ids[sumo_actor_id]
                prev_actor = self._prev.get(sumo_actor_id)
                sumo_actor = self._next.get(sumo_actor_id)
                if sumo_actor is None:
                    continue
                carla_actor = self.carla.get_actor(carla_actor_id)
                if carla_actor is None:
                    continue

                sumo_transform = _lerp_sumo_transform(prev_actor.transform,
                                                      sumo_actor.transform, f)
                carla_transform = BridgeHelper.get_carla_transform(sumo_transform,
                                                                   sumo_actor.extent)
                # Lights are a per-step state, not something to re-send every tick.
                if self.sync_vehicle_lights and sub == 1:
                    carla_lights = BridgeHelper.get_carla_lights_state(
                        carla_actor.get_light_state(), sumo_actor.signals)
                else:
                    carla_lights = None

                self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

                if self._probe is not None:
                    st, sr = sumo_transform.location, sumo_transform.rotation
                    ct, cr = carla_transform.location, carla_transform.rotation
                    self._probe.write(
                        "%d,%d,%.4f,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%.4f,%.4f,%.4f\n" %
                        (self._step, sub, f, sumo_actor_id, carla_actor.type_id,
                         st.x, st.y, st.z, sr.yaw, sr.pitch,
                         sumo_actor.extent.x, carla_actor.bounding_box.extent.x,
                         ct.x, ct.y, ct.z, cr.yaw, cr.pitch))

            # -----------------
            # carla-->sumo sync
            # -----------------
            self.carla.tick()
            self._pace()

        # Spawning new carla actors (not controlled by sumo)
        carla_spawned_actors = self.carla.spawned_actors - set(self.sumo2carla_ids.values())
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)

            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            color = carla_actor.attributes.get('color', None) if self.sync_vehicle_color else None
            if type_id is not None:
                sumo_actor_id = self.sumo.spawn_actor(type_id, color)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)

        # Destroying required carla actors in sumo.
        for carla_actor_id in self.carla.destroyed_actors:
            if carla_actor_id in self.carla2sumo_ids:
                self.sumo.destroy_actor(self.carla2sumo_ids.pop(carla_actor_id))

        # Updating carla actors in sumo.
        for carla_actor_id in self.carla2sumo_ids:
            sumo_actor_id = self.carla2sumo_ids[carla_actor_id]

            carla_actor = self.carla.get_actor(carla_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            sumo_transform = BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
            if self.sync_vehicle_lights:
                carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                if carla_lights is not None:
                    sumo_lights = BridgeHelper.get_sumo_lights_state(sumo_actor.signals,
                                                                     carla_lights)
                else:
                    sumo_lights = None
            else:
                sumo_lights = None

            self.sumo.synchronize_vehicle(sumo_actor_id, sumo_transform, sumo_lights)

        # Updates traffic lights in sumo based on carla information.
        if self.tls_manager == 'carla':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                carla_tl_state = self.carla.get_traffic_light_state(landmark_id)
                sumo_tl_state = BridgeHelper.get_sumo_traffic_light_state(carla_tl_state)

                # Updates all the sumo links related to this landmark.
                self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)

    def close(self):
        """
        Cleans synchronization.
        """
        if self._probe is not None:
            self._probe.close()
            self._probe = None
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.sumo2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        for sumo_actor_id in self.carla2sumo_ids.values():
            self.sumo.destroy_actor(sumo_actor_id)

        # Closing sumo and carla client.
        self.carla.close()
        self.sumo.close()


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    use_landmark_tls = not (args.tls_manager == 'sumo' and args.tl_table is not None)
    # --step-length is the FIXS feed period: one SUMO step per exchange. --carla-tick
    # is the CARLA world step, which may be finer; CarlaSimulation.step_length is what
    # becomes fixed_delta_seconds, so it gets the tick, not the feed.
    carla_tick = args.carla_tick or args.step_length
    if carla_tick > args.step_length + 1e-12:
        carla_tick = args.step_length
    sumo_simulation = SumoSimulation(args.sumo_cfg_file, args.step_length, args.sumo_host,
                                     args.sumo_port, args.sumo_gui, args.client_order,
                                     use_landmark_tls=use_landmark_tls)
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, carla_tick,
                                       timeout=args.carla_timeout)

    tl_table_manager = None
    if args.tls_manager == 'sumo':
        if args.tl_table is None:
            raise ValueError('--tl-table is required when --tls-manager=sumo')
        tl_table_manager = SumoCarlaTLTableManager(
            args.tl_table,
            carla_simulation.world,
            offset_x=args.offset_x,
            offset_y=args.offset_y,
            max_match_dist=args.max_match_dist)

    net_offset = (0.0, 0.0) if args.no_net_offset else None
    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                tl_table_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights,
                                                net_offset=net_offset,
                                                feed_length=args.step_length,
                                                realtime=not args.no_realtime)
    logging.info('cadence: SUMO step %.3f s | CARLA tick %.3f s (%d sub-step(s)) | '
                 'pacing %s', args.step_length, carla_tick, synchronization.sub_steps,
                 'as fast as possible' if args.no_realtime else 'realtime')
    try:
        while True:
            # Pacing lives inside tick(), per CARLA sub-tick, so it is even rather
            # than one sleep at the end of each SUMO step.
            synchronization.tick()

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')

        synchronization.close()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('sumo_cfg_file', type=str, help='sumo configuration file')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--carla-timeout',
                           metavar='S',
                           default=10.0,
                           type=float,
                           help='CARLA client connect timeout in seconds (default: 10)')
    argparser.add_argument('--no-realtime',
                           action='store_true',
                           help='do not pace to real time; run as fast as possible')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None,
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=None,
                           type=int,
                           help='TCP port to listen to (default: 8813)')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo')
    argparser.add_argument('--step-length',
                           default=0.1,
                           type=float,
                           help='SUMO step length, i.e. the FIXS feed period: one SUMO '
                                'step per exchange (default: 0.1s, the FIXS contract)')
    argparser.add_argument('--carla-tick',
                           default=None,
                           type=float,
                           help='CARLA world step / fixed_delta_seconds. Defaults to '
                                '--step-length (1:1). Finer than that and the traffic '
                                'is interpolated across the sub-steps - position and '
                                'heading - which is what the FIXS-native bridge and '
                                'the CarMaker host do with the same feed.')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--tl-table',
                           type=str,
                           default=None,
                           help='traffic light table csv used for SUMO -> CARLA traffic light sync')
    argparser.add_argument('--offset-x',
                           default=0.0,
                           type=float,
                           help='SUMO x offset applied when matching CARLA traffic lights to the table')
    argparser.add_argument('--offset-y',
                           default=0.0,
                           type=float,
                           help='SUMO y offset applied when matching CARLA traffic lights to the table')
    argparser.add_argument('--max-match-dist',
                           default=50.0,
                           type=float,
                           help='maximum SUMO-space distance for CARLA traffic light to table matching')
    argparser.add_argument('--no-net-offset',
                           action='store_true',
                           help='Do not apply the SUMO net offset to vehicle placement. Use for CARLA '
                                'maps imported independently (e.g. RoadRunner) that already sit in the '
                                'SUMO-local frame; without it vehicles are shifted off-map by the UTM offset.')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)
