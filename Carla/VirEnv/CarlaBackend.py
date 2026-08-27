"""CarlaBackend -- the CARLA half of the verb interface.

Python peer of ``VirCarlaEnv/VirCarlaEnv/CarlaBackend.{h,cpp}``. The ONLY module
on the Python bridge path that imports ``carla``; ``CommonLib/VirEnv`` stays
SDK-free. Every verb is the same lift of the per-vehicle CARLA code that the C++
backend is: try_spawn_actor / destroy, the BridgeHelper transforms, the batched
apply_transform, the junction TLS dispatch. The thin ``mainVirCarla`` driver owns
the world tick, the batch flush, the interested-vehicle readback and the
spectator.

Naming follows the C++ verbatim, including the driver hooks that are not part of
``IVirEnvBackend`` (``flushBatch``, ``freezeAndMatchTrafficLights``, ``actorOf``,
``lastAppliedPose``, ``spawnEgo``, ``auditZAlignment``).
"""

import math
import os
import sys

import carla

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from CommonLib.VirEnv.IVirEnvBackend import IVirEnvBackend, kNoHandle   # noqa: E402

from .BridgeHelper import BridgeHelper                                  # noqa: E402

__all__ = ['CarlaBackend', 'kZMismatchTolM']

#: Tolerance for the SUMO <-> CARLA per-vehicle z-alignment guard (#193
#: placeholder). Above this, a teleported car is off the CARLA road surface enough
#: to warn. With a densified elevation net the residual is ~mesh discretisation (a
#: few cm), so 0.5 m warns only on a genuine map/elevation mismatch.
kZMismatchTolM = 0.5

# The SUMO/FIXS wire carries the FRONT-of-vehicle position; a CARLA actor
# transform is the actor PIVOT, which sits at the bounding-box CENTRE
# horizontally. Landing the model's FRONT on the SUMO front therefore means
# stepping back the actor HALF-length == bounding_box.extent.x, which
# map_transfrom_Sumo_to_Carla does with extent.x. So the CORRECT extent is the
# actor's real extent -- which the reverse readback already uses, making forward
# and reverse symmetric.
#
# kDefaultExtent is ONLY a sane passenger-car fallback for the TRANSIENT spawn
# transform: an actor's bbox is not queryable until it exists, and setVehiclePose
# overwrites the pose the same tick.
_kDefaultExtent = carla.Vector3D(2.3, 1.0, 0.75)
_kSpawnOffsetZ = 0.1


class CarlaBackend(IVirEnvBackend):
    """Implements :class:`IVirEnvBackend` against the CARLA Python client API."""

    def __init__(self, world, client, useVehicleTypeAsBlueprint, verbose):
        self._world = world
        self._client = client
        self._useVType = useVehicleTypeAsBlueprint
        self._verbose = verbose

        self._bpLib = None
        self._map = None                 # cached for the z-alignment guard
        self._egoActor = None            # EgoMode >= 1: the CARLA-driven ego
        self._tmPort = 0
        self._egoUsesTM = False
        self._egoDesiredOverride = -1.0  # L2 advisory target (m/s); < 0 = none
        self._batch = []                 # apply_transform commands this tick
        self._actors = {}                # VehHandle -> carla.Vehicle
        self._lastApplied = {}           # VehHandle -> carla.Transform (A/B + z audit)
        self._trafficLightMap = {}
        self._zWarned = 0                # rate limit for the z-mismatch guard

    # --- logging -----------------------------------------------------------
    def log(self, msg):
        print(msg)

    def logError(self, msg):
        print(msg, file=sys.stderr)

    # --- helpers -----------------------------------------------------------
    @staticmethod
    def _sumoTransformOf(p):
        """(Pose) -> carla.Transform -- the raw FIXS pose as a SUMO-frame transform.

        Verbatim from the C++ ``sumoTransformOf``: Location(x, y, z),
        Rotation(grade in degrees, heading, 0).
        """
        return carla.Transform(
            carla.Location(p.x, p.y, p.z),
            carla.Rotation(p.gradeRad * 180.0 / math.pi, p.headingDeg, 0.0))

    def _extentOf(self, h):
        """The actor's real half-size, falling back on the spawn frame."""
        actor = self._actors.get(h)
        if actor is not None:
            e = actor.bounding_box.extent
            if e.x > 0.1:                # guard a not-yet-populated bbox
                return e
        return _kDefaultExtent

    # --- traffic pool lifecycle -------------------------------------------
    def loadSignalTable(self, path):
        if not path:
            return
        self._trafficLightMap = BridgeHelper.readTrafficLightTable(path)

    def initTrafficPool(self):
        """CARLA spawns lazily; this only caches the blueprint library."""
        if self._bpLib is None and self._world is not None:
            self._bpLib = self._world.get_blueprint_library()

    def spawnVehicle(self, vType, vClass, spawnPose):
        if self._world is None:
            return kNoHandle
        if self._bpLib is None:
            self._bpLib = self._world.get_blueprint_library()

        # The spawn transform is transient: setVehiclePose corrects it with the
        # real bbox the same tick (before world.tick), so a passenger-car default
        # extent is sufficient here.
        carlaTf = BridgeHelper.map_transfrom_Sumo_to_Carla(
            self._sumoTransformOf(spawnPose), _kDefaultExtent)
        carlaTf.location.z += _kSpawnOffsetZ

        bpId = vType if self._useVType else \
            BridgeHelper.map_Sumo_vClass_to_Carla_blueprintId(vClass)
        try:
            bp = self._bpLib.find(bpId)
        except (IndexError, RuntimeError):
            bp = None
        if bp is None:
            self.logError('Blueprint not found: %s' % bpId)
            return kNoHandle

        actor = self._world.try_spawn_actor(bp, carlaTf)
        if actor is None:
            if self._verbose:
                print('[Warning] Failed to spawn actor (vClass=%s)' % vClass)
            return kNoHandle
        actor.set_simulate_physics(False)

        h = int(actor.id)
        self._actors[h] = actor
        if self._verbose:
            print('Spawned Carla actor %d (%s)' % (h, bpId))
        return h

    def despawnVehicle(self, h):
        actor = self._actors.pop(h, None)
        if actor is not None:
            actor.destroy()
        # Drop the applied-pose record too. Left behind, it would grow for the
        # whole run (a thousand-plus arrived vehicles over a few minutes) and a
        # destroyed actor's pose would stay queryable through lastAppliedPose(),
        # which the per-exchange z audit and the A/B log both read.
        self._lastApplied.pop(h, None)

    # --- per-step actuation ------------------------------------------------
    def setVehiclePose(self, h, p):
        carlaTf = BridgeHelper.map_transfrom_Sumo_to_Carla(
            self._sumoTransformOf(p), self._extentOf(h))
        self._lastApplied[h] = carlaTf
        # Batched, applied in flushBatch() before the world tick -- as the C++ does.
        self._batch.append(carla.command.ApplyTransform(h, carlaTf))

    def setVehicleLights(self, h, brake, indL, indR):
        """No-op: the CARLA bridge sets no vehicle lights, matching the C++."""

    def parkSpares(self):
        """No-op: CARLA has no pre-placed spare pool."""

    def syncTrafficLight(self, junctionId, stateStr):
        linkMap = self._trafficLightMap.get(junctionId)
        if linkMap is None:
            return
        for linkId, ch in enumerate(stateStr):
            ss = BridgeHelper.get_Sumo_traffic_light_state_from_char(ch)
            cs = BridgeHelper.map_Sumo_traffic_light_state_to_Carla(ss)
            tl = linkMap.get(linkId)
            if tl is not None and tl.carlaTrafficLightActorPtr is not None:
                tl.carlaTrafficLightActorPtr.set_state(cs)

    # --- ego coupling ------------------------------------------------------
    def readEgoState(self, egoId, out):
        """Mode A: read the CARLA-driven ego back in FIXS terms.

        Returns False when no ego actor is owned (EgoMode 0 -- the driver does the
        readback itself for interested ids).
        """
        if self._egoActor is None:
            return False
        cTf = self._egoActor.get_transform()
        ext = self._egoActor.bounding_box.extent
        vel = self._egoActor.get_velocity()
        sTf = BridgeHelper.map_transfrom_Carla_to_Sumo(cTf, ext)
        out.x = sTf.location.x
        out.y = sTf.location.y
        out.z = sTf.location.z
        out.heading = sTf.rotation.yaw
        out.grade = sTf.rotation.pitch * math.pi / 180.0
        out.speed = math.sqrt(vel.x * vel.x + vel.y * vel.y)
        return True

    def applyEgoControl(self, egoId, desiredSpeed):
        """L2 actuation seam: route an EXTERNAL desired-speed advisory to the driver.

        Native TM -> ``set_desired_speed`` (km/h) on the ego's TM instance. No ego
        -> nothing to advise.
        """
        if self._egoActor is None:
            return
        self._egoDesiredOverride = desiredSpeed
        if self._egoUsesTM and self._client is not None:
            tm = self._client.get_trafficmanager(self._tmPort)
            tm.set_desired_speed(self._egoActor, desiredSpeed * 3.6)   # TM is km/h

    # --- unified debug diagnostics -----------------------------------------
    def debugHeader(self):
        return ',carla_x,carla_y,carla_z,carla_yaw,carla_pitch'

    def debugFields(self, h):
        t = self._lastApplied.get(h)
        if t is None:
            return ',,,,,'
        return ',%.4f,%.4f,%.4f,%.4f,%.4f' % (
            t.location.x, t.location.y, t.location.z, t.rotation.yaw, t.rotation.pitch)

    # ------------------------------------------------------------------------
    #  Driver hooks -- not part of IVirEnvBackend
    # ------------------------------------------------------------------------
    def flushBatch(self):
        """Apply this tick's transform commands as one batch.

        Synchronous (``do_tick=False`` but awaited): #267 measured that an async
        ``apply_batch`` could miss the very tick that rendered it, so a vehicle
        stood still for a frame and then jumped double while the camera moved
        smoothly. At CarlaTimeStep 0.1 that hit 24% of frames.
        """
        if self._client is not None and self._batch:
            self._client.apply_batch_sync(self._batch, False)
        self._batch = []

    def auditZAlignment(self):
        """SUMO <-> CARLA z-alignment audit (#193 placeholder).

        A teleported (physics-off) car sits at SUMO's z. If SUMO's road elevation
        diverges from the CARLA road surface under it -- a coarsely-sampled net
        against the xodr, or an inconsistent map pair -- the car floats or sinks.
        So compare the applied pose against the CARLA road and report past
        tolerance. Warning only; the abort / snap-to-road / dyno-invalid policy is
        #193.

        Called ONCE PER FIXS EXCHANGE by the driver, not from setVehiclePose: it
        asks whether the two MAPS agree on elevation, and two maps do not start
        agreeing halfway through a 0.1 s interval -- so re-asking on every
        interpolated sub-step bought nothing and cost a whole-map waypoint search
        per vehicle per tick.
        """
        if self._world is None:
            return
        if self._map is None:
            self._map = self._world.get_map()
        if self._map is None:
            return
        for h, tf in self._lastApplied.items():
            wp = self._map.get_waypoint(tf.location)
            if wp is None:
                continue
            dz = tf.location.z - wp.transform.location.z
            if abs(dz) > kZMismatchTolM:
                self._warnZ(h, dz)

    def _warnZ(self, h, dz):
        """Rate-limited z-mismatch warning, the peer of fixs::RS_XIL_GUARD's.

        Rate-limited per condition rather than per actor, as the C++ guard is: on a
        genuinely misaligned map every vehicle violates every exchange, and an
        unlimited log is then the run's dominant output.
        """
        self._zWarned += 1
        if self._zWarned <= 5 or self._zWarned % 1000 == 0:
            print('[XIL GUARD] sumo_carla_z_mismatch: actor %d off the CARLA road '
                  'surface by %.3f m (tolerance %.3f m) [occurrence %d]'
                  % (h, dz, kZMismatchTolM, self._zWarned), file=sys.stderr)

    def freezeAndMatchTrafficLights(self):
        """Freeze CARLA's own signal logic and match each actor to a table row.

        Freezing is what makes the traffic simulator the single source of signal
        truth: an unfrozen CARLA light runs its own program and would fight the
        state this bridge writes every exchange.
        """
        if self._world is None:
            return
        matched = 0
        for actor in self._world.get_actors().filter('traffic.traffic_light'):
            actor.freeze(True)
            sloc = BridgeHelper.map_location_Carla_to_Sumo(actor.get_location())
            junctionId, linkId = BridgeHelper.find_closest_trafficLight_id(
                self._trafficLightMap, sloc.x, sloc.y)
            if not junctionId:
                continue
            tl = self._trafficLightMap[junctionId][linkId]
            tl.carlaTrafficLightActorId = str(actor.id)
            tl.carlaTrafficLightActorPtr = actor
            matched += 1
        if self._verbose:
            print('Matched %d CARLA traffic-light actor(s) to the signal table' % matched)

    def actorOf(self, h):
        """(VehHandle) -> carla.Vehicle or None -- for readback / spectator."""
        return self._actors.get(h)

    def trafficLightMap(self):
        return self._trafficLightMap

    def lastAppliedPose(self, h):
        """(VehHandle) -> carla.Transform or None -- the pose last APPLIED to h.

        A/B instrumentation: the driver logs it keyed by SUMO id so old-vs-new can
        be diffed exactly, with no CARLA-readback / blueprint / sort confounds. It
        is also the pose the NEXT tick will render, which is why the spectator
        follow reads it here rather than reading the actor back after the tick
        (#254).
        """
        return self._lastApplied.get(h)

    # --- L0+ ego ownership (EgoMode >= 1) ----------------------------------
    def spawnEgo(self, blueprintId, spawnPose, tmPort):
        """Spawn the ego with PHYSICS ON. The driver is wired separately."""
        if self._world is None:
            return kNoHandle
        if self._bpLib is None:
            self._bpLib = self._world.get_blueprint_library()
        try:
            bp = self._bpLib.find(blueprintId)
        except (IndexError, RuntimeError):
            bp = None
        if bp is None:
            self.logError('Ego blueprint not found: %s' % blueprintId)
            return kNoHandle

        tf = BridgeHelper.map_transfrom_Sumo_to_Carla(
            self._sumoTransformOf(spawnPose), _kDefaultExtent)
        tf.location.z += 0.3        # drop-in margin: physics ON, it settles on its tires

        actor = self._world.try_spawn_actor(bp, tf)
        if actor is None:
            self.logError('Ego spawn failed (spawn point blocked?)')
            return kNoHandle
        self._egoActor = actor
        self._egoActor.set_simulate_physics(True)     # full PhysX: tire contact, dynamics
        print('L0 ego spawned: %s actor %d (physics ON)' % (blueprintId, actor.id))
        return int(actor.id)

    def enableEgoTM(self, tmPort, targetSpeedMps):
        """Hand the ego to CARLA's server-side Traffic Manager.

        Spawn and physics must already be done, and TM must be synchronous in a
        synchronous world -- ``world.tick()`` then drives TM's synchronous tick.
        The sequence mirrors the verified standalone probe; reordering it is what
        makes TM silently not drive.
        """
        if self._egoActor is None or self._client is None:
            return
        tm = self._client.get_trafficmanager(tmPort)
        tm.set_synchronous_mode(True)
        self._egoActor.set_autopilot(True, tmPort)
        tm.set_desired_speed(self._egoActor, targetSpeedMps * 3.6)   # TM speed is km/h
        self._tmPort = tmPort
        self._egoUsesTM = True
        print('L0 ego: NATIVE Traffic Manager autopilot (TM port %d, target %s m/s)'
              % (tmPort, targetSpeedMps))

    def applyEgoActuation(self, throttle, brake, steerNorm):
        """Apply an ACTUATION command supplied by an external FIXS client.

        throttle/brake in [0, 1], steerNorm in [-1, 1]. CARLA owns no in-process
        driver in this mode; it just realises the wire command on the physics ego.
        """
        if self._egoActor is None:
            return
        self._egoActor.apply_control(carla.VehicleControl(
            throttle=float(max(0.0, min(1.0, throttle))),
            brake=float(max(0.0, min(1.0, brake))),
            steer=float(max(-1.0, min(1.0, steerNorm)))))

    def egoActor(self):
        return self._egoActor

    def destroyEgo(self):
        if self._egoActor is not None:
            self._egoActor.destroy()
            self._egoActor = None
