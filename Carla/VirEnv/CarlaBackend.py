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

__all__ = ['CarlaBackend', 'kZMismatchTolM', 'kZAuditStride']

#: Tolerance for the SUMO <-> CARLA per-vehicle z-alignment guard (#193
#: placeholder). Above this, a teleported car is off the CARLA road surface enough
#: to warn. With a densified elevation net the residual is ~mesh discretisation (a
#: few cm), so 0.5 m warns only on a genuine map/elevation mismatch.
kZMismatchTolM = 0.5

#: How many exchanges the z audit takes to cover every mapped vehicle.
#:
#: 5, not 20. The check is for a static map/elevation mismatch, so in principle any
#: stride finds it eventually -- but the violations actually observed on the MLK
#: corridor ran ~5 consecutive exchanges (one vehicle crossing one bad stretch),
#: and a stride of 20 would sample a 0.5 s event about a quarter of the time. 5
#: covers every mapped vehicle within 0.5 s, so nothing that was reported before is
#: missed now, and it still removes four fifths of the cost.
kZAuditStride = 5

#: How far below the world a spare vehicle waits. Physics is off, so it neither
#: falls nor collides; this only has to clear the network, and 200 m clears every
#: corridor FIXS has imported (MLK's road surface sits near z 205).
_kParkZ = -200.0

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

    @property
    def carlaWorld(self):
        """The live carla.World, for an in-process controller that needs a map.

        FIXS holds the backend client, so a road question a CARLA-shaped agent
        asks -- `get_world().get_map().get_waypoint(...)` -- is FORWARDED to the
        real map rather than answered from a reconstruction of it. Named here
        and not on IVirEnvBackend because it is CARLA's own type: a caller that
        reaches for it is asking for CARLA, and gets nothing from any other
        backend.
        """
        return self._world

    @property
    def carlaClient(self):
        """The live carla.Client, for `fixs.carla.client`.

        Handed out rather than let a controller open its own: a synchronous
        world may only be advanced by one party, and this is the one already
        driving the run.
        """
        return self._client

    @property
    def carlaEgoActor(self):
        """The physics ego CARLA owns, for `fixs.carla.ego`.

        This is the vehicle a user's agent should be built on. It is real, its
        physics are live, and its get_velocity is truthful -- measured 9.839 m/s
        against the wire's 9.83768 at the same instant -- so there is nothing
        about the ego worth standing in for.
        """
        return self._egoActor

    def __init__(self, world, client, useVehicleTypeAsBlueprint, verbose,
                 sparePoolSize=0):
        self._world = world
        self._client = client
        self._useVType = useVehicleTypeAsBlueprint
        self._verbose = verbose

        #: CarlaSetup.SpareVehiclePool -- actors to spawn UP FRONT and hand out
        #: when the traffic arrives. 0 is off, and off is today's behaviour.
        self._sparePoolSize = max(0, int(sparePoolSize or 0))
        self._spares = {}                # blueprint id -> [unused carla.Vehicle]
        self._sparesLeft = 0             # how many are still parked
        self._spareHits = 0              # handed out instead of spawned
        self._spareMisses = 0            # wanted a spare, had none of that blueprint
        self._spareTaken = False         # has the pool been drawn from at all
        self._peakMapped = 0             # most vehicles alive at once, for the report

        self._bpLib = None
        self._map = None                 # cached for the z-alignment guard
        self._egoActor = None            # EgoMode >= 1: the CARLA-driven ego
        #: True between spawning the ego and CARLA's first snapshot of it. The
        #: actor exists on the client the instant try_spawn_actor returns, but
        #: its transform reads the ORIGIN until a tick delivers a snapshot --
        #: so readEgoState would report a pose that is not the ego's.
        self._egoAwaitingSnapshot = False
        # Backstep guard state; see readEgoState. FIXS#358.
        self._lastEgoPose = None
        self._egoBackstepHolds = 0
        self._egoBackstepGuard = os.environ.get('FIXS_EGO_BACKSTEP_GUARD', '1') != '0'
        self._egoBackstepRelease = float(
            os.environ.get('FIXS_EGO_BACKSTEP_RELEASE', '0.5'))
        self._tmPort = 0
        self._egoUsesTM = False
        self._egoDesiredOverride = -1.0  # L2 advisory target (m/s); < 0 = none
        self._batch = []                 # apply_transform commands this tick
        self._actors = {}                # VehHandle -> carla.Vehicle
        self._lastApplied = {}           # VehHandle -> carla.Transform (A/B + z audit)
        self._extentX = {}           # VehHandle -> bounding_box.extent.x (fixed per actor)
        self._trafficLightMap = {}
        self._zWarned = 0                # rate limit for the z-mismatch guard
        self._zAuditPhase = -1           # rotating slice for the z audit

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

    def _extentXOf(self, h):
        """The actor half-length used as the pose anchor, cached per handle.

        Only extent.x is ever read (the anchor steps back along the vehicle axis),
        and a spawned actor's bounding box does not change -- so this is one
        boost::python property fetch per vehicle per LIFETIME instead of two per
        vehicle per TICK. On the MLK corridor that is ~1500 fetches instead of
        ~2.5 million.

        Not cached on the tick the bbox is still unpopulated (extent.x <= 0.1):
        the spawn-frame default is returned and the real value is picked up next
        tick, so a vehicle can never be anchored on the fallback for good.
        """
        e = self._extentX.get(h)
        if e is not None:
            return e
        actor = self._actors.get(h)
        if actor is not None:
            ex = actor.bounding_box.extent.x
            if ex > 0.1:
                self._extentX[h] = ex
                return ex
        return _kDefaultExtent.x

    # --- traffic pool lifecycle -------------------------------------------
    def loadSignalTable(self, path):
        if not path:
            return
        self._trafficLightMap = BridgeHelper.readTrafficLightTable(path)

    def initTrafficPool(self):
        """Everything this backend can pay for BEFORE the first exchange.

        The core calls this on the first step, at simTime 0 -- the one iteration
        that does no recv -- so with a warm-up the bridge then blocks here for the
        whole warm-up and anything done now is done in dead time. Three things,
        each of which otherwise lands in the tick where the traffic arrives:

        * the blueprint library;
        * the MAP. ``world.get_map()`` serialises and re-parses the whole
          OpenDRIVE and is not shared between callers -- measured twice back to
          back in one process at 0.640 s and 0.578 s. The z-alignment audit is its
          only user here and first runs on the exchange that carries the traffic,
          so without this the parse is paid inside that exchange;
        * the spare pool, when CarlaSetup.SpareVehiclePool asks for one.
        """
        if self._world is None:
            return
        if self._bpLib is None:
            self._bpLib = self._world.get_blueprint_library()
        if self._map is None:
            self._map = self._world.get_map()
        if self._sparePoolSize and not self._spares:
            self._preSpawnSpares(self._sparePoolSize)

    # --- the spare pool ----------------------------------------------------
    def _parkTransform(self, i, cols=25, pitch=8.0):
        """A lattice well below the road surface, keyed off the map's own extent.

        Physics is off, so a parked actor neither falls nor collides; the depth
        only has to clear the network. Measured on the MLK corridor (road surface
        ~z 205): 250 of 250 spawned at z = -200, none failed. WHERE they sit makes
        no difference to what they cost -- 0.030 ms/tick each below the map
        against 0.039 ms/tick 20 km away -- so this is about not colliding with
        the road, not about hiding them from the camera.
        """
        cx, cy = 0.0, 0.0
        spawn = self._map.get_spawn_points() if self._map is not None else []
        if spawn:
            cx = sum(p.location.x for p in spawn) / len(spawn)
            cy = sum(p.location.y for p in spawn) / len(spawn)
        return carla.Transform(carla.Location(
            x=cx + (i % cols) * pitch, y=cy + (i // cols) * pitch, z=_kParkZ))

    def _spareBlueprints(self, n):
        """Which blueprints to stock, and how many of each.

        Round-robin over the passenger set, NOT a draw from it. The draw that
        matters belongs to the vehicles: map_Sumo_vClass_to_Carla_blueprintId
        picks from a seeded generator, and the blueprint it picks fixes
        bounding_box.extent.x, which is the pose anchor (FIXS#355). Consuming that
        generator here would hand every vehicle a different model than before. A
        spare is used only when its blueprint id MATCHES what the vehicle drew, so
        nothing about the result changes; an even spread only maximises how often
        that match is there to be had.

        A vClass whose blueprints are not stocked, and every vehicle when
        UseVehicleTypeAsBlueprint is on, falls through to try_spawn_actor exactly
        as before.
        """
        pool = BridgeHelper._BY_VCLASS.get('passenger') or ()
        if not pool:
            return []
        return [pool[i % len(pool)] for i in range(n)]

    def _preSpawnSpares(self, n):
        """Park n actors now so the arrival burst does not have to spawn them."""
        made = 0
        for i, bpId in enumerate(self._spareBlueprints(n)):
            try:
                bp = self._bpLib.find(bpId)
            except (IndexError, RuntimeError):
                continue
            actor = self._world.try_spawn_actor(bp, self._parkTransform(i))
            if actor is None:
                continue
            actor.set_simulate_physics(False)
            self._spares.setdefault(bpId, []).append(actor)
            made += 1
        self._sparesLeft = made
        print('Spare vehicle pool: %d of %d parked across %d blueprints; the '
              'arrival burst is handed these instead of spawning them.'
              % (made, n, len(self._spares)))
        if made < n:
            print('[Warning] only %d of %d spares could be parked; the rest of the '
                  'burst spawns as before.' % (made, n))

    def _takeSpare(self, bpId):
        """(string) -> carla.Vehicle or None -- an unused actor of that blueprint."""
        free = self._spares.get(bpId)
        if not free:
            if self._sparesLeft:
                self._spareMisses += 1
            return None
        actor = free.pop()
        self._sparesLeft -= 1
        self._spareHits += 1
        self._spareTaken = True
        return actor

    def spareTaken(self):
        """() -> bool -- has the pool been drawn from? The driver's cue to trim."""
        return self._spareTaken

    def trimSpares(self):
        """Destroy whatever the burst did not need. Once, and not before it.

        A parked actor is not free: measured, it costs ~0.03 ms of EVERY
        world.tick just by existing, so a pool left standing taxes the whole run
        to have saved one exchange. Removing one is cheap by comparison -- 0.16 ms
        batched against 1.6-3.8 ms to spawn it -- so the leftovers go as soon as
        the burst is over.
        """
        left = [a for free in self._spares.values() for a in free]
        self._spares = {}
        self._sparesLeft = 0
        if not left:
            return 0
        if self._client is not None:
            self._client.apply_batch_sync(
                [carla.command.DestroyActor(a.id) for a in left], False)
        else:                                   # no client (tests): per actor
            for a in left:
                a.destroy()
        print('Spare vehicle pool: trimmed %d unused (a parked actor still costs '
              'every tick).' % len(left))
        return len(left)

    def spareReport(self):
        """() -> string -- what the pool did, and what to set it to next time."""
        if not self._sparePoolSize and not self._peakMapped:
            return ''
        enough = self._sparePoolSize >= self._peakMapped
        return ('Spare vehicle pool: %d configured, %d handed out, %d found none of '
                'their blueprint and spawned; peak %d vehicles alive at once%s'
                % (self._sparePoolSize, self._spareHits, self._spareMisses,
                   self._peakMapped,
                   '.' if enough else
                   ' -- CarlaSetup.SpareVehiclePool: %d covers it.' % self._peakMapped))

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

        # A parked spare of the SAME blueprint is this vehicle, already made. The
        # pose it is parked at is as transient as a spawn transform: setVehiclePose
        # corrects both in this same step, before world.tick.
        actor = self._takeSpare(bpId)
        if actor is None:
            actor = self._world.try_spawn_actor(bp, carlaTf)
            if actor is None:
                if self._verbose:
                    print('[Warning] Failed to spawn actor (vClass=%s)' % vClass)
                return kNoHandle
            actor.set_simulate_physics(False)

        h = int(actor.id)
        self._actors[h] = actor
        if len(self._actors) > self._peakMapped:
            self._peakMapped = len(self._actors)
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
        self._extentX.pop(h, None)

    # --- per-step actuation ------------------------------------------------
    def setVehiclePose(self, h, p):
        # The numeric path: BridgeHelper.map_transfrom_Sumo_to_Carla builds an
        # intermediate carla.Transform purely to read six floats back out of it,
        # which is three boost::python constructions per vehicle per tick for
        # nothing. Same arithmetic, same function -- see sumo_to_carla_numeric.
        x, y, z, pitch, yaw, roll = BridgeHelper.sumo_to_carla_numeric(
            p.x, p.y, p.z, p.headingDeg, p.gradeRad * 180.0 / math.pi, 0.0,
            self._extentXOf(h))
        carlaTf = carla.Transform(carla.Location(x, y, z),
                                  carla.Rotation(pitch, yaw, roll))
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

        Returns False when there is no pose to report: no ego actor is owned
        (EgoMode 0 -- the driver does the readback itself for interested ids),
        or the ego was spawned this tick and CARLA has not yet snapshotted it.

        The second case is not hypothetical. A freshly spawned actor's
        get_transform() returns the ORIGIN until the next tick delivers a
        snapshot, so the first readback of a deferred ego reported (0, 0) while
        the ego was really 933 m away. Callers treat False as "not this tick",
        which is the honest answer -- and it spares every controller from
        recognising the jump by its size, which only works while the origin
        happens to be far from the route.
        """
        if self._egoActor is None or self._egoAwaitingSnapshot:
            return False
        cTf = self._egoActor.get_transform()
        ext = self._egoActor.bounding_box.extent
        vel = self._egoActor.get_velocity()
        sTf = BridgeHelper.map_transfrom_Carla_to_Sumo(cTf, ext)
        # Backstep guard: never report a pose behind the last one reported --
        # the sign is lost on the `out.speed` line below, and SUMO answers a
        # backward target by throwing the ego off its lane. FIXS#358.
        fwd = cTf.get_forward_vector()
        vLong = vel.x * fwd.x + vel.y * fwd.y
        held = False
        if self._egoBackstepGuard and self._lastEgoPose is not None:
            lx, ly, lz, lh = self._lastEgoPose
            hRad = lh * math.pi / 180.0
            ahead = ((sTf.location.x - lx) * math.sin(hRad)
                     + (sTf.location.y - ly) * math.cos(hRad))
            behind = math.sqrt((sTf.location.x - lx) ** 2
                               + (sTf.location.y - ly) ** 2)
            # vLong catches the first backward tick; the projection cannot drift.
            held = (vLong < 0.0 or ahead <= 0.0) and behind < self._egoBackstepRelease
        if held:
            out.x, out.y, out.z = lx, ly, lz
            out.heading = lh
            self._egoBackstepHolds += 1
        else:
            out.x = sTf.location.x
            out.y = sTf.location.y
            out.z = sTf.location.z
            out.heading = sTf.rotation.yaw
            self._lastEgoPose = (out.x, out.y, out.z, out.heading)
        out.grade = sTf.rotation.pitch * math.pi / 180.0
        out.speed = math.sqrt(vel.x * vel.x + vel.y * vel.y)
        return True

    def noteWorldTicked(self):
        """CARLA has advanced a tick, so every actor now has a snapshot.

        Called by the host right after world.tick(). It is what ends the window
        in which a just-spawned ego has no pose -- see readEgoState.
        """
        self._egoAwaitingSnapshot = False

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
        """Apply this tick's transform commands as ONE acknowledged batch.

        Sync, not async, and called immediately before ``world.tick()`` with the
        spectator already queued into it (#266/#267). Both halves matter and they
        are one idea: everything this tick renders is applied by one call the
        server has acknowledged.

        An async ``apply_batch`` hands the commands to the socket and returns, so
        the bridge races its own message: when the tick wins, the frame renders
        every mirrored vehicle at its PREVIOUS pose while the camera -- placed
        from the pose just commanded -- has moved on, and the next tick applies
        both so the vehicle jumps twice as far. Measured on the C++ bridge at
        CarlaTimeStep 0.1: 29% of frames lagging, the followed actor alternating
        0.000 / 0.582 m instead of a steady 0.291.

        Sync alone left 4.7%, because the camera still went out as its OWN RPC and
        a separate message can be applied in a different tick from the poses it is
        meant to be centred on. With the camera in this batch it is 0.0%.

        Not only cosmetic: the interested-id readback reports the actor transform
        back to FIXS, so a stale actor sends a pose one step old, and anything
        differencing successive poses for velocity sees 0 then 2x.
        """
        if self._client is not None and self._batch:
            self._client.apply_batch_sync(self._batch, False)
        self._batch = []

    def queueTransform(self, actorId, tf):
        """(int, carla.Transform) -> None -- put ANY actor into this tick's batch.

        Used for the spectator, so the camera lands in the same atomic apply as the
        vehicles it follows rather than in a separate RPC that can miss the tick.
        """
        self._batch.append(carla.command.ApplyTransform(actorId, tf))

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
        # Sampled, not exhaustive. This asks whether the two MAPS agree on
        # elevation -- a STATIC property of the map pair, not of the traffic -- so
        # checking every vehicle every exchange re-answers one question hundreds of
        # times per second. Measured on the MLK corridor it was 9.6 ms of a 42.7 ms
        # tick: 23% of the bridge's own work, for a #193 placeholder that only
        # warns. A rotating slice covers every mapped vehicle within kZAuditStride
        # exchanges -- 0.5 s at the 0.1 s feed, shorter than the shortest violation
        # observed on this corridor -- so nothing that was reported before is missed.
        handles = list(self._lastApplied)
        if not handles:
            return
        self._zAuditPhase = (self._zAuditPhase + 1) % kZAuditStride
        for n in range(self._zAuditPhase, len(handles), kZAuditStride):
            h = handles[n]
            tf = self._lastApplied[h]
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

    def signalHeads(self):
        """[(carla.TrafficLight, carla.Transform)] -- each light, and where its
        STOP BAR actually is.

        The table gives one row per controlled movement in the SUMO frame
        (junction, link, x, y, z, heading); this converts each to the CARLA
        frame with the same arithmetic that places every mirrored vehicle. A
        controller needs it because an agent locates the signal governing it
        from the actor's trigger volume, and on an imported corridor those
        volumes do not line up with the lanes.
        """
        out = []
        for linkMap in self._trafficLightMap.values():
            for tl in linkMap.values():
                actor = tl.carlaTrafficLightActorPtr
                if actor is None:
                    continue
                x, y, z, pitch, yaw, roll = BridgeHelper.sumo_to_carla_numeric(
                    tl.x, tl.y, tl.z, tl.heading, 0.0, 0.0, 0.0)
                out.append((actor, carla.Transform(carla.Location(x, y, z),
                                                   carla.Rotation(pitch, yaw, roll))))
        return out

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
        self._egoAwaitingSnapshot = True             # no pose until CARLA ticks
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

    def applyEgoSpeedSteer(self, speed, steerNorm, accel=None, jerk=None):
        """Apply a SPEED+STEER command: CARLA's own controller closes the loop.

        The second of CARLA's two vehicle interfaces, and the one FIXS has never
        used -- libcarla has exposed ApplyAckermannControlToVehicle all along.
        Where applyEgoActuation realises a command computed elsewhere, this hands
        CARLA a target and lets its Ackermann controller track it at the world
        step rate. A 10 Hz command therefore still gets a 50 Hz loop closed on
        it, with no in-process controller.

        What that buys is not free: the gains are CARLA's
        (AckermannControllerSettings), not ours, and they are not obviously right
        for every vehicle and grade -- EgoDriver's own comments record that
        throttle response on a 1845 kg vehicle on this arterial is sharply
        nonlinear. Measure before trusting it.
        """
        if self._egoActor is None:
            return
        self._egoActor.apply_ackermann_control(carla.VehicleAckermannControl(
            steer=float(max(-1.0, min(1.0, steerNorm))),
            speed=float(max(0.0, speed)),
            acceleration=float(accel if accel is not None else 0.0),
            jerk=float(jerk if jerk is not None else 0.0)))

    def egoActor(self):
        return self._egoActor

    def destroyEgo(self):
        # A count with no standstill in the run means backward physics. FIXS#358.
        if self._egoBackstepHolds:
            print('[carla] backstep guard held the ego pose on %d readbacks '
                  '(backward or non-advancing motion, not reported to FIXS)'
                  % self._egoBackstepHolds, flush=True)
        if self._egoActor is not None:
            self._egoActor.destroy()
            self._egoActor = None
        self._egoAwaitingSnapshot = False
