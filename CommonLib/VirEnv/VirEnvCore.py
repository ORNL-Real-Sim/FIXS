"""VirEnvCore -- the backend-agnostic VirtualEnvironment orchestration core.

Python peer of ``CommonLib/VirEnvCore.{h,cpp}`` (#174, #325). It owns the
simulator-agnostic logic -- FIXS exchange, id <-> handle mapping, spawn/despawn
bookkeeping, temporal interpolation, refresh rate-gating, signal forwarding, ego
readback/send -- and drives ONE host through the :class:`IVirEnvBackend` verbs.
Imports NEITHER ``carla`` NOR any CarMaker binding, so it loads on a machine with
neither installed; that is what lets ``tests/VirEnv/replay_core.py`` run.

Behaviour is preserved relative to the two C++ paths via the same two policy
knobs (both backends despawn on the first absent step already, so no knob is
needed there):

* ``interpolateTraffic`` -- CarMaker sub-steps at ~1 kHz and interpolates the
  0.1 s FIXS updates (True). Carla ticks 1:1 with FIXS and sets the received pose
  directly (False).
* the per-refresh spare re-park (#168) is delegated to ``backend.parkSpares()``.

Naming
------
Every public symbol is the C++ name verbatim, trailing underscores included, so
one grep finds both implementations and a reviewer can read the two seven-step
bodies side by side. Three shapes necessarily differ, and only three:

1. C++ reports errors through a ``const char** errorMsg`` out-parameter; here
   :meth:`initialization`, :meth:`runStep`, :meth:`processStep` and
   :meth:`sendData` return ``(rc, errorMsg)``.
2. The transport is ``CommonLib/fixs.py`` opened in its ``virenv`` role, not a
   hand-rolled socket. So the core does not address a socket: where the C++
   driver calls ``Sock_c.sendData(serverSock[sock0], ...)``, the Python driver
   calls :meth:`sendData`, which flushes ``Msg_c.VehDataSend_um``. ``Sock_c`` and
   ``Msg_c`` are still the objects of those names -- the ``SocketHelper`` and
   ``MsgHelper`` behind the connection -- so the step body reads identically.
3. ``Msg_c.VehDataSend_um`` is keyed by socket INDEX (0 = vehicle data,
   1 = signals) rather than by socket fd, which is what a caller addresses
   anyway (``mainVirCarla.cpp`` sets ``sock0 = 0`` and indexes with it).
"""

import math
import os
import sys
import time as _time
from dataclasses import replace

from ..MsgHelper import MsgHelper
from .FixsProtocol import kFeedHz, onFeedBoundary
from .IVirEnvBackend import EgoState, Pose, VehHandle, kNoHandle

__all__ = ['VirEnvCore', 'InitErr', 'StepErr', 'lerpHeadingDeg']


class InitErr:
    """:meth:`VirEnvCore.initialization` return codes. Values match VirEnvCore.h."""
    ERROR_INIT_READ_CONFIG = -1
    ERROR_INIT_MSG_FIELD = -2
    ERROR_INIT_SOCKET = -3
    ERROR_INIT_TRAFFIC = -4


class StepErr:
    """:meth:`VirEnvCore.runStep` return codes. Values match VirEnvCore.h."""
    ERROR_STEP_RECV = -1
    ERROR_STEP_MAP = -2
    ERROR_STEP_REMOVE = -3
    ERROR_STEP_UPDATE = -4
    ERROR_STEP_SEND = -5
    ERROR_STEP_REFRESH = -6
    ERROR_STEP_SYNC = -7


def lerpHeadingDeg(prevDeg, nextDeg, f):
    """(float, float, float) -> float -- wrap-aware blend of a FIXS wire heading.

    Peer of the file-static ``lerpHeadingDeg`` in VirEnvCore.cpp, arithmetic
    identical. Headings are degrees, north = 0, CLOCKWISE (the convention
    documented on :class:`~CommonLib.VirEnv.IVirEnvBackend.Pose`), carried verbatim
    from SUMO's ``VAR_ANGLE`` / VISSIM's heading.

    Both ends are normalised into ``[0, 360)``, then the blend follows the
    SHORTEST arc::

        d = fmod(n - p + 540, 360) - 180

    After normalisation ``n - p`` is in ``(-360, 360)``, so the ``+540`` makes the
    dividend non-negative -- required, because ``math.fmod`` keeps the sign of the
    dividend (unlike Python's ``%``) and a negative remainder would pick the long
    way round. ``d`` then lands in ``[-180, 180)``: the signed short way. ``f = 0``
    returns ``p``; ``f = 1`` returns ``n`` (in the seam case via a renormalisation,
    so within ~1e-14 deg rather than bit-exact).
    """
    p = math.fmod(math.fmod(prevDeg, 360.0) + 360.0, 360.0)
    n = math.fmod(math.fmod(nextDeg, 360.0) + 360.0, 360.0)
    d = math.fmod(n - p + 540.0, 360.0) - 180.0
    out = p + d * f
    return math.fmod(math.fmod(out, 360.0) + 360.0, 360.0)


class _Sample:
    """One staged FIXS sample: the time it applies at, the pose, the light bits."""

    __slots__ = ('t', 'pose', 'lightBits')

    def __init__(self, t=0.0, pose=None, lightBits=0):
        self.t = t
        self.pose = pose if pose is not None else Pose()
        self.lightBits = lightBits


class VirEnvCore:
    """The SDK-free heart of the virtual-environment bridge.

    Construct it, :meth:`setBackend`, fill the public config fields below, then
    :meth:`initialization` once and :meth:`runStep` per host tick.
    """

    def __init__(self):
        # --- policy knobs (host sets before initialization) -----------------
        #: Interpolate 0.1 s FIXS updates across sub-steps (CarMaker) vs set the
        #: received pose directly each tick (Carla 1:1).
        self.interpolateTraffic = True
        #: Does the core send the ego back to FIXS (CarMaker mode-A readback)?
        #: Carla sets False -- its driver sends interested-vehicle readback POST
        #: tick, so the core must not also send.
        self.sendEgoFromCore = True
        #: CarMaker receives signals on a SEPARATE FIXS port (True -> a 2nd
        #: connection); Carla receives vehicles AND signals on ONE port (False).
        self.openSignalPort = True

        # --- host-set config (neutral; not tied to a yaml section) ----------
        self.ENABLE_REALSIM = True
        self.ENABLE_SEPARATE_EGO_TRAFFIC = False   # mode B: ego on a 2nd conn
        self.SYNCHRONIZE_TRAFFIC_SIGNAL = True
        self.egoId_ = ''
        self.egoType_ = ''
        self.trafficLayerIP_ = ''
        self.vehDataPort_ = 0
        self.trafficSignalPort_ = 0
        self.trafficRefreshRate_ = 0.001
        self.cmErrorFile_ = 'RealSimVirEnv.err'

        # --- transport (see the module docstring) ---------------------------
        # Msg_c exists from construction, as the C++ value member does, so a
        # replay test can pre-fill Msg_c.VehDataRecv_um and call processStep with
        # no transport at all. initialization() rebinds it to the one behind the
        # live connection. Sock_c stays None until then -- it cannot be built
        # without a config to open.
        self.Sock_c = None
        self.Msg_c = MsgHelper()

        self._backend = None
        self._veryFirstStep = 1

        # --- id <-> backend handle map + interpolation state ----------------
        self._id2handle = {}
        self._prev = {}
        self._next = {}
        #: last raw pose the core set (replaces the C++ ``TrfObj->t_0`` readback)
        self._lastSet = {}
        self._lastRefreshSlot = -1

        self._rsDbg = None
        self._rsDebugEnabled = bool(os.environ.get('RS_DEBUG'))

        #: Seconds the last runStep spent BLOCKED in the FIXS exchange, as opposed
        #: to orchestrating. A host that wants to know whether it is slow or merely
        #: waiting for the rest of the co-simulation reads this; without it the two
        #: are indistinguishable in a tick total and have opposite fixes.
        self.lastRecvSeconds = 0.0

    # ------------------------------------------------------------------ setup
    def setBackend(self, be):
        """(IVirEnvBackend) -> None -- the host owns the backend; we keep a ref."""
        self._backend = be

    def mappedVehicles(self):
        """() -> dict -- the live id -> :data:`VehHandle` map.

        The host driver (Carla) reads this to map its interested ids to backend
        handles for the POST-tick readback. Returned live, not copied, matching
        the C++ ``const&``; treat it as read-only.
        """
        return self._id2handle

    def _logCore(self, msg):
        if self._backend is not None:
            self._backend.log(msg)
        else:
            sys.stdout.write(msg)

    @staticmethod
    def decodeLightBits(lightIndicators):
        """(int) -> (bool, bool, bool) -- the FIXS light bitfield as brake/L/R.

        Verbatim from the old refresh loop: right = bit 0, left = bit 1,
        brake = bit 3. C++ fills three out-parameters and returns 0; returning the
        triple is the Python shape of the same thing.
        """
        indR = bool((lightIndicators >> 0) & 1)
        indL = bool((lightIndicators >> 1) & 1)
        brake = bool((lightIndicators >> 3) & 1)
        return brake, indL, indR

    # ----------------------------------------------------------- lifecycle
    def initialization(self, configPath, signalTablePath):
        """(string, string) -> (int, string) -- validate, load signals, connect.

        SDK-free. The host fills the public config fields above and the message
        field set BEFORE calling this; here we only validate the subscription,
        hand the signal table to the backend, and open the FIXS connection.

        :returns: ``(0, None)``, or ``(InitErr.*, message)``.
        """
        self._veryFirstStep = 0

        # Import here, not at module scope: the core must load with no transport
        # available at all, which is what lets replay_core.py drive processStep on
        # a machine with nothing installed.
        from CommonLib import fixs

        if self.openSignalPort:
            return (InitErr.ERROR_INIT_SOCKET,
                    'RealSim: openSignalPort needs a second FIXS connection, which '
                    'the Python bridge does not open yet. It exists for the CarMaker '
                    'host (a separate TrafficSignalPort); the Carla host sets it '
                    'False and receives vehicles and signals on one port.')

        if self.ENABLE_REALSIM:
            try:
                fixs.connect(configPath, port=self.vehDataPort_, host=self.trafficLayerIP_,
                             role='virenv')
            except Exception as exc:                            # noqa: BLE001
                return (InitErr.ERROR_INIT_SOCKET,
                        'RealSim: Initialize Socket Failed: %s' % exc)
            self.Sock_c, self.Msg_c = fixs.transport()
        else:
            # Co-simulation off: no socket, but the wire format still has to be
            # known -- the backend is driven from records some other source fills
            # into Msg_c, and a MsgHelper with no fields set decodes nothing. The
            # C++ host fills Msg_c from the config itself; here connect() normally
            # does it, so with no connect we read the same key ourselves.
            from CommonLib.ConfigHelper import ConfigHelper
            cfg = ConfigHelper()
            cfg.getConfig(configPath)
            self.Msg_c.set_vehicle_message_field(
                cfg.simulation_setup.get('VehicleMessageField') or ['id', 'speed'])

        # required subscription fields (same contract as before). Checked AFTER the
        # field set is known; the C++ host fills Msg_c itself and so checks first.
        need = ('vehicleClass', 'heading', 'grade')
        have = self.Msg_c.VehicleMessageField_set
        if any(f not in have for f in need):
            if self.ENABLE_REALSIM:
                fixs.close()
            return (InitErr.ERROR_INIT_MSG_FIELD,
                    'RealSim: Must subscribe: id, speed, vehicleClass, heading, grade, '
                    'speedDesired/accelerationDesired')

        if self.SYNCHRONIZE_TRAFFIC_SIGNAL and self._backend is not None:
            self._backend.loadSignalTable(signalTablePath)

        return (0, None)

    def shutdown(self):
        """() -> None -- release the connection and forget every mapped vehicle."""
        self._logCore('RealSim shutdown \n')
        self._veryFirstStep = 1
        self._id2handle.clear()
        self._prev.clear()
        self._next.clear()
        self._lastSet.clear()
        self._lastRefreshSlot = -1
        if self._rsDbg is not None:
            self._rsDbg.close()
            self._rsDbg = None
        if self.Sock_c is not None:
            from CommonLib import fixs
            fixs.close()
            self.Sock_c = None
            self.Msg_c = None

    # ---------------------------------------------------------------- stepping
    def runStep(self, simTime):
        """(float) -> (int, string) -- receive if due, then orchestrate the step.

        The recv happens only on an exchange boundary with ``t > 0``.
        ``kFeedPeriodS`` is the protocol's exchange period, not a knob: the host
        may tick as fine as it likes, but it trades messages with TrafficLayer only
        here, and TrafficLayer steps the traffic simulator once per exchange.
        """
        from CommonLib import fixs

        simStateRecv = 0
        simTimeRecv = 0.0

        onUpdate = (simTime > 1e-5 and onFeedBoundary(simTime, 1e-5))
        self.lastRecvSeconds = 0.0
        if onUpdate and self.ENABLE_REALSIM:
            _t0 = _time.monotonic()
            try:
                fixs.recv()
            except fixs.Shutdown:
                self.lastRecvSeconds = _time.monotonic() - _t0
                return (StepErr.ERROR_STEP_RECV,
                        'RealSim: traffic simulator ended the run')
            except Exception as exc:                            # noqa: BLE001
                self.lastRecvSeconds = _time.monotonic() - _t0
                return (StepErr.ERROR_STEP_RECV,
                        'RealSim: Receive from traffic simulator failed: %s' % exc)
            simStateRecv, simTimeRecv = fixs.sim.state, fixs.sim.time
            self.lastRecvSeconds = _time.monotonic() - _t0
        elif onUpdate:
            # No transport: the caller pre-filled Msg_c.VehDataRecv_um itself.
            simTimeRecv = simTime

        return self.processStep(simTime, onUpdate, simStateRecv, simTimeRecv)

    def processStep(self, simTime, onUpdate, simStateRecv, simTimeRecv):
        """(float, bool, int, float) -> (int, string) -- the seven-step skeleton.

        Map / despawn / stage / signals / send / refresh, using whatever is already
        in ``Msg_c.VehDataRecv_um``. Split out from :meth:`runStep` exactly as in
        C++ so a replay test can drive it with no sockets, no TrafficLayer and no
        simulator by pre-filling that dict.
        """
        backend = self._backend

        # ---- step 0: first-step pool init (backend enumerates/prepares slots)
        if simTime < 0.05:
            if backend is not None:
                backend.initTrafficPool()

        recv = self.Msg_c.VehDataRecv_um

        if onUpdate:
            # ---- step 1: map received ids to backend handles (spawn new) ---
            for idTs, rec in recv.items():
                if idTs == self.egoId_:
                    continue                        # never spawn the ego
                if idTs in self._id2handle:
                    continue                        # already mapped
                sp = Pose(x=rec.positionX, y=rec.positionY, z=rec.positionZ,
                          headingDeg=rec.heading, gradeRad=rec.grade)
                h = backend.spawnVehicle(rec.type, rec.vehicleClass, sp) \
                    if backend is not None else kNoHandle
                if h == kNoHandle:
                    continue                        # backend full -> skip
                self._id2handle[idTs] = h

            # ---- step 2: despawn vehicles that disappeared this step -------
            for idTs in [k for k in self._id2handle if k not in recv]:
                if backend is not None:
                    backend.despawnVehicle(self._id2handle[idTs])
                self._prev.pop(idTs, None)
                self._next.pop(idTs, None)
                self._lastSet.pop(idTs, None)
                del self._id2handle[idTs]

            # ---- step 3: stage the k+1 target (raw FIXS); prev = the k pose -
            # Interpolate between CONSECUTIVE RECEIVED samples (k -> k+1) -- the
            # causally-correct co-sim motion: the received traffic IS the next-step
            # (k+1) state and the previous received is the current (k) state, so the
            # interval [simTime, simTimeNext] carries the vehicle exactly along the
            # k->k+1 segment. Using the previous RECEIVED pose (not the last DRAWN
            # pose) makes this exact at ANY sub-step rate; the old last-drawn anchor
            # lagged ~half a step at coarse Carla sub-steps (they never reach f=1
            # before the boundary re-stages).
            simTimeNext = math.ceil(simTime * kFeedHz + 0.001) / kFeedHz
            for idTs in self._id2handle:
                v = recv[idTs]

                # capture the previous target (the k pose) BEFORE overwriting _next
                pit = self._next.get(idTs)
                hasPrev = pit is not None
                kPose = pit.pose if hasPrev else Pose()

                nextS = _Sample(
                    t=simTimeNext,
                    pose=Pose(x=v.positionX,        # raw FIXS: front-of-vehicle, ground
                              y=v.positionY,
                              z=v.positionZ,
                              headingDeg=v.heading,  # raw wire conventions; backend converts
                              gradeRad=v.grade),
                    lightBits=v.lightIndicators)

                prevS = _Sample(
                    t=simTime,
                    # first sight: no interp yet
                    pose=kPose if hasPrev else replace(nextS.pose),
                    lightBits=nextS.lightBits)

                self._prev[idTs] = prevS
                self._next[idTs] = nextS

            # ---- step 4: traffic-signal sync (per junction; backend maps) --
            if self.SYNCHRONIZE_TRAFFIC_SIGNAL and backend is not None:
                for tls in self.Msg_c.TlsDataRecv_um.values():
                    backend.syncTrafficLight(tls.name, tls.state)

        # ---- step 5: send ego back (mode A readback) on the update boundary
        if onUpdate and self.sendEgoFromCore:
            rc, err = self._sendEgoReadback(simTime, simStateRecv, simTimeRecv)
            if rc < 0:
                return (rc, err)

        # ---- step 6: refresh -- park spares + (interp or direct) set poses -
        refreshRate = int(1.0 / self.trafficRefreshRate_) if self.trafficRefreshRate_ > 0.0 else 1
        refreshSlot = _llround(simTime * refreshRate)
        if refreshSlot != self._lastRefreshSlot:
            self._lastRefreshSlot = refreshSlot
            if backend is not None:
                backend.parkSpares()

            logLights = 'lightIndicators' in self.Msg_c.VehicleMessageField_set

            for idTs, h in self._id2handle.items():
                nS = self._next.get(idTs)
                if nS is None:
                    continue
                pS = self._prev[idTs]

                out = Pose()
                if self.interpolateTraffic and nS.t > pS.t:
                    f = (simTime - pS.t) / (nS.t - pS.t)
                    out.x = pS.pose.x + (nS.pose.x - pS.pose.x) * f
                    out.y = pS.pose.y + (nS.pose.y - pS.pose.y) * f
                    out.z = pS.pose.z + (nS.pose.z - pS.pose.z) * f
                    # Heading has to be interpolated too, on the SHORTEST ARC.
                    # Taking only the latest sample leaves a sub-stepping host
                    # sliding its traffic smoothly while the yaw snaps once per
                    # exchange -- a car that translates along a curve but rotates
                    # in 0.1 s steps. A plain lerp would be worse in one specific
                    # place: headingDeg is degrees, north = 0, CLOCKWISE, so a
                    # vehicle heading due north oscillates across the 360/0 seam
                    # (359.8 -> 0.2) and a plain lerp would sweep the long way
                    # round -- a full spin in one feed interval. lerpHeadingDeg
                    # does the wrap-aware blend. A host that ticks 1:1 with the
                    # feed never gets here (the else branch applies the raw
                    # sample), so the 1:1 path is untouched by this.
                    out.headingDeg = lerpHeadingDeg(pS.pose.headingDeg, nS.pose.headingDeg, f)
                    # Grade is a slope in radians (no wrap): plain lerp.
                    out.gradeRad = pS.pose.gradeRad + (nS.pose.gradeRad - pS.pose.gradeRad) * f
                else:
                    out.x = nS.pose.x                  # 1:1 direct
                    out.y = nS.pose.y
                    out.z = nS.pose.z
                    out.headingDeg = nS.pose.headingDeg
                    out.gradeRad = nS.pose.gradeRad

                if backend is not None:
                    backend.setVehiclePose(h, out)
                self._lastSet[idTs] = out
                if self._rsDebugEnabled:
                    self._rsDebugRow(simTime, idTs, h, nS.pose, out, nS.lightBits)

                if logLights:
                    brake, indL, indR = self.decodeLightBits(nS.lightBits)
                    if backend is not None:
                        backend.setVehicleLights(h, brake, indL, indR)

        return (0, None)

    # ----------------------------------------------------------------- sending
    def _sendEgoReadback(self, simTime, simStateRecv, simTimeRecv):
        """Step 5: read the backend's ego and put it on the wire (mode A)."""
        from CommonLib.VehDataMsgDefs import VehData

        haveEgo = False
        VehDataSend = VehData()
        if (not self.ENABLE_SEPARATE_EGO_TRAFFIC or simTime < 1e-5) and self._backend is not None:
            ego = EgoState()
            if self._backend.readEgoState(self.egoId_, ego):
                haveEgo = True
                VehDataSend.id = self.egoId_
                VehDataSend.type = self.egoType_
                VehDataSend.speed = ego.speed
                VehDataSend.heading = ego.heading
                VehDataSend.positionX = ego.x
                VehDataSend.positionY = ego.y
                VehDataSend.positionZ = ego.z
                VehDataSend.acceleration = 0.0
                VehDataSend.color = 0
                VehDataSend.linkId = 'None'
                VehDataSend.laneId = 0
                VehDataSend.distanceTravel = 0.0
                VehDataSend.speedDesired = ego.speed
                VehDataSend.accelerationDesired = 0.0
                VehDataSend.lightIndicators = int(ego.brake) * 8 + int(ego.indL) * 2 + int(ego.indR)

        if haveEgo:
            self.Msg_c.VehDataSend_um.setdefault(0, []).append(VehDataSend)
        return self.sendData(simTime)

    def sendData(self, simTime):
        """(float) -> (int, string) -- answer this tick, flushing VehDataSend_um.

        The one shape the C++ does not have. There, the driver addresses the socket
        itself (``Sock_c.sendData(serverSock[sock0], ...)``); here the connection
        belongs to ``fixs``, so the core flushes and answers. Every record staged in
        ``Msg_c.VehDataSend_um[0]`` goes out and the staging list is cleared, so a
        tick can never ship the previous tick's records.

        A tick with nothing to say still sends: a header and no records. That is
        what tells TrafficLayer this client is done and the exchange may advance.
        """
        if not self.ENABLE_REALSIM or self.Sock_c is None:
            self.Msg_c.clearSendStorage()
            return (0, None)

        from CommonLib import fixs
        try:
            for record in self.Msg_c.VehDataSend_um.get(0, ()):
                fixs.emit(record)
            self.Msg_c.clearSendStorage()
            fixs.send()
        except Exception as exc:                                # noqa: BLE001
            return (StepErr.ERROR_STEP_SEND, 'RealSim: Send failed: %s' % exc)
        return (0, None)

    # ------------------------------------------------------------- diagnostics
    def _rsDebugRow(self, simTime, idTs, h, fixsPose, applied, lightBits):
        """One unified per-vehicle diagnostic row, identical columns per backend.

        Enabled by the ``RS_DEBUG`` environment variable -- the same switch name the
        C++ uses as a compile flag, so a run is turned on the same way.

        * ``fixs_*`` -- the FIXS-canonical pose received this step (front-of-vehicle,
          the wire conventions: the simulator-neutral ground truth).
        * ``set_*`` -- the pose the core handed to the backend (== ``fixs_*`` for a
          1:1 host; interpolated for a sub-stepping one).

        Backend-specific readings are appended via ``debugFields()`` so it all stays
        in ONE csv.
        """
        if self._rsDbg is None:
            for path in (os.path.join('RealSim_tmp', 'rs_core_pos.csv'), 'rs_core_pos.csv'):
                try:
                    self._rsDbg = open(path, 'w', encoding='utf-8')
                    break
                except OSError:
                    continue
            if self._rsDbg is None:
                self._rsDebugEnabled = False          # stop retrying every vehicle
                return
            extra = self._backend.debugHeader() if self._backend is not None else ''
            self._rsDbg.write(
                'simTime,id,handle,fixs_x,fixs_y,fixs_z,fixs_heading,fixs_grade,'
                'set_x,set_y,set_z,set_heading,lightBits%s\n' % extra)

        extra = self._backend.debugFields(h) if self._backend is not None else ''
        self._rsDbg.write(
            '%.3f,%s,%d,%.4f,%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.4f,%d%s\n'
            % (simTime, idTs, h, fixsPose.x, fixsPose.y, fixsPose.z,
               fixsPose.headingDeg, fixsPose.gradeRad,
               applied.x, applied.y, applied.z, applied.headingDeg, lightBits, extra))
        self._rsDbg.flush()


def _llround(v):
    """C++ ``std::llround`` for the non-negative clock the refresh gate uses.

    Python's ``round()`` is banker's rounding -- ``round(0.5) == 0`` -- so the
    refresh slot would land one step late on every exact half. Sim time never runs
    backwards here, so the C++ round-half-away-from-zero reduces to this.
    """
    return int(v + 0.5)
