"""MockVirEnvBackend -- an IVirEnvBackend test double, no simulator required.

Python peer of ``tests/VirEnvCore/MockVirEnvBackend.h``. It (a) hands out
monotonic fake handles from a bounded per-class pool, so spawn/despawn and
capacity exhaustion behave like the CarMaker slot pool, and (b) RECORDS every
verb call as a human-diffable event string in the SAME format the C++ mock emits.

That shared format is the whole point: ``test_core_parity.py`` runs the C++
``replay_core.exe`` and the Python ``replay_core.py`` over the same trace and
diffs the two transcripts. Asserting on these strings is asserting "the core's
decisions in the canonical frame" -- which is the guard the ego-ownership risk in
#325 actually needs, because a bridge that disagrees with TrafficLayer about who
owns the ego disagrees HERE first, in which verbs it calls for which id.

Formatting note: the C++ mock prints doubles with ``std::fixed <<
setprecision(3)``, so ``fx()`` below must produce byte-identical text.
``'%.3f'`` matches it, including the ``-0.000`` a small negative rounds to.
"""

from collections import deque

from CommonLib.VirEnv.IVirEnvBackend import (EgoState, IVirEnvBackend, VehClass,
                                             kNoHandle)

__all__ = ['MockVirEnvBackend']


class MockVirEnvBackend(IVirEnvBackend):
    """A recording double with a bounded per-class handle pool.

    :param nCars: size of the car pool. Defaults match the C++ mock (64/16/0), so
        a scenario that exhausts a pool exhausts it in both languages at the same
        vehicle.
    """

    def __init__(self, nCars=64, nTrucks=16, nBuses=0):
        self._events = []
        self._carPool = deque(self._handleOf(VehClass.Car, i) for i in range(nCars))
        self._truckPool = deque(self._handleOf(VehClass.Truck, i) for i in range(nTrucks))
        self._busPool = deque(self._handleOf(VehClass.Bus, i) for i in range(nBuses))
        self._ego = EgoState()
        self._egoAvailable = True
        self._poolInit = False

    # --- recorded transcript (one line per verb call) ----------------------
    def events(self):
        """() -> list -- the transcript so far, one string per verb call."""
        return self._events

    def clear(self):
        """() -> None -- drop the transcript, keeping the pools as they are."""
        self._events.clear()

    def mark(self, label):
        """(string) -> None -- delimit one step in the transcript.

        Not a verb -- the driver calls it, the core never does. It exists so a
        transcript can be compared PER STEP: the order of two vehicles' verbs
        WITHIN one step is C++ ``unordered_map`` iteration order, which the
        standard does not define and which an insertion-ordered Python dict
        cannot reproduce. Per-step grouping compares the decisions, which are the
        thing both cores must agree on, and does not compare an order that
        carries no meaning.
        """
        self._events.append('== ' + label)

    # --- IVirEnvBackend ----------------------------------------------------
    def log(self, msg):
        self._rec('log', msg)

    def logError(self, msg):
        self._rec('logError', msg)

    def loadSignalTable(self, path):
        self._rec('loadSignalTable', path or '')

    def initTrafficPool(self):
        self._rec('initTrafficPool', '')
        self._poolInit = True

    def spawnVehicle(self, vType, vClass, spawnPose):
        cls = self._classify(vClass)
        pool = self._poolFor(cls)
        tag = '%s(%s/%s)' % (cls.name, vType, vClass)
        if not pool:
            self._rec('spawnVehicle', tag + ' -> kNoHandle')
            return kNoHandle
        h = pool.popleft()
        self._rec('spawnVehicle', '%s -> %d' % (tag, h))
        return h

    def despawnVehicle(self, h):
        self._poolFor(self._classOf(h)).append(h)
        self._rec('despawnVehicle', str(h))

    def setVehiclePose(self, h, p):
        self._rec('setVehiclePose', '%d (%s,%s,%s) hdg=%s grade=%s'
                  % (h, _fx(p.x), _fx(p.y), _fx(p.z), _fx(p.headingDeg), _fx(p.gradeRad)))

    def setVehicleLights(self, h, brake, indL, indR):
        # C++ streams bools through ostream, which prints 1/0 -- not true/false.
        self._rec('setVehicleLights', '%d brake=%d L=%d R=%d'
                  % (h, int(brake), int(indL), int(indR)))

    def syncTrafficLight(self, junctionId, stateStr):
        self._rec('syncTrafficLight', "%s '%s'" % (junctionId, stateStr))

    def readEgoState(self, egoId, out):
        self._rec('readEgoState', egoId)
        # The C++ assigns the whole canned struct through the out-parameter; copy
        # field by field so the caller's object is the one that gets filled.
        for name in ('speed', 'x', 'y', 'z', 'heading', 'grade', 'brake', 'indL', 'indR'):
            setattr(out, name, getattr(self._ego, name))
        return self._egoAvailable

    def setEgoPose(self, egoId, p):
        self._rec('setEgoPose', '%s (%s,%s,%s)' % (egoId, _fx(p.x), _fx(p.y), _fx(p.z)))

    def applyEgoControl(self, egoId, desiredSpeed):
        self._rec('applyEgoControl', '%s v*=%s' % (egoId, _fx(desiredSpeed)))

    # --- test fixtures -----------------------------------------------------
    def setMockEgo(self, ego, available=True):
        """(EgoState, bool) -> None -- what :meth:`readEgoState` will hand back."""
        self._ego = ego
        self._egoAvailable = available

    # --- internals ---------------------------------------------------------
    @staticmethod
    def _classify(vClass):
        if 'truck' in vClass:
            return VehClass.Truck
        if 'bus' in vClass:
            return VehClass.Bus
        return VehClass.Car          # car/passenger/private/default

    # encode (class, index) into a stable, decodable handle so _classOf works.
    @staticmethod
    def _handleOf(c, i):
        return int(c) * 100000 + i

    @staticmethod
    def _classOf(h):
        return VehClass(h // 100000)

    def _poolFor(self, c):
        if c == VehClass.Truck:
            return self._truckPool
        if c == VehClass.Bus:
            return self._busPool
        return self._carPool

    def _rec(self, verb, arg):
        self._events.append(verb if not arg else ('%s %s' % (verb, arg)))


def _fx(v):
    """(float) -> string -- the C++ mock's ``fixed << setprecision(3)`` format."""
    return '%.3f' % v
