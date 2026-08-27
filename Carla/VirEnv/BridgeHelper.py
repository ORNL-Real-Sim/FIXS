"""BridgeHelper -- the SUMO <-> CARLA frame, blueprint and signal mappings.

Python peer of ``VirCarlaEnv/VirCarlaEnv/BridgeHelper.{h,cpp}``. Same class name,
same method names (``map_transfrom_Sumo_to_Carla`` keeps its long-standing
spelling so the two are greppable together), same arithmetic in the same order.

Why this is a fresh port rather than a reuse of CARLA's own
``Carla/sumo/run_synchronization/sumo_integration/bridge_helper.py``: the C++
BridgeHelper has diverged from it (a different extent convention, the FIXS
traffic-light table, the vClass -> blueprint sets), and the point of this module
is to reproduce THE C++ ONE. A bridge that produced CARLA's original transforms
would be a second answer to a question that already has one.

Frames, stated once
-------------------
The FIXS wire (== the SUMO/VISSIM convention) carries the FRONT-of-vehicle
position at ground height, with heading in navigational degrees (north = 0,
clockwise). CARLA is left-handed with a Y flip relative to SUMO, and an actor's
transform is its PIVOT, which sits at the bounding-box centre horizontally. So
placing a model's FRONT on the SUMO front means stepping BACK by the half-length,
which is ``extent.x`` -- that is the ``cos/sin * extent.x`` term below, and it is
why the caller must pass the actor's REAL ``bounding_box.extent`` and not a
constant.
"""

import csv
import math
import os
import random

import carla

__all__ = ['SumoActor', 'SumoTrafficLightState', 'TrafficLight', 'BridgeHelper']


class SumoActor:
    """One traffic-simulator vehicle and its CARLA counterpart.

    Peer of the C++ ``struct SumoActor``. Carried for structural parity; the
    bridge itself keys on handles, so this is what a backend or a probe uses when
    it wants the pair in one object.
    """

    def __init__(self, id='', vType='', vClass='', sumoTransform=None, extent=None):
        self.id = id
        self.vType = vType
        self.vClass = vClass
        self.sumoTransform = sumoTransform
        self.carlaTransform = None
        #: half-size of the CARLA bounding box
        self.extent = extent
        self.spawnedInCarla = False
        self.carlaVehicleActorPtr = None


class SumoTrafficLightState:
    """SUMO's per-link signal characters, as the C++ ``enum class`` of that name.

    Plain characters rather than an Enum: they ARE the wire representation (one
    char per controlled link in the state string), and every mapping below is
    keyed on the character.
    """
    RED = 'r'
    YELLOW = 'y'
    GREEN = 'G'
    GREEN_WITHOUT_PRIORITY = 'g'
    GREEN_RIGHT_TURN = 's'
    RED_YELLOW = 'u'
    OFF_BLINKING = 'o'
    OFF = 'O'

    _ALL = (RED, YELLOW, GREEN, GREEN_WITHOUT_PRIORITY,
            GREEN_RIGHT_TURN, RED_YELLOW, OFF_BLINKING, OFF)


class TrafficLight:
    """One signal head from the FIXS traffic-light table, plus its CARLA actor."""

    def __init__(self, junctionId='', linkId=-1, x=0.0, y=0.0, z=0.0, heading=0.0):
        self.junctionId = junctionId
        self.linkId = linkId
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading
        self.state = SumoTrafficLightState.OFF
        self.carlaTrafficLightActorId = ''
        self.carlaTrafficLightActorPtr = None


class BridgeHelper:
    """Static frame / blueprint / signal mappings, mirroring the C++ class."""

    #: Global offset between the SUMO and CARLA coordinate systems. Town01 was
    #: (0.06, 328.61); Town04 (503.02, 423.76). Zero for a map imported from the
    #: scenario's own OpenDRIVE, which is every FIXS map -- the two derive from one
    #: geometry, so there is nothing to offset.
    offset = carla.Location(0.0, 0.0, 0.0)

    # ------------------------------------------------------------ transforms
    @staticmethod
    def map_transfrom_Sumo_to_Carla(in_sumo_transform, extent):
        """(carla.Transform, carla.Vector3D) -> carla.Transform.

        ``extent`` must be the actor's real ``bounding_box.extent``; see the
        module docstring for why a constant is wrong here.
        """
        in_location = in_sumo_transform.location
        in_rotation = in_sumo_transform.rotation

        yaw = -1.0 * in_rotation.yaw + 90.0
        pitch = in_rotation.pitch

        x = in_location.x - math.cos(yaw * math.pi / 180.0) * extent.x
        y = in_location.y - math.sin(yaw * math.pi / 180.0) * extent.x
        z = in_location.z - math.sin(pitch * math.pi / 180.0) * extent.x

        x -= BridgeHelper.offset.x
        y -= BridgeHelper.offset.y

        return carla.Transform(
            carla.Location(x, -y, z),
            carla.Rotation(in_rotation.pitch, in_rotation.yaw - 90.0, in_rotation.roll))

    @staticmethod
    def map_transfrom_Carla_to_Sumo(in_carla_transform, extent):
        """(carla.Transform, carla.Vector3D) -> carla.Transform -- the reverse."""
        in_location = in_carla_transform.location
        in_rotation = in_carla_transform.rotation

        yaw = -1.0 * in_rotation.yaw
        pitch = in_rotation.pitch

        x = in_location.x + math.cos(yaw * math.pi / 180.0) * extent.x
        y = in_location.y - math.sin(yaw * math.pi / 180.0) * extent.x
        z = in_location.z - math.sin(pitch * math.pi / 180.0) * extent.x

        x += BridgeHelper.offset.x
        y -= BridgeHelper.offset.y

        return carla.Transform(
            carla.Location(x, -y, z),
            carla.Rotation(in_rotation.pitch, in_rotation.yaw + 90.0, in_rotation.roll))

    @staticmethod
    def map_location_Carla_to_Sumo(in_carla_location):
        """(carla.Location) -> carla.Location -- position only, no anchor step."""
        x = in_carla_location.x + BridgeHelper.offset.x
        y = in_carla_location.y - BridgeHelper.offset.y
        return carla.Location(x, -y, in_carla_location.z)

    # ------------------------------------------------------------ blueprints
    # The C++ keeps these in unordered_set and picks with random_select_from_set,
    # which is a uniform draw over the set. Sorted tuples here so the CANDIDATE
    # LIST is deterministic across runs and interpreters; the DRAW still uses the
    # module RNG, exactly as the C++ does, so seeding `random` makes a run's
    # blueprint choices reproducible -- which the C++ (a static random_device) does
    # not offer and which a visual A/B comparison wants.
    _CARS = (
        'vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.audi.tt',
        'vehicle.bmw.grandtourer', 'vehicle.chevrolet.impala', 'vehicle.citroen.c3',
        'vehicle.dodge.charger_2020', 'vehicle.ford.crown', 'vehicle.ford.mustang',
        'vehicle.jeep.wrangler_rubicon', 'vehicle.lincoln.mkz_2017',
        'vehicle.lincoln.mkz_2020', 'vehicle.mercedes.coupe',
        'vehicle.mercedes.coupe_2020', 'vehicle.micro.microlino',
        'vehicle.mini.cooper_s', 'vehicle.mini.cooper_s_2021', 'vehicle.nissan.micra',
        'vehicle.nissan.patrol', 'vehicle.nissan.patrol_2021', 'vehicle.seat.leon',
        'vehicle.tesla.model3', 'vehicle.toyota.prius',
    )
    _TRUCKS = (
        'vehicle.carlamotors.carlacola', 'vehicle.carlamotors.european_hgv',
        'vehicle.tesla.cybertruck',
    )
    _VANS = (
        'vehicle.mercedes.sprinter', 'vehicle.volkswagen.t2',
        'vehicle.volkswagen.t2_2021',
    )
    _BUSES = ('vehicle.mitsubishi.fusorosa',)
    _MOTORCYCLES = (
        'vehicle.harley-davidson.low_rider', 'vehicle.kawasaki.ninja',
        'vehicle.vespa.zx125', 'vehicle.yamaha.yzf',
    )
    _BICYCLES = (
        'vehicle.bh.crossbike', 'vehicle.diamondback.century',
        'vehicle.gazelle.omafiets',
    )
    _PEDESTRIANS = tuple('walker.pedestrian.%04d' % i for i in range(1, 6))
    #: CARLA does not separate emergency vehicles by type, so this one set spans
    #: van / truck / car -- matching the C++ comment and set.
    _EMERGENCY = (
        'vehicle.ford.ambulance', 'vehicle.carlamotors.firetruck',
        'vehicle.dodge.charger_police', 'vehicle.dodge.charger_police_2020',
    )

    _BY_VCLASS = {
        'passenger': _CARS, 'truck': _TRUCKS, 'van': _VANS, 'bus': _BUSES,
        'motorcycle': _MOTORCYCLES, 'bicycle': _BICYCLES,
        'pedestrian': _PEDESTRIANS, 'emergency': _EMERGENCY,
    }

    #: Reported once per unknown vClass rather than per vehicle -- the C++ prints
    #: on every spawn, which on a corridor with one unmapped class buries the log.
    _warnedVClasses = set()

    @staticmethod
    def map_Sumo_vClass_to_Carla_blueprintId(vClass):
        """(string) -> string -- a blueprint id for one SUMO vehicle class."""
        pool = BridgeHelper._BY_VCLASS.get(vClass)
        if pool is None:
            if vClass not in BridgeHelper._warnedVClasses:
                BridgeHelper._warnedVClasses.add(vClass)
                print('Unknown vClass: %s\n'
                      'Currently supported vClasses are:\n'
                      '%s.\n'
                      'Defaulting to vehicle.tesla.model3.'
                      % (vClass, ', '.join(sorted(BridgeHelper._BY_VCLASS))))
            return 'vehicle.tesla.model3'          # default to a passenger car
        return random.choice(pool)

    # --------------------------------------------------------- signal states
    @staticmethod
    def get_Sumo_traffic_light_state_from_char(c):
        """(char) -> char -- validate one SUMO state character.

        The C++ maps a char to an enum and throws on anything else; here the enum
        IS the char, so this is the validation half of that. Kept as a named
        function because the throw is the point: an unrecognised character means
        the state string is not what this bridge thinks it is.
        """
        if c not in SumoTrafficLightState._ALL:
            raise ValueError('Unknown SumoTrafficLightState char: %s' % c)
        return c

    @staticmethod
    def Sumo_traffic_light_state_to_char(state):
        """(char) -> char -- identity, kept for call-site parity with the C++."""
        return state

    @staticmethod
    def map_Carla_traffic_light_state_to_Sumo(carlaTrafficLightState):
        """(carla.TrafficLightState) -> char."""
        if carlaTrafficLightState == carla.TrafficLightState.Red:
            return SumoTrafficLightState.RED
        if carlaTrafficLightState == carla.TrafficLightState.Yellow:
            return SumoTrafficLightState.YELLOW
        if carlaTrafficLightState == carla.TrafficLightState.Green:
            return SumoTrafficLightState.GREEN
        return SumoTrafficLightState.OFF          # Off and Unknown both land here

    @staticmethod
    def map_Sumo_traffic_light_state_to_Carla(sumoTrafficLightState):
        """(char) -> carla.TrafficLightState.

        RED_YELLOW ('u', the pre-green phase) maps to Red: a driver may not go.
        GREEN_RIGHT_TURN ('s') and OFF_BLINKING ('o') have no CARLA equivalent and
        map to Unknown, as in the C++.
        """
        if sumoTrafficLightState in (SumoTrafficLightState.RED,
                                     SumoTrafficLightState.RED_YELLOW):
            return carla.TrafficLightState.Red
        if sumoTrafficLightState == SumoTrafficLightState.YELLOW:
            return carla.TrafficLightState.Yellow
        if sumoTrafficLightState in (SumoTrafficLightState.GREEN,
                                     SumoTrafficLightState.GREEN_WITHOUT_PRIORITY):
            return carla.TrafficLightState.Green
        if sumoTrafficLightState == SumoTrafficLightState.OFF:
            return carla.TrafficLightState.Off
        return carla.TrafficLightState.Unknown

    # ---------------------------------------------------- traffic-light table
    @staticmethod
    def readTrafficLightTable(filename):
        """(string) -> dict -- ``{junctionId: {linkId: TrafficLight}}``.

        The FIXS traffic-light table: ``junctionId, linkId, x, y, z, heading``,
        one row per controlled link, with a header line. Its positions are in the
        SUMO frame; :meth:`find_closest_trafficLight_id` matches CARLA's own
        ``traffic.traffic_light`` actors to them by proximity.
        """
        trafficLightMap = {}
        if not filename or not os.path.isfile(filename):
            print('Failed to open file: %s' % filename)
            return trafficLightMap

        with open(filename, newline='', encoding='utf-8-sig') as f:
            reader = csv.reader(f)
            next(reader, None)                      # skip header
            for row in reader:
                if len(row) < 6:
                    continue                        # blank or short line
                junctionId = row[0]
                linkId = int(row[1])
                tl = TrafficLight(junctionId, linkId,
                                  float(row[2]), float(row[3]), float(row[4]),
                                  float(row[5]))
                trafficLightMap.setdefault(junctionId, {})[linkId] = tl
        return trafficLightMap

    @staticmethod
    def find_closest_trafficLight_id(trafficLightMap, x, y):
        """(dict, float, float) -> (junctionId, linkId), or ``('', -1)`` if empty.

        Planar nearest neighbour over every head in the table, comparing squared
        distance as the C++ does. Elevation is deliberately not part of it: a
        CARLA light actor sits at head height while the table row is at the stop
        line, so z would push every match toward whichever junction happened to be
        lowest.
        """
        minDist = float('inf')
        closest = ('', -1)
        for junctionId, linkMap in trafficLightMap.items():
            for linkId, tl in linkMap.items():
                dx = tl.x - x
                dy = tl.y - y
                distSq = dx * dx + dy * dy
                if distSq < minDist:
                    minDist = distSq
                    closest = (junctionId, linkId)
        return closest
