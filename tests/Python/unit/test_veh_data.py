import pytest
from CommonLib.VehDataMsgDefs import VehData, TrafficLightData, DetectorData


class TestVehData:
    def test_instantiation_no_args(self):
        """VehData can be created with no arguments (all fields have defaults)."""
        veh = VehData()
        assert veh is not None

    def test_default_numeric_fields(self):
        """Numeric fields default to zero."""
        veh = VehData()
        assert veh.speed == 0.0
        assert veh.acceleration == 0.0
        assert veh.positionX == 0.0
        assert veh.positionY == 0.0
        assert veh.positionZ == 0.0
        assert veh.heading == 0.0
        assert veh.color == 0
        assert veh.laneId == 0
        assert veh.hasPrecedingVehicle == 0
        assert veh.activeLaneChange == 0

    def test_default_string_fields_exist(self):
        """String fields (id, type, etc.) exist and are strings."""
        veh = VehData()
        assert isinstance(veh.id, str)
        assert isinstance(veh.type, str)
        assert isinstance(veh.vehicleClass, str)
        assert isinstance(veh.linkId, str)
        assert isinstance(veh.precedingVehicleId, str)
        assert isinstance(veh.signalLightId, str)
        assert isinstance(veh.linkIdNext, str)

    def test_expected_fields_present(self):
        """All expected fields are accessible."""
        veh = VehData()
        expected_fields = [
            'id', 'type', 'vehicleClass', 'speed', 'acceleration',
            'positionX', 'positionY', 'positionZ', 'heading', 'color',
            'linkId', 'laneId', 'distanceTravel', 'speedDesired',
            'accelerationDesired', 'hasPrecedingVehicle', 'precedingVehicleId',
            'precedingVehicleDistance', 'precedingVehicleSpeed',
            'signalLightId', 'signalLightHeadId', 'signalLightDistance',
            'signalLightColor', 'speedLimit', 'speedLimitNext',
            'speedLimitChangeDistance', 'linkIdNext', 'grade',
            'length', 'width', 'height', 'activeLaneChange',
        ]
        for f in expected_fields:
            assert hasattr(veh, f), f"VehData missing expected field: {f}"

    def test_field_assignment(self):
        """Fields can be assigned and read back."""
        veh = VehData()
        veh.id = 'ego'
        veh.speed = 27.8
        veh.positionX = 100.0
        veh.positionY = -50.5
        veh.heading = 3.14

        assert veh.id == 'ego'
        assert veh.speed == 27.8
        assert veh.positionX == 100.0
        assert veh.positionY == -50.5
        assert veh.heading == 3.14

    def test_get_method(self):
        """The .get() helper returns field values and defaults for missing keys."""
        veh = VehData()
        veh.speed = 10.0
        assert veh.get('speed') == 10.0
        assert veh.get('nonexistent_field', 99) == 99

    def test_independent_default_strings(self):
        """Two VehData instances do not share the same string default objects."""
        veh1 = VehData()
        veh2 = VehData()
        veh1.id = 'vehicle_a'
        assert veh2.id != 'vehicle_a'


class TestTrafficLightData:
    def test_instantiation_with_args(self):
        """TrafficLightData requires positional args (no defaults)."""
        tl = TrafficLightData(id=1, name='TL_north', state='green')
        assert tl.id == 1
        assert tl.name == 'TL_north'
        assert tl.state == 'green'

    def test_get_method(self):
        tl = TrafficLightData(id=2, name='TL_south', state='red')
        assert tl.get('state') == 'red'
        assert tl.get('missing', 'default') == 'default'


class TestDetectorData:
    def test_instantiation_with_args(self):
        """DetectorData requires positional args (no defaults)."""
        det = DetectorData(id=5, name='det_01', state=1)
        assert det.id == 5
        assert det.name == 'det_01'
        assert det.state == 1
