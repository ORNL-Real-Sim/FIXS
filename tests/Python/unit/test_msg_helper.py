import struct
import pytest
from CommonLib.MsgHelper import MsgHelper, MessageType
from CommonLib.VehDataMsgDefs import VehData


def make_msg_helper_with_fields(*fields):
    """Return a MsgHelper with the given vehicle fields enabled."""
    mh = MsgHelper()
    mh.set_vehicle_message_field(list(fields))
    return mh


class TestMsgHelperInit:
    def test_default_fields_all_false(self):
        mh = MsgHelper()
        for key, val in mh.vehicle_msg_field_valid.items():
            assert val is False, f"Expected {key} to be False by default"

    def test_header_sizes(self):
        mh = MsgHelper()
        assert mh.msg_header_size == 9
        assert mh.msg_each_header_size == 3


class TestSetVehicleMessageField:
    def test_set_fields_marks_valid(self):
        mh = MsgHelper()
        mh.set_vehicle_message_field(['id', 'speed', 'positionX'])
        assert mh.vehicle_msg_field_valid['id'] is True
        assert mh.vehicle_msg_field_valid['speed'] is True
        assert mh.vehicle_msg_field_valid['positionX'] is True

    def test_unset_fields_remain_false(self):
        mh = MsgHelper()
        mh.set_vehicle_message_field(['id', 'speed'])
        assert mh.vehicle_msg_field_valid['positionY'] is False
        assert mh.vehicle_msg_field_valid['heading'] is False


class TestPackUnpackString:
    def test_roundtrip_short_string(self):
        buf = bytearray(64)
        packed, idx = MsgHelper.pack_string(buf, 0, "hello")
        assert idx == 6  # 1 byte length + 5 bytes string
        result, length, new_idx, arr = MsgHelper.depack_string(packed, 0)
        assert result == "hello"
        assert length == 5
        assert new_idx == 6

    def test_roundtrip_empty_string(self):
        buf = bytearray(16)
        packed, idx = MsgHelper.pack_string(buf, 0, "")
        assert idx == 1
        result, length, new_idx, arr = MsgHelper.depack_string(packed, 0)
        assert result == ""
        assert length == 0

    def test_roundtrip_with_offset(self):
        buf = bytearray(64)
        # Put some bytes at start then pack string at offset 4
        MsgHelper.pack_string(buf, 4, "abc")
        result, length, _, _ = MsgHelper.depack_string(buf, 4)
        assert result == "abc"
        assert length == 3


class TestUnpackHelpers:
    def test_unpack_float(self):
        raw = struct.pack('f', 3.14)
        val, idx = MsgHelper.unpack_float(raw, 0)
        assert abs(val - 3.14) < 0.001
        assert idx == 4

    def test_unpack_int32(self):
        raw = struct.pack('i', -42)
        val, idx = MsgHelper.unpack_int32(raw, 0)
        assert val == -42
        assert idx == 4

    def test_unpack_uint32(self):
        raw = struct.pack('I', 999999)
        val, idx = MsgHelper.unpack_uint32(raw, 0)
        assert val == 999999

    def test_unpack_uint16(self):
        raw = struct.pack('H', 1234)
        val, idx = MsgHelper.unpack_uint16(raw, 0)
        assert val == 1234
        assert idx == 2

    def test_unpack_uint8(self):
        raw = bytes([255])
        val, idx = MsgHelper.unpack_uint8(raw, 0)
        assert val == 255
        assert idx == 1

    def test_unpack_int8(self):
        raw = struct.pack('b', -1)
        val, idx = MsgHelper.unpack_int8(raw, 0)
        assert val == -1
        assert idx == 1


class TestPackDepackVehData:
    def test_roundtrip_id_and_speed(self):
        mh = make_msg_helper_with_fields('id', 'speed')
        veh = VehData()
        veh.id = 'car1'
        veh.speed = 15.5

        buf = bytearray(256)
        packed, msg_size, end_idx = mh.pack_veh_data(buf, 0, veh)

        # depack_veh_data skips the 3-byte sub-header
        unpacked = mh.depack_veh_data(bytes(packed[mh.msg_each_header_size:end_idx]))
        assert unpacked.id == 'car1'
        assert abs(unpacked.speed - 15.5) < 0.001

    def test_roundtrip_position_fields(self):
        mh = make_msg_helper_with_fields('positionX', 'positionY', 'positionZ', 'heading')
        veh = VehData()
        veh.positionX = 100.0
        veh.positionY = 200.0
        veh.positionZ = 5.0
        veh.heading = 1.57

        buf = bytearray(256)
        packed, msg_size, end_idx = mh.pack_veh_data(buf, 0, veh)
        unpacked = mh.depack_veh_data(bytes(packed[mh.msg_each_header_size:end_idx]))

        assert abs(unpacked.positionX - 100.0) < 0.01
        assert abs(unpacked.positionY - 200.0) < 0.01
        assert abs(unpacked.positionZ - 5.0) < 0.01
        assert abs(unpacked.heading - 1.57) < 0.01

    def test_roundtrip_multiple_string_fields(self):
        mh = make_msg_helper_with_fields('id', 'type', 'vehicleClass')
        veh = VehData()
        veh.id = 'ego'
        veh.type = 'car'
        veh.vehicleClass = 'passenger'

        buf = bytearray(256)
        packed, msg_size, end_idx = mh.pack_veh_data(buf, 0, veh)
        unpacked = mh.depack_veh_data(bytes(packed[mh.msg_each_header_size:end_idx]))

        assert unpacked.id == 'ego'
        assert unpacked.type == 'car'
        assert unpacked.vehicleClass == 'passenger'

    def test_message_type_byte_is_vehicle_data(self):
        mh = make_msg_helper_with_fields('id')
        veh = VehData()
        veh.id = 'v1'

        buf = bytearray(256)
        packed, _, _ = mh.pack_veh_data(buf, 0, veh)

        # byte 2 of sub-header is the message type
        assert packed[2] == MessageType.vehicle_data

    def test_pack_header(self):
        mh = MsgHelper()
        buf = bytearray(256)
        packed, idx = mh.pack_msg_header(buf, 1, 5.5, 100)
        assert idx == mh.msg_header_size

        sim_state, sim_time, total = mh.depack_msg_header(bytes(buf[:mh.msg_header_size]))
        assert sim_state == 1
        assert abs(sim_time - 5.5) < 0.001
        assert total == 100
