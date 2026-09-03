from struct import unpack, pack
from CommonLib.VehDataMsgDefs import VehData
from CommonLib.MsgHelper import MsgHelper, MessageType
from CommonLib.ConfigHelper import ConfigHelper
import typing

class SocketHelper:

    MSG_CODER = 'utf-8'

    def __init__(self, config_helper: ConfigHelper, msg_helper: MsgHelper):
        # empty constructer
        aa = 1
        self.config_helper = config_helper
        self.msg_helper = msg_helper
        self.msg_header_size = self.msg_helper.msg_header_size
        self.msg_each_header_size = self.msg_helper.msg_each_header_size

        # Get verbose logging setting from config
        self.enable_verbose_log = config_helper.simulation_setup.get('EnableVerboseLog', False)

        # initialize lists to store sending data
        self.vehicle_data_send_list: typing.List[VehData] = []
        self.traffic_light_data_send_list = []
        self.detector_data_send_list = []


        # initialize lists to store received data
        self.vehicle_data_receive_list: typing.List[VehData] = []
        self.traffic_light_data_receive_list = []
        self.detector_data_receive_list = []

    def clear_data(self):
        self.vehicle_data_send_list.clear()
        self.traffic_light_data_send_list.clear()
        self.detector_data_send_list.clear()

        self.vehicle_data_receive_list.clear()
        self.traffic_light_data_receive_list.clear()
        self.detector_data_receive_list.clear()

        # The id-keyed views of the same records (MsgHelper.VehDataRecv_um etc.)
        # are cleared with them, so a tick never mixes two exchanges.
        self.msg_helper.clearRecvStorage()
        self.msg_helper.clearSendStorage()



    def pack_traffic_light_data(self, TrafficLightData):
        # need to skip the size part and add after all have been written
        buffer = b''

        # identifier
        buffer = buffer + pack('<B', 2)

        # message itself
        idLen = len(TrafficLightData['id'])
        buffer = buffer + pack('<B', idLen)
        buffer = buffer + pack('<'+str(idLen)+'s', bytes(TrafficLightData['id'], self.MSG_CODER))

        # sig id
        buffer = buffer + pack('<H', 0)

        idLen = len(TrafficLightData['state'])
        buffer = buffer + pack('<B', idLen)
        buffer = buffer + pack('<'+str(idLen)+'s', bytes(TrafficLightData['state'], self.MSG_CODER))

        buffer = pack('<H', len(buffer)+2) + buffer

        return buffer

    def depack_detector_data(self,buffer):
        nByte = len(buffer)
        sigLen = buffer[0]
        sigName = unpack('<{}s'.format(sigLen), buffer[1:(1+sigLen)])[0]
        sigId = unpack('<H', buffer[(1+sigLen):(1+sigLen+2)])[0]

        iByte = (1+sigLen+2)
        DetDataRecv_v = []
        while iByte<nByte:
            detIdLen = unpack('<B', buffer[iByte:iByte+1])[0]
            iByte = iByte + 1
            detId = unpack('<{}s'.format(detIdLen), buffer[iByte:iByte+detIdLen])[0]
            iByte = iByte + detIdLen
            
            iByte = iByte + 1
            detState = unpack('<B', buffer[iByte:iByte+1])[0]
            iByte = iByte + 1

            DetData_d = {'id': detId, 'state': detState}

            DetDataRecv_v.append(DetData_d)

        return DetDataRecv_v

    # #87: must match MAX_RECORD_SIZE in CommonLib/SocketHelper.h. The two ends enforce
    # the same record ceiling so a record one accepts is never one the other refuses.
    MAX_RECORD_SIZE = 8192

    @staticmethod
    def _recv_exact(sock, n):
        """Read exactly n bytes, looping until they all arrive (#87).

        socket.recv(n) returns UP TO n bytes -- whatever has arrived so far. A
        single call is only reliable while the whole message fits in one TCP
        segment, which is why this went unnoticed with small vehicle counts.
        Under 'all' subscription a message spans many segments and a partial
        read desyncs the stream permanently: the leftover body bytes are then
        parsed as the next record header.

        Mirrors SocketHelper.cpp::recvExact so both ends frame identically.
        """
        if n == 0:
            return b''
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError(
                    f'peer closed mid-message: got {len(buf)} of {n} bytes')
            buf += chunk
        return bytes(buf)

    def recv_data(self, sock):
        # initialize return lists


        # get header for entire message
        received_buffer = self._recv_exact(sock, self.msg_header_size)


        sim_state, sim_time, total_msg_size = self.msg_helper.depack_msg_header(received_buffer)
        msg_processed_size = 0
        msg_processed_size = msg_processed_size + self.msg_header_size
        # total message size is the data to be received
        # save received_buffer to local log for debugging (only if verbose logging enabled)
        if self.enable_verbose_log:
            log_file_path = "received_header_buffer.log"
            with open(log_file_path, 'a', encoding='utf-8') as log_file:  # 'a' for appending text
                log_file.write(f"[HEADER] State: {sim_state}, Time: {sim_time:.2f}, TotalSize: {total_msg_size} | Hex: {received_buffer.hex()}\n")
        
        # ONE read for the whole body, then parse it in memory.
        #
        # The loop below used to take the record header and the record body from
        # the socket separately, so a 190-vehicle exchange cost ~410 recv() calls
        # where the message header had already said exactly how many bytes were
        # coming. Measured on the MLK corridor: 615,220 recv() calls over 1501
        # exchanges, 410 per exchange, all but the first returning immediately
        # from the socket buffer. Every FIXS client pays this -- the bridge and
        # the controller both -- so it is a cost on the whole exchange, not on
        # one component.
        #
        # The per-record size checks below still apply, and matter more now: they
        # validate a size read out of THIS buffer rather than one about to be
        # trusted to size a blocking read.
        body = self._recv_exact(sock, max(0, total_msg_size - self.msg_header_size))
        body_at = 0
        while (msg_processed_size < total_msg_size):
            # get message type header
            received_buffer = body[body_at:body_at + self.msg_each_header_size]
            body_at += self.msg_each_header_size
            msg_size, msg_type = self.msg_helper.depack_msg_type(received_buffer)

            if self.enable_verbose_log:
                log_file_path = "received_each_header_buffer.log"
                with open(log_file_path, 'a', encoding='utf-8') as log_file:  # 'a' for appending text
                    log_file.write(f"[MSG_HEADER] Size: {msg_size}, Type: {msg_type} | Hex: {received_buffer.hex()}\n")

            # get message it self
            body_size = msg_size - self.msg_each_header_size
            if body_size < 0:
                raise ValueError(
                    f'record size {msg_size} is smaller than the record header '
                    f'({self.msg_each_header_size}) -- stream desync (#87)')
            # #87: same ceiling SocketHelper.cpp enforces (MAX_RECORD_SIZE). No FIXS
            # sender emits a record above it, so hitting this means the stream is
            # desynced and msg_size was read out of the middle of a record. Without the
            # check Python would block forever waiting for bytes that never come, where
            # the C++ receiver fails loudly.
            if body_size > self.MAX_RECORD_SIZE:
                raise ValueError(
                    f'record size {msg_size} exceeds MAX_RECORD_SIZE '
                    f'{self.MAX_RECORD_SIZE} -- stream desync (#87)')
            if body_at + body_size > len(body):
                raise ValueError(
                    f'record of {body_size} bytes runs past the message the header '
                    f'declared ({total_msg_size}) -- stream desync (#87)')
            received_buffer = body[body_at:body_at + body_size]
            body_at += body_size

            if self.enable_verbose_log:
                log_file_path = "received_msg_buffer.log"
                with open(log_file_path, 'a', encoding='utf-8') as log_file:  # 'a' for appending text
                    log_file.write(f"[MSG_DATA] Size: {len(received_buffer)}, Type: {msg_type} | Hex: {received_buffer.hex()}\n")

            # unpack message based on type identifier
            if msg_type == MessageType.vehicle_data:
                aa = 1
                vehicle_data_received = self.msg_helper.depack_veh_data(received_buffer)
                self.vehicle_data_receive_list.append(vehicle_data_received)
                # Also index it by id for MsgHelper.VehDataRecv_um -- the feed
                # VirEnvCore walks, and the peer of SocketHelper.cpp:1030. Raw
                # wire id, unstripped, exactly as the C++ keys it.
                self.msg_helper.VehDataRecv_um[vehicle_data_received.id] = vehicle_data_received
            elif msg_type == MessageType.traffic_light_data:
                # Wire format is fixed: name (uint8 len + bytes), id (uint16),
                # state (uint8 len + bytes). depack_traffic_light_data on
                # MsgHelper handles it.
                tls_data = self.msg_helper.depack_traffic_light_data(received_buffer)
                self.traffic_light_data_receive_list.append(tls_data)
                self.msg_helper.TlsDataRecv_um[tls_data.name] = tls_data

            elif msg_type == MessageType.detector_data:
                # DetDataRecv_v = self.depackDetectorData(received_buffer) 
                aa = 1
            else:
                aa = 1

            msg_processed_size = msg_processed_size + msg_size

        return sim_state, sim_time,
        

    def sendData(self, simState, simTime, sock):
        byte_index = 0
        total_msg_size = 0
        total_msg_size = total_msg_size + self.msg_header_size
        byte_index = byte_index + self.msg_header_size
        vehicle_byte_index = 0
        vehicle_data_buffer = bytearray(65536) 
        # Not yet implemented - see GitHub issue #130
        traffic_light_data_buffer = bytearray(8096)
        detector_data_buffer = bytearray(8096)
        data_buffer = bytearray(81728)
        for vehicle_data in self.vehicle_data_send_list:
            vehicle_data, vehicle_msg_size, vehicle_byte_index = self.msg_helper.pack_veh_data(vehicle_data_buffer, 
                                                                                                   vehicle_byte_index, 
                                                                                                   vehicle_data)
            total_msg_size = total_msg_size + vehicle_msg_size
        
        data_buffer, byte_index = self.msg_helper.pack_msg_header(data_buffer, simState, simTime, total_msg_size)
        data_buffer[byte_index:byte_index+vehicle_byte_index] = vehicle_data_buffer[0:vehicle_byte_index]
        byte_index = byte_index + vehicle_byte_index

        # send data
        sock.sendall(data_buffer[0:byte_index])