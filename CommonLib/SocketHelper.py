from struct import unpack, pack
from CommonLib.VehDataMsgDefs import VehData
from CommonLib.VehDataMsgDefs import TrafficLightData
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

        # initialize lists to store sending data
        self.vehicle_data_send_list: typing.List[VehData] = []
        self.traffic_light_data_send_list = []
        self.detector_data_send_list = []


        # initialize lists to store received data
        self.vehicle_data_receive_list: typing.List[VehData] = []
        self.traffic_light_data_receive_list: typing.List[TrafficLightData] = []
        self.detector_data_receive_list = []

    def clear_data(self):
        self.vehicle_data_send_list.clear()
        self.traffic_light_data_send_list.clear()
        self.detector_data_send_list.clear()

        self.vehicle_data_receive_list.clear()
        self.traffic_light_data_receive_list.clear()
        self.detector_data_receive_list.clear()



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
    

    def recvall(self, sock, n: int) -> bytes:
        """Read exactly n bytes from TCP stream, or raise ConnectionError/TimeoutError."""
        data = bytearray()
        while len(data) < n:
            chunk = sock.recv(n - len(data))
            if not chunk:
                raise ConnectionError(f"Socket closed while reading {n} bytes (got {len(data)})")
            data.extend(chunk)
        return bytes(data)
    

    # def recv_data(self, sock):
    #     # initialize return lists
        
        
    #     # get header for entire message
    #     received_buffer = sock.recv(self.msg_header_size)
        

    #     sim_state, sim_time, total_msg_size = self.msg_helper.depack_msg_header(received_buffer)
    #     msg_processed_size = 0
    #     msg_processed_size = msg_processed_size + self.msg_header_size
    #     # total message size is the data to be received
    #     # save received_buffer to local log for debugging
    #     log_file_path = "received_header_buffer.log"
    #     with open(log_file_path, 'ab') as log_file:  # 'ab' for appending binary data
    #         log_file.write(received_buffer)
    #         log_file.write(b'\n')
        
    #     while (msg_processed_size < total_msg_size):
    #         # get message type header
    #         received_buffer = sock.recv(self.msg_each_header_size)
    #         msg_size, msg_type = self.msg_helper.depack_msg_type(received_buffer)

    #         log_file_path = "received_each_header_buffer.log"
    #         with open(log_file_path, 'ab') as log_file:  # 'ab' for appending binary data
    #             log_file.write(received_buffer)
    #             log_file.write(b'\n')

    #         # get message it self
    #         received_buffer = sock.recv(msg_size - self.msg_each_header_size)

    #         log_file_path = "received_msg_buffer.log"
    #         with open(log_file_path, 'ab') as log_file:  # 'ab' for appending binary data
    #             log_file.write(received_buffer)
    #             log_file.write(b'\n')

    #         # unpack message based on type identifier
    #         if msg_type == MessageType.vehicle_data:
    #             aa = 1
    #             vehicle_data_received = self.msg_helper.depack_veh_data(received_buffer)
    #             self.vehicle_data_receive_list.append(vehicle_data_received)
    #         elif msg_type == MessageType.traffic_light_data:               
    #             aa = 1
    #             tls = self.msg_helper.depack_traffic_light_data(received_buffer)
    #             self.traffic_light_data_receive_list.append(tls)

    #         elif msg_type == MessageType.detector_data:
    #             # DetDataRecv_v = self.depackDetectorData(received_buffer) 
    #             aa = 1
    #         else:
    #             aa = 1

    #         msg_processed_size = msg_processed_size + msg_size

    #     return sim_state, sim_time,



    def recv_data(self, sock):
        # initialize return lists
        
        
        # get header for entire message
        # 1) header
        received_buffer = self.recvall(sock, self.msg_header_size)
        sim_state, sim_time, total_msg_size = self.msg_helper.depack_msg_header(received_buffer)
        msg_processed_size = self.msg_header_size

        # 2) sub information
        while msg_processed_size < total_msg_size:
            each_hdr = self.recvall(sock, self.msg_each_header_size)
            msg_size, msg_type = self.msg_helper.depack_msg_type(each_hdr)

            body_size = msg_size - self.msg_each_header_size
            if body_size <= 0:
                raise RuntimeError(f"Invalid body_size={body_size}, msg_size={msg_size}, msg_type={msg_type}")

            body = self.recvall(sock, body_size)

            # unpack
            if msg_type == MessageType.vehicle_data:
                vd = self.msg_helper.depack_veh_data(body)
                self.vehicle_data_receive_list.append(vd)
            elif msg_type == MessageType.traffic_light_data:
                tls = self.msg_helper.depack_traffic_light_data(body)
                self.traffic_light_data_receive_list.append(tls)
            elif msg_type == MessageType.detector_data:
                pass

            msg_processed_size += msg_size

        return sim_state, sim_time



        

    # def sendData(self, simState, simTime, sock):
    #     byte_index = 0
    #     total_msg_size = 0
    #     total_msg_size = total_msg_size + self.msg_header_size
    #     byte_index = byte_index + self.msg_header_size
    #     vehicle_byte_index = 0
    #     vehicle_data_buffer = bytearray(65536) 
    #     # TODO: add traffic light and detector data
    #     traffic_light_data_buffer = bytearray(8096)
    #     detector_data_buffer = bytearray(8096)
    #     data_buffer = bytearray(81728)
    #     for vehicle_data in self.vehicle_data_send_list:
    #         vehicle_data, vehicle_msg_size, vehicle_byte_index = self.msg_helper.pack_veh_data(vehicle_data_buffer, 
    #                                                                                                vehicle_byte_index, 
    #                                                                                                vehicle_data)
    #         total_msg_size = total_msg_size + vehicle_msg_size

    #                     # ✅ 关键检查：车辆buffer是否溢出
    #         if vehicle_byte_index > len(vehicle_data_buffer):
    #             raise RuntimeError(
    #                 f"vehicle_data_buffer overflow: vehicle_byte_index={vehicle_byte_index} > {len(vehicle_data_buffer)} "
    #                 f"(nVeh={len(self.vehicle_data_send_list)})"
    #             )

    #         # ✅ 关键检查：总包大小是否超出 data_buffer
    #         if total_msg_size > len(data_buffer):
    #             raise RuntimeError(
    #                 f"data_buffer overflow: total_msg_size={total_msg_size} > {len(data_buffer)} "
    #                 f"(vehicle_byte_index={vehicle_byte_index}, nVeh={len(self.vehicle_data_send_list)})"
    #         )
        
    #     data_buffer, byte_index = self.msg_helper.pack_msg_header(data_buffer, simState, simTime, total_msg_size)

    #     # ✅ 再检查一次 copy 是否会越界
    #     if byte_index + vehicle_byte_index > len(data_buffer):
    #         raise RuntimeError(
    #             f"data_buffer overflow on copy: {byte_index}+{vehicle_byte_index} > {len(data_buffer)}"
    #         )

    #     data_buffer[byte_index:byte_index+vehicle_byte_index] = vehicle_data_buffer[0:vehicle_byte_index]
    #     byte_index = byte_index + vehicle_byte_index

    #     # send data
    #     sock.sendall(data_buffer[0:byte_index])


    def sendData(self, simState, simTime, sock):
        vehicle_byte_index = 0
        vehicle_data_buffer = bytearray(65536)

        for vehicle_data in self.vehicle_data_send_list:
            vehicle_data, vehicle_msg_size, vehicle_byte_index = self.msg_helper.pack_veh_data(
                vehicle_data_buffer, vehicle_byte_index, vehicle_data
            )

            if vehicle_byte_index > len(vehicle_data_buffer):
                raise RuntimeError(...)


        total_msg_size = self.msg_header_size + vehicle_byte_index

        data_buffer = bytearray(total_msg_size)
        data_buffer, byte_index = self.msg_helper.pack_msg_header(data_buffer, simState, simTime, total_msg_size)


        assert byte_index == self.msg_header_size

        data_buffer[byte_index:byte_index + vehicle_byte_index] = vehicle_data_buffer[:vehicle_byte_index]
        sock.sendall(data_buffer)

