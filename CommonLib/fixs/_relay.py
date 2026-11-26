"""Framing for relayed TraCI: one request record set out, one response record set back.

message  = uint8 simState | float simTime | uint32 totalMsgSize, then records
record   = uint16 recordSize | uint8 msgType | body
request  = uint8 cmdID | int16 varID | uint8 idLen | id | uint8 more | uint32 len | chunk
response = uint8 status | uint8 more | uint32 len | chunk

Constants mirror CommonLib/FixsProtocol.h.
"""
import struct
import sys

MSG_TRACI_REQUEST = 128
MSG_TRACI_RESPONSE = 129

TRACI_OK = 0
TRACI_ERROR = 1
TRACI_REFUSED = 2

# varID of a relayed subscription, which is not a TraCI command (FixsProtocol.h).
TRACI_SUBSCRIPTION_VAR = -2

REQ_CHUNK = 7800
MAX_RECORD_SIZE = 8192

_HEADER = struct.Struct('<BfI')
_REC = struct.Struct('<HB')
_REQ_HEAD = struct.Struct('<BhB')
_REQ_TAIL = struct.Struct('<BI')
_RSP_HEAD = struct.Struct('<BBI')

HEADER_SIZE = _HEADER.size
REC_HEADER_SIZE = _REC.size


class RelayUnavailable(Exception):
    """A relayed call was made where nothing can serve it."""


def _fixs():
    # The parent package, looked up at call time because it imports this module.
    return sys.modules[__package__]


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('TrafficLayer closed the connection mid TraCI reply '
                                  '(%d of %d bytes)' % (len(buf), n))
        buf += chunk
    return bytes(buf)


def encode_request(cmdID, varID, objID, payload, simTime=0.0):
    """The request message for one command, split into records of at most REQ_CHUNK payload bytes."""
    objBytes = str(objID).encode('utf8')
    if len(objBytes) > 255:
        raise ValueError('TraCI object id longer than 255 bytes: %r' % objID)
    chunks = [payload[i:i + REQ_CHUNK] for i in range(0, len(payload), REQ_CHUNK)] or [b'']
    records = []
    for i, chunk in enumerate(chunks):
        body = (_REQ_HEAD.pack(cmdID, varID, len(objBytes)) + objBytes +
                _REQ_TAIL.pack(1 if i < len(chunks) - 1 else 0, len(chunk)) + chunk)
        records.append(_REC.pack(REC_HEADER_SIZE + len(body), MSG_TRACI_REQUEST) + body)
    total = HEADER_SIZE + sum(len(r) for r in records)
    # simState 1: TrafficLayer echoes it, and 0 would read as shutdown to a FIXS client.
    return _HEADER.pack(1, float(simTime), total) + b''.join(records)


def read_response(sock):
    """(status, body) from one response message."""
    _, _, totalMsgSize = _HEADER.unpack(_recv_exact(sock, HEADER_SIZE))
    processed, status, body = HEADER_SIZE, None, bytearray()
    while processed < totalMsgSize:
        recSize, msgType = _REC.unpack(_recv_exact(sock, REC_HEADER_SIZE))
        bodySize = recSize - REC_HEADER_SIZE
        if not 0 <= bodySize <= MAX_RECORD_SIZE:
            raise ConnectionError('record size %d out of range in a TraCI reply' % recSize)
        raw = _recv_exact(sock, bodySize)
        if msgType != MSG_TRACI_RESPONSE:
            raise ConnectionError('expected a TraCI response record, got type %d' % msgType)
        status, more, chunkLen = _RSP_HEAD.unpack_from(raw)
        body += raw[_RSP_HEAD.size:_RSP_HEAD.size + chunkLen]
        processed += recSize
        if not more:
            break
    if status is None:
        raise ConnectionError('empty TraCI reply from TrafficLayer')
    return status, bytes(body)


def request(cmdID, varID, objID, payload):
    """Relay one command on the connection fixs holds. Legal only while a tick is held."""
    fixs = _fixs()
    sock = getattr(fixs, '_sock', None)
    if sock is None:
        raise RelayUnavailable("not connected to TrafficLayer: call fixs.connect() "
                               "(or traci.start()) first.")
    if not getattr(fixs, '_armed', False):
        raise RelayUnavailable('no tick is held: call traci.simulationStep() (or fixs.recv()) '
                               'first. TrafficLayer serves relayed commands only while it '
                               'waits for this client.')
    sock.sendall(encode_request(cmdID, varID, objID, payload, getattr(fixs, '_simTime', 0.0)))
    return read_response(sock)
