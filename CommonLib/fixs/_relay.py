"""The FIXS side of the relayed-TraCI RPC (#356).

One function matters: :func:`request` sends a TraCI command to TrafficLayer on the
connection ``fixs`` already holds and blocks for the reply. Everything above it
(``fixs.traci``) is SUMO's own traci; everything below it (TrafficLayer) executes the
command on the libtraci connection it owns. This module is only the framing in
between, and it never looks inside a payload.

Wire format, mirroring CommonLib/MsgTypes.h and MsgHelper.cpp:

    message = header + one or more records

    header  = uint8 simState | float simTime | uint32 totalMsgSize      (9 bytes)
    record  = uint16 recordSize | uint8 msgType | body

    request  body = uint8 cmdID | int16 varID | uint8 idLen | id
                    | uint8 more | uint32 chunkLen | chunk
    response body = uint8 status | uint8 more | uint32 chunkLen | chunk

A payload larger than one record is split across consecutive records of the SAME
message, each flagging ``more``; the receiver concatenates in wire order. That is not
hypothetical -- MAX_RECORD_SIZE is 8192 and ``lane.getIDList()`` on a real network is
bigger than that.

'<' everywhere: the C++ side memcpy's fields contiguously with no padding.
"""

import struct
import sys

# --- wire constants, kept in step with CommonLib/MsgTypes.h --------------------
MSG_TRACI_REQUEST = 128
MSG_TRACI_RESPONSE = 129

TRACI_OK = 0        # body is the reply, positioned where traci's parsers expect
TRACI_ERROR = 1     # body is SUMO's own error text
TRACI_REFUSED = 2   # body is FIXS's own text

REQ_CHUNK = 7800    # FIXS_TRACI_REQ_CHUNK
MAX_RECORD_SIZE = 8192

_HEADER = struct.Struct('<BfI')      # simState, simTime, totalMsgSize
_REC = struct.Struct('<HB')          # recordSize, msgType
_REQ_HEAD = struct.Struct('<BhB')    # cmdID, varID, idLen
_REQ_TAIL = struct.Struct('<BI')     # more, chunkLen
_RSP_HEAD = struct.Struct('<BBI')    # status, more, chunkLen

HEADER_SIZE = _HEADER.size           # 9, == MsgHelper::msgHeaderSize
REC_HEADER_SIZE = _REC.size          # 3, == MsgHelper::msgEachHeaderSize


class RelayUnavailable(Exception):
    """Raised when a relay call is made outside the window that can serve it."""


def _fixs():
    """The parent package, without importing it at module load (it imports us)."""
    return sys.modules[__package__]


def connected():
    return getattr(_fixs(), '_sock', None) is not None


def _recv_exact(sock, n):
    """Mirrors SocketHelper._recv_exact: a short read desyncs the stream for good."""
    if n == 0:
        return b''
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError(
                'TrafficLayer closed the connection mid TraCI reply: got '
                '%d of %d bytes' % (len(buf), n))
        buf += chunk
    return bytes(buf)


def request(cmdID, varID, objID, payload):
    """Relay one TraCI command. Returns ``(status, body)``.

    Legal only while this client holds a tick -- between ``fixs.recv()`` and
    ``fixs.send()``. That is not an arbitrary restriction: TrafficLayer answers relay
    requests from inside its blocking wait for this client's tick answer, so outside
    that window there is nobody reading.
    """
    fixs = _fixs()
    sock = getattr(fixs, '_sock', None)
    if sock is None:
        raise RelayUnavailable(
            "not connected to TrafficLayer. Call fixs.connect('config.yaml') before "
            'using fixs.traci.')
    if not getattr(fixs, '_armed', False):
        raise RelayUnavailable(
            'a TraCI call is only legal while this client holds a tick -- between '
            'fixs.recv() and fixs.send(). TrafficLayer serves relayed commands from '
            'inside its wait for this client\'s answer; outside that window nothing '
            'is listening and the call would hang.')

    objBytes = str(objID).encode('utf8')
    if len(objBytes) > 255:
        raise ValueError('TraCI object id longer than 255 bytes: %r' % objID)

    # --- request message ------------------------------------------------------
    chunks = [payload[i:i + REQ_CHUNK] for i in range(0, len(payload), REQ_CHUNK)] or [b'']
    records = []
    for i, chunk in enumerate(chunks):
        body = (_REQ_HEAD.pack(cmdID, varID, len(objBytes)) + objBytes +
                _REQ_TAIL.pack(1 if i < len(chunks) - 1 else 0, len(chunk)) + chunk)
        records.append(_REC.pack(REC_HEADER_SIZE + len(body), MSG_TRACI_REQUEST) + body)

    total = HEADER_SIZE + sum(len(r) for r in records)
    # simState 1 = running. TrafficLayer echoes these back and the reply path ignores
    # them; 0 would mean shutdown to a FIXS client, so it is never sent here.
    sock.sendall(_HEADER.pack(1, float(getattr(fixs, '_simTime', 0.0)), total) +
                 b''.join(records))

    # --- response message -----------------------------------------------------
    _, _, totalMsgSize = _HEADER.unpack(_recv_exact(sock, HEADER_SIZE))
    processed = HEADER_SIZE
    status = None
    body = bytearray()
    while processed < totalMsgSize:
        recSize, msgType = _REC.unpack(_recv_exact(sock, REC_HEADER_SIZE))
        bodySize = recSize - REC_HEADER_SIZE
        if bodySize < 0 or bodySize > MAX_RECORD_SIZE:
            raise ConnectionError(
                'record size %d out of range -- stream desync on the TraCI reply'
                % recSize)
        raw = _recv_exact(sock, bodySize)
        if msgType != MSG_TRACI_RESPONSE:
            raise ConnectionError(
                'expected a TraCI response record (%d), got type %d. TrafficLayer and '
                'this client disagree about the wire format.'
                % (MSG_TRACI_RESPONSE, msgType))
        st, more, chunkLen = _RSP_HEAD.unpack(raw[:_RSP_HEAD.size])
        status = st
        body += raw[_RSP_HEAD.size:_RSP_HEAD.size + chunkLen]
        processed += recSize
        if not more:
            break

    if status is None:
        raise ConnectionError('empty TraCI reply from TrafficLayer')
    return status, bytes(body)
