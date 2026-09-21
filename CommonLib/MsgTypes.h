/*
Record types on the FIXS wire (#356).

Every record carries a 3-byte header -- uint16 size, uint8 type -- and the type is
what SocketHelper dispatches on. Until #356 the three values below were bare
integer literals in a switch in SocketHelper.cpp and again in MsgHelper.py; naming
them here is the point of this header, so the two ends cannot drift.

The range, not the individual number, is what a reader should classify from:

    1-63     DATA. State pushed one way, no answer expected. The sender writes the
             record and moves on. An unknown one may be skipped: framing survives,
             the receiver just loses a field it did not know about.

    64-127   reserved.

    128-255  RPC. The sender BLOCKS until the peer answers on the same socket.
             Request is even, its response is request + 1. An unknown one may NOT
             simply be skipped -- the peer is waiting -- which is why recvData
             answers 'unsupported' instead of dropping it (see #356).
*/

#pragma once

#include <stdint.h>
#include <string>
#include <vector>

enum FixsMsgType : uint8_t {
	// --- data -------------------------------------------------------------
	FIXS_MSG_VEHICLE       = 1,
	FIXS_MSG_TRAFFICLIGHT  = 2,
	FIXS_MSG_DETECTOR      = 3,
	//                       4-63 free for the next native record type

	// --- rpc --------------------------------------------------------------
	FIXS_MSG_TRACI_REQUEST  = 128,
	FIXS_MSG_TRACI_RESPONSE = 129,
	//                        130+ free for the next relay (a CARLA RPC, #355)
};

// Lowest type that means "the peer is blocked waiting for an answer".
#define FIXS_MSG_RPC_FIRST 128

// Status byte at the head of every FIXS_MSG_TRACI_RESPONSE record.
enum FixsTraciStatus : uint8_t {
	FIXS_TRACI_OK       = 0,  // body is the reply, positioned as traci's parsers expect
	FIXS_TRACI_ERROR    = 1,  // body is SUMO's own error text -> traci.TraCIException
	FIXS_TRACI_REFUSED  = 2,  // body is FIXS's text          -> traci.FatalTraCIError
};

// Largest payload chunk a single record may carry, so that record + header stays
// under MAX_RECORD_SIZE (8192, the wire contract #87 enforces on BOTH ends). A TraCI
// payload or reply larger than this is split across consecutive records of the SAME
// message, each flagging `more`; the receiver concatenates in wire order. It is not
// hypothetical: lane.getIDList() on a real network exceeds 8 KB.
#define FIXS_TRACI_REQ_CHUNK  7800   // worst-case header is 3+1+2+1+255+1+4 = 267 B
#define FIXS_TRACI_RSP_CHUNK  8000   // header is 3+1+1+4 = 9 B

// One relayed TraCI command, reassembled from its chunks.
struct TraciRequest_t {
	uint8_t cmdID = 0;
	int16_t varID = -1;                 // -1: no variable and no object id on the wire
	std::string objID;
	std::vector<unsigned char> payload; // opaque: packed by the client's own traci
};
