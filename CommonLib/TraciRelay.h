/*
TraciRelay -- execute one relayed TraCI command on the connection TrafficLayer
already owns (#356).

An adopter arriving with a TraCI-shaped Python script changes one import
(`import fixs.traci as traci`) and keeps their `traci.vehicle.*` calls. Those calls
are serialized by SUMO's own traci, sent to TrafficLayer as a FIXS RPC record
(FIXS_MSG_TRACI_REQUEST), and executed here against the single libtraci connection
TrafficHelper opened. No second TraCI client, no --num-clients, no stepping contract
for the adopter to get wrong.

The payload is OPAQUE end to end. Neither this file nor the FIXS wire knows what
varID 0x13 means -- the bytes were packed by traci in the client and are handed to
SUMO unread. That is what lets the relay cover the whole TraCI API without either
side enumerating it.

Measured before it was written (tests/Sumo/Probes/TraciRelay/FINDINGS.md):
Python's _pack output drops into doCommand's addData byte for byte, and doCommand's
reply lands exactly where traci's own parsers expect to start reading.
*/

#pragma once

#include <string>
#include <vector>

#include "MsgTypes.h"

namespace fixs {

struct TraciResult {
	uint8_t status = FIXS_TRACI_REFUSED;
	std::vector<unsigned char> body;   // OK: the reply. Otherwise: the message text.
};

// Executes one relayed command. Never throws: a SUMO error and a refusal both come
// back as a status and a message, because the caller is a socket loop with a client
// blocked on the other end.
//
// varID < 0 means "no variable and no object id on the wire", which is the shape
// traci uses for the base commands -- and precisely why the refusals below are
// load-bearing rather than defensive: a relayed CMD_SIMSTEP was measured moving the
// clock behind TrafficLayer's back.
TraciResult executeTraci(int cmdID, int varID, const std::string& objID,
                         const std::vector<unsigned char>& payload);

// True when this build can relay at all. False under ENABLE_LIBSUMO, where SUMO runs
// in-process and there is no Connection and no generic executor to relay onto.
bool traciRelayAvailable();

}  // namespace fixs
