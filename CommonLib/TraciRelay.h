#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "FixsProtocol.h"

// One relayed TraCI command, reassembled from its records.
struct TraciRequest_t {
	uint8_t cmdID = 0;
	int16_t varID = -1;                 // -1: no variable id and no object id on the wire
	std::string objID;
	std::vector<unsigned char> payload; // packed by the client's traci; never parsed here
};

namespace fixs {

struct TraciResult {
	uint8_t status = kTraciRefused;
	std::vector<unsigned char> body;    // the reply, or the message text
};

// Runs one command on TrafficLayer's own libtraci connection. Never throws.
TraciResult executeTraci(int cmdID, int varID, const std::string& objID,
                         const std::vector<unsigned char>& payload);

// False in an ENABLE_LIBSUMO build, which has no libtraci connection to relay onto.
bool traciRelayAvailable();

}  // namespace fixs
