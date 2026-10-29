#include "TraciRelay.h"

#include "TrafficHelper.h"   // for ENABLE_LIBSUMO

#ifndef ENABLE_LIBSUMO
#include <foreign/tcpip/storage.h>

// Declared here rather than through libtraci/Connection.h, which needs SUMO's
// utils/common/* headers. Both members are exported by libtracicpp on Windows and
// Linux; scripts/dispatch/check_libtraci_symbols.ps1 fails the Windows build if not.
namespace libtraci {
class Connection {
public:
	static Connection& getActive();
	tcpip::Storage& doCommand(int command, int var, const std::string& id,
	                          tcpip::Storage* add, int expectedType);
};
}
#endif

namespace {

bool refusedCommand(int cmdID, std::string& why) {
	switch (cmdID) {
	case 0x02:
		why = "simulationStep() is not relayed: TrafficLayer owns the clock. "
		      "fixs.recv() / fixs.send() advance the co-simulation.";
		return true;
	case 0x7F:
		why = "close() is not relayed: TrafficLayer owns the session. Use fixs.close().";
		return true;
	case 0x03:
		why = "setOrder() is not relayed: FIXS is the only TraCI client.";
		return true;
	case 0x01:
		why = "load() is not relayed: it would reload the network under a running co-simulation.";
		return true;
	default:
		break;
	}
	// CMD_SUBSCRIBE_*_CONTEXT is 0x04-0x0b and 0x80-0x8f; CMD_SUBSCRIBE_*_VARIABLE is 0x54-0x5b and 0xd0-0xdf.
	if ((cmdID >= 0x04 && cmdID <= 0x0b) || (cmdID >= 0x80 && cmdID <= 0x8f) ||
	    (cmdID >= 0x54 && cmdID <= 0x5b) || (cmdID >= 0xd0 && cmdID <= 0xdf)) {
		why = "subscriptions are not relayed: the FIXS feed is the subscription. "
		      "Read the vehicles the config names from fixs.vehicle.";
		return true;
	}
	return false;
}

fixs::TraciResult message(uint8_t status, const std::string& text) {
	fixs::TraciResult r;
	r.status = status;
	r.body.assign(text.begin(), text.end());
	return r;
}

}  // namespace

bool fixs::traciRelayAvailable() {
#ifdef ENABLE_LIBSUMO
	return false;
#else
	return true;
#endif
}

fixs::TraciResult fixs::executeTraci(int cmdID, int varID, const std::string& objID,
                                     const std::vector<unsigned char>& payload) {
#ifdef ENABLE_LIBSUMO
	(void)cmdID; (void)varID; (void)objID; (void)payload;
	return message(kTraciRefused,
		"this TrafficLayer embeds SUMO via libsumo (ENABLE_LIBSUMO): there is no "
		"TraCI connection to relay onto.");
#else
	std::string why;
	if (refusedCommand(cmdID, why)) {
		return message(kTraciRefused, why);
	}

	tcpip::Storage add;
	if (!payload.empty()) {
		add.writePacket(payload);
	}
	try {
		// TrafficLayer is single-threaded, so this cannot interleave with its own libtraci calls.
		// expectedType -1 leaves the reply where traci's Domain._getCmd starts reading.
		tcpip::Storage& in = libtraci::Connection::getActive()
		                     .doCommand(cmdID, varID, objID, payload.empty() ? nullptr : &add, -1);
		TraciResult r;
		r.status = kTraciOk;
		r.body.reserve(in.size());
		while (in.valid_pos()) {
			r.body.push_back(in.readChar());
		}
		return r;
	}
	catch (const std::exception& e) {
		return message(kTraciError, e.what());
	}
#endif
}
