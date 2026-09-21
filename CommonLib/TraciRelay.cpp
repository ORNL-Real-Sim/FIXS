#include "TraciRelay.h"

#include <mutex>

#include "TrafficHelper.h"   // for ENABLE_LIBSUMO, and nothing else

#ifndef ENABLE_LIBSUMO
#include <foreign/tcpip/storage.h>

// ---------------------------------------------------------------------------
// Binding to libtracicpp.dll
// ---------------------------------------------------------------------------
// The generic executor is exported, but libtraci/Connection.h CANNOT be vendored to
// declare it: that header includes libsumo/Subscription.h, which includes
// utils/common/SUMOVehicleClass.h and utils/common/SUMOTime.h, neither of which is in
// the shipped set -- and pulling them in drags a large part of SUMO's utils/common
// behind them. So the three members used here are declared locally. MSVC mangles a
// member function from the namespace, class name and signature alone, so this links
// against libtracicpp.lib exactly as the real declaration would.
//
// The bet that makes: a SUMO bump that changes one of these signatures breaks the
// link rather than the behaviour. scripts/dispatch/check_libtraci_symbols.ps1 (run by
// 2_core_components.bat, before TrafficLayer is built) asserts all three are present,
// so that failure arrives with an explanation instead of as an unresolved external.
namespace libtraci {
class Connection {
public:
	static Connection& getActive();
	std::mutex& getMutex() const;
	tcpip::Storage& doCommand(int command, int var, const std::string& id,
	                          tcpip::Storage* add, int expectedType);
};
}
#endif

namespace {

// Commands FIXS owns and will not relay. Each one fits the four-value contract as
// well as any getter does -- refusing them is the only thing that stops a relayed
// script from taking the clock or the session away from TrafficLayer.
//
// The subscription ranges are refused for a different reason: their varID is a PAIR
// of doubles (begin, end), which doCommand's int varID cannot carry at all. The
// client refuses them first, with a message pointing at the FIXS feed; this is the
// backstop for a client FIXS did not write.
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
		why = "setOrder() is not relayed: FIXS is the only TraCI client, so client "
		      "ordering is meaningless.";
		return true;
	case 0x01:
		why = "load() is not relayed: it would reload the network under a running "
		      "co-simulation.";
		return true;
	default:
		break;
	}
	// CMD_SUBSCRIBE_*_CONTEXT: 0x04-0x0b and 0x80-0x8f
	// CMD_SUBSCRIBE_*_VARIABLE: 0x54-0x5b and 0xd0-0xdf
	// (SUMO added the low blocks when the high ones ran out of domains. As COMMAND
	// ids these are unambiguous; the same numbers appear as VARIABLE ids elsewhere,
	// which is a different namespace.)
	if ((cmdID >= 0x04 && cmdID <= 0x0b) || (cmdID >= 0x80 && cmdID <= 0x8f) ||
	    (cmdID >= 0x54 && cmdID <= 0x5b) || (cmdID >= 0xd0 && cmdID <= 0xdf)) {
		why = "subscriptions are not relayed: the FIXS feed IS the subscription. The "
		      "vehicles and fields the config named arrive every tick -- use fixs.vehicle.";
		return true;
	}
	return false;
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
	TraciResult result;

#ifdef ENABLE_LIBSUMO
	(void)cmdID; (void)varID; (void)objID; (void)payload;
	const std::string why =
		"this TrafficLayer embeds SUMO via libsumo (ENABLE_LIBSUMO): SUMO runs "
		"in-process, there is no TraCI connection to relay onto. Build with libtraci "
		"to use fixs.traci.";
	result.status = FIXS_TRACI_REFUSED;
	result.body.assign(why.begin(), why.end());
	return result;
#else
	std::string why;
	if (refusedCommand(cmdID, why)) {
		result.status = FIXS_TRACI_REFUSED;
		result.body.assign(why.begin(), why.end());
		return result;
	}

	tcpip::Storage add;
	if (!payload.empty()) {
		add.writePacket(payload);
	}

	try {
		// Same discipline libtraci's own domains follow: the CALLER holds the mutex
		// around doCommand, and Connection::simulationStep takes it too. Holding it
		// here is what makes a relayed command safe against TrafficLayer's own
		// libtraci calls -- measured with 300 relayed getters against 300 steps on
		// another thread (FINDINGS.md).
		std::lock_guard<std::mutex> lock(libtraci::Connection::getActive().getMutex());

		// expectedType = -1, NOT 0. With -1 doCommand consumes only the status
		// message and leaves the reply positioned exactly where traci's
		// Domain._getCmd starts reading, so the bytes below need no re-framing on
		// the client. With 0 it would eat part of the response header.
		tcpip::Storage& in = libtraci::Connection::getActive()
		                     .doCommand(cmdID, varID, objID,
		                                payload.empty() ? nullptr : &add, -1);
		result.body.reserve(in.size());
		while (in.valid_pos()) {
			result.body.push_back(in.readChar());
		}
		result.status = FIXS_TRACI_OK;
	}
	catch (const std::exception& e) {
		// SUMO's own text, relayed verbatim: the client re-raises it as
		// traci.TraCIException, so an adopter's error handling is unchanged.
		const std::string what = e.what();
		result.status = FIXS_TRACI_ERROR;
		result.body.assign(what.begin(), what.end());
	}
	return result;
#endif
}
