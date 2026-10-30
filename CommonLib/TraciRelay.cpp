#include "TraciRelay.h"

#include "TrafficHelper.h"   // for ENABLE_LIBSUMO

#ifndef ENABLE_LIBSUMO
#include <foreign/tcpip/storage.h>

#include <algorithm>
#include <cstdio>

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

bool isContextSubscribe(int cmdID) {
	return (cmdID >= 0x04 && cmdID <= 0x0b) || (cmdID >= 0x80 && cmdID <= 0x8f);
}

bool isVariableSubscribe(int cmdID) {
	return (cmdID >= 0x54 && cmdID <= 0x5b) || (cmdID >= 0xd0 && cmdID <= 0xdf);
}

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
	case 0x7E:
		why = "subscription filters are not relayed: they apply to context subscriptions.";
		return true;
	default:
		break;
	}
	if (isContextSubscribe(cmdID)) {
		why = "context subscriptions are not relayed: subscribe each object with subscribe(), "
		      "or read the vehicles the config names from fixs.vehicle.";
		return true;
	}
	if (isVariableSubscribe(cmdID)) {
		why = "a raw subscription command is not relayed: use the domain's subscribe().";
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

#ifndef ENABLE_LIBSUMO

using SubscribeFn = void (*)(const std::string&, const std::vector<int>&, double, double,
                             const libsumo::TraCIResults&);
using ResultsFn = const libsumo::TraCIResults (*)(const std::string&);

struct Domain {
	int subscribeCmd;
	SubscribeFn subscribe;
	ResultsFn results;
};

#define FIXS_DOMAIN(CLASS, DOM) \
	{ libsumo::CMD_SUBSCRIBE_##DOM##_VARIABLE, &libtraci::CLASS::subscribe, &libtraci::CLASS::getSubscriptionResults }

const Domain kDomains[] = {
	FIXS_DOMAIN(InductionLoop, INDUCTIONLOOP), FIXS_DOMAIN(MultiEntryExit, MULTIENTRYEXIT),
	FIXS_DOMAIN(TrafficLight, TL), FIXS_DOMAIN(Lane, LANE), FIXS_DOMAIN(Vehicle, VEHICLE),
	FIXS_DOMAIN(VehicleType, VEHICLETYPE), FIXS_DOMAIN(Route, ROUTE), FIXS_DOMAIN(POI, POI),
	FIXS_DOMAIN(Polygon, POLYGON), FIXS_DOMAIN(Junction, JUNCTION), FIXS_DOMAIN(Edge, EDGE),
	FIXS_DOMAIN(Simulation, SIM), FIXS_DOMAIN(LaneArea, LANEAREA), FIXS_DOMAIN(Person, PERSON),
	FIXS_DOMAIN(BusStop, BUSSTOP), FIXS_DOMAIN(ParkingArea, PARKINGAREA),
	FIXS_DOMAIN(ChargingStation, CHARGINGSTATION), FIXS_DOMAIN(RouteProbe, ROUTEPROBE),
	FIXS_DOMAIN(Calibrator, CALIBRATOR), FIXS_DOMAIN(Rerouter, REROUTER),
	FIXS_DOMAIN(VariableSpeedSign, VARIABLESPEEDSIGN), FIXS_DOMAIN(MeanData, MEANDATA),
	FIXS_DOMAIN(OverheadWire, OVERHEADWIRE),
};

const Domain* findDomain(int subscribeCmd) {
	for (const Domain& d : kDomains) {
		if (d.subscribeCmd == subscribeCmd) return &d;
	}
	return nullptr;
}

// One subscribed variable, with the TraCI type SUMO answers it in: libtraci keeps only the value.
struct SubVar {
	int var;
	int type;
	int secondType;                     // TYPE_COMPOUND only: the type after the leading string
	std::vector<unsigned char> param;   // typed, as the script packed it; empty if none
};

struct ScriptSub {
	const Domain* domain;
	std::string objID;
	std::vector<SubVar> vars;
};

// The script's subscriptions, in the order SUMO answers them.
std::vector<ScriptSub> scriptSubs;

std::string hex(int v) {
	char b[8];
	snprintf(b, sizeof(b), "0x%02x", v);
	return b;
}

void skipTyped(tcpip::Storage& s, int type) {
	switch (type) {
	case libsumo::TYPE_DOUBLE: s.readDouble(); break;
	case libsumo::TYPE_INTEGER: s.readInt(); break;
	case libsumo::TYPE_STRING: s.readString(); break;
	default: throw std::invalid_argument("parameter type " + hex(type) + " is not relayed");
	}
}

std::shared_ptr<libsumo::TraCIResult> paramResult(const std::vector<unsigned char>& typed) {
	tcpip::Storage s(typed.data(), (int)typed.size());
	switch (s.readUnsignedByte()) {
	case libsumo::TYPE_DOUBLE: return std::make_shared<libsumo::TraCIDouble>(s.readDouble());
	case libsumo::TYPE_INTEGER: return std::make_shared<libsumo::TraCIInt>(s.readInt());
	case libsumo::TYPE_STRING: return std::make_shared<libsumo::TraCIString>(s.readString());
	default: return nullptr;
	}
}

// Asks SUMO for the variable once, to learn the type its subscription answers will carry.
// Returns an empty string when libtraci can carry that type, else why not.
std::string probeType(int subscribeCmd, const std::string& objID, SubVar& v) {
	tcpip::Storage add;
	if (!v.param.empty()) add.writePacket(v.param);
	tcpip::Storage& in = libtraci::Connection::getActive()
	                     .doCommand(subscribeCmd - 0x30, v.var, objID, v.param.empty() ? nullptr : &add, -1);
	if (in.readUnsignedByte() == 0) in.readInt();   // length
	in.readUnsignedByte();                          // response id
	in.readUnsignedByte();                          // variable id
	in.readString();                                // object id
	v.type = in.readUnsignedByte();
	v.secondType = -1;
	switch (v.type) {
	case libsumo::TYPE_DOUBLE: case libsumo::TYPE_INTEGER: case libsumo::TYPE_STRING:
	case libsumo::POSITION_2D: case libsumo::POSITION_3D: case libsumo::TYPE_COLOR:
	case libsumo::TYPE_STRINGLIST:
		return "";
	case libsumo::TYPE_COMPOUND:
		if (in.readInt() == 2 && in.readUnsignedByte() == libsumo::TYPE_STRING) {
			in.readString();
			v.secondType = in.readUnsignedByte();
			if (v.secondType == libsumo::TYPE_DOUBLE || v.secondType == libsumo::TYPE_STRING) return "";
		}
		break;
	default:
		break;
	}
	return "variable " + hex(v.var) + " of '" + objID + "' is answered as TraCI type " + hex(v.type) +
	       ", which TrafficLayer's libtraci cannot carry in a subscription. Read it with its getter.";
}

// The value as SUMO writes it in a subscription answer.
bool writeValue(tcpip::Storage& out, const SubVar& v, const libsumo::TraCIResult* r) {
	out.writeUnsignedByte(v.type);
	switch (v.type) {
	case libsumo::TYPE_DOUBLE:
		if (auto d = dynamic_cast<const libsumo::TraCIDouble*>(r)) { out.writeDouble(d->value); return true; }
		break;
	case libsumo::TYPE_INTEGER:
		if (auto i = dynamic_cast<const libsumo::TraCIInt*>(r)) { out.writeInt(i->value); return true; }
		break;
	case libsumo::TYPE_STRING:
		if (auto t = dynamic_cast<const libsumo::TraCIString*>(r)) { out.writeString(t->value); return true; }
		break;
	case libsumo::POSITION_2D: case libsumo::POSITION_3D:
		if (auto p = dynamic_cast<const libsumo::TraCIPosition*>(r)) {
			out.writeDouble(p->x); out.writeDouble(p->y);
			if (v.type == libsumo::POSITION_3D) out.writeDouble(p->z);
			return true;
		}
		break;
	case libsumo::TYPE_COLOR:
		if (auto c = dynamic_cast<const libsumo::TraCIColor*>(r)) {
			out.writeUnsignedByte(c->r); out.writeUnsignedByte(c->g);
			out.writeUnsignedByte(c->b); out.writeUnsignedByte(c->a);
			return true;
		}
		break;
	case libsumo::TYPE_STRINGLIST:
		if (auto l = dynamic_cast<const libsumo::TraCIStringList*>(r)) { out.writeStringList(l->value); return true; }
		break;
	case libsumo::TYPE_COMPOUND:
		out.writeInt(2);
		out.writeUnsignedByte(libsumo::TYPE_STRING);
		if (auto p = dynamic_cast<const libsumo::TraCIRoadPosition*>(r)) {
			out.writeString(p->edgeID);
			out.writeUnsignedByte(libsumo::TYPE_DOUBLE);
			out.writeDouble(p->pos);
			return true;
		}
		if (auto l = dynamic_cast<const libsumo::TraCIStringList*>(r)) {
			if (l->value.size() == 2) {
				out.writeString(l->value[0]);
				out.writeUnsignedByte(libsumo::TYPE_STRING);
				out.writeString(l->value[1]);
				return true;
			}
		}
		break;
	default:
		break;
	}
	return false;
}

// One subscription answer block, as SUMO sends it after a step; false if SUMO sent none.
bool writeBlock(std::vector<unsigned char>& out, const ScriptSub& sub) {
	const libsumo::TraCIResults res = sub.domain->results(sub.objID);
	tcpip::Storage vars;
	int n = 0;
	for (const SubVar& v : sub.vars) {
		auto it = res.find(v.var);
		if (it == res.end()) continue;
		tcpip::Storage value;
		if (!writeValue(value, v, it->second.get())) continue;
		vars.writeUnsignedByte(v.var);
		vars.writeUnsignedByte(libsumo::RTYPE_OK);
		vars.writeStorage(value);
		n++;
	}
	if (n == 0) return false;
	tcpip::Storage block;
	block.writeUnsignedByte(sub.domain->subscribeCmd + 0x10);
	block.writeString(sub.objID);
	block.writeUnsignedByte(n);
	block.writeStorage(vars);
	tcpip::Storage framed;
	framed.writeUnsignedByte(0);
	framed.writeInt(1 + 4 + (int)block.size());
	framed.writeStorage(block);
	out.insert(out.end(), framed.begin(), framed.end());
	return true;
}

fixs::TraciResult subscribe(int cmdID, const std::string& objID, const std::vector<unsigned char>& payload) {
	const Domain* domain = findDomain(cmdID);
	if (domain == nullptr) {
		return message(fixs::kTraciRefused, "subscriptions of domain " + hex(cmdID) + " are not relayed.");
	}
	double begin, end;
	std::vector<SubVar> vars;
	try {
		// double begin | double end | ubyte n | n x (ubyte var | ubyte hasParam | typed param)
		tcpip::Storage s(payload.data(), (int)payload.size());
		begin = s.readDouble();
		end = s.readDouble();
		for (int n = s.readUnsignedByte(); n > 0; n--) {
			SubVar v{ s.readUnsignedByte(), 0, -1, {} };
			if (s.readUnsignedByte()) {
				const unsigned start = s.position();
				skipTyped(s, s.readUnsignedByte());
				v.param.assign(payload.begin() + start, payload.begin() + s.position());
			}
			vars.push_back(v);
		}
	}
	catch (const std::exception& e) {
		return message(fixs::kTraciRefused, std::string("malformed relayed subscription: ") + e.what());
	}

	auto known = std::find_if(scriptSubs.begin(), scriptSubs.end(), [&](const ScriptSub& s) {
		return s.domain == domain && s.objID == objID;
	});
	if (vars.empty()) {
		// Forgotten here only: SUMO would drop TrafficLayer's own subscription of the same object too.
		if (known != scriptSubs.end()) scriptSubs.erase(known);
		fixs::TraciResult r;
		r.status = fixs::kTraciOk;
		return r;
	}

	try {
		libsumo::TraCIResults params;
		std::vector<int> varIDs;
		for (SubVar& v : vars) {
			const std::string why = probeType(cmdID, objID, v);
			if (!why.empty()) return message(fixs::kTraciRefused, why);
			if (!v.param.empty()) params[v.var] = paramResult(v.param);
			varIDs.push_back(v.var);
		}
		domain->subscribe(objID, varIDs, begin, end, params);
	}
	catch (const std::exception& e) {
		return message(fixs::kTraciError, e.what());
	}

	if (known == scriptSubs.end()) {
		scriptSubs.push_back(ScriptSub{ domain, objID, vars });
		known = scriptSubs.end() - 1;
	}
	else {
		for (const SubVar& v : vars) {
			auto same = std::find_if(known->vars.begin(), known->vars.end(),
			                         [&](const SubVar& o) { return o.var == v.var; });
			if (same == known->vars.end()) known->vars.push_back(v);
			else *same = v;
		}
	}
	fixs::TraciResult r;
	r.status = fixs::kTraciOk;
	writeBlock(r.body, *known);
	return r;
}

// int count | count x block, as after SUMO's step answer.
fixs::TraciResult subscriptionResults() {
	std::vector<unsigned char> blocks;
	int n = 0;
	for (const ScriptSub& sub : scriptSubs) {
		if (writeBlock(blocks, sub)) n++;
	}
	tcpip::Storage head;
	head.writeInt(n);
	fixs::TraciResult r;
	r.status = fixs::kTraciOk;
	r.body.assign(head.begin(), head.end());
	r.body.insert(r.body.end(), blocks.begin(), blocks.end());
	return r;
}

#endif

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
	if (varID == kTraciSubscriptionVar) {
		if (cmdID == libsumo::CMD_SIMSTEP) return subscriptionResults();
		if (isVariableSubscribe(cmdID)) return subscribe(cmdID, objID, payload);
	}
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
