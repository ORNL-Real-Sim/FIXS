#pragma once
//============================================================================
//  replay_trace  (#325) -- drive VirEnvCore through a RECORDED FIXS feed.
//----------------------------------------------------------------------------
//  The scripted scenarios in replay_core.cpp pin the core's decisions on four
//  steps and two vehicles. That catches a broken interpolation or a mis-ordered
//  spawn, and does not come close to the failure #325 names as the expensive one:
//  two bridges disagreeing about WHO OWNS THE EGO, which shows up on the tick an
//  id appears, disappears, or is skipped -- events a four-step script does not
//  contain and a real corridor contains thousands of.
//
//  So this reads a trace recorded off a live TrafficLayer
//  (tests/VirEnv/record_feed.py), drives the core through it, and emits ONE
//  digest per step. tests/VirEnv/replay_core.py --trace does the identical thing
//  in Python, and test_core_parity.py compares the two digest lists: the first
//  index that differs is the exact exchange where the two cores stopped agreeing.
//
//  Trace format (see record_feed.py; flat because this file links no JSON parser):
//    S,<simTime>,<simState>
//    V,<id>,<type>,<vehicleClass>,<x>,<y>,<z>,<heading>,<grade>,<lightIndicators>
//    T,<name>,<state>
//============================================================================

#include "../../CommonLib/VirEnvCore.h"
#include "../../CommonLib/FixsProtocol.h"
#include "MockVirEnvBackend.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace virenv {
namespace trace {

struct Exchange {
    double t = 0.0;
    int    state = 0;
    std::vector<VehFullData_t>   veh;
    std::vector<TrafficLightData_t> tls;
};

// ---------------------------------------------------------------------------
// A 64-bit FNV-1a digest, defined identically in tests/VirEnv/replay_core.py.
//
// A digest rather than the transcript itself because a 6500-exchange corridor
// trace produces millions of verb lines: comparing one 16-hex digest per step
// finds the FIRST step that differs just as exactly, and a run's output stays
// something a person can read. FNV-1a rather than a real hash because it has to be
// the same eight lines in both languages; there is no adversary here, only a diff.
// ---------------------------------------------------------------------------
inline std::uint64_t fnv1a64(const std::string& s) {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    for (unsigned char c : s) {
        h ^= (std::uint64_t)c;
        h *= 0x100000001b3ULL;
    }
    return h;
}

inline std::string hex16(std::uint64_t h) {
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long)h);
    return std::string(b);
}

inline std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    std::istringstream ss(line);
    while (std::getline(ss, cur, ',')) out.push_back(cur);
    return out;
}

inline bool readTrace(const std::string& path, std::vector<Exchange>& out) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "cannot open trace: " << path << "\n";
        return false;
    }
    std::string line;
    while (std::getline(f, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
        if (line.empty()) continue;
        std::vector<std::string> p = splitCsv(line);
        if (p[0] == "S") {
            Exchange e;
            e.t = std::stod(p[1]);
            e.state = std::stoi(p[2]);
            out.push_back(e);
        } else if (p[0] == "V" && !out.empty()) {
            VehFullData_t v{};
            v.id = p[1]; v.type = p[2]; v.vehicleClass = p[3];
            v.positionX = std::stof(p[4]); v.positionY = std::stof(p[5]);
            v.positionZ = std::stof(p[6]); v.heading = std::stof(p[7]);
            v.grade = std::stof(p[8]);
            v.lightIndicators = (uint16_t)std::stoi(p[9]);
            out.back().veh.push_back(v);
        } else if (p[0] == "T" && !out.empty()) {
            TrafficLightData_t t{};
            t.id = 0; t.name = p[1];
            // A SUMO state string cannot contain a comma, so the tail is the state
            // even if a junction name somehow did.
            for (std::size_t i = 2; i < p.size(); i++) {
                if (i > 2) t.state += ",";
                t.state += p[i];
            }
            out.back().tls.push_back(t);
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Replace backend handles with the vehicle id that owns them, and sort.
//
// Two things in a raw transcript carry no meaning and cannot match across the two
// implementations:
//
//  * WHICH HANDLE a vehicle got. The core spawns in the iteration order of its
//    received map -- unordered_map here (order unspecified by the standard),
//    an insertion-ordered dict in Python -- so when several vehicles appear in one
//    exchange they draw pool handles in different orders. The handle is opaque
//    either way; what must match is which VEHICLE got which pose.
//  * THE ORDER of verbs within one step. Same cause, and they are independent
//    writes -- Carla batches them before they reach the server.
//
// So every handle -- in the pose, lights and despawn arguments AND in the "-> N"
// a spawn reports -- is rewritten to its owning id, and the step is sorted. The
// spawn line matters as much as the rest: the mock hands handles back to a
// per-class pool on despawn, so the moment two vehicles leave in one exchange the
// two pools are ordered differently, and every LATER spawn reports a different
// number for the same vehicle. What survives is the decision: this vehicle, this
// pose, this step.
// ---------------------------------------------------------------------------
inline std::vector<std::string> canonicalise(
        const std::vector<std::string>& events,
        const std::unordered_map<VehHandle, std::string>& ownerOf) {
    std::vector<std::string> out;
    out.reserve(events.size());
    for (const std::string& e : events) {
        const std::size_t sp = e.find(' ');
        const std::string verb = (sp == std::string::npos) ? e : e.substr(0, sp);
        if (sp != std::string::npos &&
            (verb == "setVehiclePose" || verb == "setVehicleLights" || verb == "despawnVehicle")) {
            const std::string rest = e.substr(sp + 1);
            const std::size_t sp2 = rest.find(' ');
            const std::string hStr = (sp2 == std::string::npos) ? rest : rest.substr(0, sp2);
            const std::string tail = (sp2 == std::string::npos) ? "" : rest.substr(sp2);
            std::unordered_map<VehHandle, std::string>::const_iterator it =
                ownerOf.find((VehHandle)std::stoi(hStr));
            const std::string who = (it != ownerOf.end()) ? it->second : ("h" + hStr);
            out.push_back(verb + " " + who + tail);
        } else if (sp != std::string::npos && verb == "spawnVehicle") {
            const std::string rest = e.substr(sp + 1);
            const std::size_t arrow = rest.rfind(" -> ");
            if (arrow == std::string::npos || rest.substr(arrow + 4) == "kNoHandle") {
                out.push_back(e);            // pool exhausted: no handle to rewrite
            } else {
                const std::string hStr = rest.substr(arrow + 4);
                std::unordered_map<VehHandle, std::string>::const_iterator it =
                    ownerOf.find((VehHandle)std::stoi(hStr));
                const std::string who = (it != ownerOf.end()) ? it->second : ("h" + hStr);
                out.push_back(verb + " " + rest.substr(0, arrow) + " -> " + who);
            }
        } else {
            out.push_back(e);
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

struct Result {
    std::size_t exchanges = 0;
    int substeps = 1;
    std::string egoId;
    std::map<std::string, long long> counts;
    std::vector<std::string> digests;
};

inline bool replayTrace(const std::string& tracePath, int substeps,
                        const std::string& egoId, Result& res, long progressEvery = 1000) {
    std::vector<Exchange> steps;
    if (!readTrace(tracePath, steps)) return false;

    // Pools large enough that the corridor never exhausts them. A pool-full skip is
    // a real decision, but one driven by a pool size Carla does not have -- it
    // spawns lazily -- so letting it fire here would compare the mock, not the core.
    MockVirEnvBackend mock(100000, 20000, 5000);
    VirEnvCore core;
    core.setBackend(&mock);
    core.ENABLE_REALSIM = false;
    core.SYNCHRONIZE_TRAFFIC_SIGNAL = true;
    core.sendEgoFromCore = false;              // the Carla driver owns the send
    core.egoId_ = egoId;
    core.interpolateTraffic = (substeps > 1);
    core.trafficRefreshRate_ = fixs::kFeedPeriodS / substeps;
    core.Msg_c.VehicleMessageField_set = {
        "id", "type", "vehicleClass", "speed", "positionX", "positionY", "positionZ",
        "heading", "grade", "lightIndicators" };
    mock.setMockEgo(EgoState(), false);

    res.exchanges = steps.size();
    res.substeps = substeps;
    res.egoId = egoId;
    res.counts["spawn"] = res.counts["despawn"] = res.counts["pose"] = res.counts["tls"] = 0;

    const double dt = fixs::kFeedPeriodS / substeps;
    for (std::size_t i = 0; i < steps.size(); i++) {
        const Exchange& rec = steps[i];
        // A synthetic HOST clock, not the recorded traffic-simulator time: the core
        // tests its exchange boundary on the host clock, which starts at 0 and
        // reaches its first exchange one feed period in. The recorded time rides
        // along as simTimeRecv, exactly as it does on the wire.
        const double base = (double)i * fixs::kFeedPeriodS;
        for (int k = 0; k < substeps; k++) {
            const bool onUpdate = (k == 0);
            if (onUpdate) {
                core.Msg_c.clearRecvStorage();
                for (const VehFullData_t& v : rec.veh) core.Msg_c.VehDataRecv_um[v.id] = v;
                for (const TrafficLightData_t& t : rec.tls) core.Msg_c.TlsDataRecv_um[t.name] = t;
            }
            std::unordered_map<VehHandle, std::string> before;
            for (const std::pair<const std::string, VehHandle>& kv : core.mappedVehicles())
                before[kv.second] = kv.first;

            mock.clear();
            const char* err = nullptr;
            const int rc = core.processStep(base + fixs::kFeedPeriodS + k * dt,
                                            onUpdate, rec.state, (float)rec.t, &err);
            if (rc != 0) {
                std::cerr << "processStep failed at t=" << rec.t << ": "
                          << (err ? err : "?") << "\n";
                return false;
            }
            // A handle spawned this step was in the pool when the step began, so it
            // is not in `before`; a handle despawned this step is. The two sets are
            // disjoint, so before-over-after resolves every event unambiguously.
            std::unordered_map<VehHandle, std::string> owner;
            for (const std::pair<const std::string, VehHandle>& kv : core.mappedVehicles())
                owner[kv.second] = kv.first;
            for (const std::pair<const VehHandle, std::string>& kv : before)
                owner[kv.first] = kv.second;

            const std::vector<std::string> ev = canonicalise(mock.events(), owner);
            std::string joined;
            for (std::size_t n = 0; n < ev.size(); n++) {
                if (n) joined += "\n";
                joined += ev[n];
                if (ev[n].rfind("spawnVehicle", 0) == 0)        res.counts["spawn"]++;
                else if (ev[n].rfind("despawnVehicle", 0) == 0) res.counts["despawn"]++;
                else if (ev[n].rfind("setVehiclePose", 0) == 0) res.counts["pose"]++;
                else if (ev[n].rfind("syncTrafficLight", 0) == 0) res.counts["tls"]++;
            }
            res.digests.push_back(hex16(fnv1a64(joined)));
        }
        if (progressEvery > 0 && ((long)(i + 1) % progressEvery) == 0)
            std::cout << "[replay] " << (i + 1) << "/" << steps.size() << " exchanges\n";
    }
    return true;
}

// Minimal JSON writer -- the same four keys replay_core.py --digest-out emits, so
// test_core_parity.py reads both with one loader. Hand-written because this target
// links no JSON library and the shape is fixed.
inline bool writeDigestJson(const std::string& path, const Result& r) {
    std::ofstream f(path);
    if (!f.is_open()) { std::cerr << "cannot write " << path << "\n"; return false; }
    f << "{\n";
    f << " \"exchanges\": " << r.exchanges << ",\n";
    f << " \"substeps\": " << r.substeps << ",\n";
    f << " \"egoId\": \"" << r.egoId << "\",\n";
    f << " \"counts\": {";
    bool first = true;
    for (const std::pair<const std::string, long long>& kv : r.counts) {
        if (!first) f << ", ";
        f << "\"" << kv.first << "\": " << kv.second;
        first = false;
    }
    f << "},\n";
    f << " \"digests\": [\n";
    for (std::size_t i = 0; i < r.digests.size(); i++)
        f << "  \"" << r.digests[i] << "\"" << (i + 1 < r.digests.size() ? "," : "") << "\n";
    f << " ]\n}\n";
    return true;
}

}  // namespace trace
}  // namespace virenv
