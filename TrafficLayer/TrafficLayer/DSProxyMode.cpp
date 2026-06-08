#include "DSProxyMode.h"
#include "VissimDSProxyHelper.h"

#include "SocketHelper.h"
#include "MsgHelper.h"
#include "VehDataMsgDefs.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_set>
#include <vector>

namespace FIXS {
namespace DSProxy {

namespace {

// Translate VISSIM's signal-state enum to a single SUMO-style char so
// downstream consumers (CarMaker via VirtualEnvironment.lib, Carla,
// Python) keep their existing TLS-char handling. Mapping is per-signal-
// group; SUMO usually concatenates groups per intersection but VISSIM
// gives us individual (controller, group) pairs and we expose one
// TrafficLightData_t per pair.
char signalStateToSumoChar(int sigState) {
    switch (sigState) {
        case 1:  return 'r';   // Red
        case 2:  return 'u';   // Red+Amber (no SUMO equivalent; reuse 'u')
        case 3:  return 'G';   // Green
        case 4:  return 'y';   // Amber
        case 5:  return 'o';   // Off (black)
        case 6:  return '?';   // Undefined
        case 7:  return 'Y';   // Flashing Amber
        case 8:  return 'R';   // Flashing Red
        case 9:  return 'G';   // Flashing Green (collapse to G)
        case 10: return '?';   // Alternating Red/Green
        case 11: return 'G';   // Green+Amber (collapse to G)
    }
    return '?';
}

VehFullData_t toVehFull(const VISSIM_Veh_Data& v) {
    VehFullData_t out{};
    out.id = std::to_string(v.VehicleID);
    out.type = std::to_string(v.VehicleType);
    out.speed = static_cast<float>(v.Speed);
    out.positionX = static_cast<float>(v.Position_X);
    out.positionY = static_cast<float>(v.Position_Y);
    out.positionZ = static_cast<float>(v.Position_Z);
    out.heading = static_cast<float>(v.Orient_Heading);
    out.linkId = std::to_string(v.LinkID);
    out.laneId = v.LaneIndex;
    if (v.LeadingVehicleID > 0) {
        out.hasPrecedingVehicle = 1;
        out.precedingVehicleId = std::to_string(v.LeadingVehicleID);
    }
    out.activeLaneChange = static_cast<int8_t>(v.TurningIndicator);
    // grade is the link gradient at front per PTV doc §1.2 — Orient_Pitch
    // for DS-controlled vehicles reports link gradient, not vehicle pitch.
    out.grade = static_cast<float>(v.Orient_Pitch);
    return out;
}

TrafficLightData_t toTlsData(const VISSIM_Sig_Data& s) {
    TrafficLightData_t out{};
    out.id = static_cast<uint16_t>(s.SignalGroupID);
    out.name = std::to_string(s.ControllerID) + "_" + std::to_string(s.SignalGroupID);
    out.state = std::string(1, signalStateToSumoChar(s.SignalState));
    return out;
}

Simulator_Veh_Data egoFromMsg(const VehFullData_t& v) {
    Simulator_Veh_Data ego{};
    ego.Position_X = v.positionX;
    ego.Position_Y = v.positionY;
    ego.Position_Z = v.positionZ;
    ego.Orient_Heading = v.heading;
    ego.Orient_Pitch = v.grade;
    ego.Speed = v.speed;
    ego.ControlledByVissim = false;
    // VehicleID + Create flag are managed by the caller (CreateID round-trip)
    return ego;
}

// One per-port subscription entry. ConfigHelper's
// ApplicationSetup.VehicleSubscription is a list of tuples
//   (type, attr-map, ips, ports)
// and each port within a tuple becomes its own AppSocketConfig — so a
// single subscription with 3 ports gets expanded to 3 entries here.
//
// Filtering rule (per-tick):
//   - subType=="all" -> publish every vehicle to this socket
//   - subType=="ego" or "vehicleType" -> only publish vehicles whose id
//     or type matches one of `subscribedIds`
//   - otherwise -> fall back to publish-all (link / point not yet
//     filtered specifically; revisit when a real scenario needs it)
//
// The egoId field (when subType=="ego") is also used on the recv path:
// any inbound VehFullData_t with id == egoId on ANY socket is treated
// as the canonical ego inject for DSProxy SetDriverVehicles.
struct AppSocketConfig {
    int port = 0;
    std::string ip;
    std::string subType;
    std::unordered_set<std::string> subscribedIds;
    std::string egoId;          // set iff subType == "ego" (first id)

    // Populated after socket setup; -1 means "not connected yet".
    int listener = -1;
    int clientSock = -1;
};

std::vector<AppSocketConfig> resolveAppSockets(const ConfigHelper& config) {
    std::vector<AppSocketConfig> out;
    if (!config.ApplicationSetup.EnableApplicationLayer) return out;
    for (const auto& sub : config.ApplicationSetup.VehicleSubscription) {
        const std::string& subType = std::get<0>(sub);
        const auto& attMap = std::get<1>(sub);
        const auto& ips    = std::get<2>(sub);
        const auto& ports  = std::get<3>(sub);
        if (ports.empty() || ips.empty()) continue;

        std::unordered_set<std::string> subscribedIds;
        std::string firstId;
        auto idIt = attMap.find("id");
        if (idIt != attMap.end()) {
            for (const auto& id : idIt->second) {
                subscribedIds.insert(id);
                if (firstId.empty()) firstId = id;
            }
        }

        for (size_t i = 0; i < ports.size(); ++i) {
            AppSocketConfig cfg;
            cfg.port = ports[i];
            cfg.ip = (i < ips.size()) ? ips[i] : ips[0];
            cfg.subType = subType;
            cfg.subscribedIds = subscribedIds;
            if (subType == "ego") cfg.egoId = firstId;
            out.push_back(cfg);
        }
    }
    return out;
}

// Per-port outbound filter — the canonical PHASE 3 routing-rule
// implementation (see doc/fixs_tick_flow.md). The default is to publish
// every vehicle to every subscribed client — matches mainTrafficLayer's
// existing non-CarMaker behavior (`ENABLE_VEH_SIMULATOR == false`,
// ConfigHelper just uses subscription tuples to know which (ip,port)
// pairs to serve).
//
// When the XIL orchestrator (#117) lands, this function's per-port
// (subscription, message) → publish-or-skip decision becomes a row in
// the orchestrator's routing table; the body stays the same.
//
// Only `type: vehicleType` is treated as a real per-vehicle filter,
// because the YAML attribute `id: [...]` then literally lists the vehicle
// TYPE numbers (e.g. `id: ['100']` -> only Cars) and that's a deliberate
// subset selection.
//
// `type: ego` is NOT a publish filter: the ego's VISSIM VehicleID is
// assigned at simulation time (e.g. "7"), it'd never match `id: ['ego']`,
// and existing FIXS scenarios expect the full vehicle list on the ego
// subscription port. The ego id is used on the recv path instead.
bool publishesVehicle(const AppSocketConfig& sock, const VehFullData_t& v) {
    if (sock.subType == "vehicleType" && !sock.subscribedIds.empty()) {
        return sock.subscribedIds.count(v.type) > 0;
    }
    return true;
}

} // namespace

int runDSProxyMode(const ConfigHelper& config) {
    setvbuf(stdout, nullptr, _IONBF, 0);

    const auto& cfg = config.VissimSetup;

    printf("\n=== DSProxy mode (Stage B, issue #158) ===\n");
    printf("VissimVersion:      %d\n", cfg.VissimVersion);
    printf("SimulatorFrequency: %d Hz\n", cfg.SimulatorFrequency);
    printf("VisibilityRadius:   %.2f m\n", cfg.VisibilityRadius);
    printf("NetworkFile:        %s\n", cfg.NetworkFile.c_str());

    if (cfg.NetworkFile.empty()) {
        fprintf(stderr, "ERROR: VissimSetup.NetworkFile is required when EnableDSProxy: true\n");
        return 2;
    }

    std::string dllPath = cfg.DllPath.empty()
        ? defaultDllPathForVissim(cfg.VissimVersion)
        : cfg.DllPath;
    printf("DSProxy DLL:        %s\n", dllPath.c_str());

    VissimDSProxy proxy;
    if (!proxy.load(dllPath)) {
        fprintf(stderr, "ERROR: failed to load %s\n", dllPath.c_str());
        return 3;
    }

    // Resolve all application sockets. When the list is empty we fall back
    // to Stage A behavior (pump VISSIM without publishing). Each entry
    // gets its own server socket and per-tick filter.
    std::vector<AppSocketConfig> appSocks = resolveAppSockets(config);
    if (appSocks.empty()) {
        printf("App sockets:        none — pump mode only\n");
    } else {
        printf("App sockets:        %zu subscription port(s)\n", appSocks.size());
        for (const auto& s : appSocks) {
            printf("  - port %d  type=%-12s  subscribed_ids=%zu  egoId=%s\n",
                   s.port, s.subType.c_str(), s.subscribedIds.size(),
                   s.egoId.empty() ? "(none)" : s.egoId.c_str());
        }
    }
    // Cache the first non-empty egoId across all subscriptions as the
    // canonical ego id. Multi-ego support is a follow-up.
    std::string canonicalEgoId;
    for (const auto& s : appSocks) {
        if (!s.egoId.empty()) { canonicalEgoId = s.egoId; break; }
    }

    const unsigned short versionNo = connectVersionNo(cfg.VissimVersion);

    printf("calling VISSIM_Connect (versionNo=%u) ...\n", versionNo);
    const bool connected = proxy.connect(
        versionNo, cfg.NetworkFile,
        static_cast<unsigned short>(cfg.SimulatorFrequency),
        cfg.VisibilityRadius,
        static_cast<unsigned short>(cfg.MaxSimulatorVeh),
        static_cast<unsigned short>(cfg.MaxSimulatorPed),
        static_cast<unsigned short>(cfg.MaxSimulatorDet),
        static_cast<unsigned short>(cfg.MaxTotalVeh),
        static_cast<unsigned short>(cfg.MaxVissimPed),
        static_cast<unsigned short>(cfg.MaxVissimSigGrp));
    if (!connected) {
        std::wstring err = proxy.lastError();
        fwprintf(stderr, L"ERROR: VISSIM_Connect failed: %ls\n",
                 err.empty() ? L"(no detail)" : err.c_str());
        return 4;
    }
    printf("VISSIM_Connect OK\n");

    // App socket setup. One server socket per subscription port; we wait
    // for ALL clients before entering the tick loop. SocketHelper handles
    // the bind/listen/accept loop and returns clientSock[] in port order.
    SocketHelper sockHelper;
    MsgHelper msgHelper;

    if (!appSocks.empty()) {
        std::vector<int> selfPorts;
        selfPorts.reserve(appSocks.size());
        for (const auto& s : appSocks) selfPorts.push_back(s.port);
        sockHelper.socketSetup(selfPorts);
        sockHelper.disableServerTrigger();
        sockHelper.disableWaitClientTrigger();

        printf("waiting for %zu client(s) ...\n", appSocks.size());
        if (sockHelper.initConnection("TrafficLayer.err") < 0) {
            fprintf(stderr, "ERROR: initConnection failed\n");
            proxy.disconnect();
            return 6;
        }
        if (sockHelper.clientSock.size() != appSocks.size()) {
            fprintf(stderr, "ERROR: expected %zu clients, got %zu\n",
                    appSocks.size(), sockHelper.clientSock.size());
            proxy.disconnect();
            return 6;
        }
        for (size_t i = 0; i < appSocks.size(); ++i) {
            appSocks[i].clientSock = sockHelper.clientSock[i];
            printf("client connected on port %d (sock=%d)\n",
                   appSocks[i].port, appSocks[i].clientSock);
        }
        msgHelper.getConfig(const_cast<ConfigHelper&>(config));
    }

    const double endTime = config.SimulationSetup.SimulationEndTime;
    const int totalTicks = static_cast<int>(endTime * cfg.SimulatorFrequency);
    printf("Running %d ticks (end_time=%.1fs at %d Hz)\n",
           totalTicks, endTime, cfg.SimulatorFrequency);

    std::vector<Simulator_Veh_Data> egos;          // pushed each tick
    int egoVissimId = 0;                            // assigned by VISSIM via CreateID round-trip
    const int egoCreateId = 4711;                   // matches probe convention
    bool egoPending = false;                        // we've been given a pose to push but no VissimId yet

    int lastReportedTick = -25;
    int rc = 0;

    // Per-tick loop. Phase labels match doc/fixs_tick_flow.md so the future
    // XIL orchestrator refactor (#117) can absorb this body as a code-move
    // rather than a rewrite. The multi-port per-subscription filter
    // (publishesVehicle / per-port loop in PHASE 3+4) is *exactly* the
    // routing-table pattern the orchestrator will formalize — same
    // SocketHelper / MsgHelper call shape as the legacy mainTrafficLayer
    // while loop, just with multiple clients instead of one.
    for (int tick = 0; tick < totalTicks; ++tick) {
        // PHASE 1 — Advance source via DSProxy. Folds previous tick's
        // PHASE 7 (commands) into this tick's push.
        if (!proxy.setDriverVehicles(egos)) {
            std::wstring err = proxy.lastError();
            fwprintf(stderr, L"ERROR tick %d: SetDriverVehicles failed: %ls\n",
                     tick, err.c_str());
            rc = 5;
            break;
        }

        // PHASE 2 — Collect state from source.
        const auto vehicles = proxy.getTrafficVehicles();
        const auto signals  = proxy.getSignalStates();

        // Resolve egoVissimId once the Create round-trips. Intra-PHASE-2
        // bookkeeping.
        if (egoVissimId == 0 && egoPending) {
            for (const auto& v : vehicles) {
                if (v.CreateID == egoCreateId && !v.ControlledByVissim) {
                    egoVissimId = v.VehicleID;
                    printf("ego registered: VISSIM VehicleID=%d\n", egoVissimId);
                    break;
                }
            }
        }

        // PHASE 3 — Distribute to clients per subscription (pre-translate
        // once to FIXS protocol; per-port filter applied below). This is
        // exactly the routing-table responsibility the orchestrator will
        // own; the per-port `publishesVehicle` filter is the per-port
        // entry in that table.
        std::vector<VehFullData_t> vehFull;
        vehFull.reserve(vehicles.size());
        for (const auto& v : vehicles) vehFull.push_back(toVehFull(v));

        std::vector<TrafficLightData_t> tlsAll;
        tlsAll.reserve(signals.size());
        for (const auto& s : signals) tlsAll.push_back(toTlsData(s));

        const float simTime = static_cast<float>(tick) / cfg.SimulatorFrequency;
        const uint8_t simState = 1;

        // PHASE 4 — Publish to each subscribed client via SocketHelper
        // (same call shape as legacy mainTrafficLayer's per-client send;
        // we're just iterating appSocks instead of actualClientSock).
        for (const auto& asock : appSocks) {
            if (asock.clientSock <= 0) continue;
            msgHelper.VehDataSend_um[asock.clientSock].clear();
            msgHelper.TlsDataSend_um[asock.clientSock].clear();
            msgHelper.DetDataSend_um[asock.clientSock].clear();

            for (const auto& v : vehFull) {
                if (publishesVehicle(asock, v)) {
                    msgHelper.VehDataSend_um[asock.clientSock].push_back(v);
                }
            }
            // Signals are global — every subscribed client gets the full
            // table. Per-port signal filtering can come later if needed.
            for (const auto& s : tlsAll) {
                msgHelper.TlsDataSend_um[asock.clientSock].push_back(s);
            }

            if (sockHelper.sendData(asock.clientSock, 0, simTime, simState, msgHelper) < 0) {
                fprintf(stderr, "ERROR tick %d: sendData(port %d) failed\n", tick, asock.port);
                rc = 7;
                break;
            }
        }
        if (rc != 0) break;

        // PHASE 5 — Receive responses from each client via SocketHelper
        // (same call shape as legacy mainTrafficLayer's per-client recv).
        // PHASE 6 — Merge: any inbound vehicle whose id matches the
        // canonical egoId is the DSProxy ego inject for the next tick's
        // PHASE 1. Stored egos become the next setDriverVehicles input.
        // #117 Stage C will add per-recv deadline policy here.
        egos.clear();
        for (const auto& asock : appSocks) {
            if (asock.clientSock <= 0) continue;
            msgHelper.VehDataRecv_um.clear();
            int recvSimState = 0;
            float recvSimTime = 0.0f;
            if (sockHelper.recvData(asock.clientSock, &recvSimState, &recvSimTime, msgHelper) < 0) {
                fprintf(stderr, "ERROR tick %d: recvData(port %d) failed\n", tick, asock.port);
                rc = 8;
                break;
            }
            for (const auto& kv : msgHelper.VehDataRecv_um) {
                const VehFullData_t& v = kv.second;
                if (canonicalEgoId.empty() || v.id != canonicalEgoId) continue;
                Simulator_Veh_Data ego = egoFromMsg(v);
                if (egoVissimId == 0) {
                    ego.Create = true;
                    ego.CreateID = egoCreateId;
                    egoPending = true;
                } else {
                    ego.VehicleID = egoVissimId;
                    ego.Create = false;
                }
                egos.push_back(ego);
                break;     // single ego per tick; multi-ego is a refinement
            }
            if (rc != 0) break;
        }
        if (rc != 0) break;

        if (tick - lastReportedTick >= 25) {
            printf("tick %5d: vehicles=%3zu signals=%3zu egos=%zu\n",
                   tick, vehicles.size(), signals.size(), egos.size());
            lastReportedTick = tick;
        }
    }

    // Shutdown — signal every connected client.
    if (!appSocks.empty()) {
        printf("sending shutdown signal to %zu client(s) ...\n", appSocks.size());
        for (const auto& asock : appSocks) {
            if (asock.clientSock <= 0) continue;
            msgHelper.VehDataSend_um[asock.clientSock].clear();
            msgHelper.TlsDataSend_um[asock.clientSock].clear();
            msgHelper.DetDataSend_um[asock.clientSock].clear();
            sockHelper.sendData(asock.clientSock, 0, 0.0f, /*simState=*/0, msgHelper);
        }
        sockHelper.socketShutdown();
    }

    printf("calling VISSIM_Disconnect ...\n");
    proxy.disconnect();
    printf("=== DSProxy mode done (rc=%d) ===\n", rc);
    return rc;
}

} // namespace DSProxy
} // namespace FIXS
