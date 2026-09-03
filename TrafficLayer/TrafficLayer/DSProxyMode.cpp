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
#include <fstream>
#include <chrono>

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
    // VirEnvHelper places an RS_C traffic object only if vehicleClass matches a
    // known slot (~VirEnvHelper.cpp L479: "car"/"passenger"/"private" -> car,
    // "truck" -> truck); an empty/unknown class is SILENTLY DROPPED (no else),
    // so DSProxy traffic never appeared in CarMaker. The SUMO path sets this
    // from SUMO's vehicle class; here we derive it from the VISSIM type number
    // (VISSIM defaults: 100 Car, 200 HGV, 300 Bus). Map HGV -> truck, everything
    // else -> passenger so background traffic is actually placed.
    out.vehicleClass = (v.VehicleType >= 200 && v.VehicleType < 300) ? "truck" : "passenger";
    out.speed = static_cast<float>(v.Speed);
    out.positionX = static_cast<float>(v.Position_X);
    out.positionY = static_cast<float>(v.Position_Y);
    out.positionZ = static_cast<float>(v.Position_Z);
    // PTV DrivingSimulatorProxy.h: Orient_Heading is "in radians, eastbound = zero,
    // northbound = +Pi/2" (CCW from East). The FIXS protocol heading is degrees,
    // north = 0, increasing clockwise (VirEnvHelper.cpp L613). Convert between them:
    // compass_deg = 90 - math_deg.  (East: 90-0=90deg; North: 90-90=0deg.)
    {
        const float PI = 3.14159265358979f;
        float hdg = 90.0f - static_cast<float>(v.Orient_Heading) * 180.0f / PI;
        while (hdg < 0.0f)    hdg += 360.0f;
        while (hdg >= 360.0f) hdg -= 360.0f;
        out.heading = hdg;
    }
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
    // Inverse of toVehFull's heading conversion: FIXS heading (deg, north=0, CW)
    // -> VISSIM Orient_Heading (rad, east=0, CCW). VISSIM normalizes the range.
    {
        const float PI = 3.14159265358979f;
        ego.Orient_Heading = (90.0f - v.heading) * PI / 180.0f;
    }
    ego.Orient_Pitch = v.grade;
    ego.Speed = v.speed;
    // The ego's length/appearance in VISSIM comes entirely from its VehicleType
    // (PTV Simulator_Veh_Data has no length field). v.type carries
    // CarMakerSetup.EgoType; if unset/non-numeric, default to the network's car
    // type (100) so the ego renders as a car instead of a default long vehicle.
    {
        int egoType = 100;
        try { if (!v.type.empty()) egoType = std::stoi(v.type); } catch (...) {}
        if (egoType <= 0) egoType = 100;
        ego.VehicleType = egoType;
    }
    ego.ControlledByVissim = false;
    // VehicleID + Create flag are managed by the caller (CreateID round-trip)
    return ego;
}

// One app socket per (subscription, port) tuple. ConfigHelper's
// ApplicationSetup.VehicleSubscription is a list of tuples
//   (type, attr-map, ips, ports)
// and each port within a tuple becomes its own AppSocketConfig — so a
// single subscription listing 3 ports expands to 3 entries here. The
// legacy single-client path (CarMaker dyno alone) is just the N=1 case.
//
// The egoId field (set when subType == "ego") is used on the RECV path:
// an inbound VehFullData_t whose id matches is the ego pose inject for
// DSProxy's SetDriverVehicles. It is also used on the SEND path to
// re-stamp the ego's VISSIM-assigned numeric id back to the client-side
// name so the consumer recognizes and skips its own ego.
struct AppSocketConfig {
    std::string ip;
    int port = 0;
    std::string subType;
    std::unordered_set<std::string> subscribedIds;
    std::string egoId;          // set iff subType == "ego" (first id)

    // Populated during listener setup / accept; -1 means "not yet".
    int listener = -1;
    int clientSock = -1;
};

std::vector<AppSocketConfig> resolveAppSockets(const ConfigHelper& config) {
    std::vector<AppSocketConfig> out;
    if (!config.ApplicationSetup.EnableApplicationLayer) return out;

    for (const auto& sub : config.ApplicationSetup.VehicleSubscription) {
        const std::string& subType = std::get<0>(sub);
        const auto& attMap = std::get<1>(sub);   // SubAttMap_t -> {attribute: [values]}
        const auto& ips    = std::get<2>(sub);
        const auto& ports  = std::get<3>(sub);
        if (ips.empty() || ports.empty()) continue;

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

// Per-port outbound filter — the PHASE 3 routing rule. The default is to
// publish every vehicle to every subscribed client, which matches
// mainTrafficLayer's existing non-CarMaker behavior
// (`ENABLE_VEH_SIMULATOR == false`): there, subscription tuples are used
// only to know which (ip, port) pairs to serve.
//
// Only `type: vehicleType` is a real per-vehicle filter, because the YAML
// attribute `id: [...]` then literally lists the vehicle TYPE numbers
// (e.g. `id: ['100']` -> Cars only) and that is a deliberate subset.
//
// `type: ego` is deliberately NOT a publish filter. The ego's VISSIM
// VehicleID is assigned at run time (e.g. "7"), so it would never match
// `id: ['ego']`, and every existing FIXS scenario expects the full
// vehicle list on the ego subscription port. The ego id is used on the
// recv path (and for the outbound re-stamp) instead.
//
// When the XIL orchestrator (#117) lands, this (subscription, message) ->
// publish-or-skip decision becomes a row in its routing table; the body
// stays as-is.
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

    // Resolve the (optional) application sockets — one per (subscription,
    // port) tuple. When the list is empty we fall back to Stage A behavior
    // (pump VISSIM without publishing); any configured app socket triggers
    // the Stage B publish/receive loop.
    std::vector<AppSocketConfig> appSocks = resolveAppSockets(config);
    const bool appEnabled = !appSocks.empty();
    if (appEnabled) {
        printf("App sockets:        %zu subscription port(s)\n", appSocks.size());
        for (const auto& s : appSocks) {
            printf("  - port %d  type=%-12s  subscribed_ids=%zu  egoId=%s\n",
                   s.port, s.subType.c_str(), s.subscribedIds.size(),
                   s.egoId.empty() ? "(none)" : s.egoId.c_str());
        }
    } else {
        printf("App sockets:        disabled — pump mode only\n");
    }

    // The canonical ego id: the first non-empty egoId across all
    // subscriptions. One ego per DSProxy run; multi-ego is a refinement.
    std::string canonicalEgoId;
    for (const auto& s : appSocks) {
        if (!s.egoId.empty()) { canonicalEgoId = s.egoId; break; }
    }

    SocketHelper sockHelper;
    MsgHelper msgHelper;

    const bool relayDM = cfg.EnableDriverModelRelay;
    int dmSock = -1;
    int dmListener = -1;

    // Plan A (#172): when SynchronizeTrafficSignal is on, CarMaker's
    // VirtualEnvironment.lib opens a SECOND client socket to TrafficSignalPort
    // (CommonLib/VirEnvHelper.cpp). Every tick it recvs on that socket (expecting
    // signal data) and echoes its ego back on it. We serve that port here and
    // relay VISSIM signal states on it; the redundant ego is drained. Signals are
    // sent on this socket INSTEAD of the app/vehicle socket so each socket carries
    // exactly what the .lib's per-socket recv expects. TrafficSignalPort must
    // differ from every app/CarMaker port.
    const bool syncSignals = appEnabled && config.CarMakerSetup.SynchronizeTrafficSignal;
    const int signalPort = config.CarMakerSetup.TrafficSignalPort;
    int signalSock = -1;
    int signalListener = -1;

    auto bindListener = [](int port) -> int {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);    // idempotent if already initialized elsewhere
        int s = static_cast<int>(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
        if (s < 0) return -1;
        BOOL reuse = TRUE;
        setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(static_cast<u_short>(port));
        if (::bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
            fprintf(stderr, "ERROR: bind(%d) failed: %d\n", port, WSAGetLastError());
            closesocket(s);
            return -1;
        }
        if (::listen(s, 5) < 0) {
            fprintf(stderr, "ERROR: listen(%d) failed: %d\n", port, WSAGetLastError());
            closesocket(s);
            return -1;
        }
        return s;
    };

    // Open the listeners BEFORE VISSIM_Connect. With
    // EnableDriverModelRelay: true and EnableRealSim: true in the par-
    // file, the FIXS DriverModel tries to connect to TL on
    // TrafficSimulatorPort during VISSIM_Connect's own handshake. The
    // listener has to be up by then or DM's connect fails and VISSIM
    // aborts the DSProxy handshake.
    // Closes every listener opened so far. Used on each early-return path
    // below; the listener count is now data-driven, so the cleanup is too.
    auto closeListeners = [&]() {
        if (dmListener >= 0) { closesocket(dmListener); dmListener = -1; }
        if (signalListener >= 0) { closesocket(signalListener); signalListener = -1; }
        for (auto& s : appSocks) {
            if (s.listener >= 0) { closesocket(s.listener); s.listener = -1; }
        }
    };

    if (appEnabled) {
        if (relayDM) {
            dmListener = bindListener(config.SimulationSetup.TrafficSimulatorPort);
            if (dmListener < 0) { return 6; }
            printf("DriverModel listener bound on port %d\n",
                   config.SimulationSetup.TrafficSimulatorPort);
        }
        for (auto& s : appSocks) {
            s.listener = bindListener(s.port);
            if (s.listener < 0) {
                closeListeners();
                return 6;
            }
            printf("app listener bound on port %d\n", s.port);
        }

        if (syncSignals) {
            for (const auto& s : appSocks) {
                if (signalPort == s.port) {
                    fprintf(stderr,
                            "ERROR: TrafficSignalPort (%d) must differ from every app/CarMaker port (%d)\n",
                            signalPort, s.port);
                    closeListeners();
                    return 6;
                }
            }
            signalListener = bindListener(signalPort);
            if (signalListener < 0) {
                closeListeners();
                return 6;
            }
            printf("signal listener bound on port %d\n", signalPort);
        }
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
        closeListeners();
        return 4;
    }
    printf("VISSIM_Connect OK\n");

    // Accept clients. DM (if relayDM) is likely already queued from its
    // connect during VISSIM_Connect. The app clients connect after
    // VISSIM_Connect when the run_*.bat launches them.
    if (relayDM && dmListener >= 0) {
        sockaddr_in dmAddr{};
        int dmAddrLen = sizeof(dmAddr);
        dmSock = static_cast<int>(accept(dmListener, (sockaddr*)&dmAddr, &dmAddrLen));
        if (dmSock < 0) {
            fprintf(stderr, "ERROR: accept DM failed: %d\n", WSAGetLastError());
            return 6;
        }
        int nodelay = 1;
        setsockopt(dmSock, IPPROTO_TCP, TCP_NODELAY,
                   reinterpret_cast<const char*>(&nodelay), sizeof(nodelay));
        printf("DriverModel connected (sock=%d)\n", dmSock);
        // No handshake exchange — the patched FIXS DriverModel calls
        // disableServerTrigger() before initConnection, so it doesn't send
        // any "ready" bytes. The first per-tick recvData on dmSock will
        // get the DM's state from its first VISSIM callback.
    }
    // One accept per subscription port. Each listener is bound to its own
    // port, so clients may connect in any order; we simply walk the
    // configured order and wait for all of them before the tick loop.
    if (appEnabled) {
        printf("waiting for %zu app client(s) ...\n", appSocks.size());
        for (auto& s : appSocks) {
            if (s.listener < 0) continue;
            sockaddr_in appAddr{};
            int appAddrLen = sizeof(appAddr);
            s.clientSock = static_cast<int>(accept(s.listener, (sockaddr*)&appAddr, &appAddrLen));
            if (s.clientSock < 0) {
                fprintf(stderr, "ERROR: accept app client on port %d failed: %d\n",
                        s.port, WSAGetLastError());
                return 6;
            }
            printf("app client connected on port %d (sock=%d)\n", s.port, s.clientSock);
        }
        msgHelper.getConfig(const_cast<ConfigHelper&>(config));
    }
    if (syncSignals && signalListener >= 0) {
        sockaddr_in sigAddr{};
        int sigAddrLen = sizeof(sigAddr);
        signalSock = static_cast<int>(accept(signalListener, (sockaddr*)&sigAddr, &sigAddrLen));
        if (signalSock < 0) {
            fprintf(stderr, "ERROR: accept signal client failed: %d\n", WSAGetLastError());
            return 6;
        }
        int nodelay = 1;
        setsockopt(signalSock, IPPROTO_TCP, TCP_NODELAY,
                   reinterpret_cast<const char*>(&nodelay), sizeof(nodelay));
        printf("signal client connected (sock=%d)\n", signalSock);
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
    // XIL orchestrator refactor (#117) can absorb this body as a
    // code-move rather than a rewrite. Stage B+ adds the DriverModel relay
    // as a second FIXS-protocol client (alongside the app client) — DM is
    // just another endpoint exchanging VehFullData_t over SocketHelper, so
    // from the orchestrator's pub/sub matrix view it's identical to any
    // other client.
    for (int tick = 0; tick < totalTicks; ++tick) {
        const float simTime = static_cast<float>(tick) / cfg.SimulatorFrequency;

        // PHASE 1 — Advance source via DSProxy. Folds previous tick's
        // PHASE 7 (egos extracted from app recv) into this push.
        if (!proxy.setDriverVehicles(egos)) {
            std::wstring err = proxy.lastError();
            fwprintf(stderr, L"ERROR tick %d: SetDriverVehicles failed: %ls\n",
                     tick, err.c_str());
            rc = 5;
            break;
        }

        // PHASE 5+4 for the DM endpoint (interleaved between VISSIM PHASE 1
        // and PHASE 2 because DRIVER_DATA_TIME is called multiple times per
        // VISSIM tick; the second call hits send+recv even on tick 0. TL
        // must do this every tick — including tick 0 — or getTrafficVehicles
        // deadlocks):
        //   - sockHelper.recvData(dmSock, ...) = PHASE 5 (drain DM's per-
        //     tick state upload; DSProxy is canonical so the content is
        //     discarded).
        //   - sockHelper.sendData(dmSock, ...) = PHASE 4 (publish to DM
        //     the CAV behavior cmds queued from the previous tick's app
        //     PHASE 6 split).
        // SocketHelper / MsgHelper call shape is identical to the app-side
        // PHASE 4/5 below; only the destination socket changes.
        if (relayDM && dmSock > 0) {
            msgHelper.VehDataRecv_um.clear();
            msgHelper.TlsDataRecv_um.clear();
            msgHelper.DetDataRecv_um.clear();
            int dmRecvState = 0;
            float dmRecvTime = 0.0f;
            if (sockHelper.recvData(dmSock, &dmRecvState, &dmRecvTime, msgHelper) < 0) {
                fprintf(stderr, "ERROR tick %d: DM recvData failed\n", tick);
                rc = 9;
                break;
            }
            if (sockHelper.sendData(dmSock, 0, simTime, /*simState=*/1, msgHelper) < 0) {
                fprintf(stderr, "ERROR tick %d: DM sendData failed\n", tick);
                rc = 10;
                break;
            }
        }

        // PHASE 2 — Collect canonical state from DSProxy.
#ifdef RS_DEBUG
        auto _gtv0 = std::chrono::high_resolution_clock::now();
#endif
        const auto vehicles = proxy.getTrafficVehicles();
        const auto signals  = proxy.getSignalStates();
#ifdef RS_DEBUG
        {
            // --- RS_DEBUG perf: DSProxy round-trip cost (VISSIM advance + extract) and
            // the FULL per-tick interval. Bucketed every 100 ticks so the CSV write is cheap.
            double gtv_us = (double)std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - _gtv0).count();
            static auto s_prevTick = std::chrono::high_resolution_clock::now();
            auto nowT = std::chrono::high_resolution_clock::now();
            double tick_us = (double)std::chrono::duration_cast<std::chrono::microseconds>(nowT - s_prevTick).count();
            s_prevTick = nowT;
            static double s_accTick = 0.0, s_accGtv = 0.0; static long s_cnt = 0, s_lastBucket = -1;
            s_accTick += tick_us; s_accGtv += gtv_us; s_cnt++;
            long bucket = tick / 100;
            if (bucket != s_lastBucket) {
                static std::ofstream f("rs_timing_tl.csv");
                static bool h = (f << "tick,nVeh,avg_tick_us,avg_getTraffic_us,ticks" << std::endl, true);
                f << tick << "," << vehicles.size() << "," << (s_cnt ? s_accTick / s_cnt : 0.0) << ","
                  << (s_cnt ? s_accGtv / s_cnt : 0.0) << "," << s_cnt << std::endl;
                s_accTick = 0.0; s_accGtv = 0.0; s_cnt = 0; s_lastBucket = bucket;
            }
            // position dump (offsync debug)
            static std::ofstream s_rsVsPos("rs_vissim_pos.csv");
            static bool h2 = (s_rsVsPos << "tick,vissimId,vissimX,vissimY" << std::endl, true);
            for (const auto& dv : vehicles) s_rsVsPos << tick << "," << dv.VehicleID << "," << dv.Position_X << "," << dv.Position_Y << std::endl;
        }
#endif

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

        // PHASE 3 — Distribute. Translate the VISSIM state to the FIXS
        // protocol ONCE, then let each subscription port select from it via
        // publishesVehicle(). That per-port select is the routing table the
        // XIL orchestrator (#117) will own; with one subscribed port it
        // degenerates to "publish everything to the one client".
        if (appEnabled) {
            std::vector<VehFullData_t> vehFull;
            vehFull.reserve(vehicles.size());
            for (const auto& v : vehicles) {
                VehFullData_t vf = toVehFull(v);
                // Re-stamp the ego with its client-side id ("egoCm") so the .lib
                // recognizes and SKIPS its own ego. Without this the ego is sent
                // with its raw VISSIM id; the .lib (which skips by CarMakerSetup.
                // EgoId) does not match it and -- now that toVehFull sets a real
                // vehicleClass -- places it as a phantom RS_C object on top of the
                // ego (the "double vehicle"). VISSIM assigns the integer id, so
                // we map it back to the original "egoCm" here; wire-identical to
                // the SUMO path, whose ego already carries that id.
                if (egoVissimId != 0 && v.VehicleID == egoVissimId &&
                    !canonicalEgoId.empty()) {
                    vf.id = canonicalEgoId;
                }
                vehFull.push_back(vf);
            }

            std::vector<TrafficLightData_t> tlsAll;
            tlsAll.reserve(signals.size());
            for (const auto& s : signals) tlsAll.push_back(toTlsData(s));

            const uint8_t simState = 1;

            // PHASE 4 — Publish to every subscribed client via
            // SocketHelper::sendData. Same call shape as the legacy
            // mainTrafficLayer per-client send; we iterate appSocks instead
            // of actualClientSock.
            for (const auto& asock : appSocks) {
                if (asock.clientSock <= 0) continue;
                msgHelper.VehDataSend_um[asock.clientSock].clear();
                msgHelper.TlsDataSend_um[asock.clientSock].clear();
                msgHelper.DetDataSend_um[asock.clientSock].clear();

                for (const auto& vf : vehFull) {
                    if (publishesVehicle(asock, vf)) {
                        msgHelper.VehDataSend_um[asock.clientSock].push_back(vf);
                    }
                }
                // Where signals are published: on the dedicated signal socket when
                // SynchronizeTrafficSignal is on (the .lib recvs signals there), else
                // multiplexed onto the app/vehicle sockets (legacy single-socket
                // path). Signals are global — no per-port signal filter yet, so every
                // subscribed client sees the full table.
                if (!syncSignals) {
                    for (const auto& s : tlsAll) {
                        msgHelper.TlsDataSend_um[asock.clientSock].push_back(s);
                    }
                }

                if (sockHelper.sendData(asock.clientSock, 0, simTime, simState, msgHelper) < 0) {
                    fprintf(stderr, "ERROR tick %d: sendData(port %d) failed\n", tick, asock.port);
                    rc = 7;
                    break;
                }
            }
            if (rc != 0) break;

            // Plan A: relay signals on the dedicated socket the .lib opened. The
            // .lib BLOCKS on a recv from this socket every tick, so we send even
            // with zero signals -- an empty batch keeps the lockstep round-trip.
            if (syncSignals && signalSock > 0) {
                msgHelper.VehDataSend_um[signalSock].clear();
                msgHelper.TlsDataSend_um[signalSock].clear();
                msgHelper.DetDataSend_um[signalSock].clear();
                for (const auto& s : tlsAll) {
                    msgHelper.TlsDataSend_um[signalSock].push_back(s);
                }
                if (sockHelper.sendData(signalSock, 0, simTime, simState, msgHelper) < 0) {
                    fprintf(stderr, "ERROR tick %d: signal sendData failed\n", tick);
                    rc = 7;
                    break;
                }
            }

            // PHASE 5 — Receive from every app client (one round-trip per
            // client per tick).
            // PHASE 6 — Merge: split client-received vehicles by ownership.
            //   - ego → stored for next tick's PHASE 1 (DSProxy egos)
            //   - non-ego → queued for next tick's DM PHASE 4 send as
            //     CAV behavior commands
            // This split is exactly the vehicle-ownership routing the
            // orchestrator will formalize: ego.Pose is owned by DSProxy
            // (so we forward to it); non-ego.Intent is owned by the CAV
            // controller (so we forward to DriverModel which integrates).
            // #117 Stage C will add per-recv deadline policy here.
            egos.clear();
            if (relayDM && dmSock > 0) {
                msgHelper.VehDataSend_um[dmSock].clear();
                msgHelper.TlsDataSend_um[dmSock].clear();
                msgHelper.DetDataSend_um[dmSock].clear();
            }
            bool egoTaken = false;   // one ego inject per tick, whichever client sends it first
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
                    const bool isEgo = (!canonicalEgoId.empty() && v.id == canonicalEgoId);
                    if (isEgo) {
                        // Several clients may echo the same ego (e.g. an
                        // observer port mirroring what it received); only the
                        // first one this tick becomes the DSProxy inject.
                        if (egoTaken) continue;
                        Simulator_Veh_Data ego = egoFromMsg(v);
                        if (egoVissimId == 0) {
                            // Create the ego in VISSIM exactly ONCE. Resolving
                            // egoVissimId needs the CreateID round-trip (~2 ticks); if
                            // we re-sent Create every tick until then, VISSIM would
                            // spawn a DUPLICATE ego each tick -- the extras come back as
                            // traffic, never match egoVissimId, and get placed as RS_C
                            // objects overlapping the real ego (the "double ego"). After
                            // the first Create we wait (submit nothing) until resolved.
                            if (!egoPending) {
                                ego.Create = true;
                                ego.CreateID = egoCreateId;
                                egoPending = true;
                                egos.push_back(ego);
                                egoTaken = true;
                            }
                        } else {
                            ego.VehicleID = egoVissimId;
                            ego.Create = false;
                            egos.push_back(ego);
                            egoTaken = true;
                        }
                    } else if (relayDM && dmSock > 0) {
                        // CAV behavior command: relay to DriverModel as-is.
                        // DriverModel parses VehFullData_t and applies
                        // speedDesired / accelerationDesired in its next
                        // DRIVER_DATA_DESIRED_VELOCITY / _ACCELERATION callback.
                        msgHelper.VehDataSend_um[dmSock].push_back(v);
                    }
                }
            }
            if (rc != 0) break;

            // Plan A: the .lib echoes its ego on the signal socket too; drain it so
            // that socket's lockstep round-trip completes. The echo is a duplicate
            // of what the vehicle socket already delivered, so it is not merged.
            if (syncSignals && signalSock > 0) {
                msgHelper.VehDataRecv_um.clear();
                int sigState = 0;
                float sigTime = 0.0f;
                if (sockHelper.recvData(signalSock, &sigState, &sigTime, msgHelper) < 0) {
                    fprintf(stderr, "ERROR tick %d: signal recvData failed\n", tick);
                    rc = 8;
                    break;
                }
            }

            // PHASE 7 — folded: egos go to next tick's PHASE 1 (DSProxy);
            // CAV cmds (now in msgHelper.VehDataSend_um[dmSock]) go to
            // next tick's DM PHASE 4 send at the top of the loop.
        }

        if (tick - lastReportedTick >= 25) {
            printf("tick %5d: vehicles=%3zu signals=%3zu egos=%zu\n",
                   tick, vehicles.size(), signals.size(), egos.size());
            lastReportedTick = tick;
        }
    }

    // Shutdown — simState=0 to every connected client.
    if (appEnabled) {
        printf("sending shutdown signal to %zu app client(s) ...\n", appSocks.size());
        for (const auto& asock : appSocks) {
            if (asock.clientSock <= 0) continue;
            msgHelper.VehDataSend_um[asock.clientSock].clear();
            msgHelper.TlsDataSend_um[asock.clientSock].clear();
            msgHelper.DetDataSend_um[asock.clientSock].clear();
            sockHelper.sendData(asock.clientSock, 0, 0.0f, /*simState=*/0, msgHelper);
        }
        if (syncSignals && signalSock > 0) {
            msgHelper.VehDataSend_um[signalSock].clear();
            msgHelper.TlsDataSend_um[signalSock].clear();
            msgHelper.DetDataSend_um[signalSock].clear();
            sockHelper.sendData(signalSock, 0, 0.0f, /*simState=*/0, msgHelper);
        }
        if (relayDM && dmSock > 0) {
            msgHelper.VehDataSend_um[dmSock].clear();
            msgHelper.TlsDataSend_um[dmSock].clear();
            msgHelper.DetDataSend_um[dmSock].clear();
            sockHelper.sendData(dmSock, 0, 0.0f, /*simState=*/0, msgHelper);
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
