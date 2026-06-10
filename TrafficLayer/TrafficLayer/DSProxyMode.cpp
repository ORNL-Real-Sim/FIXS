#include "DSProxyMode.h"
#include "VissimDSProxyHelper.h"

#include "SocketHelper.h"
#include "MsgHelper.h"
#include "VehDataMsgDefs.h"

#include <cstdio>
#include <cstring>
#include <string>
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

// Honor ApplicationSetup.VehicleSubscription[0].port[0] as the canonical
// Stage B publish port. Full multi-subscription / multi-port routing comes
// in a follow-up; one port covers the CarMaker dyno + Python probe case.
struct AppSocketConfig {
    bool enabled = false;
    std::string ip;
    int port = 0;
    std::string egoId;   // first subscribed vehicle id, used as ego match key
};

AppSocketConfig resolveAppSocket(const ConfigHelper& config) {
    AppSocketConfig out;
    const auto& subs = config.ApplicationSetup.VehicleSubscription;
    if (!config.ApplicationSetup.EnableApplicationLayer || subs.empty()) {
        return out;
    }
    const auto& ips = std::get<2>(subs[0]);
    const auto& ports = std::get<3>(subs[0]);
    if (ips.empty() || ports.empty()) {
        return out;
    }
    out.enabled = true;
    out.ip = ips[0];
    out.port = ports[0];

    // SubAttMap_t -> {attribute: [values]}. Pick the first 'id' if present.
    const auto& attMap = std::get<1>(subs[0]);
    auto it = attMap.find("id");
    if (it != attMap.end() && !it->second.empty()) {
        out.egoId = it->second[0];
    }
    return out;
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

    // Resolve the (optional) application socket. When absent we fall back
    // to Stage A behavior (pump VISSIM without publishing). Configured app
    // socket triggers the Stage B publish/receive loop.
    const AppSocketConfig appSock = resolveAppSocket(config);
    if (appSock.enabled) {
        printf("App socket:         server on port %d (egoId=%s)\n",
               appSock.port, appSock.egoId.empty() ? "(none)" : appSock.egoId.c_str());
    } else {
        printf("App socket:         disabled — pump mode only\n");
    }

    SocketHelper sockHelper;
    MsgHelper msgHelper;
    int clientSock = -1;

    const bool relayDM = cfg.EnableDriverModelRelay;
    int dmSock = -1;
    int dmListener = -1;
    int appListener = -1;

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
    if (appSock.enabled) {
        if (relayDM) {
            dmListener = bindListener(config.SimulationSetup.TrafficSimulatorPort);
            if (dmListener < 0) { return 6; }
            printf("DriverModel listener bound on port %d\n",
                   config.SimulationSetup.TrafficSimulatorPort);
        }
        appListener = bindListener(appSock.port);
        if (appListener < 0) {
            if (dmListener >= 0) closesocket(dmListener);
            return 6;
        }
        printf("app listener bound on port %d\n", appSock.port);
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
        if (dmListener >= 0) closesocket(dmListener);
        if (appListener >= 0) closesocket(appListener);
        return 4;
    }
    printf("VISSIM_Connect OK\n");

    // Accept clients. DM (if relayDM) is likely already queued from its
    // connect during VISSIM_Connect. The app client connects after
    // VISSIM_Connect when the run_*.bat launches it.
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
    if (appSock.enabled && appListener >= 0) {
        sockaddr_in appAddr{};
        int appAddrLen = sizeof(appAddr);
        clientSock = static_cast<int>(accept(appListener, (sockaddr*)&appAddr, &appAddrLen));
        if (clientSock < 0) {
            fprintf(stderr, "ERROR: accept app client failed: %d\n", WSAGetLastError());
            return 6;
        }
        printf("app client connected (sock=%d)\n", clientSock);
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

        // PHASE 3 — Distribute to app client. Today the routing is "publish
        // every vehicle + every signal to the one configured client";
        // multi-port routing on the parallel #165 branch shows where this
        // becomes per-subscription filtering.
        // PHASE 4 — Publish to app client via SocketHelper::sendData.
        if (appSock.enabled && clientSock > 0) {
            msgHelper.VehDataSend_um[clientSock].clear();
            msgHelper.TlsDataSend_um[clientSock].clear();
            msgHelper.DetDataSend_um[clientSock].clear();

            for (const auto& v : vehicles) {
                msgHelper.VehDataSend_um[clientSock].push_back(toVehFull(v));
            }
            for (const auto& s : signals) {
                msgHelper.TlsDataSend_um[clientSock].push_back(toTlsData(s));
            }

            const float simTime = static_cast<float>(tick) / cfg.SimulatorFrequency;
            const uint8_t simState = 1;
            if (sockHelper.sendData(clientSock, 0, simTime, simState, msgHelper) < 0) {
                fprintf(stderr, "ERROR tick %d: sendData failed\n", tick);
                rc = 7;
                break;
            }

            // PHASE 5 — Receive from app client (one round-trip per tick).
            // #117 Stage C will add per-recv deadline policy here.
            msgHelper.VehDataRecv_um.clear();
            int recvSimState = 0;
            float recvSimTime = 0.0f;
            if (sockHelper.recvData(clientSock, &recvSimState, &recvSimTime, msgHelper) < 0) {
                fprintf(stderr, "ERROR tick %d: recvData failed\n", tick);
                rc = 8;
                break;
            }

            // PHASE 6 — Merge: split client-received vehicles by ownership.
            //   - ego → stored for next tick's PHASE 1 (DSProxy egos)
            //   - non-ego → queued for next tick's DM PHASE 4 send as
            //     CAV behavior commands
            // This split is exactly the vehicle-ownership routing the
            // orchestrator will formalize: ego.Pose is owned by DSProxy
            // (so we forward to it); non-ego.Intent is owned by the CAV
            // controller (so we forward to DriverModel which integrates).
            egos.clear();
            if (relayDM && dmSock > 0) {
                msgHelper.VehDataSend_um[dmSock].clear();
                msgHelper.TlsDataSend_um[dmSock].clear();
                msgHelper.DetDataSend_um[dmSock].clear();
            }
            for (const auto& kv : msgHelper.VehDataRecv_um) {
                const VehFullData_t& v = kv.second;
                const bool isEgo = (!appSock.egoId.empty() && v.id == appSock.egoId);
                if (isEgo) {
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
                        }
                    } else {
                        ego.VehicleID = egoVissimId;
                        ego.Create = false;
                        egos.push_back(ego);
                    }
                } else if (relayDM && dmSock > 0) {
                    // CAV behavior command: relay to DriverModel as-is.
                    // DriverModel parses VehFullData_t and applies
                    // speedDesired / accelerationDesired in its next
                    // DRIVER_DATA_DESIRED_VELOCITY / _ACCELERATION callback.
                    msgHelper.VehDataSend_um[dmSock].push_back(v);
                }
            }

            // PHASE 7 — folded: egos go to next tick's PHASE 1 (DSProxy);
            // CAV cmds (now in msgHelper.VehDataSend_um[dmSock]) go to
            // next tick's DM PHASE 4 send at the top of the loop.
        }

        if (tick - lastReportedTick >= 25) {
            printf("tick %5d: vehicles=%3zu signals=%3zu egos=%zu\n",
                   tick, vehicles.size(), signals.size(), egos.size());
            // [verify] record what TrafficLayer transmits for up to 3 background
            // vehicles: id, VISSIM type, and absolute position — so it can be
            // checked against the (matching) VISSIM/CarMaker coordinates.
            int shown = 0;
            for (const auto& v : vehicles) {
                if (egoVissimId != 0 && v.VehicleID == egoVissimId) continue;  // skip ego
                float fixsHdg = 90.0f - static_cast<float>(v.Orient_Heading) * 180.0f / 3.14159265358979f;
                while (fixsHdg < 0.0f) fixsHdg += 360.0f;
                printf("        bg veh id=%d type=%d pos=(%.1f, %.1f) vissim=%.3frad -> fixs=%.0fdeg\n",
                       v.VehicleID, v.VehicleType,
                       v.Position_X, v.Position_Y, v.Orient_Heading, fixsHdg);
                if (++shown >= 3) break;
            }
            lastReportedTick = tick;
        }
    }

    // Shutdown
    if (appSock.enabled && clientSock > 0) {
        printf("sending shutdown signal to clients ...\n");
        msgHelper.VehDataSend_um[clientSock].clear();
        msgHelper.TlsDataSend_um[clientSock].clear();
        msgHelper.DetDataSend_um[clientSock].clear();
        sockHelper.sendData(clientSock, 0, 0.0f, /*simState=*/0, msgHelper);
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
