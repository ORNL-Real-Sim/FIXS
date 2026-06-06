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

    for (int tick = 0; tick < totalTicks; ++tick) {
        const float simTime = static_cast<float>(tick) / cfg.SimulatorFrequency;

        // 1. Send pending DriverModel commands (CAV behavior overrides from
        //    the previous tick's app-side recv). DM's per-vehicle callback
        //    inside the next VISSIM tick (which proxy.setDriverVehicles
        //    triggers) does recv-from-TL FIRST then send-state-to-TL, so we
        //    must have data ready in the socket before VISSIM ticks or DM's
        //    recv blocks. On tick 0 the buffer is empty (no prior app recv),
        //    which DM is fine with.
        if (relayDM && dmSock > 0) {
            if (sockHelper.sendData(dmSock, 0, simTime, /*simState=*/1, msgHelper) < 0) {
                fprintf(stderr, "ERROR tick %d: DM sendData failed\n", tick);
                rc = 10;
                break;
            }
        }

        // 2. Push DS egos and advance VISSIM
        if (!proxy.setDriverVehicles(egos)) {
            std::wstring err = proxy.lastError();
            fwprintf(stderr, L"ERROR tick %d: SetDriverVehicles failed: %ls\n",
                     tick, err.c_str());
            rc = 5;
            break;
        }

        // 3. Drain DriverModel state-up. DSProxy is the canonical state
        //    source for downstream consumers so this data is discarded —
        //    but we have to recv it so DM's send buffer doesn't back up
        //    and block its next callback.
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
        }

        // 4. Pull canonical state from DSProxy
        const auto vehicles = proxy.getTrafficVehicles();
        const auto signals  = proxy.getSignalStates();

        // 5. Resolve egoVissimId once the Create round-trips
        if (egoVissimId == 0 && egoPending) {
            for (const auto& v : vehicles) {
                if (v.CreateID == egoCreateId && !v.ControlledByVissim) {
                    egoVissimId = v.VehicleID;
                    printf("ego registered: VISSIM VehicleID=%d\n", egoVissimId);
                    break;
                }
            }
        }

        // 5. Publish to app client if connected
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

            // 6. Receive ego pose + (optionally) CAV behavior commands from
            //    app client (one round-trip per tick)
            msgHelper.VehDataRecv_um.clear();
            int recvSimState = 0;
            float recvSimTime = 0.0f;
            if (sockHelper.recvData(clientSock, &recvSimState, &recvSimTime, msgHelper) < 0) {
                fprintf(stderr, "ERROR tick %d: recvData failed\n", tick);
                rc = 8;
                break;
            }

            // 7. Split client-received vehicles: ego goes to DSProxy on the
            //    next tick, everything else goes to DriverModel as a
            //    behavior command.
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
                        ego.Create = true;
                        ego.CreateID = egoCreateId;
                        egoPending = true;
                    } else {
                        ego.VehicleID = egoVissimId;
                        ego.Create = false;
                    }
                    egos.push_back(ego);
                } else if (relayDM && dmSock > 0) {
                    // CAV behavior command: relay to DriverModel as-is.
                    // DriverModel parses VehFullData_t and applies
                    // speedDesired / accelerationDesired in its next
                    // DRIVER_DATA_DESIRED_VELOCITY / _ACCELERATION callback.
                    msgHelper.VehDataSend_um[dmSock].push_back(v);
                }
            }

            // 8. CAV behavior cmds (now sitting in msgHelper.VehDataSend_um[dmSock])
            //    get sent at the START of the next tick, before the next
            //    proxy.setDriverVehicles call.
        }

        if (tick - lastReportedTick >= 25) {
            printf("tick %5d: vehicles=%3zu signals=%3zu egos=%zu\n",
                   tick, vehicles.size(), signals.size(), egos.size());
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
