//============================================================================
//  mainVirCarla  (#174)  -- now a THIN Carla tick-driver over VirEnvCore.
//----------------------------------------------------------------------------
//  Previously this file WAS the whole Carla bridge (~677 lines) with the
//  seven-step orchestration inlined in main(). That orchestration moved to the
//  backend-agnostic CommonLib/VirEnvCore (shared with CarMaker); the Carla verbs
//  moved to CarlaBackend. main() now just: brings up the Carla world + tick,
//  constructs CarlaBackend + VirEnvCore, and per tick calls core.runStep then
//  flushes the transform batch, ticks the world, and (for interested/external
//  ids) reads back POST-tick + sends to FIXS -- exactly the Carla-specific shell.
//============================================================================
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_set>
#include <cmath>
#include <chrono>

#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Actor.h>
#include <carla/client/Vehicle.h>
#include <carla/client/TimeoutException.h>
#include <carla/geom/Transform.h>
#include <carla/Memory.h>

#include "BridgeHelper.h"
#include "CarlaBackend.h"
#include "../../CommonLib/VirEnvCore.h"
#include "MsgHelper.h"
#include "ConfigHelper.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

static void show_usage(const std::string& name) {
    std::cerr << "Usage: " << name << " -f <config.yaml> [-t <tls_table.csv>]\n";
}

int main(int argc, const char* argv[]) {
    std::string configPath, tlsPath;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "-h" || a == "--help") { show_usage(argv[0]); return 0; }
        else if ((a == "-f" || a == "--file") && i + 1 < argc) configPath = argv[++i];
        else if ((a == "-t" || a == "--tls")  && i + 1 < argc) tlsPath = argv[++i];
        else { show_usage(argv[0]); return 0; }
    }

    ConfigHelper config;
    if (config.getConfig(configPath) < 0) { std::cerr << "Bad config: " << configPath << "\n"; return -1; }
    CarlaSetup_t cs = config.CarlaSetup;
    const bool   verbose      = cs.EnableVerboseLog;
    const double refreshRate  = cs.TrafficRefreshRate;
    const uint32_t simEndTime = config.SimulationSetup.SimulationEndTime;
    const bool   enableExternalControl = cs.EnableExternalControl;
    const std::string centeredViewId   = cs.CenteredViewId;
    const bool   enableTlsSync = true;

    std::unordered_set<std::string> interestedIds(cs.InterestedIds.begin(), cs.InterestedIds.end());

    try {
        // ---- Carla world: connect + synchronous mode -----------------------
        carla::client::Client client(cs.CarlaServerIP, cs.CarlaServerPort);
        client.SetTimeout(10s);
        std::cout << "Carla client " << client.GetClientVersion()
                  << " / server " << client.GetServerVersion() << "\n";
        carla::client::World world = client.GetWorld();
        carla::SharedPtr<carla::client::Actor> spectator = world.GetSpectator();

        carla::rpc::EpisodeSettings settings = world.GetSettings();
        if (!settings.synchronous_mode) {
            settings.synchronous_mode = true;
            settings.fixed_delta_seconds = refreshRate;
            world.ApplySettings(settings, 1s);
            if (verbose) std::cout << "Synchronous mode enabled.\n";
        }

        // ---- backend + core ------------------------------------------------
        virenv::CarlaBackend backend(&world, &client, cs.UseVehicleTypeAsBlueprint, verbose);
        virenv::VirEnvCore core;
        core.setBackend(&backend);
        core.interpolateTraffic        = false;  // Carla ticks 1:1 with FIXS
        core.sendEgoFromCore           = false;  // this driver owns the send (post-tick)
        core.openSignalPort            = false;  // Carla: vehicles + signals on ONE port
        core.ENABLE_REALSIM            = cs.EnableCosimulation;
        core.SYNCHRONIZE_TRAFFIC_SIGNAL = enableTlsSync;
        core.egoId_                    = "";      // Carla renders every vehicle (ego included)
        core.egoType_                  = "";
        core.trafficLayerIP_           = cs.CarlaClientIP;
        core.vehDataPort_              = cs.CarlaClientPort;
        core.trafficRefreshRate_       = refreshRate;
        core.Msg_c.getConfig(config);

        const char* err = nullptr;
        if (core.initialization(&err, configPath.c_str(), tlsPath.c_str()) < 0) {
            std::cerr << "VirEnvCore init failed: " << (err ? err : "?") << "\n";
            return -1;
        }
        if (enableTlsSync) backend.freezeAndMatchTrafficLights();

        const int sock0 = 0;
        double simTime = 0.0;
        while (simTime < simEndTime) {
            // ---- core: recv -> spawn / pose (batch) / despawn / (no send) ---
            if (core.runStep(simTime, &err) < 0) {
                if (WSAGetLastError() != WSAEINTR && WSAGetLastError() != WSAEFAULT)
                    std::cerr << "co-sim recv/step ended: " << (err ? err : "?") << "\n";
                break;
            }
            backend.flushBatch();          // ApplyBatch(transform commands)
            world.Tick(1s);                // advance Carla one frame

            // ---- POST-tick: interested-id readback + spectator -------------
            const auto& mapped = core.mappedVehicles();
            for (const std::string& iid : interestedIds) {
                auto mit = mapped.find(iid);
                if (mit == mapped.end()) continue;
                carla::SharedPtr<carla::client::Vehicle> actor = backend.actorOf(mit->second);
                if (!actor) continue;
                carla::geom::Transform cTf = actor->GetTransform();

                if (enableExternalControl) {
                    carla::geom::Vector3D ext = actor->GetBoundingBox().extent;
                    carla::geom::Vector3D vel = actor->GetVelocity();
                    carla::geom::Transform sTf = BridgeHelper::map_transfrom_Carla_to_Sumo(cTf, ext);
                    VehFullData_t d;
                    d.id = iid; d.type = "ego";
                    d.speedDesired = (float)std::sqrt(vel.x * vel.x + vel.y * vel.y);
                    d.positionX = sTf.location.x; d.positionY = sTf.location.y; d.positionZ = sTf.location.z;
                    d.heading = sTf.rotation.yaw; d.grade = (float)(sTf.rotation.pitch * M_PI / 180.0);
                    core.Msg_c.VehDataSend_um[core.Sock_c.serverSock[sock0]].push_back(d);
                }
                if (iid == centeredViewId) {
                    static carla::geom::Location smoothed;
                    carla::geom::Location top = cTf.location; top.z += 100.0f;
                    smoothed = 0.9f * smoothed + 0.1f * top;
                    spectator->SetTransform(carla::geom::Transform(smoothed, carla::geom::Rotation(-90.f, -90.f, 0.f)));
                }
            }

            // ---- driver owns the send (core.sendEgoFromCore == false) -------
            if (core.ENABLE_REALSIM) {
                if (core.Sock_c.sendData(core.Sock_c.serverSock[sock0], sock0, (float)simTime, 1, core.Msg_c) < 0) {
                    if (WSAGetLastError() != WSAEINTR && WSAGetLastError() != WSAEFAULT)
                        std::cerr << "send to traffic layer failed\n";
                    break;
                }
            }
            core.Msg_c.clearRecvStorage();
            core.Msg_c.clearSendStorage();
            simTime += refreshRate;
        }

        settings = world.GetSettings();
        if (settings.synchronous_mode) {
            settings.synchronous_mode = false;
            world.ApplySettings(settings, 1s);
        }
    }
    catch (const carla::client::TimeoutException& e) { std::cerr << "\n" << e.what() << "\n"; return 1; }
    catch (const std::exception& e) { std::cerr << "\nException: " << e.what() << "\n"; return 2; }
    return 0;
}
