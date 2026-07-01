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
#include <cstdlib>
#include <thread>

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
    // Carla render sub-step: tick finer than the 0.1 s FIXS feed and interpolate the
    // feed (the CarMaker trick) for smoother motion. <=0 or >=feed -> 1:1, no interp.
    const double carlaStep    = (cs.CarlaTimeStep > 1e-9 && cs.CarlaTimeStep < refreshRate)
                                ? cs.CarlaTimeStep : refreshRate;
    const uint32_t simEndTime = config.SimulationSetup.SimulationEndTime;
    const bool   enableExternalControl = cs.EnableExternalControl;
    const std::string centeredViewId   = cs.CenteredViewId;
    const bool   spectatorFollow  = cs.EnableSpectatorFollow && !cs.CenteredViewId.empty();
    const float  spectatorHeight  = (float)cs.SpectatorHeight;
    const bool   spectatorAlignYaw = cs.SpectatorAlignYaw;
    const bool   realtimePacing = cs.RealtimePacing;
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
            settings.fixed_delta_seconds = carlaStep;
            world.ApplySettings(settings, 1s);
            if (verbose) std::cout << "Synchronous mode enabled.\n";
        }

        // ---- backend + core ------------------------------------------------
        virenv::CarlaBackend backend(&world, &client, cs.UseVehicleTypeAsBlueprint, verbose);
        virenv::VirEnvCore core;
        core.setBackend(&backend);
        core.interpolateTraffic        = (carlaStep < refreshRate - 1e-9);  // sub-step -> interpolate the 0.1s feed
        core.sendEgoFromCore           = false;  // this driver owns the send (post-tick)
        core.openSignalPort            = false;  // Carla: vehicles + signals on ONE port
        core.ENABLE_REALSIM            = cs.EnableCosimulation;
        core.SYNCHRONIZE_TRAFFIC_SIGNAL = enableTlsSync;
        core.egoId_                    = "";      // Carla renders every vehicle (ego included)
        core.egoType_                  = "";
        core.trafficLayerIP_           = cs.CarlaClientIP;
        core.vehDataPort_              = cs.CarlaClientPort;
        core.trafficRefreshRate_       = carlaStep;   // refresh-slot fires every Carla sub-step
        core.Msg_c.getConfig(config);
        if (verbose && core.interpolateTraffic)
            std::cout << "Carla sub-stepping: tick " << carlaStep << "s, interpolate the "
                      << refreshRate << "s FIXS feed (" << (int)(refreshRate / carlaStep) << "x).\n";

        const char* err = nullptr;
        if (core.initialization(&err, configPath.c_str(), tlsPath.c_str()) < 0) {
            std::cerr << "VirEnvCore init failed: " << (err ? err : "?") << "\n";
            return -1;
        }
        if (enableTlsSync) backend.freezeAndMatchTrafficLights();

        const int sock0 = 0;
        // #174 A/B: optional applied-pose log keyed by SUMO id (set RS_POSE_LOG=path)
        std::ofstream poseLog;
        if (const char* plp = std::getenv("RS_POSE_LOG")) { poseLog.open(plp); poseLog << "simTime,id,x,y,yaw\n"; }
        long stepCount = 0;
        double simTime = 0.0;
        auto wallStart = std::chrono::steady_clock::now();   // realtime-pacing reference
        while (simTime < simEndTime) {
            // ---- core: recv (only on the 0.1s feed boundary) -> spawn / pose
            //      (batch) / despawn; the refresh interpolates EVERY sub-step ----
            if (core.runStep(simTime, &err) < 0) {
                if (WSAGetLastError() != WSAEINTR && WSAGetLastError() != WSAEFAULT)
                    std::cerr << "co-sim recv/step ended: " << (err ? err : "?") << "\n";
                break;
            }
            backend.flushBatch();          // ApplyBatch(transform commands)
            if (poseLog.is_open()) {       // A/B: log the applied Carla pose per SUMO id
                for (const auto& kv : core.mappedVehicles()) {
                    const carla::geom::Transform* tf = backend.lastAppliedPose(kv.second);
                    if (tf) poseLog << simTime << "," << kv.first << "," << tf->location.x
                                    << "," << tf->location.y << "," << tf->rotation.yaw << "\n";
                }
            }
            world.Tick(1s);                // advance Carla one sub-step

            // FIXS feed boundary (0.1 s): a recv happened this step, so send the
            // paired response + clear here. Sub-steps in between only render.
            const bool onFeed = std::fabs(simTime * 10.0 - std::llround(simTime * 10.0)) < 1e-6;

            // ---- POST-tick: interested-id readback (feed) + spectator (every tick)
            const auto& mapped = core.mappedVehicles();
            for (const std::string& iid : interestedIds) {
                auto mit = mapped.find(iid);
                if (mit == mapped.end()) continue;
                carla::SharedPtr<carla::client::Vehicle> actor = backend.actorOf(mit->second);
                if (!actor) continue;
                carla::geom::Transform cTf = actor->GetTransform();

                if (enableExternalControl && onFeed) {
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
                if (spectatorFollow && iid == centeredViewId) {
                    // Rigid top-down BEV snapped to the ego each tick: no low-pass,
                    // so no camera oscillation. The ego pose is already smooth, so
                    // the camera is too. yaw: fixed north-up, or aligned to the
                    // ego heading so its forward points "up" (SpectatorAlignYaw).
                    carla::geom::Location loc = cTf.location; loc.z += spectatorHeight;
                    const float yaw = spectatorAlignYaw ? (cTf.rotation.yaw - 90.f) : -90.f;
                    spectator->SetTransform(carla::geom::Transform(loc, carla::geom::Rotation(-90.f, yaw, 0.f)));
                }
            }

            // ---- driver owns the send: once per FIXS feed (pairs with the recv) ----
            if (onFeed && core.ENABLE_REALSIM) {
                if (core.Sock_c.sendData(core.Sock_c.serverSock[sock0], sock0, (float)simTime, 1, core.Msg_c) < 0) {
                    if (WSAGetLastError() != WSAEINTR && WSAGetLastError() != WSAEFAULT)
                        std::cerr << "send to traffic layer failed\n";
                    break;
                }
            }
            if (onFeed) {
                core.Msg_c.clearRecvStorage();
                core.Msg_c.clearSendStorage();
            }
            simTime = (++stepCount) * carlaStep;   // step counter avoids fp drift

            // Realtime pacing (viz): sleep so each sub-tick lands at its wall-clock
            // sim-time -> the sub-ticks spread evenly instead of bursting, so a
            // follow-cam renders smooth. Never over-throttles (if we fell behind,
            // sleep is skipped and the reference resyncs). OFF for XIL.
            if (realtimePacing) {
                using namespace std::chrono;
                auto target = wallStart + duration_cast<steady_clock::duration>(duration<double>(simTime));
                auto now = steady_clock::now();
                if (now < target) std::this_thread::sleep_until(target);
                else if (now - target > milliseconds(250))
                    wallStart = now - duration_cast<steady_clock::duration>(duration<double>(simTime));
            }
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
