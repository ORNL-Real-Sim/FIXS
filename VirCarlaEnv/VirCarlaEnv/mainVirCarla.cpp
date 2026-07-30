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
#include <carla/client/ActorList.h>
#include <carla/client/Vehicle.h>
#include <carla/client/TimeoutException.h>
#include <carla/geom/Transform.h>
#include <carla/Memory.h>
#include <carla/trafficmanager/TrafficManager.h>

#include "BridgeHelper.h"
#include "CarlaBackend.h"
#include "../../CommonLib/VirEnvCore.h"
#include "../../CommonLib/DataLogger.h"
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
    std::cout << std::unitbuf;   // flush every << so a crash never swallows the last diagnostic line
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
    // ---- cadence: three distinct things, one each --------------------------
    //  feed        fixs::kFeedPeriodS -- the FIXS exchange period, which is also
    //              the traffic simulator's step (TrafficLayer steps it once per
    //              exchange). A protocol constant, not a knob: see FixsProtocol.h.
    //  carlaStep   CarlaSetup.CarlaTimeStep -- the Carla world step
    //              (fixed_delta_seconds), the analogue of CarMaker's solver dt.
    //              Absent/0 -> the feed, i.e. tick 1:1 and do not interpolate.
    //  poseRefresh CarlaSetup.TrafficRefreshRate -- how often the core re-applies
    //              (interpolated) traffic poses. EXACTLY the meaning the key has on
    //              the CarMaker side (VirEnvHelper -> core_.trafficRefreshRate_):
    //              a visual/RPC-cost knob, independent of the world step. Absent/0
    //              -> every tick. Coarser than the tick = fewer ApplyBatch calls on
    //              a heavy scene while physics/sensors still run at the tick rate.
    //
    // These used to be entangled: TrafficRefreshRate doubled as the feed period AND
    // as the tick when CarlaTimeStep was unset, so setting it to 0.05 silently
    // turned interpolation OFF (carlaStep == refreshRate) and every pose was held
    // for two ticks. Nothing falls back to anything else now.
    const double feed        = fixs::kFeedPeriodS;
    const double carlaStep   = (cs.CarlaTimeStep > 1e-9) ? cs.CarlaTimeStep : feed;
    const double poseRefresh = (cs.TrafficRefreshRate > 1e-9) ? cs.TrafficRefreshRate
                                                              : carlaStep;
    // The exchange boundary is tested on the feed grid, so the world clock has to
    // LAND on it: carlaStep must divide the feed exactly. With, say, 0.03 the clock
    // steps 0.09 -> 0.12 and no tick is ever a feed boundary, so the bridge would
    // trade no messages at all and simply hang.
    {
        const double slots = feed / carlaStep;
        const double whole = (double)(long long)(slots + 0.5);
        if (carlaStep > feed + 1e-9 || std::fabs(slots - whole) > 1e-6) {
            std::cerr << "CarlaSetup.CarlaTimeStep (" << carlaStep << " s) must be the FIXS "
                      << "feed period (" << feed << " s) or an exact divisor of it "
                      << "(0.05, 0.025, 0.02, 0.01), else no Carla tick ever lands on an "
                      << "exchange boundary.\n";
            return -1;
        }
    }
    // The core gates re-application with (int)(1.0 / poseRefresh), so a value whose
    // reciprocal is not whole silently snaps to a different grid (0.03 -> 1/33 s).
    if (poseRefresh < carlaStep - 1e-9 ||
        std::fabs(1.0 / poseRefresh - (double)(long long)(1.0 / poseRefresh + 0.5)) > 1e-6) {
        std::cerr << "CarlaSetup.TrafficRefreshRate (" << poseRefresh << " s) must be >= "
                  << "CarlaTimeStep (" << carlaStep << " s) and have a whole reciprocal "
                  << "(0.1, 0.05, 0.025, 0.02, 0.01). It is the pose re-apply cadence, "
                  << "not the feed period - to tick Carla faster, set CarlaTimeStep.\n";
        return -1;
    }
    const uint32_t simEndTime = config.SimulationSetup.SimulationEndTime;
    const bool   enableExternalControl = cs.EnableExternalControl;
    const std::string centeredViewId   = cs.CenteredViewId;
    const bool   spectatorFollow  = cs.EnableSpectatorFollow && !cs.CenteredViewId.empty();
    const float  spectatorHeight  = (float)cs.SpectatorHeight;
    const bool   spectatorAlignYaw = cs.SpectatorAlignYaw;
    const bool   realtimePacing = cs.RealtimePacing;
    const bool   enableTlsSync = true;
    // #174 ego-mode ladder: 0=SumoDriver(teleport) 1=CarlaDriver(L0) 2=Advisory(L2) 3=Control(L4)
    const int    egoMode = cs.EgoMode;
    // L0 driver selection: native Carla TM autopilot vs the SDK-free EgoDriver
    // fallback module (map-agnostic). Default is TM (see ConfigHelper).
    const bool   useFallbackDriver = (cs.EgoL0Driver == "Pursuit" || cs.EgoL0Driver == "pursuit"
                                      || cs.EgoL0Driver == "Fallback" || cs.EgoL0Driver == "EgoDriver");
    // #174 unified EgoDriver: the driver is an EXTERNAL FIXS client that streams an
    // ACTUATION command on the ego's record (acceleratorPedalDesired/brakePedalDesired
    // /steerAngleDesired). Carla owns no in-process driver here -- it just applies the
    // wire command via ApplyControl. Same path serves L0/L2 (EgoDriver client) and L4
    // (a real external controller). No TM, no route needed on the Carla side.
    const bool   useWireActuation = (cs.EgoL0Driver == "Actuation" || cs.EgoL0Driver == "actuation");
    const double kMaxSteerRad = 0.7;   // must match the client's DriveCommand steer scaling
    const std::string egoId = cs.EgoId;
    if (egoMode >= 1 && cs.EgoSpawnPose.size() < 4) {
        std::cerr << "EgoMode " << egoMode << " needs EgoSpawnPose: [x, y, z, headingDeg]\n";
        return -1;
    }

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
        // Force BOTH sync mode and the tick delta: the world-load script may have
        // set sync with a different delta, and a physics ego (EgoMode >= 1) steps
        // PhysX/TM by fixed_delta_seconds -- it must equal the bridge's carlaStep.
        if (!settings.synchronous_mode ||
            !settings.fixed_delta_seconds.has_value() ||
            std::fabs(settings.fixed_delta_seconds.value_or(0.0) - carlaStep) > 1e-9) {
            settings.synchronous_mode = true;
            settings.fixed_delta_seconds = carlaStep;
            world.ApplySettings(settings, 1s);
            if (verbose) std::cout << "Synchronous mode enabled (delta " << carlaStep << " s).\n";
        }

        // Clear stale vehicle actors from prior runs: this bridge owns EVERY
        // vehicle in the world, so any pre-existing one is a crashed run's zombie
        // (it would block the ego spawn point / duplicate bg traffic).
        // NOTE: in synchronous mode the client's episode snapshot is EMPTY until a
        // tick -- GetActors() would miss the zombies without this first Tick.
        world.Tick(10s);
        {
            carla::SharedPtr<carla::client::ActorList> stale =
                world.GetActors()->Filter("vehicle.*");
            int nStale = 0;
            for (const carla::SharedPtr<carla::client::Actor>& a : *stale) {
                if (a) { a->Destroy(); nStale++; }
            }
            if (nStale > 0) std::cout << "Cleared " << nStale << " stale vehicle actor(s) from a prior run.\n";
        }

        // ---- backend + core ------------------------------------------------
        virenv::CarlaBackend backend(&world, &client, cs.UseVehicleTypeAsBlueprint, verbose);
        virenv::VirEnvCore core;
        core.setBackend(&backend);
        core.interpolateTraffic        = (carlaStep < feed - 1e-9);  // sub-step -> interpolate the feed
        core.sendEgoFromCore           = false;  // this driver owns the send (post-tick)
        core.openSignalPort            = false;  // Carla: vehicles + signals on ONE port
        core.ENABLE_REALSIM            = cs.EnableCosimulation;
        core.SYNCHRONIZE_TRAFFIC_SIGNAL = enableTlsSync;
        // EgoMode 0: Carla renders every vehicle incl. the SUMO-driven ego ("").
        // EgoMode >=1: Carla OWNS the ego -- the core must never spawn/teleport the
        // SUMO echo of it (SUMO's copy is the injected shadow of this Carla actor).
        core.egoId_                    = (egoMode >= 1) ? egoId : "";
        core.egoType_                  = "";
        core.trafficLayerIP_           = cs.CarlaClientIP;
        core.vehDataPort_              = cs.CarlaClientPort;
        core.trafficRefreshRate_       = poseRefresh;  // pose re-apply cadence (CarMaker semantics)
        core.Msg_c.getConfig(config);
        // One line that states the whole cadence, so a run never has to be reverse
        // engineered from three keys again.
        std::cout << "Cadence: FIXS feed " << feed << " s (= the traffic simulator step)"
                  << " | Carla tick " << carlaStep << " s"
                  << (core.interpolateTraffic
                        ? " (interpolated " + std::to_string((int)(feed / carlaStep + 0.5)) + "x)"
                        : " (1:1 with the feed)")
                  << " | pose refresh " << poseRefresh << " s"
                  << " | pacing " << (realtimePacing ? "realtime" : "as fast as possible") << "\n";

        const char* err = nullptr;
        if (core.initialization(&err, configPath.c_str(), tlsPath.c_str()) < 0) {
            std::cerr << "VirEnvCore init failed: " << (err ? err : "?") << "\n";
            return -1;
        }
        if (enableTlsSync) backend.freezeAndMatchTrafficLights();

        // ---- L0+ (EgoMode >= 1): Carla drives the ego ----------------------
        // Order matters (matches the proven Python sequence): spawn + physics
        // first, THEN create the TM + sync + autopilot. TM must be synchronous in
        // a synchronous world; world.Tick() then drives TM's SynchronousTick
        // automatically (in-process TM instance).
        if (egoMode >= 1) {
            virenv::Pose sp;
            sp.x = cs.EgoSpawnPose[0]; sp.y = cs.EgoSpawnPose[1];
            sp.z = cs.EgoSpawnPose[2]; sp.headingDeg = cs.EgoSpawnPose[3];
            if (backend.spawnEgo(cs.EgoBlueprint, sp, cs.TrafficManagerPort) == virenv::kNoHandle) {
                std::cerr << "EgoMode " << egoMode << ": ego spawn failed\n";
                return -1;
            }
            // Two L0 drivers (config EgoL0Driver):
            //  - native Carla TM autopilot (default): server-side, needs a
            //    routable map (the simple_loop junction-id fix makes it routable);
            //  - EgoDriver fallback module: map-agnostic pure pursuit on
            //    EgoRoutePoints, through full PhysX -- needs the route.
            if (useFallbackDriver) {
                if (cs.EgoRoutePoints.empty()) {
                    std::cerr << "EgoMode " << egoMode << " (Pursuit) needs EgoRoutePoints\n";
                    return -1;
                }
                backend.setEgoRoute(cs.EgoRoutePoints, cs.EgoRouteRepeat, cs.TrafficManagerPort);
            }
            // settle the spawned ego onto its tires before wiring the driver / loop
            for (int i = 0; i < 10; i++) world.Tick(30s);
            if (useWireActuation) {
                // No in-Carla driver: the ego stays physics-on and MANUAL (no autopilot),
                // driven each feed by the external EgoDriver client's wire actuation.
                std::cout << "EgoMode " << egoMode << " (Actuation): ego driven by external "
                          << "FIXS actuation command (no TM, no route).\n";
            }
            // native TM must be enabled AFTER spawn + physics settle (proven order)
            if (!useFallbackDriver && !useWireActuation) {
                backend.enableEgoTM(cs.TrafficManagerPort, cs.EgoTargetSpeed);
                // TM builds its InMemoryMap on the FIRST tick after autopilot -- can
                // take 15-30 s on a generated map. Absorb that ONE-TIME cost HERE
                // (generous timeout, before the co-sim loop couples with TL/SUMO), so
                // the loop's tight world.Tick() never stalls past its timeout and
                // drops the TrafficLayer/SUMO connection.
                std::cout << "Pre-building TM InMemoryMap (one-time, may take ~30 s)...\n";
                for (int i = 0; i < 5; i++) world.Tick(120s);
                std::cout << "TM InMemoryMap ready; entering co-sim loop.\n";
            }
        }

        // ---- L2 (EgoMode >= 2): artificial external speed-advisory controller ----
        // Feeds the ego's L0 driver a time-varying desired speed (the advisory a
        // real external CAV controller would stream over FIXS). SDK-free module;
        // swap-in point for a separate advisory client later. Empty profile -> the
        // advisor returns EgoTargetSpeed, so EgoMode 2 degenerates to constant L0.
        // L2 (EgoMode >= 2): the ego's target speed comes from an EXTERNAL controller
        // over FIXS -- read off the ego's received record (ego.speedDesired), which an
        // advisory client (e.g. py_ego_speed_advisor.py) feeds through TrafficLayer's
        // sequential-client path. No controller attached -> falls back to EgoTargetSpeed.
        if (egoMode >= 2)
            std::cout << "L2: external speed advisory via FIXS (ego.speedDesired) -- driver: "
                      << (useFallbackDriver ? "EgoDriver" : "TM") << "\n";
        double lastAdvisory = cs.EgoTargetSpeed;   // most-recent commanded desired speed (for the driver/log)

        const int sock0 = 0;
        // #174 A/B: optional applied-pose log keyed by SUMO id (set RS_POSE_LOG=path)
        std::ofstream poseLog;
        if (const char* plp = std::getenv("RS_POSE_LOG")) { poseLog.open(plp); poseLog << "simTime,id,x,y,yaw\n"; }
        long stepCount = 0;
        double simTime = 0.0;
        auto wallStart = std::chrono::steady_clock::now();   // realtime-pacing reference

        // ---- generic FIXS data logging (config: DataLogSetup) --------------
        // Records the vehicle-data records this bridge reports to FIXS, in the
        // SUMO/VISSIM wire convention (see CommonLib/DataLogger). Same code path
        // for every EgoL0Driver, so the CSVs are directly comparable.
        const auto& dls = config.DataLogSetup;
        fixs::DataLogger dataLog;
        auto logWanted = [&](const std::string& id) {
            if (dls.DataLogWho.empty()) return true;
            for (const std::string& w : dls.DataLogWho) if (w == id) return true;
            return false;
        };
        if (dls.EnableDataLog) {
            std::string p = (dls.DataLogPath.empty() || dls.DataLogPath == "auto")
                            ? std::string("_datalog/vircarla.csv") : dls.DataLogPath;
            if (dataLog.open(p, dls.DataLogFields))
                std::cout << "DataLogger -> " << dataLog.path() << " (FIXS/SUMO-VISSIM wire convention)\n";
            else
                std::cerr << "DataLogger: could not open " << p << "\n";
        }

        while (simTime < simEndTime) {
            // ---- core: recv (only on the 0.1s feed boundary) -> spawn / pose
            //      (batch) / despawn; the refresh interpolates EVERY sub-step ----
            if (core.runStep(simTime, &err) < 0) {
                if (WSAGetLastError() != WSAEINTR && WSAGetLastError() != WSAEFAULT)
                    std::cerr << "co-sim recv/step ended: " << (err ? err : "?") << "\n";
                break;
            }
            backend.flushBatch();          // ApplyBatch(transform commands)
            // ---- L2: apply the external speed advisory at each 0.1s FIXS feed ----
            // Set the driver target BEFORE it runs this tick. applyEgoControl routes
            // it to native TM (SetDesiredSpeed) or the EgoDriver fallback (override);
            // it persists across sub-steps until the next feed refreshes it.
            if (egoMode >= 2 && !useWireActuation) {
                const bool onFeedNow = fixs::onFeedBoundary(simTime, 1e-6);
                if (onFeedNow) {
                    // the external controller's advisory rides on the ego's received
                    // FIXS record (TrafficLayer's sequential overlay merged it in).
                    auto itAdv = core.Msg_c.VehDataRecv_um.find(egoId);
                    if (itAdv != core.Msg_c.VehDataRecv_um.end() && itAdv->second.speedDesired > 0.0f)
                        lastAdvisory = itAdv->second.speedDesired;
                    // else keep the last advisory (controller not up yet / no update)
                    backend.applyEgoControl(egoId, lastAdvisory);
                }
            }
            // #174 unified EgoDriver: apply the external client's ACTUATION command off
            // the ego's wire record (throttle/brake pedals + steer angle). One apply-path
            // for L0/L2 (EgoDriver client) and L4 (real controller); no TM/route here.
            if (useWireActuation) {
                const bool onFeedNow = fixs::onFeedBoundary(simTime, 1e-6);
                if (onFeedNow) {
                    auto itAct = core.Msg_c.VehDataRecv_um.find(egoId);
                    if (itAct != core.Msg_c.VehDataRecv_um.end()) {
                        const VehFullData_t& cmd = itAct->second;
                        backend.applyEgoActuation(cmd.acceleratorPedalDesired, cmd.brakePedalDesired,
                                                  cmd.steerAngleDesired / kMaxSteerRad);  // rad -> normalized
                    }
                }
            }
            // fallback module drives per-tick; native TM drives inside world.Tick()
            if (egoMode >= 1 && useFallbackDriver) backend.driveEgoFallback(cs.EgoTargetSpeed);
            if (poseLog.is_open()) {       // A/B: log the applied Carla pose per SUMO id
                for (const auto& kv : core.mappedVehicles()) {
                    const carla::geom::Transform* tf = backend.lastAppliedPose(kv.second);
                    if (tf) poseLog << simTime << "," << kv.first << "," << tf->location.x
                                    << "," << tf->location.y << "," << tf->rotation.yaw << "\n";
                }
            }
            world.Tick(10s);               // advance Carla one sub-step (10s: TM sync work rides on the tick)

            // FIXS feed boundary (0.1 s): a recv happened this step, so send the
            // paired response + clear here. Sub-steps in between only render.
            const bool onFeed = fixs::onFeedBoundary(simTime, 1e-6);

            // SUMO<->CARLA elevation audit, once per exchange. Here rather than inside
            // setVehiclePose because it asks whether the two MAPS agree, which no
            // interpolated sub-step can change - see CarlaBackend::auditZAlignment.
            if (onFeed) backend.auditZAlignment();

            // ---- POST-tick L0+: the Carla-driven ego -> FIXS (TL injects into SUMO)
            if (egoMode >= 1) {
                virenv::EgoState es;
                if (backend.readEgoState(egoId, es)) {
            if (onFeed && core.ENABLE_REALSIM) {
                        VehFullData_t d;
                        d.id = egoId; d.type = cs.EgoSumoType;
                        // L2: report the COMMANDED advisory as speedDesired (measured
                        // speed stays in `speed`) so the DataLogger captures both and
                        // the ego's tracking of the external target is verifiable.
                        d.speed = (float)es.speed;
                        d.speedDesired = (egoMode >= 2) ? (float)lastAdvisory : (float)es.speed;
                        d.positionX = (float)es.x; d.positionY = (float)es.y; d.positionZ = (float)es.z;
                        d.heading = (float)es.heading; d.grade = (float)es.grade;
                        core.Msg_c.VehDataSend_um[core.Sock_c.serverSock[sock0]].push_back(d);
                        if (dataLog.isOpen() && logWanted(d.id)) dataLog.logVehicle(simTime, d);
                        // Also log the SUMO-VIEW ego (what SUMO reports back this feed) on the
                        // SAME clock as the Carla view, so a plot compares them directly. It's
                        // the ego ~2 ticks stale (Carla is a step ahead + SUMO getPosition is
                        // n-1); logged as id "ego_sumo".
                        if (dataLog.isOpen()) {
                            auto itSumo = core.Msg_c.VehDataRecv_um.find(egoId);
                            if (itSumo != core.Msg_c.VehDataRecv_um.end()) {
                                VehFullData_t dsumo = itSumo->second;
                                dsumo.id = "ego_sumo";
                                dataLog.logVehicle(simTime, dsumo);
                            }
                        }
                    }
                    if (spectatorFollow && egoId == centeredViewId && backend.egoActor()) {
                        carla::geom::Transform eTf = backend.egoActor()->GetTransform();
                        carla::geom::Location loc = eTf.location; loc.z += spectatorHeight;
                        const float yaw = spectatorAlignYaw ? (eTf.rotation.yaw - 90.f) : -90.f;
                        spectator->SetTransform(carla::geom::Transform(loc, carla::geom::Rotation(-90.f, yaw, 0.f)));
                    }
                }
            }

            // ---- POST-tick: interested-id readback (feed) + spectator (every tick)
            const auto& mapped = core.mappedVehicles();
            for (const std::string& iid : interestedIds) {
                if (egoMode >= 1 && iid == egoId) continue;   // ego handled above (never mapped)
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
                    if (dataLog.isOpen() && logWanted(d.id)) dataLog.logVehicle(simTime, d);
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

        if (dataLog.isOpen()) { std::cout << "DataLogger closed: " << dataLog.path() << "\n"; dataLog.close(); }
        if (egoMode >= 1) backend.destroyEgo();
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
