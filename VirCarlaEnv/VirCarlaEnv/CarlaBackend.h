#pragma once
//============================================================================
//  CarlaBackend  (#174 -- the Carla half of the verb interface)
//----------------------------------------------------------------------------
//  Implements IVirEnvBackend against the Carla C++ client API. The only file on
//  the Carla path that needs the Carla SDK -- VirEnvCore stays SDK-free. Every
//  verb is a lift of the per-vehicle Carla code that used to live inline in
//  mainVirCarla's loop (TrySpawnActor/Destroy, BridgeHelper transforms, batched
//  ApplyTransform, the junction TLS dispatch). The thin mainVirCarla driver owns
//  the world tick + batch flush + interested-vehicle readback + spectator.
//============================================================================

#include "../../CommonLib/IVirEnvBackend.h"
#include "BridgeHelper.h"

#include <carla/client/World.h>
#include <carla/client/Client.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Vehicle.h>
#include <carla/rpc/Command.h>
#include <carla/Memory.h>

#include <string>
#include <vector>
#include <unordered_map>

namespace virenv {

class CarlaBackend : public IVirEnvBackend {
public:
    CarlaBackend(carla::client::World* world, carla::client::Client* client,
                 bool useVehicleTypeAsBlueprint, bool verbose)
        : world_(world), client_(client),
          useVType_(useVehicleTypeAsBlueprint), verbose_(verbose) {}

    void log(const char* msg) override;
    void logError(const char* msg) override;

    void loadSignalTable(const char* path) override;
    void initTrafficPool() override;                                       // lazy spawn -> caches blueprint lib

    VehHandle spawnVehicle(const std::string& vType, const std::string& vClass,
                           const Pose& spawnPose) override;
    void      despawnVehicle(VehHandle h) override;

    void setVehiclePose(VehHandle h, const Pose& p) override;              // -> batched ApplyTransform
    void setVehicleLights(VehHandle, bool, bool, bool) override {}         // mainVirCarla sets no lights
    void parkSpares() override {}                                          // no spare pool in Carla

    void syncTrafficLight(const std::string& junctionId, const std::string& stateStr) override;

    bool readEgoState(const std::string&, EgoState&) override { return false; }  // driver reads POST-tick

    // unified RS_DEBUG: append the APPLIED Carla transform to the core's csv row
    std::string debugHeader() const override;
    std::string debugFields(VehHandle h) const override;

    // ---- driver hooks (not part of IVirEnvBackend) ------------------------
    void flushBatch();                                                     // ApplyBatch(the transform commands)
    void freezeAndMatchTrafficLights();                                    // map traffic.traffic_light actors -> junctions
    carla::SharedPtr<carla::client::Vehicle> actorOf(VehHandle h);         // for interested readback / spectator

    // expose for the driver's TLS / interested handling
    std::unordered_map<std::string, std::unordered_map<int, TrafficLight>>& trafficLightMap() { return trafficLightMap_; }

    // #174 A/B instrumentation: the Carla transform last APPLIED to a handle this
    // tick (post BridgeHelper transform). The driver logs it keyed by SUMO id so
    // old-vs-new can be diffed exactly, with no Carla-readback / blueprint / sort
    // confounds. Returns nullptr if the handle has no applied pose yet.
    const carla::geom::Transform* lastAppliedPose(VehHandle h) const {
        auto it = lastApplied_.find(h);
        return (it != lastApplied_.end()) ? &it->second : nullptr;
    }

private:
    carla::client::World*  world_  = nullptr;
    carla::client::Client* client_ = nullptr;
    bool useVType_  = false;
    bool verbose_   = false;

    carla::SharedPtr<carla::client::BlueprintLibrary> bpLib_;
    std::vector<carla::rpc::Command> batch_;                               // ApplyTransform commands this tick
    std::unordered_map<VehHandle, carla::SharedPtr<carla::client::Vehicle>> actors_;
    std::unordered_map<VehHandle, carla::geom::Transform> lastApplied_;   // A/B instrumentation
    std::unordered_map<std::string, std::unordered_map<int, TrafficLight>> trafficLightMap_;

    static carla::geom::Transform sumoTransformOf(const Pose& p);
};

} // namespace virenv
