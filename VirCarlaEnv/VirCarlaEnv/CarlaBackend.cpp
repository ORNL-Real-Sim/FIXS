#include "CarlaBackend.h"

#include <carla/client/ActorList.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/TrafficLight.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Vector3D.h>

#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace virenv {

// mainVirCarla uses a fixed default bounding box + a small spawn z offset.
static const carla::geom::Vector3D kExtent(4.8f, 2.0f, 1.8f);
static const float kSpawnOffsetZ = 0.1f;

void CarlaBackend::log(const char* msg)      { std::cout << msg << std::endl; }
void CarlaBackend::logError(const char* msg) { std::cerr << msg << std::endl; }

carla::geom::Transform CarlaBackend::sumoTransformOf(const Pose& p) {
    // verbatim from mainVirCarla: Location(x,y,z), Rotation(grade_deg, heading, 0)
    carla::geom::Location loc((float)p.x, (float)p.y, (float)p.z);
    carla::geom::Rotation rot((float)(p.gradeRad * 180.0 / M_PI), (float)p.headingDeg, 0.0f);
    return carla::geom::Transform(loc, rot);
}

void CarlaBackend::loadSignalTable(const char* path) {
    if (path == nullptr || path[0] == '\0') return;
    trafficLightMap_ = BridgeHelper::readTrafficLightTable(path);
}

void CarlaBackend::initTrafficPool() {
    if (!bpLib_ && world_) bpLib_ = world_->GetBlueprintLibrary();
}

VehHandle CarlaBackend::spawnVehicle(const std::string& vType, const std::string& vClass,
                                     const Pose& spawnPose) {
    if (!world_) return kNoHandle;
    if (!bpLib_) bpLib_ = world_->GetBlueprintLibrary();

    carla::geom::Transform carlaTf = BridgeHelper::map_transfrom_Sumo_to_Carla(sumoTransformOf(spawnPose), kExtent);
    carlaTf.location.z += kSpawnOffsetZ;

    std::string bpId = useVType_ ? vType : BridgeHelper::map_Sumo_vClass_to_Carla_blueprintId(vClass);
    const carla::client::ActorBlueprint* bp = bpLib_->Find(bpId);
    if (bp == nullptr) {
        logError(("Blueprint not found: " + bpId).c_str());
        return kNoHandle;
    }
    carla::client::ActorBlueprint bpLocal = *bp;   // copy so we can modify

    carla::SharedPtr<carla::client::Actor> actor = world_->TrySpawnActor(bpLocal, carlaTf);
    if (actor == nullptr) {
        if (verbose_) std::cout << "[Warning] Failed to spawn actor (vClass=" << vClass << ")\n";
        return kNoHandle;
    }
    carla::SharedPtr<carla::client::Vehicle> veh =
        boost::static_pointer_cast<carla::client::Vehicle>(actor);
    veh->SetSimulatePhysics(false);

    VehHandle h = (VehHandle)veh->GetId();
    actors_[h] = veh;
    if (verbose_) std::cout << "Spawned Carla actor " << h << " (" << bpId << ")\n";
    return h;
}

void CarlaBackend::despawnVehicle(VehHandle h) {
    auto it = actors_.find(h);
    if (it != actors_.end()) {
        if (it->second) it->second->Destroy();
        actors_.erase(it);
    }
}

void CarlaBackend::setVehiclePose(VehHandle h, const Pose& p) {
    carla::geom::Transform carlaTf = BridgeHelper::map_transfrom_Sumo_to_Carla(sumoTransformOf(p), kExtent);
    lastApplied_[h] = carlaTf;   // A/B instrumentation (driver logs it by SUMO id)
    // batched, applied in flushBatch() before the world Tick -- same as mainVirCarla
    batch_.push_back(carla::rpc::Command::ApplyTransform((carla::rpc::ActorId)h, carlaTf));
}

void CarlaBackend::flushBatch() {
    if (client_) client_->ApplyBatch(batch_, false);
    batch_.clear();
}

void CarlaBackend::syncTrafficLight(const std::string& junctionId, const std::string& stateStr) {
    auto jit = trafficLightMap_.find(junctionId);
    if (jit == trafficLightMap_.end()) return;
    for (size_t linkId = 0; linkId < stateStr.size(); ++linkId) {
        SumoTrafficLightState ss = BridgeHelper::get_Sumo_traffic_light_state_from_char(stateStr[linkId]);
        carla::rpc::TrafficLightState cs = BridgeHelper::map_Sumo_traffic_light_state_to_Carla(ss);
        auto lit = jit->second.find((int)linkId);
        if (lit != jit->second.end() && lit->second.carlaTrafficLightActorPtr)
            lit->second.carlaTrafficLightActorPtr->SetState(cs);
    }
}

void CarlaBackend::freezeAndMatchTrafficLights() {
    if (!world_) return;
    carla::SharedPtr<carla::client::ActorList> tlActors =
        world_->GetActors()->Filter("traffic.traffic_light");
    for (const carla::SharedPtr<carla::client::Actor>& actor : *tlActors) {
        carla::SharedPtr<carla::client::TrafficLight> tlPtr =
            boost::static_pointer_cast<carla::client::TrafficLight>(actor);
        tlPtr->Freeze(true);
        std::string aid = std::to_string(actor->GetId());
        carla::geom::Location sloc = BridgeHelper::map_location_Carla_to_Sumo(actor->GetLocation());
        std::pair<std::string, int> idp =
            BridgeHelper::find_closest_trafficLight_id(trafficLightMap_, sloc.x, sloc.y);
        if (idp.first.empty()) continue;
        TrafficLight& tl = trafficLightMap_[idp.first][idp.second];
        tl.carlaTrafficLightActorId = aid;
        tl.carlaTrafficLightActorPtr = tlPtr;
    }
}

carla::SharedPtr<carla::client::Vehicle> CarlaBackend::actorOf(VehHandle h) {
    auto it = actors_.find(h);
    return (it != actors_.end()) ? it->second : nullptr;
}

} // namespace virenv
