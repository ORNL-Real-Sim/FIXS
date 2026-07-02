#include "CarlaBackend.h"

#include <carla/client/ActorList.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/TrafficLight.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Vector3D.h>
#include <carla/trafficmanager/TrafficManager.h>
#include <carla/rpc/VehicleControl.h>

#include <iostream>
#include <cmath>
#include <cstdio>
#include <string>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace virenv {

// #174 coord fix: the SUMO/FIXS wire carries the FRONT-of-vehicle position; the
// Carla actor transform is the actor PIVOT, which (empirically) sits at the
// bounding-box CENTER horizontally (bbox.location.x ~= 0). So to land the model's
// FRONT on the SUMO front we must step back the actor HALF-length == bbox.extent.x.
// map_transfrom_Sumo_to_Carla(..,extent) does exactly that with extent.x, so the
// CORRECT extent is the actor's real GetBoundingBox().extent -- which the reverse
// readback (Carla->Sumo) already uses, so this also makes forward/reverse symmetric.
//
// The OLD code passed a hard-coded (4.8,2.0,1.8) -- a placeholder for the
// commented-out (length/2,width/2,height/2). 4.8 as a half-length implies a 9.6 m
// vehicle, so every actor was anchored ~2.4 m too far back. kDefaultExtent below is
// only a sane passenger-car fallback for the TRANSIENT spawn transform (the actor's
// bbox isn't queryable until it exists; setVehiclePose overwrites it the same tick).
static const carla::geom::Vector3D kDefaultExtent(2.3f, 1.0f, 0.75f);
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

    // spawn transform is transient: setVehiclePose corrects it with the real bbox
    // the same tick (before world.Tick), so a passenger-car default is sufficient.
    carla::geom::Transform carlaTf = BridgeHelper::map_transfrom_Sumo_to_Carla(sumoTransformOf(spawnPose), kDefaultExtent);
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
    // Use the actor's REAL half-size so the model FRONT lands on the SUMO front
    // (anchor == bbox.extent.x == half-length). Symmetric with the Carla->Sumo
    // readback, which already uses GetBoundingBox().extent.
    carla::geom::Vector3D ext = kDefaultExtent;
    auto it = actors_.find(h);
    if (it != actors_.end() && it->second) {
        carla::geom::Vector3D e = it->second->GetBoundingBox().extent;
        if (e.x > 0.1f) ext = e;   // guard a not-yet-populated bbox on the spawn frame
    }
    carla::geom::Transform carlaTf = BridgeHelper::map_transfrom_Sumo_to_Carla(sumoTransformOf(p), ext);
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

//----------------------------------------------------------------------------
//  L0+ ego ownership (EgoMode >= 1): Carla drives the ego.
//----------------------------------------------------------------------------
VehHandle CarlaBackend::spawnEgo(const std::string& blueprintId, const Pose& spawnPose, int tmPort) {
    if (!world_) return kNoHandle;
    if (!bpLib_) bpLib_ = world_->GetBlueprintLibrary();

    const carla::client::ActorBlueprint* bp = bpLib_->Find(blueprintId);
    if (bp == nullptr) {
        logError(("Ego blueprint not found: " + blueprintId).c_str());
        return kNoHandle;
    }
    carla::client::ActorBlueprint bpLocal = *bp;

    carla::geom::Transform tf = BridgeHelper::map_transfrom_Sumo_to_Carla(sumoTransformOf(spawnPose), kDefaultExtent);
    tf.location.z += 0.3f;   // drop-in margin: physics ON -> the car settles on its tires

    carla::SharedPtr<carla::client::Actor> actor = world_->TrySpawnActor(bpLocal, tf);
    if (actor == nullptr) {
        logError("Ego spawn failed (spawn point blocked?)");
        return kNoHandle;
    }
    egoActor_ = boost::static_pointer_cast<carla::client::Vehicle>(actor);
    egoActor_->SetSimulatePhysics(true);                 // full PhysX: tire contact, dynamics
    (void)tmPort;  // autopilot is enabled separately (enableEgoAutopilot) AFTER TM sync setup
    std::cout << "L0 ego spawned: " << blueprintId << " actor " << egoActor_->GetId()
              << " (physics ON)\n";
    return (VehHandle)egoActor_->GetId();
}

void CarlaBackend::enableEgoAutopilot(int tmPort) {
    if (!egoActor_) return;
    egoActor_->SetAutopilot(true, (uint16_t)tmPort);     // TM drives (registers with the TM)
    std::cout << "L0 ego autopilot ON (TM port " << tmPort << ")\n";
}

void CarlaBackend::setEgoRoute(const std::vector<std::pair<double, double>>& fixsPts,
                               int /*repeat*/, int /*tmPort*/) {
    // The CarMaker-Route analog: the closed waypoint path the built-in ego driver
    // follows (pure pursuit; see driveEgo). NOTE: Carla's Traffic Manager was
    // measured UNABLE to follow routes on generated OpenDRIVE worlds (both
    // autopilot and SetCustomPath drove straight off the simple_loop corner and
    // were kill-z destroyed), so EgoMode 1 uses this bridge-internal driver.
    if (fixsPts.empty()) return;
    egoPath_.clear();
    // densify the closed polyline to ~3 m spacing (in the Carla frame: Y flip)
    for (size_t i = 0; i < fixsPts.size(); i++) {
        const std::pair<double, double>& a = fixsPts[i];
        const std::pair<double, double>& b = fixsPts[(i + 1) % fixsPts.size()];
        const double dx = b.first - a.first, dy = b.second - a.second;
        const int n = std::max(1, (int)(std::sqrt(dx * dx + dy * dy) / 3.0));
        for (int k = 0; k < n; k++)
            egoPath_.push_back(carla::geom::Location(
                (float)(a.first + dx * k / n), (float)(-(a.second + dy * k / n)), 0.0f));
    }
    egoPathIx_ = 0;
    std::cout << "L0 ego route: " << fixsPts.size() << " waypoints -> "
              << egoPath_.size() << " path points (bridge pure-pursuit driver)\n";
}

void CarlaBackend::driveEgo(double targetSpeed) {
    // Bridge-internal L0 driver: pure-pursuit steering + proportional speed hold,
    // applied through full PhysX dynamics (ApplyControl -> tires, powertrain).
    if (!egoActor_ || egoPath_.empty()) return;
    carla::geom::Transform tf = egoActor_->GetTransform();
    carla::geom::Vector3D  vel = egoActor_->GetVelocity();
    const double v = std::sqrt(vel.x * vel.x + vel.y * vel.y);

    // advance the path index to the nearest point ahead (closed loop: wrap)
    const size_t n = egoPath_.size();
    double best = 1e18; size_t bestIx = egoPathIx_;
    for (size_t k = 0; k < 40; k++) {                    // local window search
        size_t ix = (egoPathIx_ + k) % n;
        double dx = egoPath_[ix].x - tf.location.x, dy = egoPath_[ix].y - tf.location.y;
        double d = dx * dx + dy * dy;
        if (d < best) { best = d; bestIx = ix; }
    }
    egoPathIx_ = bestIx;

    // lookahead point: ~max(6 m, 1.2 s of travel)
    const double lookahead = std::max(6.0, 1.2 * v);
    size_t tgtIx = egoPathIx_;
    double acc = 0.0;
    while (acc < lookahead) {
        size_t nxt = (tgtIx + 1) % n;
        double dx = egoPath_[nxt].x - egoPath_[tgtIx].x, dy = egoPath_[nxt].y - egoPath_[tgtIx].y;
        acc += std::sqrt(dx * dx + dy * dy);
        tgtIx = nxt;
    }
    const double tx = egoPath_[tgtIx].x - tf.location.x;
    const double ty = egoPath_[tgtIx].y - tf.location.y;
    const double yawRad = tf.rotation.yaw * M_PI / 180.0;
    double alpha = std::atan2(ty, tx) - yawRad;          // heading error to lookahead
    while (alpha >  M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;
    const double wheelbase = 2.9;                        // ~tesla model3
    const double delta = std::atan2(2.0 * wheelbase * std::sin(alpha), lookahead);

    carla::rpc::VehicleControl c;
    c.steer = (float)std::max(-1.0, std::min(1.0, delta / 0.7));  // ~40 deg max wheel angle
    // curvature-aware speed: slow toward corners so pure pursuit tracks the arc
    const double curveSlow = std::max(0.4, 1.0 - 1.2 * std::fabs(alpha));
    const double dv = targetSpeed * curveSlow - v;
    if (dv >= 0) { c.throttle = (float)std::min(0.75, 0.25 * dv + 0.15); c.brake = 0.f; }
    else         { c.throttle = 0.f; c.brake = (float)std::min(1.0, -0.3 * dv); }
    egoActor_->ApplyControl(c);
}

void CarlaBackend::destroyEgo() {
    if (egoActor_) {
        egoActor_->Destroy();
        egoActor_ = nullptr;
    }
}

bool CarlaBackend::readEgoState(const std::string& /*egoId*/, EgoState& out) {
    if (!egoActor_) return false;   // EgoMode 0: no owned ego (driver readback path)
    carla::geom::Transform cTf = egoActor_->GetTransform();
    carla::geom::Vector3D  ext = egoActor_->GetBoundingBox().extent;
    carla::geom::Vector3D  vel = egoActor_->GetVelocity();
    carla::geom::Transform sTf = BridgeHelper::map_transfrom_Carla_to_Sumo(cTf, ext);
    out.x = sTf.location.x; out.y = sTf.location.y; out.z = sTf.location.z;
    out.heading = sTf.rotation.yaw;
    out.grade   = sTf.rotation.pitch * M_PI / 180.0;
    out.speed   = std::sqrt(vel.x * vel.x + vel.y * vel.y);
    return true;
}

std::string CarlaBackend::debugHeader() const {
    return ",carla_x,carla_y,carla_z,carla_yaw,carla_pitch";
}

std::string CarlaBackend::debugFields(VehHandle h) const {
    auto it = lastApplied_.find(h);
    if (it == lastApplied_.end()) return ",,,,,";
    const carla::geom::Transform& t = it->second;
    char buf[160];
    std::snprintf(buf, sizeof(buf), ",%.4f,%.4f,%.4f,%.4f,%.4f",
                  t.location.x, t.location.y, t.location.z, t.rotation.yaw, t.rotation.pitch);
    return std::string(buf);
}

} // namespace virenv
