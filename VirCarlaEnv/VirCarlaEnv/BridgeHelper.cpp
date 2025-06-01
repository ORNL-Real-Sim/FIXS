#include "BridgeHelper.h"

carla::geom::Transform BridgeHelper::map_transfrom_sumo_to_carla(const carla::geom::Transform& in_sumo_transform,
    const carla::geom::Vector3D& extent) {
    using namespace carla::geom;

    Location in_location = in_sumo_transform.location;
    Rotation in_rotation = in_sumo_transform.rotation;

    float yaw = -1.0f * in_rotation.yaw + 90.0f;
    float pitch = in_rotation.pitch;

    float x = in_location.x - std::cos(yaw * M_PI / 180.0f) * extent.x;
    float y = in_location.y - std::sin(yaw * M_PI / 180.0f) * extent.x;
    float z = in_location.z - std::sin(pitch * M_PI / 180.0f) * extent.x;

    x -= offset.x;
    y -= offset.y;

    Location out_location{ x, -y, z };
    Rotation out_rotation{ in_rotation.pitch, in_rotation.yaw - 90.0f, in_rotation.roll };

    return Transform(out_location, out_rotation);
}

carla::geom::Transform BridgeHelper::map_transfrom_Carla_to_Sumo(const carla::geom::Transform& in_carla_transform,
    const carla::geom::Vector3D& extent) {
    using namespace carla::geom;

    Location in_location = in_carla_transform.location;
    Rotation in_rotation = in_carla_transform.rotation;

    float yaw = -1.0f * in_rotation.yaw;
    float pitch = in_rotation.pitch;

    float x = in_location.x + std::cos(yaw * M_PI / 180.0f) * extent.x;
    float y = in_location.y - std::sin(yaw * M_PI / 180.0f) * extent.x;
    float z = in_location.z - std::sin(pitch * M_PI / 180.0f) * extent.x;

    x += offset.x;
    y -= offset.y;

    Location out_location{ x, -y, z };
    Rotation out_rotation{ in_rotation.pitch, in_rotation.yaw + 90.0f, in_rotation.roll };

    return Transform(out_location, out_rotation);
}

std::string BridgeHelper::map_sumo_vehicle_class_to_carla_typeId(const std::string& vclass)
{
    static const std::unordered_map<std::string, std::string> mapVclassToTypeid = {
        {"passenger",       "vehicle.tesla.model3"},
        {"passenger/hov",   "vehicle.tesla.model3"},
        {"bus",             "vehicle.mercedes.sprinter"},
        {"coach",           "vehicle.carlamotors.carlacola"},
        {"truck",           "vehicle.carlamotors.firetruck"},
        {"delivery",        "vehicle.ford.ambulance"},
        {"motorcycle",      "vehicle.kawasaki.ninja"},
        {"bicycle",         "vehicle.bh.crossbike"},
        {"pedestrian",      "walker.pedestrian.0001"},
        {"emergency",       "vehicle.nissan.patrol"},
        {"tram",            "static.prop.tram"},
    };

    auto it = mapVclassToTypeid.find(vclass);
    if (it != mapVclassToTypeid.end()) {
        return it->second;
    }
    else {
        // Default fallback
        return "vehicle.tesla.model3";
    }
}


//tempDoublePtr = static_pointer_cast<libsumo::TraCIDouble> (VehDataSubscribeTraciResults[libsumo::VAR_ANGLE]);
//CurVehData.heading = tempDoublePtr->value;

//tempDoublePtr = static_pointer_cast<libsumo::TraCIDouble> (VehDataSubscribeTraciResults[libsumo::VAR_SLOPE]);
//CurVehData.grade = tempDoublePtr->value * M_PI / 180;


//SumoActor GetActor(TraCIAPI& traci, const std::string& actor_id) {
//    // Get subscription results
//    auto results = traci.vehicle.getSubscriptionResults(actor_id);
//
//    // Extract attributes from subscription result
//    std::string type_id = results.at(traci::constants::VAR_TYPE).asString();
//    std::string vclass = results.at(traci::constants::VAR_VEHICLECLASS).asString();
//    std::vector<std::string> signals;  // Adapt if signal structure known
//
//    // Color is a 4-element tuple in SUMO: (r, g, b, a)
//    auto color_vec = results.at(traci::constants::VAR_COLOR).asColor();
//    carla::rpc::Color color(color_vec.r, color_vec.g, color_vec.b);
//
//    // Dimensions
//    float length = results.at(traci::constants::VAR_LENGTH).asFloat();
//    float width = results.at(traci::constants::VAR_WIDTH).asFloat();
//    float height = results.at(traci::constants::VAR_HEIGHT).asFloat();
//    carla::geom::Vector3D extent(length / 2.0f, width / 2.0f, height / 2.0f);
//
//    // Location (x, y, z)
//    auto pos3d = results.at(traci::constants::VAR_POSITION3D).asPosition3D();
//    float x = pos3d.x, y = pos3d.y, z = pos3d.z;
//
//    // Rotation: pitch = slope, yaw = angle, roll = 0.0
//    float pitch = results.at(traci::constants::VAR_SLOPE).asFloat();
//    float yaw = results.at(traci::constants::VAR_ANGLE).asFloat();
//    float roll = 0.0f;
//
//    carla::geom::Location location(x, y, z);
//    carla::geom::Rotation rotation(pitch, yaw, roll);
//    carla::geom::Transform transform(location, rotation);
//
//    // Construct and return the actor
//    return SumoActor(type_id, vclass, transform, signals, extent, color);
//}