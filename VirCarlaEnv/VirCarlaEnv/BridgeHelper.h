#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <string>
#include <tuple>
#include <cmath>
#include <random>
#include <stdexcept>
#include "RandomUtils.h"
#include <carla/geom/Transform.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Vector3D.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/Vehicle.h>
#include "carla/rpc/TrafficLightState.h"
#define M_PI 3.14159265358979323846
#define SET_CONTAINS_ID(set, value) ((set).find(value) != (set).end())

struct SumoActor {
    std::string id;
    std::string vType;
    std::string vClass;
    carla::geom::Transform sumoTransform;
    carla::geom::Transform carlaTransform;
    // The extent of the actor is the size of the bounding box in Carla
    carla::geom::Vector3D extent;
    // Additional properties can be added as needed, e.g.:
    // Indicates if the actor is spawned in Carla
    bool spawnedInCarla = false;
    carla::SharedPtr<carla::client::Vehicle> carlaVehicleActorPtr = nullptr;
    //std::vector<std::string> signals;
    //carla::rpc::Color color;

    SumoActor() = default;
    // Initialize an Actor with all required fields
    // (vehicle id, vehicle type, vehicle class, transform, extent)
    SumoActor(const std::string& _id,
        const std::string& _vType,
        const std::string& _vClass,
        const carla::geom::Transform& _sumoTransform,
        const carla::geom::Vector3D& _extent)
        : id(_id),
        vType(_vType),
        vClass(_vClass),
        sumoTransform(_sumoTransform),
        extent(_extent) {
    }
};

// Sumo signal state
enum class SumoTrafficLightState : char {
    RED = 'r',
    YELLOW = 'y',
    GREEN = 'G',
    GREEN_WITHOUT_PRIORITY = 'g',
    GREEN_RIGHT_TURN = 's',
    RED_YELLOW = 'u',
    OFF_BLINKING = 'o',
    OFF = 'O'
};

struct TrafficLight {
    std::string junctionId;
    int linkId;
    double x, y, z;
    double heading;
    SumoTrafficLightState state = SumoTrafficLightState::OFF;
    std::string carlaTrafficLightActorId = "";
    carla::SharedPtr<carla::client::TrafficLight> carlaTrafficLightActorPtr = nullptr;
    // Current state of the traffic light

    TrafficLight() = default;
    TrafficLight(std::string _junctionId, int _linkId, double _x, double _y, double _z, double _heading)
        : junctionId(_junctionId), linkId(_linkId), x(_x), y(_y), z(_z), heading(_heading) {}
};


class BridgeHelper {
public:
    // Global offset between SUMO and Carla coordinate systems
    static carla::geom::Location offset;

    // Convert SUMO → Carla
    static carla::geom::Transform map_transfrom_Sumo_to_Carla(const carla::geom::Transform& in_sumo_transform,
        const carla::geom::Vector3D& extent);

    // Convert Carla → SUMO
    static carla::geom::Transform map_transfrom_Carla_to_Sumo(const carla::geom::Transform& in_carla_transform,
        const carla::geom::Vector3D& extent);

    static carla::geom::Location map_location_Carla_to_Sumo(const carla::geom::Location& in_carla_location);

    static std::string map_Sumo_vClass_to_Carla_blueprintId(const std::string& vclass);

    static std::unordered_map<std::string, std::unordered_map<int, TrafficLight>> readTrafficLightTable(const std::string& filename);

    static char Sumo_traffic_light_state_to_char(SumoTrafficLightState state);

    static SumoTrafficLightState get_Sumo_traffic_light_state_from_char(char c);

    static SumoTrafficLightState map_Carla_traffic_light_state_to_Sumo(carla::rpc::TrafficLightState carlaTrafficLightState);

    static carla::rpc::TrafficLightState map_Sumo_traffic_light_state_to_Carla(SumoTrafficLightState& sumoTrafficLightState);

    static std::pair<std::string, int> find_closest_trafficLight_id(
        std::unordered_map<std::string, std::unordered_map<int, TrafficLight>>& trafficLightMap,
        double x, double y);

};


