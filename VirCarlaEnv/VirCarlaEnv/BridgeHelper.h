#pragma once

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
#include "carla/rpc/TrafficLightState.h"
#define M_PI 3.14159265358979323846
#define SET_CONTAINS_ID(set, value) ((set).find(value) != (set).end())

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

    static std::string map_Sumo_vClass_to_Carla_blueprintId(const std::string& vclass);
};



struct Actor {
    std::string id;
    std::string vClass;
    carla::geom::Transform transform;
    carla::geom::Vector3D extent;
	// Additional properties can be added as needed, e.g.:
	bool flagged = false; // Indicates if the actor is flagged for some reason
    //std::vector<std::string> signals;
    //carla::rpc::Color color;

    Actor() = default;

    Actor(const std::string& _id,
        const std::string& _vClass,
        const carla::geom::Transform& _transform,
        const carla::geom::Vector3D& _extent)
        : id(_id),
        vClass(_vClass),
        transform(_transform),
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

char Sumo_traffic_light_state_to_char(SumoTrafficLightState state);

SumoTrafficLightState get_Sumo_traffic_light_state_from_char(char c);

SumoTrafficLightState map_Carla_traffic_light_state_to_Sumo(carla::rpc::TrafficLightState carlaTrafficLightState);