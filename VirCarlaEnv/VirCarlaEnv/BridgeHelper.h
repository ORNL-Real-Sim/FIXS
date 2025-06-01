#pragma once

#include <unordered_map>
#include <string>
#include <tuple>
#include <cmath>
#include <carla/geom/Transform.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Vector3D.h>

#define M_PI 3.14159265358979323846
#define SET_CONTAINS_ID(set, value) ((set).find(value) != (set).end())

class BridgeHelper {
public:
    // Global offset between SUMO and Carla coordinate systems
    static carla::geom::Location offset;

    // Convert SUMO → Carla
    static carla::geom::Transform map_transfrom_sumo_to_carla(const carla::geom::Transform& in_sumo_transform,
        const carla::geom::Vector3D& extent);

    // Convert Carla → SUMO
    static carla::geom::Transform map_transfrom_Carla_to_Sumo(const carla::geom::Transform& in_carla_transform,
        const carla::geom::Vector3D& extent);

    static std::string map_sumo_vehicle_class_to_carla_typeId(const std::string& vclass);
};

// Default value for offset
carla::geom::Location BridgeHelper::offset = carla::geom::Location(0.0f, 0.0f, 0.0f);

struct Actor {
    std::string id;
    std::string vclass;
    carla::geom::Transform transform;
    carla::geom::Vector3D extent;
    //std::vector<std::string> signals;
    //carla::rpc::Color color;

    Actor(const std::string& _id,
              const std::string& _vclass,
              const carla::geom::Transform& _transform,
              const carla::geom::Vector3D& _extent)
        : id(_id),
          vclass(_vclass),
          transform(_transform),
          extent(_extent) {}
};