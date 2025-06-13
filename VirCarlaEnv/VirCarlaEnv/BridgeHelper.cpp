#include "BridgeHelper.h"


// Default value for offset
// Town01 0.06,328.61
// Town04 503.02,423.76
carla::geom::Location BridgeHelper::offset = carla::geom::Location(0.06f, 328.61f, 0.0f);

carla::geom::Transform BridgeHelper::map_transfrom_Sumo_to_Carla(const carla::geom::Transform& in_sumo_transform,
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

std::string BridgeHelper::map_Sumo_vClass_to_Carla_blueprintId(const std::string& vClass)
{
    static const std::unordered_set<std::string> carlaCarsBlueprints = {
        "vehicle.audi.a2",
		"vehicle.audi.etron",
        "vehicle.audi.tt",
		"vehicle.bmw.grandtourer",
        "vehicle.chevrolet.impala",
		"vehicle.citroen.c3",
		"vehicle.dodge.charger_2020",
		"vehicle.ford.mustang",
		"vehicle.jeep.wrangler_rubicon",
		"vehicle.lincoln.mkz_2017",
		"vehicle.lincoln.mkz_2020",
		"vehicle.mercedes.coupe",
		"vehicle.mercedes.coupe_2020",
		"vehicle.micro.microlino",
        "vehicle.mini.cooper_s",
        "vehicle.mini.cooper_s_2021",
		"vehicle.nissan.micra",
		"vehicle.nissan.patrol",
		"vehicle.nissan.patrol_2021",
		"vehicle.seat.leon",
        "vehicle.tesla.model3",
        "vehicle.toyota.prius",
		"vehicle.ford.crown" // Also used for taxi
	};
    static const std::unordered_set<std::string> carlaTrucksBlueprints = {
		"vehicle.carlamotors.carlacola",
        "vehicle.carlamotors.european_hgv",
        "vehicle.tesla.cybertruck",
    };
    static const std::unordered_set<std::string> carlaVansBlueprints = {
		"vehicle.mercedes.sprinter",
        "vehicle.volkswagen.t2",
		"vehicle.volkswagen.t2_2021",
	};
    static const std::unordered_set<std::string> carlaBusesBlueprints = {
        "vehicle.mitsubishi.fusorosa",
    };
    static const std::unordered_set<std::string> carlaMotorcyclesBlueprints = {
		"vehicle.harley-davidson.low_rider",
        "vehicle.kawasaki.ninja",
        "vehicle.vespa.zx125",
        "vehicle.yamaha.yzf",
    };
    static const std::unordered_set<std::string> carlaBicyclesBlueprints = {
        "vehicle.bh.crossbike",
        "vehicle.diamondback.century",
        "vehicle.gazelle.omafiets",
	};
    
    static const std::unordered_set<std::string> carlaPedestriansBlueprints = {
        "walker.pedestrian.0001",
        "walker.pedestrian.0002",
        "walker.pedestrian.0003",
        "walker.pedestrian.0004",
        "walker.pedestrian.0005",
	};
	//Note in the Carla Beprints, the emergency vehicles are not separated by type
    static const std::unordered_set<std::string> carlaEmergencyBlueprints = {
		"vehicle.ford.ambulance", // Vans
        "vehicle.carlamotors.firetruck", // Truck
		"vehicle.dodge.charger_police", // Car
        "vehicle.dodge.charger_police_2020", // Car
	};


    std::string carlaBlueprintId = "";
	//std::string carlaBlueprintId = "vehicle.tesla.model3"; // Default to be passenger car
    if (vClass == "passenger"){
		carlaBlueprintId = random_select_from_set(carlaCarsBlueprints);
	}
	else if (vClass == "truck") {
        carlaBlueprintId = random_select_from_set(carlaTrucksBlueprints);
    }
    else if (vClass == "van") {
        carlaBlueprintId = random_select_from_set(carlaVansBlueprints);
    }
    else if (vClass == "bus") {
        carlaBlueprintId = random_select_from_set(carlaBusesBlueprints);
    }
    else if (vClass == "motorcycle") {
        carlaBlueprintId = random_select_from_set(carlaMotorcyclesBlueprints);
    }
    else if (vClass == "bicycle") {
        carlaBlueprintId = random_select_from_set(carlaBicyclesBlueprints);
    }
    else if (vClass == "pedestrian") {
        carlaBlueprintId = random_select_from_set(carlaPedestriansBlueprints);
    }
    else if (vClass == "emergency") {
        carlaBlueprintId = random_select_from_set(carlaEmergencyBlueprints);
    }
    else {
		std::cerr << "Unknown vClass: " << vClass << std::endl
		<< "Currently supported vClasses are:" << std::endl
        << "passenger, truck, van, bus, motorcycle, bicycle, pedestrian, emergency." << std::endl
		<< "Defaulting to vehicle.tesla.model3." << std::endl;
        carlaBlueprintId = "vehicle.tesla.model3"; // Default to be passenger car
    }
	return carlaBlueprintId;
}

SumoTrafficLightState map_Carla_traffic_light_state_to_Sumo(carla::rpc::TrafficLightState carlaTrafficLightState) {
    using namespace carla::rpc;
    switch (carlaTrafficLightState) {
    case TrafficLightState::Red:
        return SumoTrafficLightState::RED;
    case TrafficLightState::Yellow:
        return SumoTrafficLightState::YELLOW;
    case TrafficLightState::Green:
        return SumoTrafficLightState::GREEN;
    case TrafficLightState::Off:
        return SumoTrafficLightState::OFF;
    case TrafficLightState::Unknown:
    default:
        return SumoTrafficLightState::OFF;
    }
}

SumoTrafficLightState get_Sumo_traffic_light_state_from_char(char c) {
    switch (c) {
    case 'r': return SumoTrafficLightState::RED;
    case 'y': return SumoTrafficLightState::YELLOW;
    case 'G': return SumoTrafficLightState::GREEN;
    case 'g': return SumoTrafficLightState::GREEN_WITHOUT_PRIORITY;
    case 's': return SumoTrafficLightState::GREEN_RIGHT_TURN;
    case 'u': return SumoTrafficLightState::RED_YELLOW;
    case 'o': return SumoTrafficLightState::OFF_BLINKING;
    case 'O': return SumoTrafficLightState::OFF;
    default:
        throw std::invalid_argument("Unknown SumoTrafficLightState char: " + std::string(1, c));
    }
}

char Sumo_traffic_light_state_to_char(SumoTrafficLightState state) {
    return static_cast<char>(state);
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