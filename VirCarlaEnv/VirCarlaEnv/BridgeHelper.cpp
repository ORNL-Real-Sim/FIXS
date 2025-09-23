#include "BridgeHelper.h"


// Default value for offset
// Town01 0.06,328.61
// Town04 503.02,423.76
//carla::geom::Location BridgeHelper::offset = carla::geom::Location(0.06f, 328.61f, 0.0f);
carla::geom::Location BridgeHelper::offset = carla::geom::Location(0.0f, 0.0f, 0.0f);
carla::geom::Transform BridgeHelper::map_transfrom_Sumo_to_Carla(const carla::geom::Transform& in_sumo_transform,
    const carla::geom::Vector3D& extent) {

    carla::geom::Location in_location = in_sumo_transform.location;
    carla::geom::Rotation in_rotation = in_sumo_transform.rotation;

    float yaw = -1.0f * in_rotation.yaw + 90.0f;
    float pitch = in_rotation.pitch;

    float x = in_location.x - std::cos(yaw * M_PI / 180.0f) * extent.x;
    float y = in_location.y - std::sin(yaw * M_PI / 180.0f) * extent.x;
    float z = in_location.z - std::sin(pitch * M_PI / 180.0f) * extent.x;

    x -= offset.x;
    y -= offset.y;

    carla::geom::Location out_location{ x, -y, z };
    carla::geom::Rotation out_rotation{ in_rotation.pitch, in_rotation.yaw - 90.0f, in_rotation.roll };

    return carla::geom::Transform(out_location, out_rotation);
}

carla::geom::Transform BridgeHelper::map_transfrom_Carla_to_Sumo(const carla::geom::Transform& in_carla_transform,
    const carla::geom::Vector3D& extent) {

    carla::geom::Location in_location = in_carla_transform.location;
    carla::geom::Rotation in_rotation = in_carla_transform.rotation;

    float yaw = -1.0f * in_rotation.yaw;
    float pitch = in_rotation.pitch;

    float x = in_location.x + std::cos(yaw * M_PI / 180.0f) * extent.x;
    float y = in_location.y - std::sin(yaw * M_PI / 180.0f) * extent.x;
    float z = in_location.z - std::sin(pitch * M_PI / 180.0f) * extent.x;

    x += offset.x;
    y -= offset.y;

    carla::geom::Location out_location{ x, -y, z };
    carla::geom::Rotation out_rotation{ in_rotation.pitch, in_rotation.yaw + 90.0f, in_rotation.roll };

    return carla::geom::Transform(out_location, out_rotation);
}

carla::geom::Location BridgeHelper::map_location_Carla_to_Sumo(const carla::geom::Location& in_carla_location) {

    carla::geom::Location in_location = in_carla_location;

    float x = in_location.x;
    float y = in_location.y;
    float z = in_location.z;

    x += offset.x;
    y -= offset.y;

    carla::geom::Location out_location{ x, -y, z };

    return out_location;
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

SumoTrafficLightState BridgeHelper::map_Carla_traffic_light_state_to_Sumo(carla::rpc::TrafficLightState carlaTrafficLightState) {

    switch (carlaTrafficLightState) {
    case carla::rpc::TrafficLightState::Red:
        return SumoTrafficLightState::RED;
    case carla::rpc::TrafficLightState::Yellow:
        return SumoTrafficLightState::YELLOW;
    case carla::rpc::TrafficLightState::Green:
        return SumoTrafficLightState::GREEN;
    case carla::rpc::TrafficLightState::Off:
        return SumoTrafficLightState::OFF;
    case carla::rpc::TrafficLightState::Unknown:
    default:
        return SumoTrafficLightState::OFF;
    }
}


carla::rpc::TrafficLightState BridgeHelper::map_Sumo_traffic_light_state_to_Carla(SumoTrafficLightState& sumoTrafficLightState) {
    // Map SumoTrafficLightState to carla::rpc::TrafficLightState
    if (sumoTrafficLightState == SumoTrafficLightState::RED ||
        sumoTrafficLightState == SumoTrafficLightState::RED_YELLOW) {
		return carla::rpc::TrafficLightState::Red;
	}
    else if (sumoTrafficLightState == SumoTrafficLightState::YELLOW) {
		return carla::rpc::TrafficLightState::Yellow;
	}
    else if (sumoTrafficLightState == SumoTrafficLightState::GREEN ||
        sumoTrafficLightState == SumoTrafficLightState::GREEN_WITHOUT_PRIORITY) {
		return carla::rpc::TrafficLightState::Green;
	}
    else if (sumoTrafficLightState == SumoTrafficLightState::OFF) {
		return carla::rpc::TrafficLightState::Off;
	}
	else { // SumoTrafficLightState::GREEN_RIGHT_TURN and SumoTrafficLightState::OFF_BLINKING
    		return carla::rpc::TrafficLightState::Unknown;
    	}
}


SumoTrafficLightState BridgeHelper::get_Sumo_traffic_light_state_from_char(char c) {
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

char BridgeHelper::Sumo_traffic_light_state_to_char(SumoTrafficLightState state) {
    return static_cast<char>(state);
}

std::unordered_map<std::string, std::unordered_map<int, TrafficLight>> BridgeHelper::readTrafficLightTable(const std::string& filename) {
    std::unordered_map<std::string, std::unordered_map<int, TrafficLight>> trafficLightMap;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return trafficLightMap;
    }

    // Skip header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        std::string junctionId;
        int linkId;
        double x, y, z, heading;

        std::getline(ss, junctionId, ',');
        std::getline(ss, token, ',');
        linkId = std::stoi(token);
        std::getline(ss, token, ',');
        x = std::stod(token);
        std::getline(ss, token, ',');
        y = std::stod(token);
        std::getline(ss, token, ',');
        z = std::stod(token);
        std::getline(ss, token, ',');
        heading = std::stod(token);

        TrafficLight trafficLight(junctionId, linkId, x, y, z, heading);
        trafficLightMap[junctionId][linkId] = trafficLight;
    }

    return trafficLightMap;
}


std::pair<std::string, int> BridgeHelper::find_closest_trafficLight_id(
    std::unordered_map<std::string, std::unordered_map<int, TrafficLight>>& trafficLightMap,
    double x, double y
) {
    double min_dist = std::numeric_limits<double>::max();
    std::pair<std::string, int> closest_ids = { "", -1 };

    for (const std::pair<std::string, std::unordered_map<int, TrafficLight>>& pair : trafficLightMap) {
        const std::string& junctionId = pair.first;
        const std::unordered_map<int, TrafficLight>& linkMap = pair.second;
        for (const std::pair<int, TrafficLight>& pair : linkMap) {
            const int& linkId = pair.first;
            const TrafficLight& trafficLight = pair.second;
            double dx = trafficLight.x - x;
            double dy = trafficLight.y - y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist) {
                min_dist = dist_sq;
                closest_ids = { junctionId, linkId };
            }
        }
    }

    return closest_ids;
}