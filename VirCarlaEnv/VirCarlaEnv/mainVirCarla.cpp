#pragma once
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
#include <chrono>

#include <carla/client/World.h>
#include <carla/Time.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/DebugHelper.h>
#include <carla/client/Client.h>
#include <carla/client/ActorList.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/trafficmanager/TrafficManager.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Rotation.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <carla/rpc/ActorDescription.h>
#include <carla/Memory.h>
#include "BridgeHelper.h"
#include "MsgHelper.h"
#include "SocketHelper.h"
#include "ConfigHelper.h"
#include "DebugHelper.h"

//preset delay durations
carla::time_duration timeout_10s = std::chrono::seconds(10);
carla::time_duration timeout_1s = std::chrono::seconds(1);
carla::time_duration timeout_50ms = std::chrono::milliseconds(50);
carla::time_duration timeout_100ms = std::chrono::duration<double>(0.1);  // 0.1 second

using namespace std::chrono_literals;
using namespace std::string_literals;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }
#define SET_CONTAINS_ID(set, value) ((set).find(value) != (set).end())
#define MAP_CONTAINS_KEY(map, key) ((map).find(key) != (map).end())
#define SPAWN_OFFSET_Z 0.1f // Offset for spawning actors above the ground

static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << std::endl
        << "Options:\n"
        << "\t-h,--help\t\tShow this help message\n"
        << "\t-f,--file PATH\\FILENAME\tSpecify the path and filename of configuration yaml file"
        << std::endl;
}

double periodicCosineSpeed(double t, double T_period, double v_max) {
    if (T_period <= 0.0) return 0.0;

    // Time within the current period
    double t_in_period = std::fmod(t, T_period);

    // Compute speed using cosine profile within the period
    return 0.5 * v_max * (1 - std::cos(2 * M_PI * t_in_period / T_period));
}

float lowPassFilter(float currentValue, float lastValue, float alpha) {
    return alpha * currentValue + (1 - alpha) * lastValue;
}

int main(int argc, const char* argv[]) {
    std::string CarlaClientLogFile = "CarlaClient.log";

    time_t simStartTimestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char simStartTimestampChar[100];
    ctime_s(simStartTimestampChar, sizeof simStartTimestampChar, &simStartTimestamp);
    std::fstream f(CarlaClientLogFile, std::fstream::in | std::fstream::out | std::fstream::app);
    f << std::endl << "=============================================" << std::endl;
    f << "Carla Client Starts at  " << simStartTimestampChar << std::endl;
    f.close();
    MsgHelper msgHelper;
    ConfigHelper configHelper;
    SocketHelper socketHelper;
    std::string configPath = "";
    std::string trafficLightMapPath = "";
    // ===========================================================================
	// 			Parse Arguments
    // ===========================================================================
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            show_usage(argv[0]);
            return 0;
        }
        else if (arg == "-f" || arg == "--file") {
            if (i + 1 < argc) {
                configPath = argv[++i];
            }
            else {
                std::cerr << "--path option requires one argument." << std::endl;
                return -1;
            }
        }
        else if (arg == "-t" || arg == "--tls") {
            if (i + 1 < argc) {
                trafficLightMapPath = argv[++i];
            }
            else {
                std::cerr << "--path option requires one argument." << std::endl;
                return -1;
            }
        }
        else {
            printf("Check options\n");
            show_usage(argv[0]);
            return 0;
        }
    }
    // ===========================================================================
    // 			READ Config File
    // ===========================================================================
    printf("Reading Configuration file %s\n", configPath.c_str());
    if (configHelper.getConfig(configPath) < 0) {
        printf("Please check path and filename of the configuration yaml\n");
        show_usage(argv[0]);
        exit(-1);
    }
    else {
        printf("Read configuration file success\n");
    }
    // Set the simulation end time
    uint32_t simEndTime = configHelper.SimulationSetup.SimulationEndTime;
    bool enableVerboseLog = configHelper.CarlaSetup.EnableVerboseLog;
    CarlaSetup_t carlaSetup = configHelper.CarlaSetup;

    std::string carlaServerIp = carlaSetup.CarlaServerIP;
    int carlaServerPort = carlaSetup.CarlaServerPort;
	std::string carlaClientIp = carlaSetup.CarlaClientIP;
	int carlaClientPort = carlaSetup.CarlaClientPort;
	//std::string carlaMapName = carlaSetup.CarlaMapName;
	double trafficRefreshRate = carlaSetup.TrafficRefreshRate;
    const std::chrono::milliseconds step_duration_ms(static_cast<int>(trafficRefreshRate * 1000));
	// If the vehicle type is used as the blueprint ID, set this to true
    bool useVhicleTypeAsBlueprint = carlaSetup.UseVehicleTypeAsBlueprint;
	// Enable external control for interested vehicles, if turned off, the interested vehicles will be controlled by the Sumo
    bool enableExternalControl = carlaSetup.EnableExternalControl;
    // centeredViewId is the id of the camera that is used to render the view in Carla      
    std::string centeredViewId = configHelper.CarlaSetup.CenteredViewId;
    // Enable traffic light synchronization
    /*bool enableTlsSync = configHelper.CarlaSetup.EnableTrafficLightSync;*/
    bool enableTlsSync = true;
    std::unordered_set<std::string> setInterestedIds;
    for (const std::string& id : carlaSetup.InterestedIds) {
        setInterestedIds.insert(id);
	}
	// This is important for the reveived data parser to work correctly
    msgHelper.getConfig(configHelper);
	// Setup Connection to the Traffic Server
    //void SocketHelper::socketSetup(vector <string> SERVERADDR_UserInput, vector <int> SERVERPORT_UserInput) {
	std::vector<std::string> SERVERADDR_UserInput = { carlaClientIp };
    std::vector<int> SERVERPORT_UserInput = { carlaClientPort };
    socketHelper.disableWaitClientTrigger();
    socketHelper.disableServerTrigger();
	socketHelper.socketSetup(SERVERADDR_UserInput, SERVERPORT_UserInput);
    // ===========================================================================
    // 			READ Traffic Light Table
    // ===========================================================================
    std::unordered_map<std::string, std::unordered_map<int, TrafficLight>> trafficLightMap = BridgeHelper::readTrafficLightTable(trafficLightMapPath);

    if (trafficLightMap.empty()) {
		std::cerr << "No traffic light data found in the file: " << trafficLightMapPath << std::endl;
		return -1;
	}

    if (socketHelper.initConnection(CarlaClientLogFile) < 0) {
        printf("Connect to Traffic Layer failed!\n");
        exit(-1);
    }
	// there is only one traffic layer server
    int sockId = 0; 
    try {
        carla::client::Client carlaClient = carla::client::Client(carlaServerIp, carlaServerPort);
		// initialize carla client and world
        std::cout << "Client API version : " << carlaClient.GetClientVersion() << '\n';
        std::cout << "Server API version : " << carlaClient.GetServerVersion() << '\n';
        carlaClient.SetTimeout(timeout_10s);
		carla::client::World carlaWorld = carlaClient.GetWorld();
        carla::SharedPtr<carla::client::Actor> carlaSpectator = carlaWorld.GetSpectator();

        // Enable synchronous mode
        carla::rpc::EpisodeSettings carlaWorldSettings = carlaWorld.GetSettings();
        if (!carlaWorldSettings.synchronous_mode) {
            carlaWorldSettings.synchronous_mode = true;            // Turn on synchronous mode
			carlaWorldSettings.fixed_delta_seconds = trafficRefreshRate;         // time step of 0.1 seconds
            //the time_out is of carla::time_duration
            carlaWorld.ApplySettings(carlaWorldSettings, timeout_1s);
            if (enableVerboseLog) std::cout << "Synchronous mode enabled.\n";
        }
        carla::SharedPtr<carla::client::BlueprintLibrary> blueprint_library = carlaWorld.GetBlueprintLibrary();

		// Map the Sumo Ids to Carla Ids
		std::unordered_map<std::string, std::string> mapSumoToCarla;
		// Map the Carla Ids to Sumo Ids
		std::unordered_map<std::string, std::string> mapCarlaToSumo;

        float simTime = 0; // Initialize the current simulation time
        std::unordered_map<std::string, SumoActor> mapSumoActor;
		// command batch to set the transform of the actors (non-interested actors)
        std::vector<carla::rpc::Command> transformCommandBatch;

        if (enableTlsSync) {
            carla::SharedPtr <carla::client::ActorList> carlaTrafficLightActors = carlaWorld.GetActors()->Filter("traffic.traffic_light");

            for (const carla::SharedPtr<carla::client::Actor>& carlaActor : *carlaTrafficLightActors) {
                carla::SharedPtr<carla::client::TrafficLight> carlaTrafficLightActorPtr = boost::static_pointer_cast<carla::client::TrafficLight>(carlaActor);
                carlaTrafficLightActorPtr->Freeze(true);
                std::string carlaTrafficLightActorId = std::to_string(carlaActor->GetId());
                // Convert Carla location to SUMO coordinate
                carla::geom::Location carlaLocation = carlaActor->GetLocation();
                carla::geom::Location sumoLocation = BridgeHelper::map_location_Carla_to_Sumo(carlaLocation);

                // Find closest TLS from map
                std::pair<std::string, int> trafficLightId = BridgeHelper::find_closest_trafficLight_id(
                    trafficLightMap,
                    sumoLocation.x,
                    sumoLocation.y
                );
                std::string junctionId = trafficLightId.first;
                int linkId = trafficLightId.second;

                if (junctionId == "") {
                    std::cerr << "No matching traffic light found for actor " << carlaTrafficLightActorId << "\n";
                    continue;
                }

                // Assign traffic light attributes
                TrafficLight& trafficLight = trafficLightMap[junctionId][linkId];
                trafficLight.carlaTrafficLightActorId = carlaTrafficLightActorId; // Store the Carla actor ID in the traffic light data
                trafficLight.carlaTrafficLightActorPtr = carlaTrafficLightActorPtr; // Store the Carla actor pointer in the traffic light data
            }
        }
        
        while (simTime < simEndTime) {
            std::chrono::steady_clock::time_point loop_start_time = std::chrono::steady_clock::now();
            ///***********************
            // RUN one-step simulation
            ///***********************


            // ==========================================
			// Receive data from the traffic layer server
            // ==========================================
            int simStateRecv;
            float simTimeRecv;
            msgHelper.clearRecvStorage();
            for (unsigned int iServer = 0; iServer < socketHelper.serverSock.size(); iServer++) {

                int simStateRecv;
                float simTimeRecv;

                if (enableVerboseLog) {
                    printf("receiving server at port %d\n", socketHelper.SERVERPORT[iServer]);

                    FILE* f = fopen(CarlaClientLogFile.c_str(), "a");
                    fprintf(f, "recv server: %d\n", socketHelper.SERVERPORT[iServer]);
                    fclose(f);
                }

                // save received message into Msg_c recv storages
                if (socketHelper.recvData(socketHelper.serverSock[iServer], &simStateRecv, &simTimeRecv, msgHelper) < 0) {
                    if (WSAGetLastError() != WSAEINTR && WSAGetLastError() != WSAEFAULT) {
                        printf("ERROR: receive from server fails\n");
                    }
                    socketHelper.socketShutdown();
                    exit(-1);
                };
            }

            std::unordered_set<std::string> setCurrentSumoIds;
            for (const std::pair<const std::string, VehFullData_t>& pair : msgHelper.VehDataRecv_um) {
                const VehFullData_t& tmpVehData = pair.second;
				carla::geom::Location tmpLocation(tmpVehData.positionX, tmpVehData.positionY, tmpVehData.positionZ);
				// The grade received from FIXS is in radians, convert to degrees
                //carla::geom::Rotation tmpRotation(tmpVehData.grade * 180/M_PI, tmpVehData.heading, 0.0f);
                carla::geom::Rotation tmpRotation(0.0f, std::round(tmpVehData.heading / 5.0f) * 5.0f, 0.0f);
                //carla::geom::Vector3D tmpExtent(tmpVehData.length / 2, tmpVehData.width / 2, tmpVehData.height / 2);
                carla::geom::Vector3D tmpExtent(4.8f, 2.0f, 1.8f); // Default extent, can be modified later
                carla::geom::Transform tmpTransform(tmpLocation, tmpRotation);

                if (!MAP_CONTAINS_KEY(mapSumoActor, tmpVehData.id)) {
                    mapSumoActor[tmpVehData.id] = SumoActor(tmpVehData.id, tmpVehData.type, tmpVehData.vehicleClass, tmpTransform, tmpExtent);
                    std::cout << "Vehicle Class: " << tmpVehData.vehicleClass << std::endl;
                }
                else {
                    //// Update the existing actor's transform and extent
                    //carla::geom::Location prvLocation = mapSumoActor[tmpVehData.id].sumoTransform.location;
                    //carla::geom::Rotation prvRotation = mapSumoActor[tmpVehData.id].sumoTransform.rotation;
                    //carla::geom::Transform updatedTransform(
                    //    carla::geom::Location(
                    //        lowPassFilter(tmpTransform.location.x, prvLocation.x, 0.5f),
                    //        lowPassFilter(tmpTransform.location.y, prvLocation.y, 0.5f),
                    //        lowPassFilter(tmpTransform.location.z, prvLocation.z, 0.5f)
                    //    ), carla::geom::Rotation(
                    //        lowPassFilter(tmpTransform.rotation.pitch, prvRotation.pitch, 0.5f),
                    //        lowPassFilter(tmpTransform.rotation.yaw, prvRotation.yaw, 0.5f),
                    //        lowPassFilter(tmpTransform.rotation.roll, prvRotation.roll, 0.5f)
                    //    )
                    //);
                    mapSumoActor[tmpVehData.id].sumoTransform = tmpTransform;
                    mapSumoActor[tmpVehData.id].extent = tmpExtent;
				}
				setCurrentSumoIds.insert(tmpVehData.id);
            }
            if (enableTlsSync) {
                for (const std::pair<const std::string, TrafficLightData_t>& pair : msgHelper.TlsDataRecv_um) {
                    const std::string& junctionID = pair.first;
                    const TrafficLightData_t& tmpTrafficLightData = pair.second;
                    const std::string& tafficLightStateStr = tmpTrafficLightData.state;
                    if (enableVerboseLog) {
                        std::cout << "Received Traffic Light Data for junction: " << junctionID << std::endl;
                        std::cout << "ID: " << tmpTrafficLightData.id << std::endl;
                        std::cout << "Name: " << tmpTrafficLightData.name << std::endl;
                        std::cout << "State: " << tmpTrafficLightData.state << std::endl;
                    }
                    // traverse each char in the state str
                    for (size_t linkId = 0; linkId < tafficLightStateStr.size(); ++linkId) {
                        char stateChar = tafficLightStateStr[linkId];
                        SumoTrafficLightState sumoTrafficLightState = BridgeHelper::get_Sumo_traffic_light_state_from_char(stateChar);
                        carla::rpc::TrafficLightState carlaTrafficLightState = BridgeHelper::map_Sumo_traffic_light_state_to_Carla(sumoTrafficLightState);
                        // set the traffic light state in Carla, if the traffic light exists in the map
                        TrafficLight& trafficLight = trafficLightMap[junctionID][linkId];

                        if (trafficLight.carlaTrafficLightActorPtr) {
                            trafficLight.carlaTrafficLightActorPtr->SetState(carlaTrafficLightState);
                        }
                        else {
                            if (enableVerboseLog) std::cerr << "Carla actor for traffic light " << tmpTrafficLightData.id << "Junction Id:" << junctionID << "Link Id: " << linkId << " not found." << std::endl;
                        }
                    }
                }
            }
            
			// Check if the mapSumoActor contains vehicles that are not in the current step (vehicles have left the simulation)
			// If so, remove them from the mapSumoActor and mapCarlaActor
            for (std::unordered_map<std::string, SumoActor>::iterator it = mapSumoActor.begin(); it != mapSumoActor.end(); ) {
                std::string sumoActorId = it->first;
                SumoActor& sumoActor = it->second;
                if (!SET_CONTAINS_ID(setCurrentSumoIds, sumoActorId)) {
                    if(enableVerboseLog) std::cout << "Removing Sumo actor with ID: " << sumoActorId << std::endl;
                    // Remove associated Carla actor
                    std::string carlaActorId = mapSumoToCarla[sumoActorId];
                    carla::SharedPtr<carla::client::Vehicle> carlaVehicleActorPtr = sumoActor.carlaVehicleActorPtr;
                    if (carlaVehicleActorPtr) {
                        carlaVehicleActorPtr->Destroy();
                        if (enableVerboseLog) std::cout << "Destroyed Carla actor with ID: " << carlaActorId << std::endl;
                    }
                    mapSumoToCarla.erase(sumoActorId);
                    mapCarlaToSumo.erase(carlaActorId);
                    // Safely erase from mapSumoActor and advance the iterator
					sumoActor.carlaVehicleActorPtr = nullptr; // Clear the pointer to avoid dangling references
                    it = mapSumoActor.erase(it);
                }
                else {
                    ++it;
                }
            }

			// Get the Carla Actor Role Name Mapping
            carla::SharedPtr <carla::client::ActorList> carlaActors = carlaWorld.GetActors()->Filter("vehicle.*");
			std::unordered_map<std::string, std::string> mapActorIdToRoleName;
            std::unordered_map<std::string, std::string> mapRoleNameToActorId;
            for (const carla::SharedPtr<carla::client::Actor>& carlaActor : *carlaActors) {
                const std::vector<carla::client::ActorAttributeValue> carlaAttributes = carlaActor->GetAttributes(); // Ensure the attributes are loaded
                for (const carla::client::ActorAttributeValue& attr : carlaAttributes) {
                    //std::cout << "Attribute ID: " << attr.GetId() << ", value: " << attr.GetValue() << std::endl;
                    if (attr.GetId() == "role_name") {
                        std::string carlaRoleName = attr.GetValue();
						std::string carlaActorId = std::to_string(carlaActor->GetId());
                        mapActorIdToRoleName[carlaActorId] = carlaRoleName;
						mapRoleNameToActorId[carlaRoleName] = carlaActorId;
						break; // No need to check further attributes for this actor
                    }

                }
			}

            for (std::pair<const std::string, SumoActor>& pair : mapSumoActor) {
                const std::string& sumoActorId = pair.first;
                SumoActor& sumoActor = pair.second;
                

                carla::geom::Transform carlaTransform = BridgeHelper::map_transfrom_Sumo_to_Carla(sumoActor.sumoTransform, sumoActor.extent);

                //carla::SharedPtr<carla::client::Waypoint> carlaWaypoint = carlaMap->GetWaypoint(carlaTransform.location);
                //carla::geom::Transform waypointTransform = carlaWaypoint->GetTransform();
                // =======================================================
                // [Optianal] Get the waypoint of the carla transform
                // This is to ensure that the carla transform is on the road
                // =======================================================
                //carlaTransform.location = waypointTransform.location;
                //Use the waypoint's rotation to ensure the vehicle is aligned with the road
				//carlaTransform.rotation = waypointTransform.rotation; 
                //print the sumo and carla transform

                
                // =======================================================
                // Spawn the Sumo actors that are not in the current step
                // =======================================================
                if (!MAP_CONTAINS_KEY(mapSumoToCarla , sumoActorId) || !sumoActor.spawnedInCarla || sumoActor.carlaVehicleActorPtr ==nullptr) {
                    // Add a small offset to the z coordinate to avoid collision with the ground
                    carlaTransform.location.z = carlaTransform.location.z + SPAWN_OFFSET_Z;
					

                    std::string carlaActorTypeId;
                    if (useVhicleTypeAsBlueprint) {
                        carlaActorTypeId = sumoActor.vType;
                    }
                    else {
                        carlaActorTypeId = BridgeHelper::map_Sumo_vClass_to_Carla_blueprintId(sumoActor.vClass);
                    }

                    const carla::client::ActorBlueprint* vehicle_blueprint = blueprint_library->Find(carlaActorTypeId);
					// C++ API does not allow modifying a blueprint after retrieval.
                    // To work around this in C++, we need to make a copy of the ActorBlueprint,
                    // then modify the attributes on the copy � not the original const pointer returned by Find.
                    carla::client::ActorBlueprint vehicle_blueprint_local = *vehicle_blueprint;
                    vehicle_blueprint_local.SetAttribute("role_name", sumoActorId); // Set the role name to the Sumo actor ID
                    
                    if (!vehicle_blueprint) {
                        std::cerr << "Blueprint not found: " << carlaActorTypeId << std::endl;
                        return 1;
                    }

                    carla::SharedPtr<carla::client::Vehicle> carlaVehicleActorPtr;
                    // If the Intertested vehicle has been spawned in the Carla
                    // Note: For the interested vehicles spawned by the extrernal control script, its role_name should be set to the sumo id name
					// And their blueprint should be the same as its vehicle type
                    if (SET_CONTAINS_ID(setInterestedIds, sumoActorId) && MAP_CONTAINS_KEY(mapRoleNameToActorId, sumoActorId) && useVhicleTypeAsBlueprint) {
                        carlaVehicleActorPtr = boost::static_pointer_cast<carla::client::Vehicle>(carlaWorld.GetActor(std::stoul(mapRoleNameToActorId[sumoActorId])));
                        carlaVehicleActorPtr->SetTransform(carlaTransform); // Update the transform of the existing actor to the sumo position
                        if (enableVerboseLog) std::cout << "Found Intertested Vehicle with Carla Type ID: " << carlaActorTypeId << " SUMO ID:" << sumoActorId << std::endl;
                    }
                    else {
                        if (enableVerboseLog) std::cout << "Spawning actor with Carla Type ID: " << carlaActorTypeId << " SUMO ID:" << sumoActorId << std::endl;

                        carlaVehicleActorPtr = boost::static_pointer_cast<carla::client::Vehicle>(carlaWorld.TrySpawnActor(vehicle_blueprint_local, carlaTransform));
                        
                        if (carlaVehicleActorPtr != nullptr) {
                            
                            carlaVehicleActorPtr->SetSimulatePhysics(false);
                            sumoActor.spawnedInCarla = true; // Mark the Sumo actor as spawned in Carla
                            sumoActor.carlaVehicleActorPtr = carlaVehicleActorPtr; // Store the Carla actor in the SumoActor

                            if (enableVerboseLog) std::cout << "Spawned actor with Carla ID: " << carlaVehicleActorPtr->GetId() << " SUMO ID:" << sumoActorId << std::endl;
                            if (enableVerboseLog && sumoActorId == "ego") {
                                std::cout << "Sumo Transform:" << std::endl;
                                std::cout << "  Location -> x: " << sumoActor.sumoTransform.location.x
                                    << ", y: " << sumoActor.sumoTransform.location.y
                                    << ", z: " << sumoActor.sumoTransform.location.z << std::endl;

                                std::cout << "  Rotation -> pitch: " << sumoActor.sumoTransform.rotation.pitch
                                    << ", yaw: " << sumoActor.sumoTransform.rotation.yaw
                                    << ", roll: " << sumoActor.sumoTransform.rotation.roll << std::endl;

                                std::cout << "Carla Transform:" << std::endl;
                                std::cout << "  Location -> x: " << carlaTransform.location.x
                                    << ", y: " << carlaTransform.location.y
                                    << ", z: " << carlaTransform.location.z << std::endl;

                                std::cout << "  Rotation -> pitch: " << carlaTransform.rotation.pitch
                                    << ", yaw: " << carlaTransform.rotation.yaw
                                    << ", roll: " << carlaTransform.rotation.roll << std::endl;
                            }
                            // convert the carla actor id (uint_32 to string)
                            std::string carlaActorId = std::to_string(carlaVehicleActorPtr->GetId());
                            mapCarlaToSumo[carlaActorId] = sumoActorId;
                            mapSumoToCarla[sumoActorId] = carlaActorId;
                            sumoActor.carlaTransform = carlaTransform; // Store the Carla transform in the SumoActor
                        }
                        else {
                            std::cout << "[Warning] Failed to spawn actor with SUMO ID:" << sumoActorId << std::endl;
                        }
                        
                    }
                    
                }
                else {
                    // ==============================================================================================================
                    //  Update the existing carla actor's transform (except for the Interested Ids, which are controlled by the user)
                    // ==============================================================================================================

					// string to uint32_t conversion
					carla::rpc::ActorId carlaActorId = static_cast<uint32_t>(std::stoul(mapSumoToCarla[sumoActorId]));
					
                    if (sumoActor.spawnedInCarla && sumoActor.carlaVehicleActorPtr !=nullptr) {
                        if (SET_CONTAINS_ID(setInterestedIds, sumoActorId) && enableExternalControl) {
                            if (enableVerboseLog) {
                                carla::SharedPtr<carla::client::Vehicle> carlaActor = mapSumoActor[mapSumoToCarla[sumoActorId]].carlaVehicleActorPtr;
                                carla::geom::Transform carlaTransform = carlaActor->GetTransform();
                                std::cout << "Carla Transform:" << std::endl;
                                std::cout << "  Location -> x: " << carlaTransform.location.x
                                    << ", y: " << carlaTransform.location.y
                                    << ", z: " << carlaTransform.location.z << std::endl;
                            }
                        }  
                        else {
                            // If not interested or the lateral control is not enabled, update its position according to the sumo actor
                            // convert the id back to uint32_t
                            //carla::SharedPtr<carla::client::Actor> carlaActor = carlaWorld.GetActor(std::stoul(carlaActorId));
							carla::SharedPtr<carla::client::Vehicle>& carlaActor = sumoActor.carlaVehicleActorPtr;
                            const std::vector<carla::client::ActorAttributeValue> carlaAttributes = carlaActor->GetAttributes(); // Ensure the attributes are loaded
                            if (enableVerboseLog) {
                                for (const carla::client::ActorAttributeValue& attr : carlaAttributes) {

                                    //std::cout << "Attribute ID: " << attr.GetId() << ", value: " << attr.GetValue() << std::endl;

                                    if (attr.GetId() == "role_name") {
                                        std::string role_name = attr.GetValue();

                                        std::cout << "This vehicle's role_name is: " << role_name << std::endl;
                                    }
                                }
                            }
                            // update the actor's transform
                            carla::rpc::Command::ApplyTransform applyTransformCommand(carlaActorId, carlaTransform);
                            transformCommandBatch.push_back(applyTransformCommand);
                            if (enableVerboseLog) std::cout << "Updating actor with ID: " << carlaActorId << " SUMO ID:" << sumoActorId << std::endl;
                            if (enableVerboseLog && sumoActorId == "ego") {
                                std::cout << "Sumo Transform:" << std::endl;
                                std::cout << "  Location -> x: " << sumoActor.sumoTransform.location.x
                                    << ", y: " << sumoActor.sumoTransform.location.y
                                    << ", z: " << sumoActor.sumoTransform.location.z << std::endl;

                                std::cout << "  Rotation -> pitch: " << sumoActor.sumoTransform.rotation.pitch
                                    << ", yaw: " << sumoActor.sumoTransform.rotation.yaw
                                    << ", roll: " << sumoActor.sumoTransform.rotation.roll << std::endl;

                                std::cout << "Carla Transform:" << std::endl;
                                std::cout << "  Location -> x: " << carlaTransform.location.x
                                    << ", y: " << carlaTransform.location.y
                                    << ", z: " << carlaTransform.location.z << std::endl;

                                std::cout << "  Rotation -> pitch: " << carlaTransform.rotation.pitch
                                    << ", yaw: " << carlaTransform.rotation.yaw
                                    << ", roll: " << carlaTransform.rotation.roll << std::endl;
                            }
                            sumoActor.carlaTransform = carlaTransform; // Update the Carla transform in the SumoActor
                        
                        }
                    } else {
                        std::cerr << "Carla actor not found in the Actor Map for ID: " << carlaActorId << std::endl;
					}
                }
            }
			// false means that the command is not applied immediately, but in the next tick
			// true  means that the command is applied immediately (i.e., advance the world one frame)
            carlaClient.ApplyBatch(transformCommandBatch, false);
            // After updating the non-interested actors, we can now handle the interested actors.
            // We use another script (another Carla client) to control the interested actors;
            // these are the ego vehicles in this case. We want to get the updated positions of the interested actors
            // by sending back the updated positions to the Sumo server.
            // Note: In the control script, the control commands should be applied before the world.wait_for_tick() function.
            carlaWorld.Tick(timeout_1s);
			
            // Clear the command batch for the next iteration
            transformCommandBatch.clear(); 

			//The posions (transform) of the interested actors should be updated by the control script, so we just retrive the current transform
            //of the interested actors
            for (const auto& sumoId : setInterestedIds) {
				SumoActor& sumoActor = mapSumoActor[sumoId];
                if (sumoActor.spawnedInCarla && sumoActor.carlaVehicleActorPtr != nullptr) {
                    carla::SharedPtr<carla::client::Vehicle>& carlaActor = sumoActor.carlaVehicleActorPtr;
                    carla::geom::Transform carlaTransform = carlaActor->GetTransform();
                    carla::geom::Vector3D carlaExtent = carlaActor->GetBoundingBox().extent;
                    carla::geom::Vector3D carlaVelocity = carlaActor->GetVelocity(); // This will update the velocity of the actor
                    carla::geom::Transform sumoTransform = BridgeHelper::map_transfrom_Carla_to_Sumo(carlaTransform, carlaExtent);
					double carlaSpeed = std::sqrt(carlaVelocity.x * carlaVelocity.x + carlaVelocity.y * carlaVelocity.y);
					
					// Update the Sumo actor's transform
					
					mapSumoActor[sumoId].sumoTransform = sumoTransform;
                    carla::geom::Location sumoLocation = sumoTransform.location;
                    carla::geom::Rotation sumoRotation = sumoTransform.rotation;
                    VehFullData_t tmpVehData;
					tmpVehData.id = sumoId;
					tmpVehData.type = mapSumoActor[sumoId].vClass;
                    double simulatedSpeed = periodicCosineSpeed(simTime, 8.0f, 20.0f);
                    double speedDesired = carlaSpeed;
      //              if (enableExternalControl) {
      //                  // If the vehicle is controlled by the external script, use the speed from the script
      //                  speedDesired = carlaSpeed; // Use the speed from the Carla actor
      //              }
      //              else {
						//speedDesired = simulatedSpeed; // Use the simulated speed, this can be further changed with desired speed like from the Eco-Pilot
      //              }
                    if (enableVerboseLog) std::cout << "Speed of the Sumo actor with ID: " << sumoId << " is: " << speedDesired << std::endl;
                    if (enableExternalControl) {
					    tmpVehData.speedDesired = speedDesired;
					    tmpVehData.positionX = sumoLocation.x;
					    tmpVehData.positionY = sumoLocation.y;
					    tmpVehData.positionZ = sumoLocation.z;
					    tmpVehData.heading = sumoRotation.yaw;
					    tmpVehData.grade = sumoRotation.pitch * M_PI / 180.0; // Convert to radians
					    
					    //tmpVehData.length = carlaExtent.x * 2; // The extent is half the length, so multiply by 2
					    //tmpVehData.width = carlaExtent.y * 2; // The extent is half the width, so multiply by 2
					    //tmpVehData.height = carlaExtent.z * 2; // The extent is half the height, so multiply by 2
					    msgHelper.VehDataSend_um[socketHelper.serverSock[sockId]].push_back(tmpVehData);
                    }
                    
                    if (sumoId == centeredViewId) {

						// Set the spectator to follow the centered view actor
						// Note: The spectator is a special camera that follows the actor
						if (enableVerboseLog) std::cout << "Setting spectator to follow actor with ID: " << sumoId << std::endl;
                        static carla::geom::Location smoothedLocation;
                        carla::geom::Location tmpLocation = carlaTransform.location;
                        tmpLocation.z += 100.0f;// Set height to 100 meters above the ground
                        smoothedLocation = 0.9f * smoothedLocation + 0.1f * tmpLocation;
                        //pitch = -90.0, yaw = 0.0, roll = 0.0
                        carla::geom::Rotation tmpRotation(-90.0f, -90.0f, 0.0f);
                        carla::geom::Transform tmpTransform(smoothedLocation, tmpRotation);
                        carlaSpectator->SetTransform(tmpTransform);
                        //drawCircle(carlaWorld.MakeDebugHelper(), tmpLocation, 2.5f, 32, 0.5f, 0.1f, 0.2f);
					}
                }
            }
            
            // =======================================================================================
			// Semd data to the traffic layer server, to update the positions of the interested actors
            // =======================================================================================
            uint8_t simStateSend = 1;

            for (unsigned int iServer = 0; iServer < socketHelper.serverSock.size(); iServer++) {

                int simStateRecv;
                float simTimeRecv;

                if (enableVerboseLog) {
                    printf("sending server at port %d\n", socketHelper.SERVERPORT[iServer]);

                    FILE* f = fopen(CarlaClientLogFile.c_str(), "a");
                    fprintf(f, "send server: %d\n", socketHelper.SERVERPORT[iServer]);
                    fclose(f);
                }

				// send data to the traffic layer server
                if (socketHelper.sendData(socketHelper.serverSock[iServer], iServer, simTime, simStateSend, msgHelper) < 0) {
                    if (WSAGetLastError() != WSAEINTR && WSAGetLastError() != WSAEFAULT) {
                        printf("ERROR: send to server fails\n");
                    }
                    socketHelper.socketShutdown();
                    exit(-1);
                };
            }
            simTime += trafficRefreshRate;
            
			msgHelper.clearRecvStorage();
            msgHelper.clearSendStorage();
            //const std::chrono::steady_clock::time_point loop_end_time = std::chrono::steady_clock::now();
            //std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
            //std::chrono::milliseconds sleep_duration = std::chrono::milliseconds(step_duration_ms) - elapsed;
            //if (sleep_duration.count() > 0) {
            //    std::this_thread::sleep_for(sleep_duration);
            //}
        }
        
        carlaWorldSettings = carlaWorld.GetSettings();
        if (carlaWorldSettings.synchronous_mode) {
            carlaWorldSettings.synchronous_mode = false;            // Turn off synchronous mode
            carlaWorld.ApplySettings(carlaWorldSettings, timeout_1s);
            if (enableVerboseLog) std::cout << "Synchronous mode disabled.\n";
        }
    }
    catch (const carla::client::TimeoutException& e) {
        socketHelper.socketShutdown();
        std::cout << '\n' << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception& e) {
        socketHelper.socketShutdown();
        std::cout << "\nException: " << e.what() << std::endl;
        return 2;
    }
}
