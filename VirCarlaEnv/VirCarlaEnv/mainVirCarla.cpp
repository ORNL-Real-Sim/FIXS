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
#include <carla/client/Client.h>
#include <carla/client/ActorList.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <carla/Memory.h>
#include "BridgeHelper.h"
#include "MsgHelper.h"
#include "SocketHelper.h"
#include "ConfigHelper.h"

//preset delay durations
carla::time_duration timeout_1s = std::chrono::seconds(1);
carla::time_duration timeout_50ms = std::chrono::milliseconds(50);
carla::time_duration timeout_100ms = std::chrono::duration<double>(0.1);  // 0.1 second
namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace cr = carla::rpc;
using namespace std::chrono_literals;
using namespace std::string_literals;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }
#define SET_CONTAINS_ID(set, value) ((set).find(value) != (set).end())
#define MAP_CONTAINS_KEY(map, key) ((map).find(key) != (map).end())

static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << std::endl
        << "Options:\n"
        << "\t-h,--help\t\tShow this help message\n"
        << "\t-f,--file PATH\\FILENAME\tSpecify the path and filename of configuration yaml file"
        << std::endl;
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
    std::string configPath = ".\\defaultConfig.yaml";
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
    bool enableVerbose = configHelper.SimulationSetup.EnableVerboseLog;
    CarlaSetup_t carlaSetup = configHelper.CarlaSetup;

    std::string carlaServerIp = carlaSetup.CarlaServerIP;
    int carlaServerPort = carlaSetup.CarlaServerPort;
	std::string carlaClientIp = carlaSetup.CarlaClientIP;
	int carlaClientPort = carlaSetup.CarlaClientPort;
	std::string carlaMap = carlaSetup.CarlaMap;
	double trafficRefreshRate = carlaSetup.TrafficRefreshRate;
	
    std::unordered_set<std::string> setInterestedIds;
    for (const std::string& id : carlaSetup.InterestedIds) {
        setInterestedIds.insert(id);
	}

	// Setup Connection to the Traffic Server
    //void SocketHelper::socketSetup(vector <string> SERVERADDR_UserInput, vector <int> SERVERPORT_UserInput) {
	std::vector<std::string> SERVERADDR_UserInput = { carlaClientIp };
    std::vector<int> SERVERPORT_UserInput = { carlaClientPort };
	socketHelper.socketSetup(SERVERADDR_UserInput, SERVERPORT_UserInput);
    socketHelper.disableWaitClientTrigger();
    socketHelper.disableServerTrigger();
    if (socketHelper.initConnection(CarlaClientLogFile) < 0) {
        printf("Connect to Traffic Layer failed!\n");
        exit(-1);
    }
	// there is only one traffic layer server
    int sockId = 0; 
    try {
        cc::Client client = cc::Client(carlaServerIp, carlaServerPort);
		// initialize carla client and world
        cc::World world = client.GetWorld();
        std::cout << "Client API version : " << client.GetClientVersion() << '\n';
        std::cout << "Server API version : " << client.GetServerVersion() << '\n';
        // Enable synchronous mode
        cr::EpisodeSettings settings = world.GetSettings();
        // To be replaced with the actual Carla Map from configuration file
        std::cout << "Loading world: " << carlaMap << std::endl;
        cc::World world = client.LoadWorld(carlaMap);
        auto settings = world.GetSettings();
        if (!settings.synchronous_mode) {
            settings.synchronous_mode = true;            // Turn on synchronous mode
			settings.fixed_delta_seconds = trafficRefreshRate;         // time step of 0.1 seconds
            //the time_out is of carla::time_duration
            world.ApplySettings(settings, timeout_1s);
            std::cout << "Synchronous mode enabled.\n";
        }
        carla::SharedPtr<cc::BlueprintLibrary> blueprint_library = world.GetBlueprintLibrary();

		// Map the Sumo Ids to Carla Ids
		std::unordered_map<std::string, std::string> mapSumoToCarla;
		// Map the Carla Ids to Sumo Ids
		std::unordered_map<std::string, std::string> mapCarlaToSumo;

		uint32_t simTime = 0; // Initialize the current simulation time
        std::unordered_map<std::string, Actor> mapSumoActor;
		std::unordered_map<std::string, Actor> mapCarlaActor;

        
        while (simTime < simEndTime) {
            ///***********************
            // RUN one-step simulation
            ///***********************


            // ==========================================
			// Receive data from the traffic layer server
            // ==========================================
            int simStateRecv;
            float simTimeRecv;
            for (unsigned int iServer = 0; iServer < socketHelper.serverSock.size(); iServer++) {

                int simStateRecv;
                float simTimeRecv;

                if (enableVerbose) {
                    printf("receiving server at port %d\n", socketHelper.serverSock[iServer]);

                    FILE* f = fopen(CarlaClientLogFile.c_str(), "a");
                    fprintf(f, "recv server: %d\n", socketHelper.serverSock[iServer]);
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
            for (const auto& pair : msgHelper.VehDataRecv_um) {
				VehFullData_t tmpVehData = pair.second;
				cg::Location tmpLocation(tmpVehData.positionX, tmpVehData.positionY, tmpVehData.positionZ);
				// The grade received from FIXS is in radians, convert to degrees
				cg::Rotation tmpRotation(tmpVehData.grade * 180/M_PI, tmpVehData.heading, 0.0f);
				cg::Vector3D tmpExtent(tmpVehData.length / 2, tmpVehData.width / 2, tmpVehData.height / 2);
				cg::Transform tmpTransform(tmpLocation, tmpRotation);
				mapSumoActor[tmpVehData.id] = Actor(tmpVehData.id, tmpVehData.type, tmpTransform, tmpExtent);
				setCurrentSumoIds.insert(tmpVehData.id);
            }
			// Check if the mapSumoActor contains vehicles that are not in the current step
			// If so, remove them from the mapSumoActor and mapCarlaActor
            for (auto it : mapSumoActor) {
                std::string sumoActorId = it.first;

                if (!SET_CONTAINS_ID(setCurrentSumoIds, sumoActorId)) {
                    // If the sumo actor is not in the current step, remove it from the map
                    std::cout << "Removing Sumo actor with ID: " << sumoActorId << std::endl;
					// Remove from mapSumoActor
					mapSumoActor.erase(sumoActorId);
					// Remove from mapCarlaActor
                    if (MAP_CONTAINS_KEY(mapSumoToCarla, sumoActorId)) {
                        std::string carlaActorId = mapSumoToCarla[sumoActorId];
                        if (MAP_CONTAINS_KEY(mapCarlaActor, carlaActorId)) {
                            std::cout << "Removing Carla actor with ID: " << carlaActorId << std::endl;
                            // Remove the actor from Carla world
                            carla::SharedPtr<cc::Actor> carlaActor = world.GetActor(std::stoul(carlaActorId));
                            if (carlaActor) {
                                carlaActor->Destroy();
                                std::cout << "Destroyed Carla actor with ID: " << carlaActorId << std::endl;
                            }
                        }
                        // Remove from mapCarlaActor
                        mapCarlaActor.erase(carlaActorId);
                        // Remove from mapSumoToCarla
                        mapSumoToCarla.erase(sumoActorId);
                        // Remove from mapCarlaToSumo
                        mapCarlaToSumo.erase(carlaActorId);

                    }
                }
            }

            for (auto it : mapSumoActor) {
                std::string sumoActorId = it.first;
                Actor sumoActor = it.second;
                cg::Transform carlaTransform = BridgeHelper::map_transfrom_sumo_to_carla(sumoActor.transform, sumoActor.extent);
                // =======================================================
                // Spawn the Sumo actors that are not in the current step
                // =======================================================
                if (!MAP_CONTAINS_KEY(mapCarlaActor, sumoActorId)) {
					
                    std::string carlaActorTypeId = BridgeHelper::map_sumo_vehicle_class_to_carla_typeId(sumoActor.vclass);
                    auto blueprint = blueprint_library->Find(carlaActorTypeId);
                    if (!blueprint) {
                        std::cerr << "Blueprint not found: " << carlaActorTypeId << std::endl;
                        return 1;
                    }
                    auto carlaActor = world.TrySpawnActor(*blueprint, carlaTransform);
                    if (carlaActor) {
                        std::cout << "Spawned actor with ID: " << carlaActor->GetId() << std::endl;
                    }
                    else {
                        std::cerr << "Failed to spawn actor. " << carlaActorTypeId << std::endl;
                        return 1;
                    }
                    // convert the carla actor id (uint_32 to string)
                    std::string carlaActorId = std::to_string(carlaActor->GetId());
					mapCarlaToSumo[carlaActorId] = sumoActorId;
					mapSumoToCarla[sumoActorId] = carlaActorId;
					cg::Vector3D carlaActorExtent = carlaActor->GetBoundingBox().extent;
					mapCarlaActor[carlaActorId] = Actor(carlaActorId, carlaActorTypeId, carlaTransform, carlaActorExtent);
                }
                else {
                    // ==============================================================================================================
                    //  Update the existing carla actor's transform (except for the Interested Ids, which are controlled by the user)
                    // ==============================================================================================================
                    std::string carlaActorId = mapSumoToCarla[sumoActorId];
					
                    if (MAP_CONTAINS_KEY(mapCarlaActor, carlaActorId)) {
                        if (SET_CONTAINS_ID(setInterestedIds, sumoActorId)) {
                            
                        }
                        else {
                            // If not interested, update its position according to the sumo actor
                            // convert the id back to uint32_t
                            carla::SharedPtr<cc::Actor> carlaActor = world.GetActor(std::stoul(carlaActorId));
                            // update the actor's transform
                            if (carlaActor) {
                                carlaActor->SetTransform(carlaTransform);
                                std::cout << "Updated actor with ID: " << carlaActorId << std::endl;
                                mapCarlaActor[carlaActorId].transform = carlaTransform;
                            }
                            else {
                                std::cerr << "Carla actor not found in Carla for ID: " << carlaActorId << std::endl;
                            }
                        }
                    } else {
                        std::cerr << "Carla actor not found in the Actor Map for ID: " << carlaActorId << std::endl;
					}
                }
            }
            
            // After updating the non-interested actors, we can now handle the interested actors.
            // We use another script (another Carla client) to control the interested actors;
            // these are the ego vehicles in this case. We want to get the updated positions of the interested actors
            // by sending back the updated positions to the Sumo server.
            // Note: In the control script, the control commands should be applied before the world.wait_for_tick() function.
            world.Tick(timeout_1s);

			//The posions (transform) of the interested actors should be updated by the other script, so we just retrive the current transform
            //of the interested actors
            for (const auto& sumoId : setInterestedIds) {
                if (MAP_CONTAINS_KEY(mapSumoToCarla, sumoId)) {
                    std::string carlaActorId = mapSumoToCarla[sumoId];
                    carla::SharedPtr<cc::Actor> carlaActor = world.GetActor(std::stoul(carlaActorId));
					cg::Transform carlaTransform = carlaActor->GetTransform();
                    cg::Vector3D carlaExtent = carlaActor->GetBoundingBox().extent;
                    cg::Transform sumoTransform = BridgeHelper::map_transfrom_Carla_to_Sumo(carlaTransform, carlaExtent);
					// Update the Sumo actor's transform
					if (MAP_CONTAINS_KEY(mapSumoActor, sumoId)) {
						mapSumoActor[sumoId].transform = sumoTransform;
                        cg::Location sumoLocation = sumoTransform.location;
                        cg::Rotation sumoRotation = sumoTransform.rotation;
                        VehFullData_t tmpVehData;
						tmpVehData.id = sumoId;
						tmpVehData.type = mapSumoActor[sumoId].vclass;
						tmpVehData.positionX = sumoLocation.x;
						tmpVehData.positionY = sumoLocation.y;
						tmpVehData.positionZ = sumoLocation.z;
						tmpVehData.heading = sumoRotation.yaw;
						tmpVehData.grade = sumoRotation.pitch * M_PI / 180.0; // Convert to radians
						tmpVehData.length = carlaExtent.x * 2; // The extent is half the length, so multiply by 2
						tmpVehData.width = carlaExtent.y * 2; // The extent is half the width, so multiply by 2
						tmpVehData.height = carlaExtent.z * 2; // The extent is half the height, so multiply by 2
						msgHelper.VehDataSend_um[socketHelper.serverSock[sockId]].push_back(tmpVehData);
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

                if (enableVerbose) {
                    printf("sending server at port %d\n", socketHelper.serverSock[iServer]);

                    FILE* f = fopen(CarlaClientLogFile.c_str(), "a");
                    fprintf(f, "send server: %d\n", socketHelper.serverSock[iServer]);
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
            socketHelper.socketShutdown();
			msgHelper.clearRecvStorage();
            msgHelper.clearSendStorage();
        }
        
    }
    catch (const cc::TimeoutException& e) {
        std::cout << '\n' << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception& e) {
        std::cout << "\nException: " << e.what() << std::endl;
        return 2;
    }
}
