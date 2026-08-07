#pragma once
/*Define vehicle data structures using only Plain Old Data (POD), i.e. string, double, int, etc., no complex struct, class what so ever*/
//#include <any>
#include <unordered_map>
#include <vector>
#include <string>
// #65: unconditional, not #ifdef RS_DSPACE. The fixed-width types below are
// declared by <cstdint>; MSVC and gcc <= 12 happen to pull it in transitively,
// gcc 13 (Ubuntu 24.04) does not. Guarding it on RS_DSPACE only papered over
// the dSPACE toolchain that noticed first.
#include <cstdint>
// MESSAGE IDENTIFIER 1
// Full vehicle data structure that will be shared between SUMO and other simulators
// !!! This does not necessary mean all data directly communicated between simulators
typedef struct  {
	std::string id; 
	std::string type; 
	std::string vehicleClass;
	float speed; 
	float acceleration;
	float positionX;
	float positionY; 
	float positionZ; 
	float heading; 
	uint32_t color; 
	std::string linkId;
	int32_t laneId;
	float distanceTravel;
	float speedDesired;
	float accelerationDesired;

	int8_t hasPrecedingVehicle;
	std::string precedingVehicleId;
	float precedingVehicleDistance; // distance to preceding vehicle
	float precedingVehicleSpeed; // absolute speed of preceding vehicle
	std::string signalLightId;
	int32_t signalLightHeadId;
	float signalLightDistance; // distance to next signal light
	int8_t signalLightColor; // color of next signal light
	float speedLimit;
	float speedLimitNext;
	float speedLimitChangeDistance; 

	std::string linkIdNext;
	float grade; 


	int8_t activeLaneChange; // 1 to the left, -1 to the right, 0 stay on the lane, 


	// variables not retrievable from VISSIM yet
	float length;
	float width;
	float height;
	// variables not retrievable yet
	float weight;


	//double positionOnLink;

	uint16_t lightIndicators;

	// #174 EgoDriver command channel (L2/L4). Serialized at the END of
	// packVehData/depackVehData, gated by VehicleMessageField like every field.
	// steer is a physical angle; pedals are unitless positions (as in a real car).
	float steerAngleDesired;        // rad, desired front road-wheel steer angle (L4)
	float acceleratorPedalDesired;  // [0,1] accelerator pedal position (L4)
	float brakePedalDesired;        // [0,1] brake pedal position (L4)

}VehFullData_t;


typedef struct  {
	
	double positionOnLink;

	//int isNextSpeedLimitFound;
	//double nextSpeedLimitDistanceCalc;
	//double nextSpeedLimit;

	std::vector <std::string> nextLink_v;

	double positionXrear;
	double positionYrear;

	double accelerationDesired;


}VehicleDataAuxiliary_t;




// MESSAGE IDENTIFIER 2
typedef struct  {
	// unique number of signal light 
	// in VISSIM, this is the index for each signal group
	uint16_t id;

	// this is name of the signal light
	// in SUMO, it is naturally using string to distinguish each TLS
	// in VISSIM, this name can be empty if not set in VISSIM
	std::string name; 

	// this is state, consistent to SUMO definition, 
	// so state of each signal group (VISSIM terminology) or each phase will be represented by one single string, 
	// starting from left to right, that is e.g. phase 1 (signal group 1)-> phase 8 (signal group 8)
	std::string state; 

}TrafficLightData_t;

// MESSAGE IDENTIFIER 3
typedef struct  {
	//std::string intersectionId; // intersection this detector associated with
	uint8_t id; // port number of detector/detector id 
	std::string name; //
	uint8_t state; // state of detector 0 off, 1 active

}DetectorData_t;

// intersection id, intersection name, vector of detectors
typedef std::tuple <int, std::string, std::vector < DetectorData_t>> TlsDetector_t;





typedef struct {
	double x;
	double y;
	double z;
}COORD_t;
