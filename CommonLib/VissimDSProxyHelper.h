#pragma once

// C++ wrapper around PTV's DrivingSimulatorProxy.dll.
//
// Loaded dynamically via LoadLibrary so the FIXS build does not depend on
// the PTV install layout. The struct/enum surface mirrors PTV's
// `<VISSIM>/API/DrivingSimulator_DLL/include/DrivingSimulatorProxy.h`
// (2022 SP00); the 2022/2026 header diff is doc-comment-only, so the same
// declarations are correct for both VISSIM versions — only the runtime
// `versionNo` passed to `Connect()` differs (2200 vs 2600).
//
// Empirical validation lives under tests/Vissim/Probes/DSProxy_*/
// from issue #156's investigation. This is Stage A of issue #158.

#ifndef WINDOWS_INCLUDED
#define WINDOWS_INCLUDED
#define _WINSOCKAPI_
#include <windows.h>
#endif

#include <string>
#include <vector>

namespace FIXS {
namespace DSProxy {

constexpr int NAME_MAX_LENGTH = 100;
constexpr int MAX_UDA = 16;

enum class TurningIndicator : int {
    Left  =  1,
    None  =  0,
    Right = -1,
};

enum class SignalStateType : int {
    Red                  =  1,
    RedAmber             =  2,
    Green                =  3,
    Amber                =  4,
    Off                  =  5,
    Undefined            =  6,
    FlashingAmber        =  7,
    FlashingRed          =  8,
    FlashingGreen        =  9,
    AlternatingRedGreen  = 10,
    GreenAmber           = 11,
};

const char* signalStateName(SignalStateType s);

#pragma pack(push, 8)

struct Simulator_Veh_Data {
    int    VehicleID;
    int    VehicleType;
    double Position_X;
    double Position_Y;
    double Position_Z;
    double Orient_Heading;
    double Orient_Pitch;
    double Speed;
    bool   Create;
    int    CreateID;
    bool   Delete;
    bool   ControlledByVissim;
    int    RoutingDecisionNo;
    int    RouteNo;
};

struct Simulator_Ped_Data {
    double Position_X;
    double Position_Y;
    double Position_Z;
    double Orient_Heading;
    double DistanceSinceBirth;
    double Speed;
};

struct VISSIM_Veh_Data {
    int    VehicleID;
    int    VehicleType;
    char   ModelFileName[NAME_MAX_LENGTH];
    int    color;
    double Position_X;
    double Position_Y;
    double Position_Z;
    double Orient_Heading;
    double Orient_Pitch;
    double Speed;
    int    LeadingVehicleID;
    int    TrailingVehicleID;
    int    LinkID;
    char   LinkName[NAME_MAX_LENGTH];
    double LinkCoordinate;
    int    LaneIndex;
    int    TurningIndicator;
    int    PreviousIndex;
    int    NumUDAs;
    double UDA[MAX_UDA];
    int    CreateID;
    bool   ControlledByVissim;
};

struct VISSIM_Sig_Data {
    int ControllerID;
    int SignalGroupID;
    int SignalState;
};

#pragma pack(pop)

struct VehicleListsDelta {
    std::vector<int> newIds;
    std::vector<int> newTypes;
    std::vector<int> movedIds;
    std::vector<int> deletedIds;
};

// Resolve the per-install DSProxy DLL path from a VISSIM major version
// (e.g. 2022 -> "C:\Program Files\PTV Vision\PTV Vissim 2022\API\..."). If
// vissimVersion is 0, the caller is expected to supply an explicit path via
// the overload below.
std::string defaultDllPathForVissim(int vissimVersion);

// Convert a VISSIM major version (2022 -> 2200, 2026 -> 2600) to the
// `versionNo` literal that VISSIM_Connect expects. Throws on unknown input.
unsigned short connectVersionNo(int vissimVersion);

class VissimDSProxy {
public:
    VissimDSProxy();
    ~VissimDSProxy();

    VissimDSProxy(const VissimDSProxy&) = delete;
    VissimDSProxy& operator=(const VissimDSProxy&) = delete;

    // Loads the PTV DLL. Returns true on success. lastError() carries the
    // reason on failure.
    bool load(const std::string& dllPath);
    bool isLoaded() const { return mModule != nullptr; }

    // VISSIM_Connect — starts a GUI VISSIM instance via COM and opens the
    // shared-memory channel to it. Returns true on success. The std::string
    // overload accepts a UTF-8 path (e.g. from ConfigHelper).
    bool connect(unsigned short versionNo,
                 const std::wstring& networkFile,
                 unsigned short simulatorFrequency = 10,
                 double         visibilityRadius   = -1.0,
                 unsigned short maxSimulatorVeh    = 10,
                 unsigned short maxSimulatorPed    = 0,
                 unsigned short maxSimulatorDet    = 0,
                 unsigned short maxTotalVeh        = 50000,
                 unsigned short maxVissimPed       = 0,
                 unsigned short maxVissimSigGrp    = 1000);

    bool connect(unsigned short versionNo,
                 const std::string& networkFile,
                 unsigned short simulatorFrequency = 10,
                 double         visibilityRadius   = -1.0,
                 unsigned short maxSimulatorVeh    = 10,
                 unsigned short maxSimulatorPed    = 0,
                 unsigned short maxSimulatorDet    = 0,
                 unsigned short maxTotalVeh        = 50000,
                 unsigned short maxVissimPed       = 0,
                 unsigned short maxVissimSigGrp    = 1000);

    bool disconnect();

    // Per-frame push of driving-simulator-controlled vehicle data. Pass an
    // empty vector to advance VISSIM without pushing any ego.
    bool setDriverVehicles(const std::vector<Simulator_Veh_Data>& vehicles);

    // Optional: fake a detector hit.
    bool setDetection(int detectorId, int controllerId);

    // Non-blocking probe: has VISSIM finished computing the next frame?
    bool dataReady();

    // Per-frame pull. Returned vectors are copies of VISSIM's internal
    // buffers; safe to hold across the next set/get cycle.
    std::vector<VISSIM_Veh_Data> getTrafficVehicles();
    std::vector<VISSIM_Sig_Data> getSignalStates();
    VehicleListsDelta            getVehicleLists();

    // Returns the last error string reported by the DLL (empty if none).
    std::wstring lastError() const;

private:
    HMODULE mModule = nullptr;

    // PTV DLL function pointer types. Names match the exported symbols.
    using PFN_VISSIM_Connect = bool (*)(unsigned short, const wchar_t*,
                                        unsigned short, double,
                                        unsigned short, unsigned short, unsigned short,
                                        unsigned short, unsigned short, unsigned short);
    using PFN_VISSIM_Disconnect            = bool (*)();
    using PFN_VISSIM_SetDriverVehicles     = bool (*)(int, Simulator_Veh_Data*);
    using PFN_VISSIM_SetDetection          = bool (*)(int, int);
    using PFN_VISSIM_DataReady             = bool (*)();
    using PFN_VISSIM_GetTrafficVehicles    = void (*)(int*, VISSIM_Veh_Data**);
    using PFN_VISSIM_GetSignalStates       = void (*)(int*, VISSIM_Sig_Data**);
    using PFN_VISSIM_GetVehicleLists       = void (*)(int*, int**, int**,
                                                      int*, int**,
                                                      int*, int**);
    using PFN_VISSIM_GetLastErrorMessage   = const wchar_t* (*)();

    PFN_VISSIM_Connect            pConnect = nullptr;
    PFN_VISSIM_Disconnect         pDisconnect = nullptr;
    PFN_VISSIM_SetDriverVehicles  pSetDriverVehicles = nullptr;
    PFN_VISSIM_SetDetection       pSetDetection = nullptr;
    PFN_VISSIM_DataReady          pDataReady = nullptr;
    PFN_VISSIM_GetTrafficVehicles pGetTrafficVehicles = nullptr;
    PFN_VISSIM_GetSignalStates    pGetSignalStates = nullptr;
    PFN_VISSIM_GetVehicleLists    pGetVehicleLists = nullptr;
    PFN_VISSIM_GetLastErrorMessage pGetLastErrorMessage = nullptr;
};

} // namespace DSProxy
} // namespace FIXS
