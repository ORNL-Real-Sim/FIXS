#include "VissimDSProxyHelper.h"

#include <stdexcept>

namespace FIXS {
namespace DSProxy {

const char* signalStateName(SignalStateType s) {
    switch (s) {
        case SignalStateType::Red:                 return "Red";
        case SignalStateType::RedAmber:            return "RedAmber";
        case SignalStateType::Green:               return "Green";
        case SignalStateType::Amber:               return "Amber";
        case SignalStateType::Off:                 return "Off";
        case SignalStateType::Undefined:           return "Undefined";
        case SignalStateType::FlashingAmber:       return "FlashingAmber";
        case SignalStateType::FlashingRed:         return "FlashingRed";
        case SignalStateType::FlashingGreen:       return "FlashingGreen";
        case SignalStateType::AlternatingRedGreen: return "AlternatingRedGreen";
        case SignalStateType::GreenAmber:          return "GreenAmber";
    }
    return "?";
}

std::string defaultDllPathForVissim(int vissimVersion) {
    switch (vissimVersion) {
        case 2022:
            return R"(C:\Program Files\PTV Vision\PTV Vissim 2022\API\DrivingSimulator_DLL\bin\x64\DrivingSimulatorProxy.dll)";
        case 2026:
            return R"(C:\Program Files\PTV Vision\PTV Vissim 2026\API\DrivingSimulator_DLL\bin\x64\DrivingSimulatorProxy.dll)";
    }
    throw std::invalid_argument("Unsupported VissimVersion for DSProxy (expected 2022 or 2026)");
}

unsigned short connectVersionNo(int vissimVersion) {
    switch (vissimVersion) {
        case 2022: return 2200;
        case 2026: return 2600;
    }
    throw std::invalid_argument("Unsupported VissimVersion for DSProxy (expected 2022 or 2026)");
}

VissimDSProxy::VissimDSProxy() = default;

VissimDSProxy::~VissimDSProxy() {
    if (mModule) {
        FreeLibrary(mModule);
        mModule = nullptr;
    }
}

bool VissimDSProxy::load(const std::string& dllPath) {
    mModule = LoadLibraryA(dllPath.c_str());
    if (!mModule) {
        return false;
    }

    // Resolve all required entry points up front so a partial load is detected
    // here, not at first use. Any miss tears down the module and reports false.
    pConnect              = reinterpret_cast<PFN_VISSIM_Connect>(GetProcAddress(mModule, "VISSIM_Connect"));
    pDisconnect           = reinterpret_cast<PFN_VISSIM_Disconnect>(GetProcAddress(mModule, "VISSIM_Disconnect"));
    pSetDriverVehicles    = reinterpret_cast<PFN_VISSIM_SetDriverVehicles>(GetProcAddress(mModule, "VISSIM_SetDriverVehicles"));
    pSetDetection         = reinterpret_cast<PFN_VISSIM_SetDetection>(GetProcAddress(mModule, "VISSIM_SetDetection"));
    pDataReady            = reinterpret_cast<PFN_VISSIM_DataReady>(GetProcAddress(mModule, "VISSIM_DataReady"));
    pGetTrafficVehicles   = reinterpret_cast<PFN_VISSIM_GetTrafficVehicles>(GetProcAddress(mModule, "VISSIM_GetTrafficVehicles"));
    pGetSignalStates      = reinterpret_cast<PFN_VISSIM_GetSignalStates>(GetProcAddress(mModule, "VISSIM_GetSignalStates"));
    pGetVehicleLists      = reinterpret_cast<PFN_VISSIM_GetVehicleLists>(GetProcAddress(mModule, "VISSIM_GetVehicleLists"));
    pGetLastErrorMessage  = reinterpret_cast<PFN_VISSIM_GetLastErrorMessage>(GetProcAddress(mModule, "VISSIM_GetLastErrorMessage"));

    const bool allBound = pConnect && pDisconnect && pSetDriverVehicles
                       && pSetDetection && pDataReady && pGetTrafficVehicles
                       && pGetSignalStates && pGetVehicleLists && pGetLastErrorMessage;
    if (!allBound) {
        FreeLibrary(mModule);
        mModule = nullptr;
    }
    return allBound;
}

bool VissimDSProxy::connect(unsigned short versionNo,
                            const std::wstring& networkFile,
                            unsigned short simulatorFrequency,
                            double         visibilityRadius,
                            unsigned short maxSimulatorVeh,
                            unsigned short maxSimulatorPed,
                            unsigned short maxSimulatorDet,
                            unsigned short maxTotalVeh,
                            unsigned short maxVissimPed,
                            unsigned short maxVissimSigGrp) {
    return pConnect(versionNo, networkFile.c_str(),
                    simulatorFrequency, visibilityRadius,
                    maxSimulatorVeh, maxSimulatorPed, maxSimulatorDet,
                    maxTotalVeh, maxVissimPed, maxVissimSigGrp);
}

bool VissimDSProxy::connect(unsigned short versionNo,
                            const std::string& networkFile,
                            unsigned short simulatorFrequency,
                            double         visibilityRadius,
                            unsigned short maxSimulatorVeh,
                            unsigned short maxSimulatorPed,
                            unsigned short maxSimulatorDet,
                            unsigned short maxTotalVeh,
                            unsigned short maxVissimPed,
                            unsigned short maxVissimSigGrp) {
    // ConfigHelper-side paths arrive as UTF-8 std::string; VISSIM_Connect
    // wants wchar_t*. This overload is the only widening point so callers
    // don't have to roll their own.
    std::wstring w;
    if (!networkFile.empty()) {
        const int needed = MultiByteToWideChar(CP_UTF8, 0, networkFile.c_str(),
                                               static_cast<int>(networkFile.size()),
                                               nullptr, 0);
        w.resize(needed);
        MultiByteToWideChar(CP_UTF8, 0, networkFile.c_str(),
                            static_cast<int>(networkFile.size()),
                            w.data(), needed);
    }
    return connect(versionNo, w, simulatorFrequency, visibilityRadius,
                   maxSimulatorVeh, maxSimulatorPed, maxSimulatorDet,
                   maxTotalVeh, maxVissimPed, maxVissimSigGrp);
}

bool VissimDSProxy::disconnect() {
    return pDisconnect();
}

bool VissimDSProxy::setDriverVehicles(const std::vector<Simulator_Veh_Data>& vehicles) {
    const int n = static_cast<int>(vehicles.size());
    if (n == 0) {
        return pSetDriverVehicles(0, nullptr);
    }
    // PTV's signature is non-const, but the DLL does not mutate the caller's
    // buffer in practice. cast-away-const here is safe and keeps the wrapper's
    // interface const-correct.
    auto* buf = const_cast<Simulator_Veh_Data*>(vehicles.data());
    return pSetDriverVehicles(n, buf);
}

bool VissimDSProxy::setDetection(int detectorId, int controllerId) {
    return pSetDetection(detectorId, controllerId);
}

bool VissimDSProxy::dataReady() {
    return pDataReady();
}

std::vector<VISSIM_Veh_Data> VissimDSProxy::getTrafficVehicles() {
    int n = 0;
    VISSIM_Veh_Data* raw = nullptr;
    pGetTrafficVehicles(&n, &raw);
    if (n <= 0 || raw == nullptr) {
        return {};
    }
    return std::vector<VISSIM_Veh_Data>(raw, raw + n);
}

std::vector<VISSIM_Sig_Data> VissimDSProxy::getSignalStates() {
    int n = 0;
    VISSIM_Sig_Data* raw = nullptr;
    pGetSignalStates(&n, &raw);
    if (n <= 0 || raw == nullptr) {
        return {};
    }
    return std::vector<VISSIM_Sig_Data>(raw, raw + n);
}

VehicleListsDelta VissimDSProxy::getVehicleLists() {
    int numNew = 0, numMoved = 0, numDeleted = 0;
    int* newIds = nullptr;
    int* newTypes = nullptr;
    int* movedIds = nullptr;
    int* deletedIds = nullptr;
    pGetVehicleLists(&numNew, &newIds, &newTypes,
                     &numMoved, &movedIds,
                     &numDeleted, &deletedIds);
    VehicleListsDelta out;
    if (numNew > 0 && newIds && newTypes) {
        out.newIds.assign(newIds, newIds + numNew);
        out.newTypes.assign(newTypes, newTypes + numNew);
    }
    if (numMoved > 0 && movedIds) {
        out.movedIds.assign(movedIds, movedIds + numMoved);
    }
    if (numDeleted > 0 && deletedIds) {
        out.deletedIds.assign(deletedIds, deletedIds + numDeleted);
    }
    return out;
}

std::wstring VissimDSProxy::lastError() const {
    if (!pGetLastErrorMessage) {
        return {};
    }
    const wchar_t* msg = pGetLastErrorMessage();
    return msg ? std::wstring(msg) : std::wstring();
}

} // namespace DSProxy
} // namespace FIXS
