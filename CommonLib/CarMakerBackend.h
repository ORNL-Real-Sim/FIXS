#pragma once
//============================================================================
//  CarMakerBackend  (#174 -- the CarMaker half of the verb interface)
//----------------------------------------------------------------------------
//  Implements IVirEnvBackend against the CarMaker C API. This is the ONLY file
//  on the CarMaker path that needs <CarMaker.h> -- VirEnvCore stays SDK-free.
//  Every verb is a verbatim lift of the CarMaker-specific code that used to live
//  inline in VirEnvHelper::runStep (slot pool, TrfObj->t_0[]/r_zyx[] + the
//  Cfg.l/h/zOff pose anchor, Traffic_Lights_*, tlsChar2CmState, Vehicle readback).
//============================================================================

#include "IVirEnvBackend.h"

#include <CarMaker.h>
#include <Lights.h>

#include <string>
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace virenv {

class CarMakerBackend : public IVirEnvBackend {
public:
    CarMakerBackend() = default;

    void log(const char* msg) override;
    void logError(const char* msg) override;

    void loadSignalTable(const char* path) override;
    void initTrafficPool() override;

    VehHandle spawnVehicle(const std::string& vType, const std::string& vClass) override;
    void      despawnVehicle(VehHandle h) override;

    void setVehiclePose(VehHandle h, const Pose& p) override;
    void setVehicleLights(VehHandle h, bool brake, bool indL, bool indR) override;
    void parkSpares() override;

    void syncTrafficLight(const std::string& junctionId, const std::string& stateStr) override;

    bool readEgoState(const std::string& egoId, EgoState& out) override;

private:
    // CarMaker traffic-object naming for the pre-placed RS slot pool
    std::string carName_  = "RS_C";
    std::string truckName_ = "RS_T";

    std::queue<int> carQueue_, truckQueue_;
    std::unordered_set<int> spawned_;     // slots currently mapped (for parkSpares)

    // signal table: SUMO controller -> [(head-id index into the state string,
    // CarMaker TrfLight.Objs index)]
    std::unordered_map<std::string, std::vector<std::pair<int, int>>> signalCtrl2HeadIdx_;

    bool isRsCar(const std::string& nm)   const { return nm.find(carName_)  != std::string::npos; }
    bool isRsTruck(const std::string& nm) const { return nm.find(truckName_) != std::string::npos; }
    bool isRs(const std::string& nm)      const { return isRsCar(nm) || isRsTruck(nm); }

    static tTLState tlsChar2CmState(char charState);
};

} // namespace virenv
