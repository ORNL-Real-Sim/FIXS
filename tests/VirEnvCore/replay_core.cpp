//============================================================================
//  replay_core.cpp  (#174 simulator-free guard -- the real core test)
//----------------------------------------------------------------------------
//  Drives VirEnvCore::processStep with a hand-authored FIXS traffic trace and a
//  MockVirEnvBackend, asserting the core's VERB decisions (spawn/despawn, the
//  Carla direct-set vs CarMaker interpolation policy). Runs with NO CarMaker,
//  NO Carla, NO server -- the baseline that protects the extract-class refactor.
//
//  Build: see build_and_run.bat (links CommonLib VirEnvCore/MsgHelper/Socket/
//  Config + yaml-cpp + ws2_32; no SDK).
//============================================================================
#include "../../CommonLib/VirEnvCore.h"
#include "MockVirEnvBackend.h"

#include <iostream>
#include <cassert>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

using namespace virenv;
using std::string;
using std::vector;

static VehFullData_t veh(const string& id, const string& vclass,
                         double x, double y, double z, double hdg,
                         double grade = 0.0, int lights = 0) {
    VehFullData_t v{};
    v.id = id; v.type = "car"; v.vehicleClass = vclass;
    v.positionX = (float)x; v.positionY = (float)y; v.positionZ = (float)z;
    v.heading = (float)hdg; v.grade = (float)grade; v.lightIndicators = (uint16_t)lights;
    return v;
}

static void step(VirEnvCore& core, MockVirEnvBackend& mock, double simTime,
                 bool onUpdate, const vector<VehFullData_t>& vs) {
    { std::ostringstream os; os << "step t=" << std::fixed << std::setprecision(2) << simTime
                               << " onUpdate=" << (onUpdate ? 1 : 0); mock.mark(os.str()); }
    core.Msg_c.clearRecvStorage();
    for (const auto& v : vs) core.Msg_c.VehDataRecv_um[v.id] = v;
    const char* err = nullptr;
    int rc = core.processStep(simTime, onUpdate, 1, (float)simTime, &err);
    assert(rc == 0 && "processStep returned an error");
}

static bool hasInOrder(const vector<string>& ev, const vector<string>& needles) {
    size_t i = 0;
    for (const auto& e : ev) if (i < needles.size() && e.find(needles[i]) != string::npos) ++i;
    return i == needles.size();
}
static void dump(const char* title, const vector<string>& ev) {
    std::cout << "--- " << title << " transcript (" << ev.size() << ") ---\n";
    for (const auto& e : ev) std::cout << "  " << e << "\n";
    std::cout.flush();  // so the transcript survives an assert() abort
}

int main() {
    // ===== Scenario 1: Carla-style -- interpolate=false, 1:1 @ 0.1 s =====
    {
        MockVirEnvBackend mock;
        VirEnvCore core; core.setBackend(&mock);
        core.ENABLE_REALSIM = false; core.SYNCHRONIZE_TRAFFIC_SIGNAL = false;
        core.ENABLE_SEPARATE_EGO_TRAFFIC = false;
        core.egoId_ = "__none__"; core.trafficRefreshRate_ = 0.1; core.interpolateTraffic = false;
        core.Msg_c.VehicleMessageField_set = { "vehicleClass","heading","grade","lightIndicators" };
        mock.setMockEgo({}, false);  // readEgoState -> false (keeps transcript clean)

        step(core, mock, 0.00, false, {});                                            // initTrafficPool
        step(core, mock, 0.10, true,  { veh("v1","passenger", 10,0,0.1, 90) });       // v1 appears @ A
        step(core, mock, 0.20, true,  { veh("v1","passenger", 20,0,0.1, 90),
                                  veh("v2","truck",      5,5,0.1, 0) });          // v1 @ B, v2 appears
        step(core, mock, 0.30, true,  { veh("v2","truck",      6,5,0.1, 0) });          // v1 gone

        dump("Carla-style", mock.events());
        assert(hasInOrder(mock.events(), {
            "initTrafficPool",
            "spawnVehicle Car(car/passenger) -> 0",
            "setVehiclePose 0 (10.000,0.000,0.100)",   // direct @ A (step 0.1 refresh)
            "spawnVehicle Truck(car/truck)",           // v2 appears (step 0.2 map)
            "setVehiclePose 0 (20.000,0.000,0.100)",   // direct @ B, NO interpolation (step 0.2 refresh)
            "despawnVehicle 0",                         // v1 destroyed the first absent step (0.3)
        }));
        std::cout << "Scenario 1 (Carla direct-set, immediate despawn) PASS\n\n";
    }

    // ===== Scenario 2: CarMaker-style -- interpolate=true, sub-steps =====
    {
        MockVirEnvBackend mock;
        VirEnvCore core; core.setBackend(&mock);
        core.ENABLE_REALSIM = false; core.SYNCHRONIZE_TRAFFIC_SIGNAL = false;
        core.egoId_ = "__none__"; core.trafficRefreshRate_ = 0.05; core.interpolateTraffic = true;
        core.Msg_c.VehicleMessageField_set = { "vehicleClass","heading","grade" };
        mock.setMockEgo({}, false);

        step(core, mock, 0.00, false, {});
        step(core, mock, 0.10, true,  { veh("v1","passenger", 10,0,0.1, 90) });  // first sight -> A
        step(core, mock, 0.20, true,  { veh("v1","passenger", 20,0,0.1, 90) });  // stage prev=A@0.2, next=B@0.3
        step(core, mock, 0.25, false, {});                                       // sub-step -> interp midpoint

        dump("CarMaker-style", mock.events());
        // f=(0.25-0.2)/(0.3-0.2)=0.5 -> midpoint (15,0,0.1)
        assert(hasInOrder(mock.events(), { "setVehiclePose 0 (15.000,0.000,0.100)" }));
        std::cout << "Scenario 2 (CarMaker interpolation) PASS\n\n";
    }

    std::cout << "REPLAY PASS: VirEnvCore drives the verbs correctly, SDK-free, no sockets.\n";
    return 0;
}
