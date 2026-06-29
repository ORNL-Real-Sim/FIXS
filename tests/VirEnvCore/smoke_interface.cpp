//============================================================================
//  smoke_interface.cpp  (#174 simulator-free guard -- precursor)
//----------------------------------------------------------------------------
//  Proves IVirEnvBackend + MockVirEnvBackend compile and run with NO CarMaker /
//  Carla / Carla-server (the SDK-free guarantee), and exercises every verb +
//  the spawn/despawn pool-exhaustion path. This stands in for CI until the
//  VirEnvCore extraction lands a real recorded-trace replay test on top of it.
//
//  Build (no SDK, any C++17 compiler), e.g. from a VS dev shell:
//    cl /std:c++17 /EHsc /nologo tests\VirEnvCore\smoke_interface.cpp /Fe:smoke.exe
//  or:  g++ -std=c++17 tests/VirEnvCore/smoke_interface.cpp -o smoke
//============================================================================
#include "MockVirEnvBackend.h"
#include <iostream>
#include <cassert>

using namespace virenv;

int main() {
    // Drive the backend ONLY through the abstract interface -- the core will too.
    MockVirEnvBackend mock(/*cars*/2, /*trucks*/1, /*buses*/0);
    IVirEnvBackend& be = mock;

    be.loadSignalTable("");   // signal-free scenario: no-op
    be.initTrafficPool();

    // spawn within capacity
    VehHandle a = be.spawnVehicle("car", "passenger", Pose{});
    VehHandle b = be.spawnVehicle("car", "passenger", Pose{});
    assert(a != kNoHandle && b != kNoHandle && a != b);
    // capacity exhausted -> kNoHandle (mirrors the full-queue `continue` in runStep)
    VehHandle c = be.spawnVehicle("car", "passenger", Pose{});
    assert(c == kNoHandle);
    // despawn returns the slot; next spawn reuses it
    be.despawnVehicle(a);
    VehHandle d = be.spawnVehicle("car", "passenger", Pose{});
    assert(d == a);

    // actuation verbs in the raw FIXS frame
    Pose p; p.x = 12.5; p.y = -0.4; p.z = 0.1; p.headingDeg = 78.0; p.gradeRad = 0.0;
    be.setVehiclePose(b, p);
    be.setVehicleLights(b, /*brake*/true, /*L*/false, /*R*/true);
    be.syncTrafficLight("J1", "rG");

    // ego mode A readback (canned)
    EgoState canned; canned.speed = 7.2; canned.x = 12.2; canned.y = -0.42; canned.heading = 12.5;
    mock.setMockEgo(canned, /*available*/true);
    EgoState got;
    bool ok = be.readEgoState("egoCm", got);
    assert(ok && got.speed == 7.2);

    // mode B + L2 hooks (default-able verbs)
    be.setEgoPose("egoSim", p);
    be.applyEgoControl("egoCm", 13.4);

    std::cout << "=== recorded verb transcript (" << mock.events().size() << " events) ===\n";
    for (const auto& e : mock.events()) std::cout << "  " << e << "\n";

    std::cout << "\nSMOKE PASS: IVirEnvBackend + MockVirEnvBackend compile & run SDK-free.\n";
    return 0;
}
