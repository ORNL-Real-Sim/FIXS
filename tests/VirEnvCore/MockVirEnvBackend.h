#pragma once
//============================================================================
//  MockVirEnvBackend  (#174 simulator-free guard)
//----------------------------------------------------------------------------
//  An IVirEnvBackend test double that needs NO CarMaker, NO Carla, NO Carla
//  server. It (a) hands out monotonic fake handles from a bounded per-class pool
//  (so spawn/despawn + capacity-exhaustion behave like the CarMaker slot pool),
//  and (b) RECORDS every verb call as a human-diffable event string.
//
//  This is the backend half of the "core test that runs green with no simulator
//  installed" -- the baseline that protects the VirEnvHelper -> VirEnvCore
//  extraction: drive VirEnvCore with a recorded FIXS traffic trace + this mock,
//  then assert the recorded verb sequence (active handles, spawn/despawn, poses,
//  TLS, ego readback) is unchanged across the refactor. Asserting on these verb
//  strings == asserting "the core's decisions in the canonical frame".
//============================================================================

#include "../../CommonLib/IVirEnvBackend.h"

#include <vector>
#include <queue>
#include <string>
#include <sstream>
#include <iomanip>

namespace virenv {

class MockVirEnvBackend : public IVirEnvBackend {
public:
    // Bounded pools mimic the CarMaker pre-placed slots (default 20 cars / 5
    // trucks / 0 buses -- enough for SimpleLoop's ~40 looping veh? tune per test).
    MockVirEnvBackend(int nCars = 64, int nTrucks = 16, int nBuses = 0) {
        for (int i = 0; i < nCars; ++i)   carPool_.push(handleOf(VehClass::Car, i));
        for (int i = 0; i < nTrucks; ++i) truckPool_.push(handleOf(VehClass::Truck, i));
        for (int i = 0; i < nBuses; ++i)  busPool_.push(handleOf(VehClass::Bus, i));
    }

    // --- recorded transcript (one line per verb call) ----------------------
    const std::vector<std::string>& events() const { return events_; }
    void clear() { events_.clear(); }

    // --- IVirEnvBackend ----------------------------------------------------
    void log(const char* msg) override      { rec("log", msg); }
    void logError(const char* msg) override { rec("logError", msg); }

    void loadSignalTable(const char* path) override { rec("loadSignalTable", path ? path : ""); }
    void initTrafficPool() override { rec("initTrafficPool", ""); poolInit_ = true; }

    VehHandle spawnVehicle(const std::string& vType, const std::string& vClass) override {
        VehClass cls = classify(vClass);
        std::queue<VehHandle>& pool = poolFor(cls);
        std::string tag = clsName(cls) + "(" + vType + "/" + vClass + ")";
        if (pool.empty()) { rec("spawnVehicle", tag + " -> kNoHandle"); return kNoHandle; }
        VehHandle h = pool.front(); pool.pop();
        rec("spawnVehicle", tag + " -> " + std::to_string(h));
        return h;
    }
    void despawnVehicle(VehHandle h) override {
        poolFor(classOf(h)).push(h);
        rec("despawnVehicle", std::to_string(h));
    }

    void setVehiclePose(VehHandle h, const Pose& p) override {
        std::ostringstream os;
        os << h << " (" << fx(p.x) << "," << fx(p.y) << "," << fx(p.z)
           << ") hdg=" << fx(p.headingDeg) << " grade=" << fx(p.gradeRad);
        rec("setVehiclePose", os.str());
    }
    void setVehicleLights(VehHandle h, bool brake, bool indL, bool indR) override {
        std::ostringstream os; os << h << " brake=" << brake << " L=" << indL << " R=" << indR;
        rec("setVehicleLights", os.str());
    }
    void syncTrafficLight(const std::string& junctionId, const std::string& stateStr) override {
        rec("syncTrafficLight", junctionId + " '" + stateStr + "'");
    }

    bool readEgoState(const std::string& egoId, EgoState& out) override {
        rec("readEgoState", egoId);
        out = ego_;            // tests inject a canned ego via setMockEgo()
        return egoAvailable_;
    }
    void setEgoPose(const std::string& egoId, const Pose& p) override {
        std::ostringstream os; os << egoId << " (" << fx(p.x) << "," << fx(p.y) << "," << fx(p.z) << ")";
        rec("setEgoPose", os.str());
    }
    void applyEgoControl(const std::string& egoId, double desiredSpeed) override {
        std::ostringstream os; os << egoId << " v*=" << fx(desiredSpeed);
        rec("applyEgoControl", os.str());
    }

    // --- test fixtures -----------------------------------------------------
    void setMockEgo(const EgoState& e, bool available = true) { ego_ = e; egoAvailable_ = available; }

private:
    static std::string clsName(VehClass c) {
        switch (c) { case VehClass::Car: return "Car"; case VehClass::Truck: return "Truck";
                     case VehClass::Bus: return "Bus"; } return "?";
    }
    static VehClass classify(const std::string& vClass) {
        if (vClass.find("truck") != std::string::npos) return VehClass::Truck;
        if (vClass.find("bus")   != std::string::npos) return VehClass::Bus;
        return VehClass::Car;  // car/passenger/private/default
    }
    // encode (class,index) into a stable, decodable handle so classOf() works.
    static VehHandle handleOf(VehClass c, int i) { return static_cast<int>(c) * 100000 + i; }
    static VehClass  classOf(VehHandle h) { return static_cast<VehClass>(h / 100000); }
    std::queue<VehHandle>& poolFor(VehClass c) {
        switch (c) { case VehClass::Truck: return truckPool_; case VehClass::Bus: return busPool_;
                     default: return carPool_; }
    }
    static std::string fx(double v) { std::ostringstream o; o << std::fixed << std::setprecision(3) << v; return o.str(); }
    void rec(const char* verb, const std::string& arg) {
        events_.push_back(arg.empty() ? std::string(verb) : (std::string(verb) + " " + arg));
    }

    std::vector<std::string> events_;
    std::queue<VehHandle> carPool_, truckPool_, busPool_;
    EgoState ego_;
    bool egoAvailable_ = true;
    bool poolInit_ = false;
};

} // namespace virenv
