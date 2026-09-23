#include "VirEnvCore.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

namespace virenv {

void VirEnvCore::logCore(const char* msg) {
    if (backend_) backend_->log(msg);
    else std::cout << msg;
}

#ifdef RS_DEBUG
// One unified per-vehicle diagnostic row, identical columns for every backend.
//   fixs_*  : the FIXS-canonical pose received this step (front-of-vehicle, the
//             wire conventions -- this is the simulator-neutral ground truth).
//   set_*   : the pose the core handed to the backend (== fixs_* for Carla 1:1;
//             interpolated x/y/z for CarMaker sub-steps).
// Backend-specific readings (e.g. the applied Carla transform, or CarMaker t_0 /
// Vehicle.v) are appended via debugFields() so it all stays in ONE csv.
void VirEnvCore::rsDebugRow_(double simTime, const std::string& id, VehHandle h,
                             const Pose& fixs, const Pose& applied, int lightBits) {
    if (rsDbg_ == nullptr) {
        rsDbg_ = std::fopen("RealSim_tmp\\rs_core_pos.csv", "w");
        if (rsDbg_ == nullptr) rsDbg_ = std::fopen("rs_core_pos.csv", "w");  // CWD fallback
        if (rsDbg_ != nullptr) {
            std::fprintf(rsDbg_,
                "simTime,id,handle,fixs_x,fixs_y,fixs_z,fixs_heading,fixs_grade,"
                "set_x,set_y,set_z,set_heading,lightBits%s\n",
                backend_ ? backend_->debugHeader().c_str() : "");
        }
    }
    if (rsDbg_ == nullptr) return;
    const std::string extra = backend_ ? backend_->debugFields(h) : std::string();
    std::fprintf(rsDbg_,
        "%.3f,%s,%d,%.4f,%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.4f,%d%s\n",
        simTime, id.c_str(), h, fixs.x, fixs.y, fixs.z, fixs.headingDeg, fixs.gradeRad,
        applied.x, applied.y, applied.z, applied.headingDeg, lightBits, extra.c_str());
    std::fflush(rsDbg_);
}
#endif

// FIXS lightIndicators bitfield -> brake / left / right (verbatim from the old
// refresh loop: right=bit0, left=bit1, brake=bit3).
int VirEnvCore::decodeLightBits(int lightIndicators, bool& brake, bool& indL, bool& indR) {
    indR  = (lightIndicators & (1 << 0)) >> 0;
    indL  = (lightIndicators & (1 << 1)) >> 1;
    brake = (lightIndicators & (1 << 3)) >> 3;
    return 0;
}

//============================================================================
//  initialization -- SDK-free. The host fills the public config fields + Msg_c
//  (VehicleMessageField) BEFORE calling this; here we only validate, hand the
//  signal table to the backend, set up the FIXS sockets, and connect.
//============================================================================
int VirEnvCore::initialization(const char** errorMsg, const char* /*configPath*/,
                               const char* signalTablePath) {
    veryFirstStep_ = 0;
    // config fields + Msg_c are filled by the host before this call.

    // required subscription fields (same contract as before)
    if (Msg_c.VehicleMessageField_set.find("vehicleClass") == Msg_c.VehicleMessageField_set.end()
        || Msg_c.VehicleMessageField_set.find("heading") == Msg_c.VehicleMessageField_set.end()
        || Msg_c.VehicleMessageField_set.find("grade") == Msg_c.VehicleMessageField_set.end()) {
        *errorMsg = "RealSim: Must subscribe: id, speed, vehicleClass, heading, grade, speedDesired/accelerationDesired";
        return ERROR_INIT_MSG_FIELD;
    }

    if (SYNCHRONIZE_TRAFFIC_SIGNAL && backend_) {
        backend_->loadSignalTable(signalTablePath);
    }

    try {
        serverAddr_.clear(); serverPort_.clear();
        serverAddr_.push_back(trafficLayerIP_);
        serverPort_.push_back(vehDataPort_);
        if (SYNCHRONIZE_TRAFFIC_SIGNAL && openSignalPort) {
            serverAddr_.push_back(trafficLayerIP_);
            serverPort_.push_back(trafficSignalPort_);
        }
        Sock_c.socketSetup(serverAddr_, serverPort_);
        Sock_c.disableServerTrigger();
        Sock_c.disableWaitClientTrigger();

        if (ENABLE_REALSIM) {
            if (Sock_c.initConnection(cmErrorFile_) > 0) {
                printf("Connect to RealSim failed! Make sure start TrafficLayer first\n");
                *errorMsg = "RealSim: Initialize Socket Failed";
                return ERROR_INIT_SOCKET;
            }
        }
    }
    catch (const std::exception& e) {
        Sock_c.socketShutdown();
        std::cout << e.what();
        *errorMsg = "RealSim: Initialize Socket Failed";
        return ERROR_INIT_SOCKET;
    }
    catch (...) {
        Sock_c.socketShutdown();
        *errorMsg = "RealSim: Initialize Socket Failed";
        return ERROR_INIT_SOCKET;
    }
    return 0;
}

void VirEnvCore::shutdown() {
    logCore("RealSim shutdown \n");
    veryFirstStep_ = 1;
    id2handle_.clear();
    prev_.clear(); next_.clear(); lastSet_.clear();
    lastRefreshSlot_ = -1;
}

//============================================================================
//  runStep -- the seven-step skeleton, now driven through IVirEnvBackend.
//============================================================================
int VirEnvCore::runStep(double simTime, const char** errorMsg) {
    string errorMsgStr;
    int   simStateRecv = 0;
    float simTimeRecv  = 0;

    // ---- recv from FIXS (only on an exchange boundary, t>0) ----------------
    // fixs::kFeedPeriodS is the protocol's exchange period, not a knob: the host
    // may tick as fine as it likes, but it trades messages with TrafficLayer only
    // here, and TrafficLayer steps the traffic simulator once per exchange.
    bool onUpdate = (simTime > 1e-5 && fixs::onFeedBoundary(simTime, 1e-5));
    lastRecvSeconds = 0.0;
    if (onUpdate) {
        const auto recvT0 = std::chrono::steady_clock::now();
        Msg_c.clearRecvStorage();
        if (ENABLE_REALSIM && simTime) {
            for (unsigned int iS = 0; iS < Sock_c.serverSock.size(); iS++) {
                if (Sock_c.recvData(Sock_c.serverSock[iS], &simStateRecv, &simTimeRecv, Msg_c) < 0) {
                    *errorMsg = "RealSim: Receive from traffic simulator failed";
                    return ERROR_STEP_RECV;
                }
            }
        }
        lastRecvSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - recvT0).count();
    }
    return processStep(simTime, onUpdate, simStateRecv, simTimeRecv, errorMsg);
}

// Wrap-aware interpolation of a FIXS wire heading (degrees, north = 0, CLOCKWISE
// -- the convention documented on Pose in IVirEnvBackend.h, carried verbatim from
// SUMO's VAR_ANGLE / VISSIM's heading; the backend, not the core, converts it to
// its host's frame).
//
// Both ends are normalised into [0, 360), then the blend follows the SHORTEST arc:
//     d = fmod(n - p + 540, 360) - 180
// After normalisation n - p is in (-360, 360), so the +540 makes the dividend
// non-negative -- required, because C's fmod keeps the sign of the DIVIDEND and a
// negative remainder would pick the long way round. d then lands in [-180, 180):
// the signed short way. f = 0 returns p; f = 1 returns n (in the seam case via a
// renormalisation, so within ~1e-14 deg rather than bit-exact).
static double lerpHeadingDeg(double prevDeg, double nextDeg, double f) {
    const double p = std::fmod(std::fmod(prevDeg, 360.0) + 360.0, 360.0);
    const double n = std::fmod(std::fmod(nextDeg, 360.0) + 360.0, 360.0);
    const double d = std::fmod(n - p + 540.0, 360.0) - 180.0;
    const double out = p + d * f;
    return std::fmod(std::fmod(out, 360.0) + 360.0, 360.0);
}

//============================================================================
//  processStep -- map / despawn / interpolate / signals / send / refresh,
//  using whatever is already in Msg_c (so a replay test can drive it with no
//  sockets by pre-filling VehDataRecv_um and calling this directly).
//============================================================================
int VirEnvCore::processStep(double simTime, bool onUpdate, int simStateRecv, float simTimeRecv,
                            const char** errorMsg) {
    // ---- step 0: first-step pool init (backend enumerates/prepares slots) --
    if (simTime < 0.05) {
        if (backend_) backend_->initTrafficPool();
    }

    if (onUpdate) {
        // ---- step 1: map received ids to backend handles (spawn new) ------
        for (auto it : Msg_c.VehDataRecv_um) {
            const string& idTs = it.second.id;
            if (idTs == egoId_) continue;                          // never spawn the ego
            if (id2handle_.find(idTs) != id2handle_.end()) continue; // already mapped
            Pose sp;
            sp.x = it.second.positionX; sp.y = it.second.positionY; sp.z = it.second.positionZ;
            sp.headingDeg = it.second.heading; sp.gradeRad = it.second.grade;
            VehHandle h = backend_ ? backend_->spawnVehicle(it.second.type, it.second.vehicleClass, sp)
                                   : kNoHandle;
            if (h == kNoHandle) continue;                          // backend full -> skip
            id2handle_[idTs] = h;
        }

        // ---- step 2: despawn vehicles that disappeared this step ----------
        for (auto it = id2handle_.begin(); it != id2handle_.end(); ) {
            const string& idTs = it->first;
            if (Msg_c.VehDataRecv_um.find(idTs) == Msg_c.VehDataRecv_um.end()) {
                if (backend_) backend_->despawnVehicle(it->second);
                prev_.erase(idTs); next_.erase(idTs); lastSet_.erase(idTs);
                it = id2handle_.erase(it);
            } else {
                ++it;
            }
        }

        // ---- step 3: stage the k+1 target (raw FIXS); prev = the k pose --------
        // Interpolate between CONSECUTIVE RECEIVED samples (k -> k+1) -- the
        // causally-correct co-sim motion: the received traffic IS the next-step
        // (k+1) state and the previous received is the current (k) state, so the
        // interval [simTime, simTimeNext] carries the vehicle exactly along the
        // k->k+1 segment. Using the previous RECEIVED pose (not the last DRAWN
        // pose) makes this exact at ANY sub-step rate; the old last-drawn anchor
        // lagged ~half a step at coarse Carla sub-steps (they never reach f=1
        // before the boundary re-stages).
        double simTimeNext = ceil(simTime * fixs::kFeedHz + 0.001) / fixs::kFeedHz;
        for (auto& kv : id2handle_) {
            const string& idTs = kv.first;
            const VehFullData_t& v = Msg_c.VehDataRecv_um[idTs];

            // capture the previous target (the k pose) BEFORE overwriting next_
            auto pit = next_.find(idTs);
            const bool hasPrev = (pit != next_.end());
            const Pose kPose = hasPrev ? pit->second.pose : Pose();

            Sample nextS;
            nextS.t = simTimeNext;
            nextS.pose.x = v.positionX;          // raw FIXS: front-of-vehicle, ground
            nextS.pose.y = v.positionY;
            nextS.pose.z = v.positionZ;
            nextS.pose.headingDeg = v.heading;   // raw wire conventions; backend converts
            nextS.pose.gradeRad   = v.grade;
            nextS.lightBits = v.lightIndicators;

            Sample prevS;
            prevS.t = simTime;
            prevS.pose = hasPrev ? kPose : nextS.pose;   // first sight: no interp yet
            prevS.lightBits = nextS.lightBits;

            prev_[idTs] = prevS;
            next_[idTs] = nextS;
        }

        // ---- step 4: traffic-signal sync (per junction; backend maps) -----
        if (SYNCHRONIZE_TRAFFIC_SIGNAL && backend_) {
            for (auto it : Msg_c.TlsDataRecv_um) {
                backend_->syncTrafficLight(it.second.name, it.second.state);
            }
        }
    }

    // ---- step 5: send ego back (mode A readback) on the update boundary ---
    if (onUpdate && sendEgoFromCore) {
        VehFullData_t VehDataSend;
        bool haveEgo = false;
        if ((!ENABLE_SEPARATE_EGO_TRAFFIC || simTime < 1e-5) && backend_) {
            EgoState ego;
            if (backend_->readEgoState(egoId_, ego)) {
                haveEgo = true;
                VehDataSend.id = egoId_;
                VehDataSend.type = egoType_;
                VehDataSend.speed = (float)ego.speed;
                VehDataSend.heading = (float)ego.heading;
                VehDataSend.positionX = (float)ego.x;
                VehDataSend.positionY = (float)ego.y;
                VehDataSend.positionZ = (float)ego.z;
                VehDataSend.acceleration = 0.f;
                VehDataSend.color = (uint32_t)0;
                VehDataSend.linkId = string("None");
                VehDataSend.laneId = (int32_t)0;
                VehDataSend.distanceTravel = 0.f;
                VehDataSend.speedDesired = (float)ego.speed;
                VehDataSend.accelerationDesired = 0.f;
                VehDataSend.lightIndicators = (uint16_t)(ego.brake * 8 + ego.indL * 2 + ego.indR);
            }
        }
        if (ENABLE_REALSIM) {
            for (unsigned int iS = 0; iS < Sock_c.serverSock.size(); iS++) {
                int iByte = 0;
                char sendServerBuffer[8096];
                Msg_c.packHeader(simStateRecv, simTimeRecv, MSG_HEADER_SIZE, sendServerBuffer, &iByte);
                Sock_c.sendServerByte[iS] = { MSG_HEADER_SIZE };
                if (haveEgo && (!ENABLE_SEPARATE_EGO_TRAFFIC || simTime < 1e-5) && iS == 0) {
                    Msg_c.packVehData(VehDataSend, sendServerBuffer, &Sock_c.sendServerByte[iS]);
                }
                iByte = 0;
                Msg_c.packHeader(simStateRecv, simTimeRecv, Sock_c.sendServerByte[iS], sendServerBuffer, &iByte);
                if (send(Sock_c.serverSock[iS], sendServerBuffer, Sock_c.sendServerByte[iS], 0) != Sock_c.sendServerByte[iS]) {
                    *errorMsg = "RealSim: Send failed";
                    return ERROR_STEP_SEND;
                }
            }
        }
    }

    // ---- step 6: refresh -- park spares + (interp or direct) set poses ----
    int refreshRate = (trafficRefreshRate_ > 0.0) ? (int)(1.0 / trafficRefreshRate_) : 1;
    long refreshSlot = (long)std::llround((double)simTime * refreshRate);
    if (refreshSlot != lastRefreshSlot_) {
        lastRefreshSlot_ = refreshSlot;
        if (backend_) backend_->parkSpares();

        for (auto& kv : id2handle_) {
            const string& idTs = kv.first;
            VehHandle h = kv.second;
            if (next_.find(idTs) == next_.end()) continue;

            const Sample& pS = prev_[idTs];
            const Sample& nS = next_[idTs];

            Pose out;
            if (interpolateTraffic && nS.t > pS.t) {
                double f = (simTime - pS.t) / (nS.t - pS.t);
                out.x = pS.pose.x + (nS.pose.x - pS.pose.x) * f;
                out.y = pS.pose.y + (nS.pose.y - pS.pose.y) * f;
                out.z = pS.pose.z + (nS.pose.z - pS.pose.z) * f;
                // Heading has to be interpolated too, on the SHORTEST ARC. Taking
                // only the latest sample (what this did before) leaves a sub-stepping
                // host sliding its traffic smoothly while the yaw snaps once per
                // exchange -- a car that translates along a curve but rotates in
                // 0.1 s steps. Interpolating it linearly would be worse in one
                // specific place: headingDeg is the FIXS wire convention (degrees,
                // north = 0, CLOCKWISE, see IVirEnvBackend.h), so a vehicle heading
                // due north oscillates across the 360/0 seam (359.8 -> 0.2), and a
                // plain lerp would sweep the long way round -- a full spin in one
                // feed interval. lerpHeadingDeg does the wrap-aware blend. A host
                // that ticks 1:1 with the feed never gets here (the else branch
                // applies the raw sample), so the #174 byte-identical path is
                // untouched by this.
                out.headingDeg = lerpHeadingDeg(pS.pose.headingDeg, nS.pose.headingDeg, f);
                // Grade is a slope in radians (no wrap): plain lerp.
                out.gradeRad = pS.pose.gradeRad + (nS.pose.gradeRad - pS.pose.gradeRad) * f;
            } else {
                out.x = nS.pose.x; out.y = nS.pose.y; out.z = nS.pose.z;  // 1:1 direct
                out.headingDeg = nS.pose.headingDeg;
                out.gradeRad   = nS.pose.gradeRad;
            }

            if (backend_) backend_->setVehiclePose(h, out);
            lastSet_[idTs] = out;
#ifdef RS_DEBUG
            rsDebugRow_(simTime, idTs, h, nS.pose, out, nS.lightBits);
#endif

            if (Msg_c.VehicleMessageField_set.find("lightIndicators") != Msg_c.VehicleMessageField_set.end()) {
                bool brake, indL, indR;
                decodeLightBits(nS.lightBits, brake, indL, indR);
                if (backend_) backend_->setVehicleLights(h, brake, indL, indR);
            }
        }
    }
    return 0;
}

} // namespace virenv
