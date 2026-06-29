#include "VirEnvCore.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

namespace virenv {

void VirEnvCore::logCore(const char* msg) {
    if (backend_) backend_->log(msg);
    else std::cout << msg;
}

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
        if (SYNCHRONIZE_TRAFFIC_SIGNAL) {
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

    // ---- recv from FIXS (only on the 0.1 s update boundary, t>0) ----------
    bool onUpdate = (simTime > 1e-5 && std::fabs(simTime * 10 - ceil(simTime * 10 - 0.5)) < 1e-5);
    if (onUpdate) {
        Msg_c.clearRecvStorage();
        if (ENABLE_REALSIM && simTime) {
            for (unsigned int iS = 0; iS < Sock_c.serverSock.size(); iS++) {
                if (Sock_c.recvData(Sock_c.serverSock[iS], &simStateRecv, &simTimeRecv, Msg_c) < 0) {
                    *errorMsg = "RealSim: Receive from traffic simulator failed";
                    return ERROR_STEP_RECV;
                }
            }
        }
    }
    return processStep(simTime, onUpdate, simStateRecv, simTimeRecv, errorMsg);
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
            VehHandle h = backend_ ? backend_->spawnVehicle(it.second.type, it.second.vehicleClass)
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

        // ---- step 3: stage the new target pose (raw FIXS) + interp prev ---
        double simTimeNext = ceil(simTime * 10 + 0.001) / 10;
        for (auto& kv : id2handle_) {
            const string& idTs = kv.first;
            const VehFullData_t& v = Msg_c.VehDataRecv_um[idTs];

            Sample nextS;
            nextS.t = simTimeNext;
            nextS.pose.x = v.positionX;          // raw FIXS: front-of-vehicle, ground
            nextS.pose.y = v.positionY;
            nextS.pose.z = v.positionZ;
            nextS.pose.headingDeg = v.heading;   // raw wire conventions; backend converts
            nextS.pose.gradeRad   = v.grade;
            nextS.lightBits = v.lightIndicators;
            next_[idTs] = nextS;

            Sample prevS;
            prevS.t = simTime;
            if (lastSet_.find(idTs) == lastSet_.end()) {
                prevS.pose = nextS.pose;         // first sight: no interpolation yet
            } else {
                prevS.pose = lastSet_[idTs];     // last raw pose we drew (== old t_0)
            }
            prevS.lightBits = nextS.lightBits;
            prev_[idTs] = prevS;
        }

        // ---- step 4: traffic-signal sync (per junction; backend maps) -----
        if (SYNCHRONIZE_TRAFFIC_SIGNAL && backend_) {
            for (auto it : Msg_c.TlsDataRecv_um) {
                backend_->syncTrafficLight(it.second.name, it.second.state);
            }
        }
    }

    // ---- step 5: send ego back (mode A readback) on the update boundary ---
    if (onUpdate) {
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
            } else {
                out.x = nS.pose.x; out.y = nS.pose.y; out.z = nS.pose.z;  // Carla 1:1 direct
            }
            // heading/grade are NOT interpolated (take the latest, as before)
            out.headingDeg = nS.pose.headingDeg;
            out.gradeRad   = nS.pose.gradeRad;

            if (backend_) backend_->setVehiclePose(h, out);
            lastSet_[idTs] = out;

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
