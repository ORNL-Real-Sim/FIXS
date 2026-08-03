#include "CarMakerBackend.h"

#include <cmath>
#include <fstream>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

namespace virenv {

void CarMakerBackend::log(const char* msg)      { Log(msg); }
void CarMakerBackend::logError(const char* msg) { LogErrF(EC_General, msg); }

//----------------------------------------------------------------------------
//  loadSignalTable -- verbatim from VirEnvHelper::readSignalTable
//----------------------------------------------------------------------------
void CarMakerBackend::loadSignalTable(const char* path) {
    if (path == nullptr) return;
    string p = path;
    if (!(p.size() >= 4 && p.substr(p.size() - 4) == ".csv")) p += ".csv";
    Log("SignalTable name %s\n", p.c_str());

    ifstream f(p);
    if (!f) return;

    string line;
    getline(f, line);  // header
    while (getline(f, line)) {
        istringstream ls(line);
        string el;
        string ctrl; int grp, head, cmIdx; string cmCtrl;
        getline(ls, el, ','); ctrl = el;
        getline(ls, el, ','); grp = atoi(el.c_str());
        getline(ls, el, ','); head = atoi(el.c_str());
        getline(ls, el, ','); cmIdx = atoi(el.c_str());
        getline(ls, el, ','); cmCtrl = el;
        (void)grp; (void)cmCtrl;
        if (cmIdx > -1) {
            signalCtrl2HeadIdx_[ctrl].push_back(make_pair(head, cmIdx));
        }
    }
}

//----------------------------------------------------------------------------
//  initTrafficPool -- enumerate the pre-placed RS_C/RS_T slots, classify them,
//  and park them off-road. From the simTime<0.05 block of runStep.
//----------------------------------------------------------------------------
void CarMakerBackend::initTrafficPool() {
    if (!carQueue_.empty() || !truckQueue_.empty()) return;  // once
    for (int iObj = 0; iObj < Traffic.nObjs; iObj++) {
        tTrafficObj* T = Traffic_GetByTrfId(iObj);
        if (T == nullptr) continue;
        string name = T->Cfg.Name;
        if (!isRs(name)) continue;
        if (isRsCar(name))        carQueue_.push(iObj);
        else if (isRsTruck(name)) truckQueue_.push(iObj);
        T->t_0[0] = 0; T->t_0[1] = 0; T->t_0[2] = -5000;
    }
}

//----------------------------------------------------------------------------
//  spawn / despawn -- pop/return a class slot; park on release. From the
//  map-ids + cleanup-vehicles steps.
//----------------------------------------------------------------------------
VehHandle CarMakerBackend::spawnVehicle(const std::string& /*vType*/, const std::string& vClass,
                                        const Pose& /*spawnPose*/) {
    if (vClass.find("car") != string::npos || vClass.find("passenger") != string::npos
        || vClass.find("private") != string::npos) {
        if (carQueue_.empty()) return kNoHandle;
        int h = carQueue_.front(); carQueue_.pop(); spawned_.insert(h); return h;
    }
    if (vClass.find("truck") != string::npos) {
        if (truckQueue_.empty()) return kNoHandle;
        int h = truckQueue_.front(); truckQueue_.pop(); spawned_.insert(h); return h;
    }
    return kNoHandle;  // class CarMaker has no pool for -> skip (matches old fall-through)
}

void CarMakerBackend::despawnVehicle(VehHandle h) {
    tTrafficObj* T = Traffic_GetByTrfId(h);
    if (T != nullptr) {
        T->t_0[0] = 0; T->t_0[1] = 0; T->t_0[2] = -5000;
        string name = T->Cfg.Name;
        if (isRsCar(name))        carQueue_.push(h);
        else if (isRsTruck(name)) truckQueue_.push(h);
    }
    spawned_.erase(h);
}

//----------------------------------------------------------------------------
//  setVehiclePose -- apply the CarMaker pose anchor + heading->yaw convention to
//  the raw FIXS pose. From update_traffic_state (anchor) + refresh (the write).
//   RS position is FRONT of vehicle; CM t_0 is the rearmost surface.
//   RS z is ground; CM is CoM (Cfg.h/2 + zOff), tilted by grade (Cfg.l*sin grade).
//----------------------------------------------------------------------------
void CarMakerBackend::setVehiclePose(VehHandle h, const Pose& p) {
    tTrafficObj* T = Traffic_GetByTrfId(h);
    if (T == nullptr) return;
    const double heading = p.headingDeg;
    const double grade   = p.gradeRad;

    T->t_0[0] = p.x - T->Cfg.l * sin(heading * M_PI / 180);
    T->t_0[1] = p.y - T->Cfg.l * cos(heading * M_PI / 180);
    T->t_0[2] = p.z + T->Cfg.h / 2 + T->Cfg.zOff - T->Cfg.l * sin(grade);

    double yaw;
    if (heading >= 0 && heading <= 1.5 * 180) yaw = 0.5 * M_PI - heading * M_PI / 180;
    else                                      yaw = 2.5 * M_PI - heading * M_PI / 180;

    T->r_zyx[1] = -grade;   // RS positive = climbing; CM negative
    T->r_zyx[2] = yaw;
}

void CarMakerBackend::setVehicleLights(VehHandle h, bool brake, bool indL, bool indR) {
    tTrafficObj* T = Traffic_GetByTrfId(h);
    if (T == nullptr) return;
    tLights* pLights = Traffic_Lights_GetByObjId(T->Cfg.ObjId);
    Lights_Set_CtrlElem_Ignition(pLights, 1);
    Lights_Set_LightElem_Brake(pLights, brake);
    if (indR)      Lights_Set_CtrlElem_Indicator(pLights, -1);
    else if (indL) Lights_Set_CtrlElem_Indicator(pLights, 1);
    else           Lights_Set_CtrlElem_Indicator(pLights, 0);
}

//----------------------------------------------------------------------------
//  parkSpares -- re-park every UNMAPPED RS slot at z=-5000 each refresh so the
//  FreeMotion spares hold at low UpdRate (#168). From the refresh re-park loop.
//----------------------------------------------------------------------------
void CarMakerBackend::parkSpares() {
    for (int iObj = 0; iObj < Traffic.nObjs; iObj++) {
        if (spawned_.find(iObj) != spawned_.end()) continue;
        tTrafficObj* T = Traffic_GetByTrfId(iObj);
        if (T == nullptr) continue;
        if (!isRs(T->Cfg.Name)) continue;
        T->t_0[2] = -5000;
    }
}

//----------------------------------------------------------------------------
//  syncTrafficLight -- map one junction's SUMO state string onto the CarMaker
//  TrfLight objects via the signal table. From the sync-signals step.
//----------------------------------------------------------------------------
void CarMakerBackend::syncTrafficLight(const std::string& junctionId, const std::string& stateStr) {
    auto it = signalCtrl2HeadIdx_.find(junctionId);
    if (it == signalCtrl2HeadIdx_.end()) return;
    for (const auto& hk : it->second) {
        int headIdx = hk.first, cmIdx = hk.second;
        if (headIdx >= 0 && headIdx < (int)stateStr.size())
            TrfLight.Objs[cmIdx].State = tlsChar2CmState(stateStr.at(headIdx));
    }
}

//----------------------------------------------------------------------------
//  readEgoState -- ego pose back from CarMaker (mode A), in raw FIXS terms.
//  From the send-ego block; removes the CM Bdy1_CoM anchor.
//----------------------------------------------------------------------------
bool CarMakerBackend::readEgoState(const std::string& /*egoId*/, EgoState& out) {
    out.speed = Vehicle.v;

    double heading;
    if (Vehicle.Yaw >= -M_PI && Vehicle.Yaw <= 0.5 * M_PI)
        heading = (-Vehicle.Yaw + 0.5 * M_PI) * 180 / M_PI;
    else
        heading = (-Vehicle.Yaw + 2.5 * M_PI) * 180 / M_PI;
    out.heading = heading;

    // The FIXS wire position is the vehicle FRONT. Anchor at the ego's true front
    // bumper from the loaded model's geometry: PoI -> front axle (PoI2AxleFront)
    // -> bumper (OverhangFront). This generalizes to ANY ego car. The old code
    // used Bdy1_CoM[0] (the CoM), which sits ~1.07 m behind the real front on the
    // default 5 m sedan -- it reported the ego that far back to SUMO/FIXS.
    const double frontOff = Vehicle.Cfg.PoI2AxleFront + Vehicle.Cfg.OverhangFront;
    out.x = Vehicle.PoI_Pos[0] + frontOff * sin(heading * M_PI / 180);
    out.y = Vehicle.PoI_Pos[1] + frontOff * cos(heading * M_PI / 180);
    out.z = Vehicle.PoI_Pos[2] - Vehicle.Cfg.Bdy1_CoM[2];

    out.brake = VehicleControl.Lights.Brake;
    out.indL  = VehicleControl.Lights.IndL;
    out.indR  = VehicleControl.Lights.IndR;
    return true;
}

//----------------------------------------------------------------------------
//  tlsChar2CmState -- verbatim from VirEnvHelper.
//----------------------------------------------------------------------------
tTLState CarMakerBackend::tlsChar2CmState(char charState) {
    tTLState s = State_Off;
    if (charState == 'r') s = State_Red;
    else if (charState == 'y') s = State_Yellow;
    else if (charState == 'g' || charState == 'G') s = State_Green;
    else if (charState == 'u') s = State_RedYellow;
    else if (charState == 'O') s = State_Off;
    return s;
}

} // namespace virenv
