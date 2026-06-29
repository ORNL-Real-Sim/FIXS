#pragma once
//============================================================================
//  IVirEnvBackend  (#174 backend-agnostic VirtualEnvironment core)
//----------------------------------------------------------------------------
//  The verb interface that lets ONE orchestration core (VirEnvCore, lifted out
//  of VirEnvHelper) drive BOTH the CarMaker bridge and the Carla bridge. The
//  core owns the simulator-agnostic logic (FIXS sockets, id<->handle mapping,
//  spawn/despawn bookkeeping, temporal interpolation, TLS dispatch, rate-gating,
//  ego mode A/B). Each host supplies the backend-specific "verbs" below.
//
//  Design rules (from issue #174):
//   - This header is SDK-FREE: no <CarMaker.h>, no Carla headers -> the core that
//     consumes it compiles for CarMaker, Carla, AND the dSPACE real-time image.
//   - Poses cross this boundary in the CANONICAL FIXS frame (front-of-vehicle
//     reference point, z at ground; yaw east=0 CCW rad). The BACKEND applies its
//     own pose ANCHOR (CarMaker rear-surface/CoM via TrfObj->Cfg.l/h/zOff; Carla
//     via BridgeHelper) -- the anchor never lives in the core. Interpolation
//     stays in the core, BEFORE this boundary -> verb-boundary asserts compare
//     canonical decisions, not backend coordinate output.
//   - Handles are opaque ints: a CarMaker pre-placed slot index, or a Carla actor
//     id. >=0 is valid; spawnVehicle returns kNoHandle when the backend is full.
//
//  Mapping to today's monolithic CommonLib/VirEnvHelper.cpp (the extraction map):
//    log/logError      <- CM_Log / CM_LogErrF (Log / LogErrF)
//    initTrafficPool   <- the simTime<0.05 block (enumerate Traffic.nObjs, classify
//                         by Cfg.Name, park t_0[2]=-5000, build CmAvailable*_queue)
//    spawnVehicle      <- pop CmAvailable<Class>Id_queue  (map_ids step)
//    despawnVehicle    <- park t_0[2]=-5000 + push queue   (cleanup_vehicles step)
//    setVehiclePose    <- TrfObj->t_0[]/r_zyx[] write + Cfg.l/h/zOff anchor
//    setVehicleLights  <- Traffic_Lights_GetByObjId + Lights_Set_*
//    setTrafficLight   <- TrfLight.Objs[idx].State = tlsChar2CmState(state)
//    readEgoState      <- Vehicle.v/Yaw/PoI_Pos + Cfg.Bdy1_CoM anchor + Lights
//    setEgoPose        <- (mode B teleport-in; not yet wired on the CarMaker side)
//============================================================================

#include <string>
#include <cstdint>

namespace virenv {

// Vehicle classes the bridge distinguishes (CarMaker keeps a separate slot pool
// per class; Carla maps to blueprint categories). Matches the RS_C / RS_T naming
// + the CmAvailable{Car,Truck,Bus}Id_queue split in VirEnvHelper.
enum class VehClass { Car, Truck, Bus };

// Opaque backend handle for a spawned/mapped vehicle. CarMaker: Traffic slot id.
// Carla: actor id. Returned by spawnVehicle; passed back to pose/lights/despawn.
using VehHandle = int;
static constexpr VehHandle kNoHandle = -1;  // backend full / nothing to spawn into

// Pose in the CANONICAL FIXS frame (pre-anchor). x/y/z is the vehicle's FRONT
// reference point at GROUND height; the backend translates to its own ref point.
//  yaw   : rad, east=0, north=+pi/2, CCW positive (already converted from the FIXS
//          wire heading by the core).
//  pitch : rad, RealSim grade convention negated (matches VehDataAuxiliary.pitch).
struct Pose {
    double x = 0.0, y = 0.0, z = 0.0;
    double pitch = 0.0, yaw = 0.0;
};

// Ego state read back from a backend that owns ego dynamics (mode A). Returned in
// the CANONICAL FIXS frame so the core can pack it straight onto the wire; the
// backend removes its own anchor (e.g. CarMaker Vehicle.Cfg.Bdy1_CoM) before
// returning. heading is the FIXS wire convention (deg, north=0, CW).
struct EgoState {
    double speed = 0.0;                 // m/s
    double x = 0.0, y = 0.0, z = 0.0;   // FIXS front-of-vehicle, ground
    double heading = 0.0;               // deg, north=0, clockwise
    bool brake = false, indL = false, indR = false;
};

// Host-supplied verbs. A backend implements these against its SDK; the core never
// sees a CarMaker or Carla symbol. All methods are called from the core's runStep
// in the order of the seven-step skeleton.
class IVirEnvBackend {
public:
    virtual ~IVirEnvBackend() = default;

    // --- logging -----------------------------------------------------------
    virtual void log(const char* msg) = 0;
    virtual void logError(const char* msg) = 0;

    // --- traffic pool lifecycle -------------------------------------------
    // Called once at the first step: prepare the pool (CarMaker enumerates its
    // pre-placed RS_C/RS_T slots and parks them off-road; Carla may no-op and
    // spawn lazily). After this, spawnVehicle must be serviceable.
    virtual void initTrafficPool() = 0;

    // Acquire a backend handle for a newly-seen traffic vehicle of the given
    // class. Returns kNoHandle if the backend has no capacity (core then skips
    // the vehicle this step, exactly like the full-queue `continue` today).
    virtual VehHandle spawnVehicle(VehClass cls) = 0;

    // Release a handle whose vehicle left the sim (CarMaker parks the slot at
    // z=-5000 and returns it to the pool; Carla destroys the actor).
    virtual void despawnVehicle(VehHandle h) = 0;

    // --- per-step actuation (canonical frame in; backend applies anchor) ---
    virtual void setVehiclePose(VehHandle h, const Pose& p) = 0;
    virtual void setVehicleLights(VehHandle h, bool brake, bool indL, bool indR) = 0;

    // Traffic-signal dispatch. lightIndex is the backend light slot (CarMaker
    // TrfLight.Objs index); sumoState is the raw SUMO TLS char ('r'/'y'/'g'/'G'
    // /'u'/'O'). The backend owns the char->native-state mapping (CarMaker's
    // tlsChar2CmState). NOTE: this stays a SUMO char until #156 defines a
    // FIXS-canonical signal type for the VISSIM signal-routing path.
    virtual void setTrafficLight(int lightIndex, char sumoState) = 0;

    // --- ego coupling ------------------------------------------------------
    // Mode A "backend owns ego": read the ego pose back so the core sends it out.
    // Returns false if the ego is unavailable this step.
    virtual bool readEgoState(const std::string& egoId, EgoState& out) = 0;

    // Mode B "external owns ego": teleport an externally-driven ego into the
    // backend (render + sensors only). Default no-op so mode-A-only backends need
    // not implement it yet (CarMaker mode B is unwired today).
    virtual void setEgoPose(const std::string& /*egoId*/, const Pose& /*p*/) {}

    // Optional L2 hook: advise the backend's in-sim driver of a desired speed
    // (e.g. Carla TM set_desired_speed). Default no-op.
    virtual void applyEgoControl(const std::string& /*egoId*/, double /*desiredSpeed*/) {}
};

} // namespace virenv
