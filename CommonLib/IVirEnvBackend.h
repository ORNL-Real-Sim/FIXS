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

// Pose in the RAW FIXS wire frame (pre-anchor, pre-convention). x/y/z is the
// vehicle's FRONT reference point at GROUND height; headingDeg/gradeRad are the
// FIXS wire conventions verbatim. The BACKEND does ALL of: convention conversion
// (heading->CarMaker yaw / Carla rotation) AND its pose anchor (CarMaker Cfg.l/h
// /zOff; Carla extent). Keeping the raw values here -- not a pre-converted yaw --
// is what lets each backend reproduce its ORIGINAL math bit-for-bit (the #174
// byte-identical goal), since the core never round-trips the angle.
//  headingDeg : FIXS wire heading, north=0, clockwise (degrees).
//  gradeRad   : FIXS wire grade, positive = climbing (radians).
struct Pose {
    double x = 0.0, y = 0.0, z = 0.0;
    double headingDeg = 0.0;
    double gradeRad = 0.0;
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

    // Load whatever the backend needs to dispatch traffic signals -- CarMaker
    // reads the RSsignalTable.csv (controller -> head id + TrfLight index); Carla
    // matches its traffic.traffic_light actors to the junction map. Default no-op
    // (a signal-free scenario like SimpleLoop never calls it). Called once in init.
    virtual void loadSignalTable(const char* /*path*/) {}

    // --- traffic pool lifecycle -------------------------------------------
    // Called once at the first step: prepare the pool (CarMaker enumerates its
    // pre-placed RS_C/RS_T slots and parks them off-road; Carla may no-op and
    // spawn lazily). After this, spawnVehicle must be serviceable.
    virtual void initTrafficPool() = 0;

    // Acquire a backend handle for a newly-seen traffic vehicle. vType is the
    // FIXS vehicle type, vClass the SUMO/VISSIM class string -- the backend does
    // its own classification (CarMaker: vClass -> car/truck slot pool; Carla:
    // vType or vClass -> blueprint). spawnPose is the raw FIXS pose to place the
    // vehicle at (Carla's TrySpawnActor needs a transform; CarMaker's pre-placed
    // slots ignore it). Returns kNoHandle if it cannot place the vehicle (e.g.
    // CarMaker pool exhausted), and the core skips it this step exactly like the
    // full-queue `continue` today.
    virtual VehHandle spawnVehicle(const std::string& vType, const std::string& vClass,
                                   const Pose& spawnPose) = 0;

    // Release a handle whose vehicle left the sim (CarMaker parks the slot at
    // z=-5000 and returns it to the pool; Carla destroys the actor).
    virtual void despawnVehicle(VehHandle h) = 0;

    // --- per-step actuation (raw FIXS pose in; backend converts + anchors) -
    virtual void setVehiclePose(VehHandle h, const Pose& p) = 0;
    virtual void setVehicleLights(VehHandle h, bool brake, bool indL, bool indR) = 0;

    // Called once per refresh slot, before the mapped-vehicle pose loop. CarMaker
    // must re-park its UNMAPPED RS_C/RS_T spare slots at z=-5000 EVERY refresh
    // (FreeMotion slots drift at low UpdRate -- #168); Carla has no spare pool so
    // this is a no-op. Lets the core stay ignorant of the backend's spare set.
    virtual void parkSpares() {}

    // Dispatch one junction's signal state. junctionId is the FIXS/SUMO TLS id,
    // stateStr its per-link SUMO state string ("rGyu..."). The backend owns the
    // per-link mapping + char->native-state conversion (CarMaker's signal table +
    // tlsChar2CmState; Carla's trafficLightMap + SetState). NOTE: stays a SUMO
    // state string until #156 defines a FIXS-canonical signal type. Default no-op.
    virtual void syncTrafficLight(const std::string& /*junctionId*/, const std::string& /*stateStr*/) {}

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

    // --- unified RS_DEBUG diagnostics --------------------------------------
    // When the core is built with RS_DEBUG it writes ONE canonical per-vehicle
    // CSV (simTime, id, the FIXS-canonical pose, the pose handed to the backend).
    // A backend may append its OWN columns to that SAME row so every simulator
    // shares one log instead of N divergent CSVs: debugHeader() returns the extra
    // column names (comma-led, e.g. ",carla_x,carla_y,carla_yaw") written once;
    // debugFields(h) returns the matching values for one vehicle each step.
    // Default empty -> the core log has identical columns regardless of backend.
    virtual std::string debugHeader() const { return std::string(); }
    virtual std::string debugFields(VehHandle /*h*/) const { return std::string(); }
};

} // namespace virenv
