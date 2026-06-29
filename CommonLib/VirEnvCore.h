#pragma once
//============================================================================
//  VirEnvCore  (#174 backend-agnostic VirtualEnvironment orchestration core)
//----------------------------------------------------------------------------
//  The SDK-free heart of the virtual-environment bridge, extracted from the
//  CarMaker-bound VirEnvHelper. It owns the simulator-agnostic logic -- FIXS
//  sockets, id<->handle mapping, spawn/despawn bookkeeping, temporal
//  interpolation, refresh rate-gating, signal forwarding, ego readback/send --
//  and drives ONE host (CarMaker or Carla) through the IVirEnvBackend verbs.
//  Compiles with NEITHER CarMaker nor Carla SDK (no <CarMaker.h>, no Carla
//  headers) -> linkable by both hosts and into the dSPACE real-time image.
//
//  Behaviour is preserved bit-for-bit relative to the two old paths via two
//  policy knobs (both backends despawn on the first absent step already, so no
//  knob is needed there):
//   - interpolateTraffic : CarMaker sub-steps at ~1 kHz and interpolates the
//                          0.1 s FIXS updates (true). Carla ticks 1:1 with FIXS
//                          and sets the received pose directly (false).
//   - the per-refresh spare re-park (#168) is delegated to backend.parkSpares().
//============================================================================

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "SocketHelper.h"
#include "MsgHelper.h"
#include "IVirEnvBackend.h"

namespace virenv {

class VirEnvCore {
public:
    VirEnvCore() = default;

    // The host owns the backend; the core keeps a non-owning pointer.
    void setBackend(IVirEnvBackend* be) { backend_ = be; }

    // Policy: interpolate 0.1 s FIXS updates across sub-steps (CarMaker) vs set
    // the received pose directly each tick (Carla 1:1). Host sets before init.
    bool interpolateTraffic = true;

    // Policy: does the core send the ego back to FIXS (CarMaker mode-A readback,
    // true)? Carla sets false -- its driver sends interested-vehicle readback
    // POST-tick (as mainVirCarla does), so the core must not also send.
    bool sendEgoFromCore = true;

    // Same return codes as the old VirEnvHelper, so callers (User.c) are unchanged.
    enum InitErr { ERROR_INIT_READ_CONFIG = -1, ERROR_INIT_MSG_FIELD = -2,
                   ERROR_INIT_SOCKET = -3, ERROR_INIT_TRAFFIC = -4 };
    enum StepErr { ERROR_STEP_RECV = -1, ERROR_STEP_MAP = -2, ERROR_STEP_REMOVE = -3,
                   ERROR_STEP_UPDATE = -4, ERROR_STEP_SEND = -5, ERROR_STEP_REFRESH = -6,
                   ERROR_STEP_SYNC = -7 };

    // The host fills the config below + Msg_c (VehicleMessageField) BEFORE
    // initialization. (The CarMaker shim copies from Config_c.CarMakerSetup; the
    // Carla host from Config_c.CarlaSetup -- that is what makes the core neutral.)
    int  initialization(const char** errorMsg, const char* configPath, const char* signalTablePath);
    int  runStep(double simTime, const char** errorMsg);
    // Orchestration without the socket recv -- callable directly by a replay test
    // that pre-fills Msg_c.VehDataRecv_um (no CarMaker/Carla/server needed).
    int  processStep(double simTime, bool onUpdate, int simStateRecv, float simTimeRecv,
                     const char** errorMsg);
    void shutdown();

    // Public so the host can reach the socket/msg layer (mirrors old VirEnvHelper).
    SocketHelper Sock_c;
    MsgHelper    Msg_c;

    // --- host-set config (neutral; not tied to a yaml section) -------------
    bool        ENABLE_REALSIM = true;
    bool        ENABLE_SEPARATE_EGO_TRAFFIC = false;  // mode B: ego on a 2nd conn
    bool        SYNCHRONIZE_TRAFFIC_SIGNAL = true;
    std::string egoId_, egoType_, trafficLayerIP_;
    int         vehDataPort_ = 0, trafficSignalPort_ = 0;
    double      trafficRefreshRate_ = 0.001;
    std::string cmErrorFile_ = "RealSimVirEnv.err";

private:
    IVirEnvBackend* backend_ = nullptr;

    int  veryFirstStep_ = 1;

    // --- id <-> backend handle map + interpolation state -------------------
    std::unordered_map<std::string, VehHandle> id2handle_;

    struct Sample { double t = 0.0; Pose pose; int lightBits = 0; };
    std::unordered_map<std::string, Sample> prev_, next_;
    std::unordered_map<std::string, Pose>   lastSet_;  // last raw pose the core set
                                                       // (replaces TrfObj->t_0 readback)
    std::vector<std::string> serverAddr_;
    std::vector<int>         serverPort_;

    long lastRefreshSlot_ = -1;

    void  logCore(const char* msg);
    static int decodeLightBits(int lightIndicators, bool& brake, bool& indL, bool& indR);
};

} // namespace virenv
