#pragma once
//============================================================================
//  VirEnvHelper  (#174)  -- now a THIN CarMaker host-shim over VirEnvCore.
//----------------------------------------------------------------------------
//  Previously this class WAS the whole CarMaker-bound bridge (~1150 lines). The
//  backend-agnostic orchestration moved to CommonLib/VirEnvCore (SDK-free) and
//  the CarMaker verbs to CommonLib/CarMakerBackend. VirEnvHelper now just: owns
//  a VirEnvCore + a CarMakerBackend, copies the CarMakerSetup config into the
//  core, and delegates. The C wrapper (VirEnv_Wrapper.cpp -> User.c) is unchanged.
//============================================================================

#include <string>
#include <vector>

#include "VirEnvCore.h"
#include "CarMakerBackend.h"

#ifndef RS_DSPACE
#include "ConfigHelper.h"
#endif

class VirEnvHelper {
public:
    VirEnvHelper();

    int veryFirstStep = 1;

    int CM_Log(const char* MsgChar);
    int CM_LogErrF(const char* MsgChar);

    int  initialization(const char** errorMsg, const char* configPath, const char* signalTablePath);
    int  runStep(double simTime, const char** errorMsg);
    void shutdown();

#ifndef RS_DSPACE
    ConfigHelper Config_c;
#else
    // dSPACE config struct -- still filled field-by-field by VirEnv_readConfig in
    // the C wrapper, then copied into the core in initialization().
    typedef struct {
        std::vector<std::string> VehicleMessageField_v;
        bool        EnableCosimulation;
        bool        EnableEgoSimulink;
        std::string EgoId;
        std::string EgoType;
        std::string TrafficLayerIP;
        int         CarMakerPort;
        double      TrafficRefreshRate;
        int         TrafficSignalPort;
        bool        SynchronizeTrafficSignal;
        std::string SignalTableFilename;
    } Config_t;
    Config_t Config_s;
#endif

private:
    virenv::VirEnvCore       core_;
    virenv::CarMakerBackend  backend_;
};
