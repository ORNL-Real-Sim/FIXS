#include "DSProxyMode.h"
#include "VissimDSProxyHelper.h"

#include <cstdio>
#include <string>

namespace FIXS {
namespace DSProxy {

int runDSProxyMode(const ConfigHelper& config) {
    // Unbuffer stdout so per-frame summary lines appear in real time even
    // when this process is run with stdout piped to a file (Stage A probe).
    setvbuf(stdout, nullptr, _IONBF, 0);

    const auto& cfg = config.VissimDSProxySetup;

    printf("\n=== DSProxy mode (Stage A, issue #158) ===\n");
    printf("VissimVersion:      %d\n", cfg.VissimVersion);
    printf("SimulatorFrequency: %d Hz\n", cfg.SimulatorFrequency);
    printf("VisibilityRadius:   %.2f m\n", cfg.VisibilityRadius);
    printf("NetworkFile:        %s\n", cfg.NetworkFile.c_str());

    if (cfg.NetworkFile.empty()) {
        fprintf(stderr, "ERROR: VissimDSProxySetup.NetworkFile is required when Enable: true\n");
        return 2;
    }

    std::string dllPath = cfg.DllPath.empty()
        ? defaultDllPathForVissim(cfg.VissimVersion)
        : cfg.DllPath;
    printf("DSProxy DLL:        %s\n", dllPath.c_str());

    VissimDSProxy proxy;
    if (!proxy.load(dllPath)) {
        fprintf(stderr, "ERROR: failed to load %s\n", dllPath.c_str());
        return 3;
    }

    const unsigned short versionNo = connectVersionNo(cfg.VissimVersion);

    printf("calling VISSIM_Connect (versionNo=%u) ...\n", versionNo);
    const bool connected = proxy.connect(
        versionNo, cfg.NetworkFile,
        static_cast<unsigned short>(cfg.SimulatorFrequency),
        cfg.VisibilityRadius,
        static_cast<unsigned short>(cfg.MaxSimulatorVeh),
        static_cast<unsigned short>(cfg.MaxSimulatorPed),
        static_cast<unsigned short>(cfg.MaxSimulatorDet),
        static_cast<unsigned short>(cfg.MaxTotalVeh),
        static_cast<unsigned short>(cfg.MaxVissimPed),
        static_cast<unsigned short>(cfg.MaxVissimSigGrp));
    if (!connected) {
        std::wstring err = proxy.lastError();
        fwprintf(stderr, L"ERROR: VISSIM_Connect failed: %ls\n",
                 err.empty() ? L"(no detail)" : err.c_str());
        return 4;
    }
    printf("VISSIM_Connect OK\n");

    // Stage A loops until SimulationEndTime elapses in simulator time, with
    // a hard cap of (end_time * frequency) ticks. Empty ego array each tick:
    // CarMaker / dyno injection arrives in Stage B.
    const double endTime = config.SimulationSetup.SimulationEndTime;
    const int totalTicks = static_cast<int>(endTime * cfg.SimulatorFrequency);
    printf("Running %d ticks (end_time=%.1fs at %d Hz)\n",
           totalTicks, endTime, cfg.SimulatorFrequency);

    const std::vector<Simulator_Veh_Data> noEgos;
    int lastReportedTick = -25;
    int rc = 0;

    for (int tick = 0; tick < totalTicks; ++tick) {
        if (!proxy.setDriverVehicles(noEgos)) {
            std::wstring err = proxy.lastError();
            fwprintf(stderr, L"ERROR tick %d: SetDriverVehicles failed: %ls\n",
                     tick, err.c_str());
            rc = 5;
            break;
        }

        const auto vehicles = proxy.getTrafficVehicles();
        const auto signals  = proxy.getSignalStates();

        if (tick - lastReportedTick >= 25) {
            printf("tick %5d: vehicles=%3zu signals=%3zu\n",
                   tick, vehicles.size(), signals.size());
            lastReportedTick = tick;
        }
    }

    printf("calling VISSIM_Disconnect ...\n");
    proxy.disconnect();
    printf("=== DSProxy mode done (rc=%d) ===\n", rc);
    return rc;
}

} // namespace DSProxy
} // namespace FIXS
