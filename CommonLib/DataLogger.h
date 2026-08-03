#pragma once
//============================================================================
//  DataLogger  (#174 -- FIXS infrastructure data logging)
//----------------------------------------------------------------------------
//  A generic, config-driven CSV time-series logger for the FIXS vehicle-data
//  stream. It is deliberately NOT tied to any scenario, backend, or test: any
//  FIXS component (TrafficLayer, VirCarlaEnv, the CarMaker bridge, ...) can
//  instantiate one and feed it the VehFullData records it exchanges.
//
//  IMPORTANT -- the logged values are in the FIXS WIRE convention, which is the
//  SUMO / VISSIM convention:
//     * position = front-of-vehicle anchor, metres
//     * heading  = navigational degrees, 0 = North, clockwise
//     * speed    = m/s
//  The traffic simulators (SUMO, VISSIM) share this convention; the vehicle-
//  dynamics / virtual-environment backends (Carla, CarMaker) each have their own
//  native frames (Carla Y-flip + bbox-centre pivot + its own yaw zero; CarMaker
//  its own) and diverge more. Those are normalized to the wire convention by the
//  bridge (BridgeHelper) BEFORE the record reaches this logger -- so logs from
//  ANY backend are directly comparable. That is the whole reason to log at the
//  FIXS-message layer rather than at a backend-native transform.
//
//  Analysis/plotting of a logger's output is TEST-SPECIFIC and lives under the
//  relevant tests/ folder (e.g. a probe's plot_*.py), never in this class.
//============================================================================

#include "VehDataMsgDefs.h"

#include <string>
#include <vector>
#include <fstream>

namespace fixs {

class DataLogger {
public:
    // Open a CSV at 'path', logging the given VehFullData field names as columns
    // after (simTime, id). Empty 'fields' -> a sensible default core set. Parent
    // directories are created as needed. Returns false if the file can't be
    // opened (the logger then no-ops, so callers need no extra guard).
    bool open(const std::string& path, const std::vector<std::string>& fields);

    // Append one vehicle record at 'simTime'. The record's values must already be
    // in the FIXS-wire (SUMO/VISSIM) convention documented above.
    void logVehicle(double simTime, const VehFullData_t& v);

    void close();
    bool isOpen() const { return ofs_.is_open(); }
    const std::string& path() const { return path_; }

private:
    std::ofstream            ofs_;
    std::string              path_;
    std::vector<std::string> fields_;

    // Format one VehFullData field by name (numeric -> %.5f, string -> as-is,
    // unknown -> empty). Central place mapping column names to the wire record.
    static std::string cell(const std::string& field, const VehFullData_t& v);
};

} // namespace fixs
