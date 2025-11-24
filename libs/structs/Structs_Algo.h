/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Struct_algo.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <string>
#include <vector>
#include <sstream>
#include <filesystem>

namespace Struct_Algo {

struct Config_struct {
    std::filesystem::path log_path;
    std::string log_name;
    std::string signal_server_path;
    std::string executable_path;
    double threshold;
    int max_neighbor;
    double max_distance_for_neighbor;
    int max_ortools_time;
};

enum class Status {
    EXPECTING_DATA,
    ERROR,
    CALCULATING,
    FINISH
};

inline std::string to_string(Status status) {
    switch (status) {
        case Status::EXPECTING_DATA:
            return "EXPECTING_DATA";
        case Status::ERROR:
            return "ERROR";
        case Status::CALCULATING:
            return "CALCULATING";
        case Status::FINISH:
            return "FINISH";
        default:
            return std::string();
    }
}

struct CoveragePoint {
    double lat;
    double lon;
    double value;   // señal en dBm
    bool hasCoverage;
};

struct Coordinate {
    double lon;
    double lat;

    Coordinate(const double &lon, const double &lat): lon(lon), lat(lat) {}

    Coordinate(const CoveragePoint& cp) : lon(cp.lon), lat(cp.lat) {}
};

struct DroneData {
    int num_drones;
    std::vector<Coordinate> pos_targets;
};

struct SignalServerConfig {
    // --- File and directory paths ---
    std::string sdfDirectory{};       // -d : Directory containing .sdf tiles, mandatory argument
    std::string outputFile{};         // -o : Output filename (required), mandatory argument
    std::string userTerrainFile{};    // -udt : User-defined terrain filename
    std::string terrainBackground{};  // -t : Terrain background image (optional)

    // --- Geographic position and heights ---
    double latitude{};                // -lat : Transmitter latitude (decimal degrees), mandatory argument
    double longitude{};               // -lon : Transmitter longitude (0–360 or -180–180), mandatory argument
    double txHeight{};                // -txh : Transmitter height above ground, mandatory argument
    std::vector<double> rxHeights;    // -rxh : Receiver height(s), can be multiple

    // --- Transmission parameters ---
    double frequencyMHz{};            // -f : Frequency in MHz (20 MHz – 100 GHz), mandatory argument
    double erpWatts{};                // -erp : Effective radiated power (Watts), mandatory argument
    double rxThreshold{};             // -rt : Receiver threshold (dB / dBm / dBuV/m)
    bool horizontalPol{};             // -hp : Horizontal polarization (default is vertical)

    // --- Environment and terrain parameters ---
    double groundClutter{};           // -gc : Ground clutter height (feet/meters)
    int terrainCode{};                // -te : Terrain code (1–6)
    double terrainDielectric{};       // -terdic : Terrain dielectric constant (2–80)
    double terrainConductivity{};     // -tercon : Terrain conductivity (0.01–0.0001)
    int climateCode{};                // -cl : Climate code (1–6)

    // --- Model and computation options ---
    int propagationModel{};           // -pm : Propagation model (1=ITM, 2=LOS, etc.), mandatory argument
    bool knifeEdgeDiff{};             // -ked : Enable knife-edge diffraction
    bool win32TileNames{};            // -wf : Use Win32-style SDF tile names
    bool debugMode{};                 // -dbg : Enable debug mode
    bool metricUnits{};               // -m : Use metric units
    bool plotDbm{};                   // -dbm : Plot received signal power instead of field strength

    // --- Coverage and map parameters ---
    double radius{};                  // -R : Coverage radius (miles/kilometers), mandatory argument
    int resolution{};                 // -res : Pixels per degree (300/600/1200/3600), mandatory argument

    bool toCommand(const std::string& exePath, std::string &cmd_final) const {
        std::ostringstream cmd;
        try {
            cmd << exePath;
            if (!sdfDirectory.empty()) cmd << " -d " << sdfDirectory; //Mandatory argument
            else return false;
            cmd << " -lat " << latitude;
            cmd << " -lon " << longitude;
            cmd << " -txh " << txHeight;
            cmd << " -f " << frequencyMHz;
            cmd << " -erp " << erpWatts;
            if (!rxHeights.empty()) {
                cmd << " -rxh ";
                for (size_t i = 0; i < rxHeights.size(); ++i) {
                    cmd << rxHeights[i];
                    if (i < rxHeights.size() - 1) cmd << ",";
                }
            }
            cmd << " -R " << radius;
            cmd << " -pm " << propagationModel;
            cmd << " -res " << resolution;
            if (rxThreshold != 0.0) cmd << " -rt " << rxThreshold;
            if (horizontalPol) cmd << " -hp";
            if (knifeEdgeDiff) cmd << " -ked";
            if (win32TileNames) cmd << " -wf";
            if (debugMode) cmd << " -dbg";
            if (metricUnits) cmd << " -m";
            if (plotDbm) cmd << " -dbm";
            if (groundClutter != 0.0) cmd << " -gc " << groundClutter;
            if (terrainCode != 0) cmd << " -te " << terrainCode;
            if (terrainDielectric != 0.0) cmd << " -terdic " << terrainDielectric;
            if (terrainConductivity != 0.0) cmd << " -tercon " << terrainConductivity;
            if (climateCode != 0) cmd << " -cl " << climateCode;
            if (!userTerrainFile.empty()) cmd << " -udt " << userTerrainFile;
            if (!terrainBackground.empty()) cmd << " -t " << terrainBackground;
            if (!outputFile.empty()) cmd << " -o " << outputFile; //Mandatory argument
            else return false;
        } catch (...) {
            return false;
        }
        cmd_final = cmd.str();
        return true;
    }
};


};
