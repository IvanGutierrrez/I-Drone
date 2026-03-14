/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Struct_Planner.cpp                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Structs_Planner.h"
#include <string>
#include <vector>
#include <sstream>
#include <filesystem>

namespace Struct_Planner {

std::ostream& operator<<(std::ostream& os, const DroneData& d)
{
    os << "DroneData {\n"
       << "  num_drones : " << d.num_drones << "\n"
       << "  pos_targets: [";

    for (size_t i = 0; i < d.pos_targets.size(); ++i) {
        os << d.pos_targets[i];
        if (i + 1 < d.pos_targets.size()) os << ", ";
    }

    os << "]\n"
       << "}";

    return os;
}

bool SignalServerConfig::toCommand(const std::string& exePath, std::string &cmd_final) const 
{
    std::ostringstream cmd;
    try {
        cmd << exePath;
        if (!filePaths.sdfDirectory.empty()) cmd << " -d " << filePaths.sdfDirectory; //Mandatory argument
        else return false;
        cmd << " -lat " << position.latitude;
        cmd << " -lon " << position.longitude;
        cmd << " -txh " << position.txHeight;
        cmd << " -f " << transmission.frequencyMHz;
        cmd << " -erp " << transmission.erpWatts;
        if (!position.rxHeights.empty()) {
            cmd << " -rxh ";
            for (size_t i = 0; i < position.rxHeights.size(); ++i) {
                cmd << position.rxHeights[i];
                if (i < position.rxHeights.size() - 1) cmd << ",";
            }
        }
        cmd << " -R " << coverage.radius;
        cmd << " -pm " << options.propagationModel;
        cmd << " -res " << coverage.resolution;
        if (transmission.rxThreshold != 0.0) cmd << " -rt " << transmission.rxThreshold;
        if (transmission.horizontalPol) cmd << " -hp";
        if (options.knifeEdgeDiff) cmd << " -ked";
        if (options.win32TileNames) cmd << " -wf";
        if (options.debugMode) cmd << " -dbg";
        if (options.metricUnits) cmd << " -m";
        if (options.plotDbm) cmd << " -dbm";
        if (environment.groundClutter != 0.0) cmd << " -gc " << environment.groundClutter;
        if (environment.terrainCode != 0) cmd << " -te " << environment.terrainCode;
        if (environment.terrainDielectric != 0.0) cmd << " -terdic " << environment.terrainDielectric;
        if (environment.terrainConductivity != 0.0) cmd << " -tercon " << environment.terrainConductivity;
        if (environment.climateCode != 0) cmd << " -cl " << environment.climateCode;
        if (!filePaths.userTerrainFile.empty()) cmd << " -udt " << filePaths.userTerrainFile;
        if (!filePaths.terrainBackground.empty()) cmd << " -t " << filePaths.terrainBackground;
        if (!filePaths.outputFile.empty()) cmd << " -o " << filePaths.outputFile; //Mandatory argument
        else return false;
    } catch (...) {
        return false;
    }
    cmd_final = cmd.str();
    return true;
}

std::string print(const SignalServerConfig& c)
{
    std::stringstream ss;

    ss << "SignalServerConfig {\n"
       << "  sdfDirectory        : " << c.filePaths.sdfDirectory << "\n"
       << "  outputFile          : " << c.filePaths.outputFile << "\n"
       << "  userTerrainFile     : " << c.filePaths.userTerrainFile << "\n"
       << "  terrainBackground   : " << c.filePaths.terrainBackground << "\n\n"

       << "  latitude            : " << c.position.latitude << "\n"
       << "  longitude           : " << c.position.longitude << "\n"
       << "  txHeight            : " << c.position.txHeight << "\n"
       << "  rxHeights           : [";

    for (size_t i = 0; i < c.position.rxHeights.size(); ++i) {
        ss << c.position.rxHeights[i];
        if (i + 1 < c.position.rxHeights.size()) ss << ", ";
    }

    ss << "]\n\n"

       << "  frequencyMHz        : " << c.transmission.frequencyMHz << "\n"
       << "  erpWatts            : " << c.transmission.erpWatts << "\n"
       << "  rxThreshold         : " << c.transmission.rxThreshold << "\n"
       << "  horizontalPol       : " << std::boolalpha << c.transmission.horizontalPol << "\n\n"

       << "  groundClutter       : " << c.environment.groundClutter << "\n"
       << "  terrainCode         : " << c.environment.terrainCode << "\n"
       << "  terrainDielectric   : " << c.environment.terrainDielectric << "\n"
       << "  terrainConductivity : " << c.environment.terrainConductivity << "\n"
       << "  climateCode         : " << c.environment.climateCode << "\n\n"

       << "  propagationModel    : " << c.options.propagationModel << "\n"
       << "  knifeEdgeDiff       : " << std::boolalpha << c.options.knifeEdgeDiff << "\n"
       << "  win32TileNames      : " << c.options.win32TileNames << "\n"
       << "  debugMode           : " << c.options.debugMode << "\n"
       << "  metricUnits         : " << c.options.metricUnits << "\n"
       << "  plotDbm             : " << c.options.plotDbm << "\n\n"

       << "  radius              : " << c.coverage.radius << "\n"
       << "  resolution          : " << c.coverage.resolution << "\n"
       << "}";

    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const SignalServerConfig& c)
{
    os << print(c);
    return os;
}

void DroneData::clear()
{
    num_drones = 0;
    pos_targets.clear();
}

void SignalServerConfig::clear()
{
    filePaths.sdfDirectory.clear();
    filePaths.outputFile.clear();
    filePaths.userTerrainFile.clear();
    filePaths.terrainBackground.clear();

    position.latitude = 0;
    position.longitude = 0;
    position.txHeight = 0;
    position.rxHeights.clear();

    transmission.frequencyMHz = 0;
    transmission.erpWatts = 0;
    transmission.rxThreshold = 0;
    transmission.horizontalPol = false;

    environment.groundClutter = 0;
    environment.terrainCode = 0;
    environment.terrainDielectric = 0;
    environment.terrainConductivity = 0;
    environment.climateCode = 0;

    options.propagationModel = 0;
    options.knifeEdgeDiff = false;
    options.win32TileNames = false;
    options.debugMode = false;
    options.metricUnits = false;
    options.plotDbm = false;

    coverage.radius = 0;
    coverage.resolution = 0;
}

};