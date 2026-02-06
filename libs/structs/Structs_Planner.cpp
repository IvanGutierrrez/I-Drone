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

std::ostream& operator<<(std::ostream& os, const SignalServerConfig& c)
{
    os << "SignalServerConfig {\n"
       << "  sdfDirectory        : " << c.sdfDirectory << "\n"
       << "  outputFile          : " << c.outputFile << "\n"
       << "  userTerrainFile     : " << c.userTerrainFile << "\n"
       << "  terrainBackground   : " << c.terrainBackground << "\n\n"

       << "  latitude            : " << c.latitude << "\n"
       << "  longitude           : " << c.longitude << "\n"
       << "  txHeight            : " << c.txHeight << "\n"
       << "  rxHeights           : [";

    for (size_t i = 0; i < c.rxHeights.size(); ++i) {
        os << c.rxHeights[i];
        if (i + 1 < c.rxHeights.size()) os << ", ";
    }

    os << "]\n\n"

       << "  frequencyMHz        : " << c.frequencyMHz << "\n"
       << "  erpWatts            : " << c.erpWatts << "\n"
       << "  rxThreshold         : " << c.rxThreshold << "\n"
       << "  horizontalPol       : " << std::boolalpha << c.horizontalPol << "\n\n"

       << "  groundClutter       : " << c.groundClutter << "\n"
       << "  terrainCode         : " << c.terrainCode << "\n"
       << "  terrainDielectric   : " << c.terrainDielectric << "\n"
       << "  terrainConductivity : " << c.terrainConductivity << "\n"
       << "  climateCode         : " << c.climateCode << "\n\n"

       << "  propagationModel    : " << c.propagationModel << "\n"
       << "  knifeEdgeDiff       : " << std::boolalpha << c.knifeEdgeDiff << "\n"
       << "  win32TileNames      : " << c.win32TileNames << "\n"
       << "  debugMode           : " << c.debugMode << "\n"
       << "  metricUnits         : " << c.metricUnits << "\n"
       << "  plotDbm             : " << c.plotDbm << "\n\n"

       << "  radius              : " << c.radius << "\n"
       << "  resolution          : " << c.resolution << "\n"
       << "}";
    return os;
}

void DroneData::clear()
{
    num_drones = 0;
    pos_targets.clear();
}

void SignalServerConfig::clear()
{
    sdfDirectory.clear();
    outputFile.clear();
    userTerrainFile.clear();
    terrainBackground.clear();

    latitude = 0;
    longitude = 0;
    txHeight = 0;
    rxHeights.clear();

    frequencyMHz = 0;
    erpWatts = 0;
    rxThreshold = 0;
    horizontalPol = 0;

    groundClutter = 0;
    terrainCode = 0;
    terrainDielectric = 0;
    terrainConductivity = 0;
    climateCode = 0;

    propagationModel = 0;
    knifeEdgeDiff = 0;
    win32TileNames = 0;
    debugMode = 0;
    metricUnits = 0;
    plotDbm = 0;

    radius = 0;
    resolution = 0;
}

};