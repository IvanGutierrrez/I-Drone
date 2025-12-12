/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Signal_Cal.h                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "structs/Structs_Planner.h"
#include <iostream>
#include <memory>
#include <map>

struct RGB {
    unsigned char r, g, b;
    bool operator<(const RGB& other) const {
        return std::tie(r,g,b) < std::tie(other.r,other.g,other.b);
    }
    bool operator==(const RGB& other) const {
        return r==other.r && g==other.g && b==other.b;
    }
};

using CoverageMatrix = std::vector<std::vector<double>>;


class Signal_Cal {

public:
    Signal_Cal();
    std::vector<Struct_Planner::Coordinate> calculate_signal(const Struct_Planner::Config_struct &global_config,const Struct_Planner::SignalServerConfig &signal_server_conf);

private:
    CoverageMatrix read_Coverage_File(const std::string& filename, const std::string& dcfFilename);
    std::vector<double> parse_Bounds(const std::string& str);
    std::map<RGB, double> read_DCF(const std::string& dcf_filename);
};