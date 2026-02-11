/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Planner_Recorder.h                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <iostream>
#include <memory>
#include "common_libs/Recorder.h"
#include "structs/Structs_Planner.h"

class Planner_Recorder {

public:
    Planner_Recorder(const std::filesystem::path &path);

    bool write_signal_output(const std::vector<Struct_Planner::Coordinate> &points);
    bool write_message_received(const Struct_Planner::SignalServerConfig &sng_data, const Struct_Planner::DroneData &drone_data);
    bool write_or_output(const std::string &data);
    void close_all();

private:
    std::unique_ptr<Recorder> recorder_msg;
    std::unique_ptr<Recorder> recorder_sgn;
    std::unique_ptr<Recorder> recorder_or;

};