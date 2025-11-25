/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Algorithm_Recorder.h                  
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
#include "structs/Structs_Algo.h"

class Algorithm_Recorder {

public:
    Algorithm_Recorder(const std::filesystem::path &path);

    bool write_signal_output(const std::vector<Struct_Algo::Coordinate> &points);
    bool write_message_received(const Struct_Algo::SignalServerConfig &sng_data, const Struct_Algo::DroneData &drone_data);
    bool write_or_output(const std::string &data);

private:
    Recorder recorder_msg;
    Recorder recorder_sgn;
    Recorder recorder_or;

};