/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_Planner.h                 
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "generated_proto/messages_planner.pb.h"
#include "structs/Structs_Planner.h"
#include <string>

namespace Enc_Dec_Planner {

    enum class Planner {
        UNKNOWN,
        ERROR,
        ConfigMessage,
    };
    
    std::pair<Planner, std::unique_ptr<google::protobuf::Message>> decode_to_planner(const std::string& data);
    
    bool encode_config_message(const Struct_Planner::SignalServerConfig& msg, const Struct_Planner::DroneData& drone_msg, std::string &data);
    bool decode_signal_server(const SignalServerConfigProto& protoMsg, Struct_Planner::SignalServerConfig &msg);
    bool decode_drone_data(const DroneData& protoMsg, Struct_Planner::DroneData &msg);
};