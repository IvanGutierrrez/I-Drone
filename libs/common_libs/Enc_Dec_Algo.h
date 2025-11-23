/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_Algo.h                 
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "generated_proto/messages_algo.pb.h"
#include "structs/Structs_Algo.h"
#include <string>

namespace Enc_Dec_Algo {

    enum class Algo {
        UNKNOWN,
        ERROR,
        ConfigMessage,
    };
    
    std::pair<Algo, std::unique_ptr<google::protobuf::Message>> decode_to_algo(const std::string& data);
    
    bool encode_config_message(const Struct_Algo::SignalServerConfig& msg, const Struct_Algo::DroneData& drone_msg, std::string &data);
    bool decode_signal_server(const SignalServerConfigProto& protoMsg, Struct_Algo::SignalServerConfig &msg);
    bool decode_drone_data(const DroneData& protoMsg, Struct_Algo::DroneData &msg);
};