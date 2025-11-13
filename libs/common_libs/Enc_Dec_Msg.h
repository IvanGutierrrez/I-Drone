/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_Msg.h                 
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

namespace Enc_Dec {

    enum class Algo {
        UNKNOWN,
        ERROR,
        SignalServerConfig,
        Status
    };
    
    std::pair<Algo, std::unique_ptr<google::protobuf::Message>> decode_to_algo(const std::string& data);
    
    bool encode_signal_server(const Struct_Algo::SignalServerConfig& msg, std::string &data);
    bool decode_signal_server(const SignalServerConfigProto& protoMsg, Struct_Algo::SignalServerConfig &msg);
    bool encode_status_algo(const Struct_Algo::Status &status, std::string &message);
};