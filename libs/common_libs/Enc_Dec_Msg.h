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
#include "generated_proto/test.pb.h"
#include <string>

namespace Enc_Dec {

    enum class Algo {
        UNKNOWN,
        ERROR,
        MyMessage
    };
    
    std::pair<Algo, std::unique_ptr<google::protobuf::Message>> decode_to_algo(const std::string& data);
    
    bool encode_test(const MyMessage& msg, std::string &data);
    bool decode_test(const std::string& data, MyMessage &msg);
};