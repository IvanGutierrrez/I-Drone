/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_PLD.h                 
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "generated_proto/messages_pld.pb.h"
#include "structs/Structs_Algo.h"
#include <string>
namespace Enc_Dec_PLD {

    enum class PLD {
        UNKNOWN,
        ERROR,
        ALGO_RESPONSE,
        STATUS_ALGO,
        STATUS_DRONE
    };

    std::pair<PLD, std::unique_ptr<google::protobuf::Message>> decode_from_algo(const std::string& data);
    std::pair<PLD, std::unique_ptr<google::protobuf::Message>> decode_from_drone(const std::string& data);

    bool encode_algo_response(const std::vector<std::vector<Struct_Algo::Coordinate>> &result, std::string &msg);
    bool decode_algo_response(const AlgoResponseList &msg, std::vector<std::vector<Struct_Algo::Coordinate>> &result);
    
    bool encode_status_algo(const Struct_Algo::Status &status, std::string &message);
};