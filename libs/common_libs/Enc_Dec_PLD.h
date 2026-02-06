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
#include "structs/Structs_Planner.h"
#include "structs/Structs_PLD.h"
#include <string>
namespace Enc_Dec_PLD {

    enum class PLD {
        UNKNOWN,
        ERROR,
        Planner_RESPONSE,
        STATUS_Planner,
        STATUS_DRONE,
        CONFIG_MISSION
    };

    std::pair<PLD, std::unique_ptr<google::protobuf::Message>> decode_from_planner(const std::string& data);
    std::pair<PLD, std::unique_ptr<google::protobuf::Message>> decode_from_drone(const std::string& data);
    std::pair<PLD, std::unique_ptr<google::protobuf::Message>> decode_from_client(const std::string& data);

    bool encode_planner_response(const std::vector<std::vector<Struct_Planner::Coordinate>> &result, std::string &msg);
    bool decode_planner_response(const PlannerResponseList &msg, std::vector<std::vector<Struct_Planner::Coordinate>> &result);
    
    bool encode_status_planner(const Struct_Planner::Status &status, std::string &message);
    bool encode_status_pld(const Structs_PLD::Status &status, std::string &message);

    bool encode_config_mission(const Structs_PLD::Config_mission &config, std::string &message);
    bool decode_config_mission(const Config_mission &proto, Structs_PLD::Config_mission &config);
};