/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_Drone.h                 
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "generated_proto/messages_drone.pb.h"
#include "structs/Structs_Drone.h"
#include "structs/Structs_Planner.h"
#include <string>

namespace Enc_Dec_Drone {

    enum class Drone {
        UNKNOWN,
        ERROR,
        COMMAND
    };

    std::pair<Drone, std::unique_ptr<google::protobuf::Message>> decode_to_drone(const std::string& data);
    
    bool decode_PX4_command(const DroneCommandString &msg, Struct_Drone::MessagePX4 &command);

    bool encode_status_drone(const Struct_Drone::Status &status, std::string &message);

    bool create_message_to_drone(const std::string &drone_sim, const Struct_Planner::Coordinate &coord_point, const std::string &type, std::string &msg);
};