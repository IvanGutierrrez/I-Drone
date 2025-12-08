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
#include <string>

namespace Enc_Dec_Drone {

    enum class Drone {
        UNKNOWN,
        ERROR,
        COMMAND
    };

    std::pair<Drone, std::unique_ptr<google::protobuf::Message>> decode_to_drone(const std::string& data);

    bool encode_command(const Struct_Drone::MessagePX4 &command, std::string &response);
    
    bool decode_command(const DroneCommandString &msg, Struct_Drone::MessagePX4 &command);
    
    bool encode_status_drone(const Struct_Drone::Status &status, std::string &message);
};