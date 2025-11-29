/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Structs_Drone.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <string>
#include <vector>
#include <sstream>
#include <ostream>
#include <iomanip>
#include <filesystem>

namespace Struct_Drone {

    struct Config_struct {
        std::filesystem::path data_path;
        std::filesystem::path log_path;
        std::string log_name;
    };

    enum class Status {
        STARTING_SIM,
        ERROR,
        EXECUTING_COMMAND,
        WAITING_COMMAND,
        FINISH
    };

    inline std::string to_string(Status status) {
        switch (status) {
            case Status::STARTING_SIM:
                return "STARTING_SIM";
            case Status::ERROR:
                return "ERROR";
            case Status::EXECUTING_COMMAND:
                return "EXECUTING_COMMAND";
            case Status::WAITING_COMMAND:
                return "WAITING_COMMAND";
            case Status::FINISH:
                return "FINISH";
            default:
                return std::string();
        }
    }
};