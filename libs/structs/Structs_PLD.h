/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Structs_PLD.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <string>
#include <filesystem>
#include "Structs_Planner.h"

namespace Structs_PLD {

    struct Config_struct {
        std::filesystem::path data_path;
        std::filesystem::path log_path;
        std::string log_name;
    };

    enum class Status {
        UNKNOWN,
        ERROR,
        WAITING_INFO,
        PLANNING_MISSION,
        EXECUTING_MISSION,
        FINISH
    };

    inline std::string to_string(Status status) {
        switch (status) {
            case Status::UNKNOWN:
                return "UNKNOWN";
            case Status::ERROR:
                return "ERROR";
            case Status::WAITING_INFO:
                return "WAITING_INFO";
            case Status::PLANNING_MISSION:
                return "PLANNING_MISSION";
            case Status::EXECUTING_MISSION:
                return "EXECUTING_MISSION";
            case Status::FINISH:
                return "FINISH";
            default:
                return std::string();
        }
    }

    struct Info_Module {
        std::string docker_name;
        std::string docker_file;
        std::string module_ip;
        std::string ssh_ip;
        std::string port;
        std::string user;
        std::string key;

        void clear(){
            docker_name.clear();
            docker_file.clear();
            module_ip.clear();
            ssh_ip.clear();
            port.clear();
            user.clear();
            key.clear();
        }
    };

    struct Config_mission {
        Struct_Planner::Planner_info planner_info;
        Info_Module planner_module_data;
        Info_Module drone_module_data;
        std::string drone_sim;

        void clear(){
            planner_info.clear();
            planner_module_data.clear();
            drone_module_data.clear();
            drone_sim = std::string();
        }
    };

    struct Config_drone {
        std::vector<std::vector<Struct_Planner::Coordinate>> coor_points;
        Info_Module drone_module_data;
        std::string drone_sim;

        void clear(){
            coor_points.clear();
            drone_module_data.clear();
            drone_sim = std::string();
        }
    };
}