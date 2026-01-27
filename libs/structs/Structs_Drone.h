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

    struct Drone_Config {
        int drone_id = 0;
        std::string connection_url;
        std::string command_px4;
        bool autostart_px4 = true;
        double home_lat = 0.0;
        double home_lon = 0.0;
        double home_alt = 0.0;
    };

    struct Config_struct {
        std::filesystem::path data_path;
        std::filesystem::path log_path;
        std::string log_name;
        int num_drones = 1;
        std::vector<Drone_Config> drones;
    };

    enum class CameraAction {
        None,
        TakePhoto,
        StartPhotoInterval,
        StopPhotoInterval,
        StartVideo,
        StopVideo,
        StartPhotoDistance,
        StopPhotoDistance
    };

    struct MissionItem {
        double latitude_deg;
        double longitude_deg;
        float relative_altitude_m;
        float speed_m_s;
        bool is_fly_through;
        float gimbal_pitch_deg;
        float gimbal_yaw_deg;
        CameraAction camera_action;
    };

    struct MessagePX4 {
        std::string type;
        MissionItem mission_item;
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