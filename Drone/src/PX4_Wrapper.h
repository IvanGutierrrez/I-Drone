/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : PX4_Wrapper.h                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef PX4_ENGINE_H
#define PX4_ENGINE_H

#pragma once
#include "Engine.h"
#include "structs/Structs_Drone.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <memory>
#include <optional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>


// Interface
class PX4_Wrapper: public Engine {

private:
    std::vector<Struct_Drone::MessagePX4> commands_;
    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Mission> mission_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    
    std::mutex mission_mutex_;
    std::condition_variable cv_mission_complete_;
    std::atomic<bool> command_upload_{false};

    mavsdk::Mission::MissionItem make_mission_item(
        double latitude_deg,
        double longitude_deg,
        float relative_altitude_m,
        float speed_m_s,
        bool is_fly_through,
        float gimbal_pitch_deg,
        float gimbal_yaw_deg,
        mavsdk::Mission::MissionItem::CameraAction camera_action);
        
    void execute_mission();

public:
    PX4_Wrapper(const Struct_Drone::Config_struct &config);
    ~PX4_Wrapper() override = default;
    void start_engine() override;
    void send_command(const std::string &command) override;
    void set_handler(Handlers f) override;
};

#endif
