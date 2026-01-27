/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : PX4_Wrapper.cpp                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "PX4_Wrapper.h"
#include "common_libs/Logger.h"
#include "common_libs/Enc_Dec_Drone.h"
#include <chrono>
#include <thread>
#include <sstream>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

constexpr double autopilot_timeout_s = 220.0;

PX4_Wrapper::PX4_Wrapper(const Struct_Drone::Drone_Config &drone_config, const Struct_Drone::Config_struct &common_config) : Engine(drone_config, common_config),
    mavsdk_(nullptr), system_(nullptr), action_(nullptr), mission_(nullptr), telemetry_(nullptr)
{
    std::promise<void> ready_promise;
    ready_promise.set_value();
    start_signal_ = ready_promise.get_future().share();
}

Mission::MissionItem PX4_Wrapper::make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    Mission::MissionItem::CameraAction camera_action)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

void PX4_Wrapper::set_handler(Handlers f)
{
    handlers_ = f;
}

void PX4_Wrapper::set_start_signal(std::shared_future<void> start_signal)
{
    std::lock_guard<std::mutex> lock(start_signal_mutex_);
    start_signal_ = std::move(start_signal);
}

void PX4_Wrapper::mark_commands_ready()
{
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        command_upload_ = true;
    }
    cv_mission_complete_.notify_one();
}

void PX4_Wrapper::start_engine()
{
    // Launch PX4 in background (command already has & at the end)
    if (drone_config_.autostart_px4 && !drone_config_.command_px4.empty()) {
        Logger::log_message(Logger::Type::INFO, "Starting PX4 Sim...");
        Logger::log_message(Logger::Type::INFO, "PX4 Command: " + drone_config_.command_px4);
        int ret = std::system(drone_config_.command_px4.c_str());
        Logger::log_message(Logger::Type::INFO, "PX4 command executed with return code: " + std::to_string(ret));
        // Give it a moment to start
        sleep_for(std::chrono::seconds(2));
    } else {
        Logger::log_message(Logger::Type::INFO, "Skipping PX4 autostart (autostart=" + 
                           std::to_string(drone_config_.autostart_px4) + 
                           ", command_empty=" + std::to_string(drone_config_.command_px4.empty()) + ")");
    }
    
    // Create MAVSDK instance as GroundStation
    mavsdk_ = std::make_unique<Mavsdk>(Mavsdk::Configuration{ComponentType::GroundStation});
    
    // Attempt to connect
    Logger::log_message(Logger::Type::INFO, "Connecting to PX4 at " + drone_config_.connection_url);
    ConnectionResult connection_result = mavsdk_->add_any_connection(drone_config_.connection_url);
    
    if (connection_result != ConnectionResult::Success) {
        Logger::log_message(Logger::Type::ERROR, "Connection failed: " + std::to_string(static_cast<int>(connection_result)));
        return;
    }
    
    // Wait for autopilot to appear
    Logger::log_message(Logger::Type::INFO, "Waiting for autopilot...");
    auto system = mavsdk_->first_autopilot(autopilot_timeout_s);
    
    if (!system) {
        Logger::log_message(Logger::Type::ERROR, "Timeout waiting for autopilot system");
        return;
    }
    
    // Store the system and create plugins
    system_ = system.value();
    action_ = std::make_unique<Action>(system_);
    mission_ = std::make_unique<Mission>(system_);
    telemetry_ = std::make_unique<Telemetry>(system_);
    
    // Wait for system to be ready
    Logger::log_message(Logger::Type::INFO, "Waiting for system to be ready...");
    while (!telemetry_->health_all_ok()) {
        sleep_for(seconds(1));
    }
    
    Logger::log_message(Logger::Type::INFO, "PX4 is ready!");
    execute_mission();
}

void PX4_Wrapper::send_command(const std::string &command)
{
    if (command_upload_) return;
    auto [type, decoded_msg] = Enc_Dec_Drone::decode_to_drone(command);

    switch (type) {
        case Enc_Dec_Drone::Drone::UNKNOWN:
            Logger::log_message(Logger::Type::WARNING, "Unknown message received");
            handlers_.error();
            break;
        case Enc_Dec_Drone::Drone::ERROR:
            Logger::log_message(Logger::Type::WARNING, "Error decoding message");
            handlers_.error();
            break;
        case Enc_Dec_Drone::Drone::COMMAND: {
            auto my_msg = dynamic_cast<DroneCommandString*>(decoded_msg.get());
            if (my_msg) {

                Struct_Drone::MessagePX4 mission_cmd;
                if (!Enc_Dec_Drone::decode_command(*my_msg, mission_cmd))
                {
                    Logger::log_message(Logger::Type::WARNING, "Unabled to decode Command message");
                    return;
                }
                Logger::log_message(Logger::Type::INFO, "Processing command: " + mission_cmd.type);
                Logger::log_message(Logger::Type::INFO, "Mission coords: (" + 
                    std::to_string(mission_cmd.mission_item.latitude_deg) + ", " + 
                    std::to_string(mission_cmd.mission_item.longitude_deg) + ")");
                
                {
                    std::lock_guard<std::mutex> lock(mission_mutex_);
                    commands_.push_back(mission_cmd);
                    
                    if (mission_cmd.type == "FINISH") {
                        command_upload_ = true;
                        cv_mission_complete_.notify_one();
                    }
                }
            }
            break;
        }
        default:
            Logger::log_message(Logger::Type::WARNING, "Unhandled message type");
            handlers_.error();
            break;
    }
}


void PX4_Wrapper::execute_mission()
{
    if (!system_ || !mission_ || !action_ || !telemetry_) {
        Logger::log_message(Logger::Type::ERROR, "System not initialized. Call start_engine() first");
        handlers_.error();
        return;
    }

    Logger::log_message(Logger::Type::INFO, "Waiting for last mission command...");
    
    // Wait until last command is received (type == "FINISH")
    while (!command_upload_) {
        std::unique_lock<std::mutex> completion_lock(mission_mutex_);
        cv_mission_complete_.wait(completion_lock, [this] { return command_upload_.load(); });
    }
    
    Logger::log_message(Logger::Type::INFO, "All mission commands received, ready to execute");

    std::vector<Mission::MissionItem> mission_items;

    for (auto each : commands_) {
        
        // Convert Struct_Drone::CameraAction to MAVSDK CameraAction
        Mission::MissionItem::CameraAction camera_action;
        switch (each.mission_item.camera_action) {
            case Struct_Drone::CameraAction::TakePhoto:
                camera_action = Mission::MissionItem::CameraAction::TakePhoto;
                break;
            case Struct_Drone::CameraAction::StartPhotoInterval:
                camera_action = Mission::MissionItem::CameraAction::StartPhotoInterval;
                break;
            case Struct_Drone::CameraAction::StopPhotoInterval:
                camera_action = Mission::MissionItem::CameraAction::StopPhotoInterval;
                break;
            case Struct_Drone::CameraAction::StartVideo:
                camera_action = Mission::MissionItem::CameraAction::StartVideo;
                break;
            case Struct_Drone::CameraAction::StopVideo:
                camera_action = Mission::MissionItem::CameraAction::StopVideo;
                break;
            case Struct_Drone::CameraAction::StartPhotoDistance:
                camera_action = Mission::MissionItem::CameraAction::StartPhotoDistance;
                break;
            case Struct_Drone::CameraAction::StopPhotoDistance:
                camera_action = Mission::MissionItem::CameraAction::StopPhotoDistance;
                break;
            default:
                camera_action = Mission::MissionItem::CameraAction::None;
                break;
        }
        
        mission_items.push_back(make_mission_item(
            each.mission_item.latitude_deg,
            each.mission_item.longitude_deg,
            each.mission_item.relative_altitude_m,
            each.mission_item.speed_m_s,
            each.mission_item.is_fly_through,
            each.mission_item.gimbal_pitch_deg,
            each.mission_item.gimbal_yaw_deg,
            camera_action
        ));
    }

    // Upload mission
    Mission::MissionPlan mission_plan{};
    mission_plan.mission_items = mission_items;
    const Mission::Result upload_result = mission_->upload_mission(mission_plan);
    
    if (upload_result != Mission::Result::Success) {
        Logger::log_message(Logger::Type::ERROR, "Mission upload failed: " + std::to_string(static_cast<int>(upload_result)));
        handlers_.error();
        return;
    }
    
    Logger::log_message(Logger::Type::INFO, "Mission uploaded successfully");

    // Wait for system to be ready (like in the original MAVSDK example)
    Logger::log_message(Logger::Type::INFO, "Waiting for system to be healthy...");
    while (!telemetry_->health_all_ok()) {
        Logger::log_message(Logger::Type::INFO, "Vehicle is getting ready to arm");
        sleep_for(seconds(1));
    }
    Logger::log_message(Logger::Type::INFO, "System is healthy and ready");

    Logger::log_message(Logger::Type::INFO, "Arming...");
    const Action::Result arm_result = action_->arm();
    if (arm_result != Action::Result::Success) {
        std::stringstream log;
        log << "Arming failed: " << arm_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
        handlers_.error();
        return;
    }
    Logger::log_message(Logger::Type::INFO, "Armed.");
    
    std::atomic<bool> want_to_pause{false};
    // Before starting the mission, we want to be sure to subscribe to the mission progress.
    mission_->subscribe_mission_progress([&want_to_pause](Mission::MissionProgress mission_progress) {
        std::stringstream log;
        log << "Mission status update: " << mission_progress.current << " / "
                  << mission_progress.total << '\n';

        Logger::log_message(Logger::Type::INFO, log.str());

        if (mission_progress.current >= 2) {
            // We can only set a flag here. If we do more request inside the callback,
            // we risk blocking the system.
            want_to_pause = true;
        }
    });

    {
        std::shared_future<void> start_signal_copy;
        {
            std::lock_guard<std::mutex> lock(start_signal_mutex_);
            start_signal_copy = start_signal_;
        }

        if (start_signal_copy.valid()) {
            Logger::log_message(Logger::Type::INFO, "Waiting for synchronized mission start signal...");
            start_signal_copy.wait();
            Logger::log_message(Logger::Type::INFO, "Start signal received, starting mission...");
        }
    }

    Logger::log_message(Logger::Type::INFO, "Starting mission...");
    Mission::Result start_mission_result = mission_->start_mission();
    if (start_mission_result != Mission::Result::Success) {
        std::stringstream log;
        log << "Starting mission failed: " << start_mission_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
        handlers_.error();
        return;
    }
    
    Logger::log_message(Logger::Type::INFO, "Mission started successfully!");

    while (!want_to_pause) {
        sleep_for(seconds(1));
    }

    Logger::log_message(Logger::Type::INFO, "Pausing mission...");
    const Mission::Result pause_mission_result = mission_->pause_mission();

    if (pause_mission_result != Mission::Result::Success) {
        std::stringstream log;
        log << "Failed to pause mission:" << pause_mission_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
    }
    Logger::log_message(Logger::Type::INFO, "Mission paused.");

    // Pause for 5 seconds.
    sleep_for(seconds(5));

    // Then continue.
    Mission::Result start_mission_again_result = mission_->start_mission();
    if (start_mission_again_result != Mission::Result::Success) {
        std::stringstream log;
        log << "Starting mission again failed: " << start_mission_again_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
        handlers_.error();
        return;
    }
    
    Logger::log_message(Logger::Type::INFO, "Mission started successfully");
    
    // Wait for mission to complete
    while (!mission_->is_mission_finished().second) {
        sleep_for(seconds(1));
    }
    
    Logger::log_message(Logger::Type::INFO, "Mission completed, returning home");

    // We are done, and can do RTL to go home.
    const Action::Result rtl_result = action_->return_to_launch();
    if (rtl_result != Action::Result::Success) {
        std::stringstream log;
        log << "Failed to command RTL: " << rtl_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
        handlers_.error();
        return;
    }

    // We need to wait a bit, otherwise the armed state might not be correct yet.
    sleep_for(seconds(2));

    while (telemetry_->armed()) {
        // Wait until we're done.
        sleep_for(seconds(1));
    }
    handlers_.mission_complete();
}