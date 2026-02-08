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
#include <fstream>
#include <signal.h>
#include <sys/wait.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

constexpr double autopilot_timeout_s = 220.0;

static double get_current_timestamp() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

PX4_Wrapper::PX4_Wrapper(const Struct_Drone::Drone_Config &drone_config, const Struct_Drone::Config_struct &common_config, std::shared_ptr<Drone_Recorder> recorder) 
    : Engine(drone_config, common_config),
    mavsdk_(nullptr), system_(nullptr), action_(nullptr), mission_(nullptr), telemetry_(nullptr), recorder_(recorder)
{
    std::promise<void> ready_promise;
    ready_promise.set_value();
    start_signal_ = ready_promise.get_future().share();
}

PX4_Wrapper::~PX4_Wrapper()
{
    cleanup_px4_process();
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

void PX4_Wrapper::flush_recorder()
{
    if (recorder_) {
        recorder_->flush();
    }
}

void PX4_Wrapper::start_engine()
{
    // Launch PX4 in background (command already has & at the end)
    if (drone_config_.autostart_px4 && !drone_config_.command_px4.empty()) {
        Logger::log_message(Logger::Type::INFO, "Starting PX4 Sim...");
        Logger::log_message(Logger::Type::INFO, "PX4 Command: " + drone_config_.command_px4);
        
        // Fork to capture child PID
        pid_t pid = fork();
        if (pid == 0) {
            // Child process - execute the command
            execl("/bin/sh", "sh", "-c", drone_config_.command_px4.c_str(), (char*)NULL);
            // If execl returns, it failed
            exit(1);
        } else if (pid > 0) {
            // Parent process - store the PID
            px4_pid_ = pid;
            Logger::log_message(Logger::Type::INFO, "PX4 process started with PID: " + std::to_string(px4_pid_));
            
            // Register in simulation processes file for cleanup
            std::ofstream pid_file("/tmp/simulation_processes.pid", std::ios::app);
            if (pid_file.is_open()) {
                pid_file << "px4_drone_" << drone_config_.drone_id << ":" << px4_pid_ << "\n";
                pid_file.close();
            }
            
            // Give it a moment to start
            sleep_for(std::chrono::seconds(2));
        } else {
            Logger::log_message(Logger::Type::ERROR, "Failed to fork PX4 process for drone " + std::to_string(drone_config_.drone_id));
            if (handlers_.error) {
                handlers_.error(drone_config_.drone_id);
            }
            return;
        }
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
        if (handlers_.error) {
            handlers_.error(drone_config_.drone_id);
        }
        return;
    }
    
    // Wait for autopilot to appear
    Logger::log_message(Logger::Type::INFO, "Waiting for autopilot...");
    auto system = mavsdk_->first_autopilot(autopilot_timeout_s);
    
    if (!system) {
        Logger::log_message(Logger::Type::ERROR, "Timeout waiting for autopilot system");
        if (handlers_.error) {
            handlers_.error(drone_config_.drone_id);
        }
        return;
    }
    
    // Store the system and create plugins
    system_ = system.value();
    action_ = std::make_unique<Action>(system_);
    mission_ = std::make_unique<Mission>(system_);
    telemetry_ = std::make_unique<Telemetry>(system_);
    
    // Subscribe to telemetry updates for recording
    if (recorder_) {
        telemetry_->subscribe_position_velocity_ned([this](Telemetry::PositionVelocityNed pvn) {
            static auto last_log = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            // Log at 1Hz
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 1) {
                auto pos = telemetry_->position();
                auto vel_ned = telemetry_->velocity_ned();
                auto attitude = telemetry_->attitude_euler();
                auto battery = telemetry_->battery();
                auto armed = telemetry_->armed();
                auto in_air = telemetry_->in_air();
                auto flight_mode = telemetry_->flight_mode();
                
                Drone_Recorder::Telemetry_Record rec{
                    get_current_timestamp(),
                    drone_config_.drone_id,
                    pos.latitude_deg,
                    pos.longitude_deg,
                    pos.absolute_altitude_m,
                    pos.relative_altitude_m,
                    vel_ned.north_m_s,
                    vel_ned.east_m_s,
                    vel_ned.down_m_s,
                    attitude.roll_deg,
                    attitude.pitch_deg,
                    attitude.yaw_deg,
                    armed,
                    in_air,
                    std::to_string(static_cast<int>(flight_mode)),
                    battery.voltage_v,
                    battery.remaining_percent
                };
                recorder_->log_telemetry(rec);
                last_log = now;
            }
        });
    }
    
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
            if (handlers_.error) {
                handlers_.error(drone_config_.drone_id);
            }
            break;
        case Enc_Dec_Drone::Drone::ERROR:
            Logger::log_message(Logger::Type::WARNING, "Error decoding message");
            if (handlers_.error) {
                handlers_.error(drone_config_.drone_id);
            }
            break;
        case Enc_Dec_Drone::Drone::COMMAND: {
            auto my_msg = dynamic_cast<DroneCommandString*>(decoded_msg.get());
            if (my_msg) {

                Struct_Drone::MessagePX4 mission_cmd;
                if (!Enc_Dec_Drone::decode_PX4_command(*my_msg, mission_cmd))
                {
                    Logger::log_message(Logger::Type::WARNING, "Unabled to decode Command message");
                    return;
                }
                Logger::log_message(Logger::Type::INFO, "Processing command: " + mission_cmd.type);
                Logger::log_message(Logger::Type::INFO, "Mission coords: (" + 
                    std::to_string(mission_cmd.mission_item.latitude_deg) + ", " + 
                    std::to_string(mission_cmd.mission_item.longitude_deg) + ")");
                
                if (recorder_) {
                    Drone_Recorder::Command_Log cmd_log{
                        get_current_timestamp(),
                        drone_config_.drone_id,
                        "COMMAND_RECEIVED",
                        "SUCCESS",
                        "Type: " + mission_cmd.type + " at (" + 
                        std::to_string(mission_cmd.mission_item.latitude_deg) + ", " +
                        std::to_string(mission_cmd.mission_item.longitude_deg) + ")"
                    };
                    recorder_->log_command(cmd_log);
                }
                
                {
                    std::lock_guard<std::mutex> lock(mission_mutex_);
                    commands_.push_back(mission_cmd);
                }
            }
            break;
        }
        default:
            Logger::log_message(Logger::Type::WARNING, "Unhandled message type");
            if (handlers_.error) {
                handlers_.error(drone_config_.drone_id);
            }
            break;
    }
}


Mission::MissionItem::CameraAction PX4_Wrapper::convert_camera_action(Struct_Drone::CameraAction action) {
    switch (action) {
        case Struct_Drone::CameraAction::TakePhoto:
            return Mission::MissionItem::CameraAction::TakePhoto;
        case Struct_Drone::CameraAction::StartPhotoInterval:
            return Mission::MissionItem::CameraAction::StartPhotoInterval;
        case Struct_Drone::CameraAction::StopPhotoInterval:
            return Mission::MissionItem::CameraAction::StopPhotoInterval;
        case Struct_Drone::CameraAction::StartVideo:
            return Mission::MissionItem::CameraAction::StartVideo;
        case Struct_Drone::CameraAction::StopVideo:
            return Mission::MissionItem::CameraAction::StopVideo;
        case Struct_Drone::CameraAction::StartPhotoDistance:
            return Mission::MissionItem::CameraAction::StartPhotoDistance;
        case Struct_Drone::CameraAction::StopPhotoDistance:
            return Mission::MissionItem::CameraAction::StopPhotoDistance;
        default:
            return Mission::MissionItem::CameraAction::None;
    }
}

std::vector<Mission::MissionItem> PX4_Wrapper::build_mission_items() {
    Logger::log_message(Logger::Type::INFO, "Building mission from " + std::to_string(commands_.size()) + " received commands");
    
    std::vector<Mission::MissionItem> mission_items;

    for (const auto& cmd : commands_) {
        Logger::log_message(Logger::Type::INFO, "Adding waypoint [" + cmd.type + "] (" + 
                           std::to_string(cmd.mission_item.latitude_deg) + ", " +
                           std::to_string(cmd.mission_item.longitude_deg) + ") alt: " +
                           std::to_string(cmd.mission_item.relative_altitude_m) + "m");
        
        mission_items.push_back(make_mission_item(
            cmd.mission_item.latitude_deg,
            cmd.mission_item.longitude_deg,
            cmd.mission_item.relative_altitude_m,
            cmd.mission_item.speed_m_s,
            cmd.mission_item.is_fly_through,
            cmd.mission_item.gimbal_pitch_deg,
            cmd.mission_item.gimbal_yaw_deg,
            convert_camera_action(cmd.mission_item.camera_action)
        ));
    }
    
    Logger::log_message(Logger::Type::INFO, "Mission contains " + std::to_string(mission_items.size()) + " waypoints");
    return mission_items;
}

bool PX4_Wrapper::upload_mission_plan(const std::vector<Mission::MissionItem>& mission_items) {

    Mission::MissionPlan mission_plan{};
    mission_plan.mission_items = mission_items;
    const Mission::Result upload_result = mission_->upload_mission(mission_plan);
    
    if (upload_result != Mission::Result::Success) {
        Logger::log_message(Logger::Type::ERROR, "Mission upload failed: " + std::to_string(static_cast<int>(upload_result)));
        if (handlers_.error) {
            handlers_.error(drone_config_.drone_id);
        }
        return false;
    }
    
    Logger::log_message(Logger::Type::INFO, "Mission uploaded successfully");
    return true;
}

bool PX4_Wrapper::wait_system_healthy() {

    Logger::log_message(Logger::Type::INFO, "Waiting for system to be healthy...");
    while (!telemetry_->health_all_ok()) {
        Logger::log_message(Logger::Type::INFO, "Vehicle is getting ready to arm");
        sleep_for(seconds(1));
    }
    Logger::log_message(Logger::Type::INFO, "System is healthy and ready");
    return true;
}

bool PX4_Wrapper::arm_vehicle() {

    Logger::log_message(Logger::Type::INFO, "Arming...");
    const Action::Result arm_result = action_->arm();
    if (arm_result != Action::Result::Success) {
        std::stringstream log;
        log << "Arming failed: " << arm_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
        if (handlers_.error) {
            handlers_.error(drone_config_.drone_id);
        }
        return false;
    }
    Logger::log_message(Logger::Type::INFO, "Armed.");
    
    Logger::log_message(Logger::Type::INFO, "Setting takeoff altitude to " + 
                       std::to_string(drone_config_.takeoff_altitude_m) + "m");
    const Action::Result set_takeoff_result = action_->set_takeoff_altitude(drone_config_.takeoff_altitude_m);
    if (set_takeoff_result != Action::Result::Success) {
        std::stringstream log;
        log << "Set takeoff altitude failed: " << set_takeoff_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
    }
    return true;
}

void PX4_Wrapper::wait_for_start_signal() {
    
    std::shared_future<void> start_signal_copy;
    {
        std::lock_guard<std::mutex> lock(start_signal_mutex_);
        start_signal_copy = start_signal_;
    }

    if (start_signal_copy.valid()) {
        Logger::log_message(Logger::Type::INFO, "Waiting for synchronized mission start signal...");
        start_signal_copy.wait();
        Logger::log_message(Logger::Type::INFO, "Start signal received, taking off...");
    }
}

bool PX4_Wrapper::perform_takeoff() {

    Logger::log_message(Logger::Type::INFO, "Taking off...");
    const Action::Result takeoff_result = action_->takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::stringstream log;
        log << "Takeoff failed: " << takeoff_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
        if (handlers_.error) {
            handlers_.error(drone_config_.drone_id);
        }
        return false;
    }
    Logger::log_message(Logger::Type::INFO, "Takeoff command sent, waiting to reach altitude...");
    
    float current_altitude = 0.0f;
    while (current_altitude < (drone_config_.takeoff_altitude_m - 0.5f)) {
        current_altitude = telemetry_->position().relative_altitude_m;
        sleep_for(std::chrono::milliseconds(100));
    }
    Logger::log_message(Logger::Type::INFO, "Reached takeoff altitude (" + 
                       std::to_string(current_altitude) + "m), starting mission...");
    
    // Log takeoff event
    if (recorder_) {
        auto pos = telemetry_->position();
        Drone_Recorder::Mission_Event event{
            get_current_timestamp(),
            drone_config_.drone_id,
            "TAKEOFF",
            -1,
            pos.latitude_deg,
            pos.longitude_deg
        };
        recorder_->log_mission_event(event);
    }
    
    return true;
}

bool PX4_Wrapper::start_mission_execution() {
    Mission::Result start_mission_result = mission_->start_mission();
    if (start_mission_result != Mission::Result::Success) {
        std::stringstream log;
        log << "Starting mission failed: " << start_mission_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
        if (handlers_.error) {
            handlers_.error(drone_config_.drone_id);
        }
        return false;
    }
    
    Logger::log_message(Logger::Type::INFO, "Mission started successfully!");
    
    // Log mission start event
    if (recorder_) {
        auto pos = telemetry_->position();
        Drone_Recorder::Mission_Event event{
            get_current_timestamp(),
            drone_config_.drone_id,
            "MISSION_START",
            -1,
            pos.latitude_deg,
            pos.longitude_deg
        };
        recorder_->log_mission_event(event);
    }
    
    return true;
}

void PX4_Wrapper::wait_mission_completion() {
    while (!mission_->is_mission_finished().second) {
        sleep_for(seconds(1));
    }
    Logger::log_message(Logger::Type::INFO, "Mission completed, returning home");
    
    // Log mission complete event
    if (recorder_) {
        auto pos = telemetry_->position();
        Drone_Recorder::Mission_Event event{
            get_current_timestamp(),
            drone_config_.drone_id,
            "MISSION_COMPLETE",
            -1,
            pos.latitude_deg,
            pos.longitude_deg
        };
        recorder_->log_mission_event(event);
    }
}

bool PX4_Wrapper::return_to_launch() {

    const Action::Result rtl_result = action_->return_to_launch();
    if (rtl_result != Action::Result::Success) {
        std::stringstream log;
        log << "Failed to command RTL: " << rtl_result;
        Logger::log_message(Logger::Type::ERROR, log.str());
        if (handlers_.error) {
            handlers_.error(drone_config_.drone_id);
        }
        return false;
    }

    // Log RTL start event
    if (recorder_) {
        auto pos = telemetry_->position();
        Drone_Recorder::Mission_Event event{
            get_current_timestamp(),
            drone_config_.drone_id,
            "RTL_START",
            -1,
            pos.latitude_deg,
            pos.longitude_deg
        };
        recorder_->log_mission_event(event);
    }

    sleep_for(seconds(2));
    while (telemetry_->armed()) {
        sleep_for(seconds(1));
    }
    return true;
}

void PX4_Wrapper::execute_mission()
{
    if (!system_ || !mission_ || !action_ || !telemetry_) {
        Logger::log_message(Logger::Type::ERROR, "System not initialized. Call start_engine() first");
        if (handlers_.error) {
            handlers_.error(drone_config_.drone_id);
        }
        return;
    }

    Logger::log_message(Logger::Type::INFO, "Waiting for last mission command...");
    while (!command_upload_) {
        std::unique_lock<std::mutex> completion_lock(mission_mutex_);
        cv_mission_complete_.wait(completion_lock, [this] { return command_upload_.load(); });
    }
    
    Logger::log_message(Logger::Type::INFO, "All mission commands received, ready to execute");

    auto mission_items = build_mission_items();
    
    // Check if mission is trivial (same start/end point with no intermediate waypoints)
    if (mission_items.size() == 2 && 
        std::abs(mission_items[0].latitude_deg - mission_items[1].latitude_deg) < 0.0001 &&
        std::abs(mission_items[0].longitude_deg - mission_items[1].longitude_deg) < 0.0001) {
        Logger::log_message(Logger::Type::WARNING, "Mission has only 2 identical waypoints, skipping execution and marking as complete");
        
        if (recorder_) {
            Drone_Recorder::Command_Log cmd_log{
                get_current_timestamp(),
                drone_config_.drone_id,
                "MISSION_SKIPPED",
                "SUCCESS",
                "Mission only contains duplicate waypoints, no flight needed"
            };
            recorder_->log_command(cmd_log);
            recorder_->flush();
        }
        
        handlers_.mission_complete();
        return;
    }
    
    if (!upload_mission_plan(mission_items)) return;
    if (!wait_system_healthy()) return;
    if (!arm_vehicle()) return;
    
    mission_->subscribe_mission_progress([this, mission_items](Mission::MissionProgress mission_progress) {
        std::stringstream log;
        log << "Mission status update: " << mission_progress.current << " / " << mission_progress.total;
        Logger::log_message(Logger::Type::INFO, log.str());
        
        // Log waypoint reached event
        if (recorder_ && mission_progress.current < mission_items.size()) {
            const auto& item = mission_items[mission_progress.current];
            Drone_Recorder::Mission_Event event{
                get_current_timestamp(),
                drone_config_.drone_id,
                "WAYPOINT_REACHED",
                mission_progress.current,
                item.latitude_deg,
                item.longitude_deg
            };
            recorder_->log_mission_event(event);
        }
    });
    
    wait_for_start_signal();
    if (!perform_takeoff()) return;
    if (!start_mission_execution()) return;
    wait_mission_completion();
    if (!return_to_launch()) return;
    
    // Flush all remaining data
    if (recorder_) {
        recorder_->flush();
        Logger::log_message(Logger::Type::INFO, "Recorder data flushed");
    }
    
    handlers_.mission_complete();
}

void PX4_Wrapper::cleanup_px4_process()
{
    if (px4_pid_ > 0) {
        Logger::log_message(Logger::Type::INFO, "Cleaning up PX4 process tree (root PID: " + std::to_string(px4_pid_) + ") for drone " + std::to_string(drone_config_.drone_id));
        
        // Kill the entire process group to ensure all child processes are terminated
        // The px4_pid_ is the shell, which spawns multiple children (gz model, px4 binary, etc.)
        pid_t pgid = getpgid(px4_pid_);
        if (pgid > 0) {
            // Kill process group with SIGTERM first
            Logger::log_message(Logger::Type::INFO, "Killing process group " + std::to_string(pgid) + " with SIGTERM");
            killpg(pgid, SIGTERM);
            
            // Wait up to 3 seconds for graceful shutdown
            int wait_count = 0;
            while (wait_count < 30) {
                int status;
                pid_t result = waitpid(px4_pid_, &status, WNOHANG);
                if (result == px4_pid_ || result == -1) {
                    Logger::log_message(Logger::Type::INFO, "PX4 process group terminated gracefully");
                    px4_pid_ = -1;
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                wait_count++;
            }
            
            // If still running, force kill the process group
            Logger::log_message(Logger::Type::WARNING, "PX4 process group did not terminate gracefully, forcing shutdown with SIGKILL");
            killpg(pgid, SIGKILL);
            waitpid(px4_pid_, nullptr, 0);
            
            // Also try killing individual known child processes
            std::string kill_cmd = "pkill -9 -P " + std::to_string(px4_pid_) + " 2>/dev/null";
            std::system(kill_cmd.c_str());
        }
        
        px4_pid_ = -1;
        Logger::log_message(Logger::Type::INFO, "PX4 process cleanup complete for drone " + std::to_string(drone_config_.drone_id));
    }
}