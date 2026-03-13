/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Engine.cpp                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "structs/Structs_Drone.h"
#include <functional>
#include <future>

using f_handler_normal = std::function<void()>;

// Interface
class Engine {

public:
    struct Handlers {
        f_handler_normal mission_complete = nullptr;
        std::function<void(int drone_id)> error = nullptr;
    };

    Engine(const Struct_Drone::Drone_Config &drone_config, const Struct_Drone::Config_struct &common_config) 
        : drone_config_(drone_config), common_config_(common_config) {}
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;
    Engine(Engine&&) = delete;
    Engine& operator=(Engine&&) = delete;
    virtual ~Engine();
    virtual void start_engine();
    virtual void send_command(const std::string &command);
    virtual void set_handler(Handlers f);
    virtual void set_start_signal(std::shared_future<void> start_signal);
    virtual void mark_commands_ready();
    virtual void flush_recorder();

protected:
    const Struct_Drone::Drone_Config& drone_config() const { return drone_config_; }
    const Struct_Drone::Config_struct& common_config() const { return common_config_; }
    Handlers& handlers() { return handlers_; }
    const Handlers& handlers() const { return handlers_; }
    std::shared_future<void>& start_signal_ref() { return start_signal_; }
    const std::shared_future<void>& start_signal_ref() const { return start_signal_; }

private:
    Struct_Drone::Drone_Config drone_config_;
    Struct_Drone::Config_struct common_config_;
    Handlers handlers_;
    std::shared_future<void> start_signal_;
};