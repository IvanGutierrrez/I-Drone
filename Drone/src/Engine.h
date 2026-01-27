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
        f_handler_normal mission_complete = 0;
        f_handler_normal error = 0;
    };

    Engine(const Struct_Drone::Drone_Config &drone_config, const Struct_Drone::Config_struct &common_config) 
        : drone_config_(drone_config), common_config_(common_config) {}
    virtual ~Engine();
    virtual void start_engine();
    virtual void send_command(const std::string &command);
    virtual void set_handler(Handlers f);
    virtual void set_start_signal(std::shared_future<void> start_signal);
    virtual void mark_commands_ready();

protected:
    Struct_Drone::Drone_Config drone_config_;
    Struct_Drone::Config_struct common_config_;
    Handlers handlers_;
    std::shared_future<void> start_signal_;
};