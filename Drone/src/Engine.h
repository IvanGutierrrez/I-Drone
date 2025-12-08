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

using f_handler_normal = std::function<void()>;

// Interface
class Engine {

public:
    struct Handlers {
        f_handler_normal mission_complete = 0;
        f_handler_normal error = 0;
    };

    Engine(const Struct_Drone::Config_struct &config) : config_(config){}
    virtual ~Engine();
    virtual void start_engine();
    virtual void send_command(const std::string &command);
    virtual void set_handler(Handlers f);

protected:
    Struct_Drone::Config_struct config_;
    Handlers handlers_;
};