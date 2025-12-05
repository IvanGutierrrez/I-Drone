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

// Interface
class Engine {
protected:
    Struct_Drone::Config_struct config_;


public:
    Engine(const Struct_Drone::Config_struct &config) : config_(config){}
    virtual ~Engine();
    virtual void start_engine();
    virtual void send_command(const std::string &command);
};