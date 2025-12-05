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

// Interface
class PX4_Wrapper: public Engine {


public:
    PX4_Wrapper(const Struct_Drone::Config_struct &config);
    ~PX4_Wrapper() override = default;
    void start_engine() override;
    void send_command(const std::string &command) override;
};

#endif
