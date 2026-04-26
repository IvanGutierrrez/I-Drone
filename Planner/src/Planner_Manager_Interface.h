/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Planner_Manager_Interface.h                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <iostream>
#include <memory>
#include "Communication_Manager.h"
#include "Planner_Recorder.h"

// Interface
class Planner_Manager_Interface {

public:
    virtual ~Planner_Manager_Interface();
    virtual void calculate(const std::vector<Struct_Planner::SignalServerConfig> &configs, Struct_Planner::DroneData drone_data);
};