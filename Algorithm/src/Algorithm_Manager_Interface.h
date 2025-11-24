/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Algorithm_Manager_Interface.h                    
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
#include "Algorithm_Recorder.h"

// Interface
class Algorithm_Manager_Interface {

public:
    virtual ~Algorithm_Manager_Interface();
    virtual void calculate(const Struct_Algo::SignalServerConfig &config, Struct_Algo::DroneData drone_data);
};