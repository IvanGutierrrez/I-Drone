/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Controller.h                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <iostream>
#include <memory>
#include "structs/Structs_Drone.h"
#include "Communication_Manager.h"
#include "Drone_Recorder.h"

class Controller {

public:
    Controller(std::shared_ptr<Communication_Manager> comm_mng, 
               std::shared_ptr<Drone_Recorder> rec_mng,
               const Struct_Drone::Config_struct cnf);
    ~Controller();
    void handler_message(const std::string &message);

private: 

    std::shared_ptr<Communication_Manager> comm_mng_ptr_;
    std::shared_ptr<Drone_Recorder> recorder_ptr_;
    Struct_Drone::Config_struct global_config_;
    std::vector<std::string> commands_;
};