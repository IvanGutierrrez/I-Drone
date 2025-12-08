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
#include <memory>
#include <thread>
#include "structs/Structs_Drone.h"
#include "Communication_Manager.h"
#include "Drone_Recorder.h"
#include "Engine.h"

class Controller {

public:
    Controller(std::shared_ptr<Communication_Manager> comm_mng, 
               std::shared_ptr<Drone_Recorder> rec_mng,
               std::shared_ptr<Engine> engine,
               const Struct_Drone::Config_struct cnf);
    ~Controller();
    void handler_message(const std::string &message);
    void mission_complete();
    void error_processing_command();

private:
    void engine_started();

    std::shared_ptr<Communication_Manager> comm_mng_ptr_;
    std::shared_ptr<Drone_Recorder> recorder_ptr_;
    std::shared_ptr<Engine> engine_;
    Struct_Drone::Config_struct global_config_;
    
    std::thread start_engine_thread_;
};