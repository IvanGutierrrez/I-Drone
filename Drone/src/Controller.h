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
#include "Multi_Drone_Manager.h"

class Controller {

public:
    Controller(std::shared_ptr<Communication_Manager> comm_mng, 
               std::shared_ptr<Multi_Drone_Manager> drone_manager,
               const Struct_Drone::Config_struct &cnf);
    ~Controller();
    void handler_message(const std::string &message);
    void mission_complete();
    void error_callback(int drone_id);
    void missions_ready();

private:
    std::shared_ptr<Communication_Manager> comm_mng_ptr_;
    std::shared_ptr<Multi_Drone_Manager> drone_manager_;
    Struct_Drone::Config_struct global_config_;
};