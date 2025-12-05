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
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
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
    void command_complete();

private: 
    void process_commands();

    std::shared_ptr<Communication_Manager> comm_mng_ptr_;
    std::shared_ptr<Drone_Recorder> recorder_ptr_;
    std::shared_ptr<Engine> engine_;
    Struct_Drone::Config_struct global_config_;
    
    std::vector<std::string> commands_;
    std::thread worker_thread_;
    std::mutex commands_mutex_;
    std::condition_variable cv_add_command_;
    std::condition_variable cv_command_complete_;
    std::atomic<bool> running_;
    std::atomic<bool> command_completed_;
    bool first_complete_;
};