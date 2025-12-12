/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Planner_Manager.h                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <iostream>
#include <memory>
#include <map>
#include "Planner_Manager_Interface.h"
#include "Communication_Manager.h"
#include "Planner_Recorder.h"
#include "Path_Cal.h"
#include "Signal_Cal.h"

class Planner_Manager: public Planner_Manager_Interface {

private:
    std::shared_ptr<Communication_Manager> comm_mng_ptr_;
    std::shared_ptr<Planner_Recorder> recorder_ptr_;
    std::shared_ptr<Path_Cal> path_cal_ptr_;
    std::shared_ptr<Signal_Cal> signal_cal_ptr_;
    Struct_Planner::Config_struct global_config_;

public:
    Planner_Manager(std::shared_ptr<Communication_Manager> comm_mng, 
                      std::shared_ptr<Planner_Recorder> rec_mng, 
                      std::shared_ptr<Path_Cal> path_cal,
                      std::shared_ptr<Signal_Cal> signal_cal,
                      const Struct_Planner::Config_struct cnf);
    ~Planner_Manager() override;
    void calculate(const Struct_Planner::SignalServerConfig &config, Struct_Planner::DroneData drone_data) override;
};