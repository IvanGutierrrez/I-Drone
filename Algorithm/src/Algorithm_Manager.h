/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Algorithm_Manager.h                    
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
#include "Algorithm_Manager_Interface.h"
#include "Communication_Manager.h"
#include "Algorithm_Recorder.h"
#include "Path_Cal.h"
#include "Signal_Cal.h"

class Algorithm_Manager: public Algorithm_Manager_Interface {

private:
    std::shared_ptr<Communication_Manager> comm_mng_ptr_;
    std::shared_ptr<Algorithm_Recorder> recorder_ptr_;
    std::shared_ptr<Path_Cal> path_cal_ptr_;
    std::shared_ptr<Signal_Cal> signal_cal_ptr_;
    Struct_Algo::Config_struct global_config_;

public:
    Algorithm_Manager(std::shared_ptr<Communication_Manager> &comm_mng, 
                      std::shared_ptr<Algorithm_Recorder> &rec_mng, 
                      std::shared_ptr<Path_Cal> &path_cal,
                      std::shared_ptr<Signal_Cal> &signal_cal,
                      const Struct_Algo::Config_struct cnf);
    ~Algorithm_Manager() override;
    void calculate(const Struct_Algo::SignalServerConfig &config, const Struct_Algo::DroneData &drone_data) override;
};