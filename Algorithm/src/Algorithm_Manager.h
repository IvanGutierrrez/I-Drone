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
#include "Algorithm_Manager_Interface.h"
#include "Communication_Manager.h"
#include "Algorithm_Recorder.h"

class Algorithm_Manager: public Algorithm_Manager_Interface {

public:
    std::shared_ptr<Communication_Manager> comm_mng_ptr_;
    std::shared_ptr<Algorithm_Recorder> recorder_ptr_;

public:
    Algorithm_Manager(std::shared_ptr<Communication_Manager> &comm_mng, std::shared_ptr<Algorithm_Recorder> &rec_mng);
    ~Algorithm_Manager() override;
    void calculate() override;
};