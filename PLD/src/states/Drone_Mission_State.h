/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Drone_Mission_State.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "State.h"

class Drone_Mission_State: public State {
public:
    Drone_Mission_State(std::shared_ptr<State_Machine> state_machine_ptr);

    void start() override;
    void end() override;
    void handleMessage(const std::string &message) override;

};