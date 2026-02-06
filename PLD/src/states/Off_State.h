/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Off_State.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "State.h"
#include "structs/Structs_PLD.h"

class Off_State: public State {
public:
    Off_State(std::shared_ptr<State_Machine> state_machine_ptr);

    void start() override;
    void end() override;
    void handleMessage(const std::string &message) override;

private:
    Structs_PLD::Config_mission config_;

};