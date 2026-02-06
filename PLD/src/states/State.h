/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : State.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <memory>
#include <string>

// Forward declaration
class State_Machine;

class State {
public:
    State(std::shared_ptr<State_Machine> state_machine_ptr);
    virtual void start();
    virtual void end();
    virtual void handleMessage(const std::string &message);

protected:

    std::shared_ptr<State_Machine> state_machine_ptr_;

};