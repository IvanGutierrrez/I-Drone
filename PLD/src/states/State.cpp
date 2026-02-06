/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : State.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "State.h"

State::State(std::shared_ptr<State_Machine> state_machine_ptr): state_machine_ptr_(state_machine_ptr)
{
}

void State::start()
{
}

void State::end()
{
}

void State::handleMessage(const std::string &message)
{
}