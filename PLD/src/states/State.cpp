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
	// Intentionally empty: concrete state classes define startup behavior.
}

void State::end()
{
	// Intentionally empty: concrete state classes define teardown behavior.
}

void State::handleMessage(const std::string &message)
{
	// Intentionally empty: concrete state classes handle incoming messages.
}