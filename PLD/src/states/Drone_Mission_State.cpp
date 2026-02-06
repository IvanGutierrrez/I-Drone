/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Drone_Mission_State.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Drone_Mission_State.h"
#include "../State_Machine.h"
#include "Off_State.h"
#include "common_libs/Logger.h"

Drone_Mission_State::Drone_Mission_State(std::shared_ptr<State_Machine> state_machine_ptr): State(state_machine_ptr)
{
}

void Drone_Mission_State::start()
{
    Logger::log_message(Logger::Type::INFO, "Entering Drone Mission State");
    state_machine_ptr_->getCommunicationManager()->set_status(Structs_PLD::Status::EXECUTING_MISSION);
}

void Drone_Mission_State::end()
{
    Logger::log_message(Logger::Type::INFO, "Drone State functionality complete, transitioning to Off State to wait until next mission");
    std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
    state_machine_ptr_->transitionTo(std::move(off_state));
}

void Drone_Mission_State::handleMessage(const std::string &message)
{
    //TODO Process message finish
}