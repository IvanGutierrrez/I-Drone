/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : State_Machine.cpp                
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "State_Machine.h"
#include "./states/Off_State.h"
#include "common_libs/Logger.h"

State_Machine::State_Machine(std::shared_ptr<Communication_Manager> comm_mng)
    : cmm_manager_(std::move(comm_mng))
{
    cmm_manager_->set_message_handler(std::bind(&State_Machine::handleMessage, this, std::placeholders::_1));
}

void State_Machine::transitionTo(std::unique_ptr<State> next_state)
{
    actual_state_ = std::move(next_state);
    Logger::log_message(Logger::Type::INFO, "Transition complete");
    if (actual_state_) {
        actual_state_->start();
    }
}

void State_Machine::handleMessage(const std::string &message)
{
    if (actual_state_) {
        actual_state_->handleMessage(message);
    }
}

std::shared_ptr<Communication_Manager> State_Machine::getCommunicationManager() const
{
    return cmm_manager_;
}

boost::asio::io_context& State_Machine::get_io_context() const
{
    return cmm_manager_->get_io_context();
}