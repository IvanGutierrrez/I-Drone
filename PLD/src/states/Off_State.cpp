/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Off_State.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Off_State.h"
#include "Planner_State.h"
#include "../State_Machine.h"
#include "common_libs/Enc_Dec_PLD.h"
#include "common_libs/Logger.h"

Off_State::Off_State(std::shared_ptr<State_Machine> state_machine_ptr): State(state_machine_ptr)
{
}

void Off_State::start()
{
    Logger::log_message(Logger::Type::INFO, "Entering Off State, waiting for Client");
    state_machine_ptr_->getCommunicationManager()->set_status(Structs_PLD::Status::PLANNING_MISSION);
}

void Off_State::end()
{
    std::unique_ptr<Planner_State> planner_state = std::make_unique<Planner_State>(state_machine_ptr_);
    planner_state->set_data(config_);

    Logger::log_message(Logger::Type::INFO, "Off State functionality complete, transitioning to the next state");
    state_machine_ptr_->transitionTo(std::move(planner_state));
}

void Off_State::handleMessage(const std::string &message)
{
    //TODO Process message finish
    auto [type, proto_msg] = Enc_Dec_PLD::decode_from_client(message);
    
    if (type != Enc_Dec_PLD::PLD::CONFIG_MISSION) {
        Logger::log_message(Logger::Type::WARNING, "Unexpected message received from Client");
        return;
    }

    Config_mission* config_proto = dynamic_cast<Config_mission*>(proto_msg.get());
    if (!config_proto) {
        Logger::log_message(Logger::Type::ERROR, "Unable to decode message from Client");
        return;
    }

    if (!Enc_Dec_PLD::decode_config_mission(*config_proto, config_)) {
        Logger::log_message(Logger::Type::ERROR, "Unable to decode message from Client");
        return;
    }
    
    // Transitionate to the next state
    this->end();
}