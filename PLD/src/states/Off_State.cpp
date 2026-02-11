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
    state_machine_ptr_->getCommunicationManager()->set_status(Structs_PLD::Status::WAITING_INFO);
    
    // Start new recording session when entering Off State
    if (state_machine_ptr_->getRecorder()) {
        state_machine_ptr_->getRecorder()->start_new_session();
        state_machine_ptr_->getRecorder()->write_state_transition("N/A", "Off_State");
    }
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
    auto [type, proto_msg] = Enc_Dec_PLD::decode_from_client(message);
    
    if (type == Enc_Dec_PLD::PLD::CONFIG_MISSION) {
        Config_mission* config_proto = dynamic_cast<Config_mission*>(proto_msg.get());
        if (!config_proto) {
            Logger::log_message(Logger::Type::ERROR, "Unable to decode message config mission from Client");
            if (state_machine_ptr_->getRecorder()) {
                state_machine_ptr_->getRecorder()->write_error("Unable to decode CONFIG_MISSION from Client");
                state_machine_ptr_->getRecorder()->write_raw_message("Client", message);
            }
            return;
        }

        if (!Enc_Dec_PLD::decode_config_mission(*config_proto, config_)) {
            Logger::log_message(Logger::Type::ERROR, "Unable to decode message config mission from Client");
            if (state_machine_ptr_->getRecorder()) {
                state_machine_ptr_->getRecorder()->write_error("Unable to decode config mission from Client");
            }
            return;
        }
    
        // Record received message
        if (state_machine_ptr_->getRecorder()) {
            state_machine_ptr_->getRecorder()->write_message_received("Client", "CONFIG_MISSION", "Config mission received");
        }
        
        // Transitionate to the next state
        end();


    } else if (type == Enc_Dec_PLD::PLD::COMMAND) {
        Command* command = dynamic_cast<Command*>(proto_msg.get());
        if (!command) {
            Logger::log_message(Logger::Type::WARNING, "Unable to decode command from Client");
            if (state_machine_ptr_->getRecorder()) {
                state_machine_ptr_->getRecorder()->write_error("Unable to decode COMMAND from Client");
                state_machine_ptr_->getRecorder()->write_raw_message("Client", message);
            }
            return;
        }

        if (state_machine_ptr_->getRecorder()) {
            state_machine_ptr_->getRecorder()->write_message_received("Client", "COMMAND", command->command());
        }

        if (command->command() == "FINISH") {
            Logger::log_message(Logger::Type::WARNING, "FINISH command received in Off State, shuting down...");
            state_machine_ptr_->get_io_context().stop();
        } else {
            Logger::log_message(Logger::Type::WARNING, "Unexpected command received from Client: " + command->command());
            if (state_machine_ptr_->getRecorder()) {
                state_machine_ptr_->getRecorder()->write_error("Unexpected command: " + command->command());
            }
        }

    } else {
        Logger::log_message(Logger::Type::WARNING, "Unexpected message received from Client, type: " + Enc_Dec_PLD::to_string(type));
        if (state_machine_ptr_->getRecorder()) {
            state_machine_ptr_->getRecorder()->write_error("Unexpected message type: " + Enc_Dec_PLD::to_string(type));
            state_machine_ptr_->getRecorder()->write_raw_message("Client", message);
        }
        return;
    }
}