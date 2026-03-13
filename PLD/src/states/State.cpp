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
#include "../State_Machine.h"
#include "common_libs/Enc_Dec_PLD.h"
#include "common_libs/Logger.h"

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
	auto [type, proto_msg] = Enc_Dec_PLD::decode_from_client(message);

	if (type == Enc_Dec_PLD::PLD::CONFIG_MISSION) {
		Logger::log_message(Logger::Type::WARNING, std::string("unexpected CONFIG MISSION message received in ") + state_name() + ", ignoring");
		if (state_machine()->getRecorder()) {
			state_machine()->getRecorder()->write_message_received("Client", "CONFIG_MISSION", std::string("Unexpected in ") + state_name());
		}
		return;
	}

	if (type == Enc_Dec_PLD::PLD::COMMAND) {
		const auto* command = dynamic_cast<const Command*>(proto_msg.get());
		if (!command) {
			Logger::log_message(Logger::Type::WARNING, "Unable to decode command from Client");
			if (state_machine()->getRecorder()) {
				state_machine()->getRecorder()->write_error("Unable to decode COMMAND from Client");
				state_machine()->getRecorder()->write_raw_message("Client", message);
			}
			return;
		}

		if (state_machine()->getRecorder()) {
			state_machine()->getRecorder()->write_message_received("Client", "COMMAND", command->command());
		}

		if (command->command() == "FINISH") {
			handle_finish_command();
			return;
		}

		Logger::log_message(Logger::Type::WARNING, "Unexpected command received from Client: " + command->command());
		if (state_machine()->getRecorder()) {
			state_machine()->getRecorder()->write_error("Unexpected command: " + command->command());
		}
		return;
	}

	Logger::log_message(Logger::Type::WARNING, "unexpected message received from Client, type: " + Enc_Dec_PLD::to_string(type));
	if (state_machine()->getRecorder()) {
		state_machine()->getRecorder()->write_error("Unexpected message type: " + Enc_Dec_PLD::to_string(type));
		state_machine()->getRecorder()->write_raw_message("Client", message);
	}
}

const char* State::state_name() const
{
	return "State";
}

void State::handle_finish_command()
{
	Logger::log_message(Logger::Type::WARNING, std::string("FINISH command received in ") + state_name() + ", ignoring");
}