/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : main.cpp                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <iostream>
#include <boost/asio.hpp>
#include "Config.h"
#include "common_libs/Logger.h"
#include "common_libs/Signal_Handler.h"
#include "Communication_Manager.h"
#include "State_Machine.h"
#include "PLD_Recorder.h"
#include "./states/Off_State.h"
#include "structs/Structs_PLD.h"

int main(int argc, char* argv[]) {
    // Initialize logger
    Structs_PLD::Config_struct cnf = Config::get_config();
    if (!Logger::initialize(cnf.log_path,cnf.log_name))
    {
        std::cout << "Error initializing logger. Exiting program...\n";
        return EXIT_FAILURE;
    }

    Logger::log_message(Logger::Type::INFO, "PLD module started");

    // Parser arguments
    std::string own_address;
    int own_port = -1;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--Own_Address" && i + 1 < argc) {
            own_address = argv[++i];
        } else if (arg == "--Own_port" && i + 1 < argc) {
            own_port = std::stoi(argv[++i]);
        }
    }

    if (own_address.empty() || own_port == -1) {
        Logger::log_message(Logger::Type::ERROR, "Mandatory arguments missing --Own_Address and --Own_port");
        return EXIT_FAILURE;
    }
    boost::asio::io_context io_context;

    boost::asio::ip::tcp::endpoint own_endpoint;
    try {
        boost::asio::ip::tcp::resolver resolver(io_context);
        auto endpoints = resolver.resolve(own_address, std::to_string(own_port));

        if (endpoints.begin() == endpoints.end()) {
            Logger::log_message(Logger::Type::ERROR, "No endpoints found for own Server: " + own_address + ":" + std::to_string(own_port));
            return EXIT_FAILURE;
        }

        own_endpoint = *endpoints.begin();
        Logger::log_message(Logger::Type::INFO,"Own Server endpoint:  " + own_endpoint.address().to_string() + ":" + std::to_string(own_endpoint.port()));
    } catch (const std::exception& e) {
        Logger::log_message(Logger::Type::ERROR, std::string("Error creating own Server endpoint: ") + e.what());
        return EXIT_FAILURE;
    }

    std::shared_ptr<Communication_Manager> comm_mng_ptr = std::make_shared<Communication_Manager>(io_context, own_endpoint);
    std::shared_ptr<PLD_Recorder> recorder_ptr = std::make_shared<PLD_Recorder>(cnf.data_path);

    std::shared_ptr<State_Machine> state_machine_ptr = std::make_shared<State_Machine>(comm_mng_ptr, recorder_ptr);
    std::unique_ptr<State> off_state = std::make_unique<Off_State>(state_machine_ptr);
    state_machine_ptr->transitionTo(std::move(off_state));

    Signal_Handler signal_handler(io_context, [&recorder_ptr]() {
        if (recorder_ptr) {
            recorder_ptr->close();
        }
        Logger::log_message(Logger::Type::INFO, "Shutdown complete");
    });

    Logger::log_message(Logger::Type::INFO, "Starting io_context...");
    io_context.run();

    Logger::log_message(Logger::Type::INFO, "No more async functions to do, exiting program...");
    
    if (recorder_ptr) {
        recorder_ptr->close();
    }
    Logger::close();
    return 0;
}