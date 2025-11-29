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
#include "Controller.h"
#include "Communication_Manager.h"
#include "Drone_Recorder.h"
#include "common_libs/Logger.h"
#include "common_libs/Signal_Handler.h"

int main(int argc, char* argv[]) {
    // Initialize logger
    Struct_Drone::Config_struct cnf = Config::get_config();
    if (!Logger::initialize(cnf.log_path,cnf.log_name))
    {
        std::cout << "Error initializing logger. Exiting program...\n";
        return EXIT_FAILURE;
    }

    Logger::log_message(Logger::Type::INFO, "Drone module started");

    // Parser arguments
    std::string pld_address;
    int pld_port = -1;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--PLD_Address" && i + 1 < argc) {
            pld_address = argv[++i];
        } else if (arg == "--PLD_port" && i + 1 < argc) {
            pld_port = std::stoi(argv[++i]);
        }
    }

    if (pld_address.empty() || pld_port == -1) {
        Logger::log_message(Logger::Type::ERROR, "Mandatory arguments missing --PLD_Address y --PLD_port");
        return EXIT_FAILURE;
    }

    boost::asio::io_context io_context;
    boost::asio::ip::tcp::endpoint pld_endpoint;
    try {
        boost::asio::ip::tcp::resolver resolver(io_context);
        auto endpoints = resolver.resolve(pld_address, std::to_string(pld_port));

        if (endpoints.begin() == endpoints.end()) {
            Logger::log_message(Logger::Type::ERROR, "No endpoints found for PLD: " + pld_address + ":" + std::to_string(pld_port));
            return EXIT_FAILURE;
        }

        pld_endpoint = *endpoints.begin();
        Logger::log_message(Logger::Type::INFO,"PLD endpoint:  " + pld_endpoint.address().to_string() + ":" + std::to_string(pld_endpoint.port()));
    } catch (const std::exception& e) {
        Logger::log_message(Logger::Type::ERROR, std::string("Error creating PLD endpoint: ") + e.what());
        return EXIT_FAILURE;
    }

    std::shared_ptr<Drone_Recorder> rec_mng_ptr = std::make_shared<Drone_Recorder>();

    std::shared_ptr<Communication_Manager> comm_mng_ptr = std::make_shared<Communication_Manager>(io_context,pld_endpoint,rec_mng_ptr);

    std::shared_ptr<Controller> controller_ptr = std::make_shared<Controller>(comm_mng_ptr,
                                                                              rec_mng_ptr,
                                                                              cnf);
    
    Signal_Handler signal_handler(io_context, []() {
        Logger::log_message(Logger::Type::INFO, "Shutdown complete");
    });

    Logger::log_message(Logger::Type::INFO, "Starting io_context...");
    io_context.run();

    Logger::log_message(Logger::Type::INFO, "No more async functions to do, exiting program...");
    Logger::close();
}