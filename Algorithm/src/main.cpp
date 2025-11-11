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
#include "Communication_Manager.h"
#include "Algorithm_Manager_Interface.h"
#include "Algorithm_Manager.h"

int main(int argc, char* argv[]) {
    // Initialize logger
    if (!Logger::initialize(Config::log_path,Config::log_name))
    {
        std::cout << "Error initializing logger. Exiting program...\n";
        return EXIT_FAILURE;
    }

    Logger::log_message(Logger::TYPE::INFO, "Algorithm module started");

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
        Logger::log_message(Logger::TYPE::ERROR, "Mandatory arguments missing --PLD_Address y --PLD_port");
        return EXIT_FAILURE;
    }

    boost::asio::ip::tcp::endpoint pld_endpoint;
    try {
        pld_endpoint = boost::asio::ip::tcp::endpoint(
            boost::asio::ip::address::from_string(pld_address),
            static_cast<unsigned short>(pld_port)
        );

    } catch (const std::exception& e) {
        Logger::log_message(Logger::TYPE::ERROR, std::string("Error creating PLD endpoint: ") + e.what());
        return EXIT_FAILURE;
    }

    boost::asio::io_context io_context;

    std::shared_ptr<Communication_Manager> comm_mng_ptr = std::make_shared<Communication_Manager>(io_context,pld_endpoint);

    std::shared_ptr<Algorithm_Recorder> rec_mng_ptr = std::make_shared<Algorithm_Recorder>();

    std::shared_ptr<Algorithm_Manager_Interface> algo_mng_ptr = std::make_shared<Algorithm_Manager>(comm_mng_ptr,rec_mng_ptr);

    io_context.run();

    Logger::log_message(Logger::TYPE::INFO, "No more async functions to do, exiting program...");
}