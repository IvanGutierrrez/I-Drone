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
#include "Planner_Manager_Interface.h"
#include "Planner_Manager.h"
#include "structs/Structs_Planner.h"
#include "Path_Cal.h"
#include "Signal_Cal.h"

int main(int argc, char* argv[]) {
    // Initialize logger
    Struct_Planner::Config_struct cnf = Config::get_config();
    if (!Logger::initialize(cnf.log_path,cnf.log_name))
    {
        std::cout << "Error initializing logger. Exiting program...\n";
        return EXIT_FAILURE;
    }

    Logger::log_message(Logger::Type::INFO, "Planner module started");

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
            std::ostringstream error_msg;
            error_msg << "No endpoints found for PLD: " << pld_address << ':' << pld_port;
            Logger::log_message(Logger::Type::ERROR, error_msg.str());
            return EXIT_FAILURE;
        }

        pld_endpoint = *endpoints.begin();
        std::ostringstream error_msg;
        error_msg << "PLD endpoint:  " << pld_endpoint.address() << ':' << pld_endpoint.port();
        Logger::log_message(Logger::Type::ERROR, error_msg.str());
    } catch (const std::exception& e) {
        Logger::log_message(Logger::Type::ERROR, std::string("Error creating PLD endpoint: ") + e.what());
        return EXIT_FAILURE;
    }

    auto rec_mng_ptr = std::make_shared<Planner_Recorder>(cnf.data_path);

    auto comm_mng_ptr = std::make_shared<Communication_Manager>(io_context,pld_endpoint,rec_mng_ptr);

    auto path_cal_ptr = std::make_shared<Path_Cal>(cnf);

    auto signal_cal_ptr = std::make_shared<Signal_Cal>();

    std::shared_ptr<Planner_Manager_Interface> planner_mng_ptr = std::make_shared<Planner_Manager>(comm_mng_ptr,
                                                                                                    rec_mng_ptr,
                                                                                                    path_cal_ptr,
                                                                                                    signal_cal_ptr,
                                                                                                    cnf);
    
    Signal_Handler signal_handler(io_context, [&rec_mng_ptr, &comm_mng_ptr]() {
        comm_mng_ptr->shutdown();
        rec_mng_ptr->close_all();
        Logger::log_message(Logger::Type::INFO, "Shutdown complete");
    });

    Logger::log_message(Logger::Type::INFO, "Starting io_context...");
    io_context.run();

    Logger::log_message(Logger::Type::INFO, "No more async functions to do, exiting program...");
    Logger::close();
}