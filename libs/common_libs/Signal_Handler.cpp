/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Signal_Handler.cpp                 
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Signal_Handler.h"
#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>
#include "common_libs/Logger.h"

Signal_Handler::Signal_Handler(boost::asio::io_service &io_service, const shutdown_callback& callback)
    : io_service_(io_service),
      signals_(io_service_, SIGINT, SIGTERM),
      shutdown_callback_(callback)
{
    wait_for_signals();
}

void Signal_Handler::wait_for_signals()
{
    signals_.async_wait([this](const boost::system::error_code&, int signal_number) {
        std::string signal_name;
        switch(signal_number) {
            case SIGINT:  signal_name = "SIGINT";  break;
            case SIGTERM: signal_name = "SIGTERM"; break;
            default:      return;
        }

        if (shutting_down_.exchange(true)) {
            return; // Already shutting down
        }
        
        std::stringstream log;
        log << "Signal received: " << signal_name << ". Initiating shutdown...";
        Logger::log_message(Logger::Type::WARNING, log.str());

        if (shutdown_callback_) {
            try {
                shutdown_callback_();
            } catch (const std::exception& e) {
                std::stringstream log_er;
                log_er << "Exception in shutdown callback: " << e.what();
                Logger::log_message(Logger::Type::ERROR, log_er.str());
            } catch (...) {
                Logger::log_message(Logger::Type::ERROR, "Unknown exception in shutdown callback");
            }
        }
        
        io_service_.stop();
    });
}