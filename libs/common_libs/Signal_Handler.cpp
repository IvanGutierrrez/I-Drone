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
#include "common_libs/Logger.h"

Signal_Handler::Signal_Handler(boost::asio::io_service &io_service, executor_type &work, const std::function<void()> &handler)
    : io_service_(io_service),
      work_(work),
      handler_(handler),
      signals_(io_service_, SIGINT, SIGTERM)
{
    wait_for_signals();
}

void Signal_Handler::wait_for_signals()
{
    signals_.async_wait([this](const boost::system::error_code& ec, int signal_number) {
        std::string signal_name;
        switch(signal_number) {
            case SIGINT:  signal_name = "SIGINT";  break;
            case SIGTERM: signal_name = "SIGTERM"; break;
            default:      return;
        }
        std::stringstream log;
        log << "Signal received: " << signal_name << ". Stopping io_service...";
        Logger::log_message(Logger::Type::WARNING, log.str());

        std::thread shutdown_thread([handler = handler_]() {
            handler();
        });
        shutdown_thread.detach();
        work_.reset();
        io_service_.stop();
    });
}