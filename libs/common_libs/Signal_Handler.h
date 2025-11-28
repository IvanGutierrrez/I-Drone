/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Signal_Handler.h                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <boost/asio.hpp>
#include <atomic>

using shutdown_callback = std::function<void()>;

class Signal_Handler {

public:
    explicit Signal_Handler(boost::asio::io_service &io_service, const shutdown_callback& callback);

    void wait_for_signals();

private:
    boost::asio::io_service &io_service_;
    boost::asio::signal_set signals_;
    shutdown_callback shutdown_callback_;
    std::atomic<bool> shutting_down_{false};

};