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

using executor_type = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;

class Signal_Handler {

public:
    explicit Signal_Handler(boost::asio::io_service &io_service, executor_type &work);

    void wait_for_signals();

private:
    boost::asio::io_service &io_service_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
    boost::asio::signal_set signals_;

};