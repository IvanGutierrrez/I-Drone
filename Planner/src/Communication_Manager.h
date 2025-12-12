/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Communication_Manager.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <boost/asio.hpp>
#include <functional>
#include "Planner_Recorder.h"
#include "common_libs/Server.h"
#include "structs/Structs_Planner.h"

using boost::asio::ip::tcp;

using calculate_handler = std::function<void(const Struct_Planner::SignalServerConfig&, Struct_Planner::DroneData)>;;

class Communication_Manager {

private:
    boost::asio::io_context& io_context_;
    boost::asio::thread_pool calculation_pool_;
    Server server_;
    tcp::endpoint endpoint_;
    std::shared_ptr<Planner_Recorder> recorder_ptr_;
    boost::asio::steady_timer status_timer;
    std::shared_ptr<boost::asio::steady_timer> retry_timer_;
    int attemps_ = 0;
    Struct_Planner::Status status_;
    calculate_handler calculate_handler_;
    std::mutex mutex_status_;
    std::mutex mutex_deliver_;
    std::atomic<bool> shutting_down_{false};

    void on_connect();
    void on_error(const boost::system::error_code& ec, const Type_Error &type_error);
    void on_message(const std::string& msg);
    void send_status_message(const boost::system::error_code& ec);
    Struct_Planner::Status get_status();

public:
    Communication_Manager(boost::asio::io_context& io_context, const tcp::endpoint& endpoint,const std::shared_ptr<Planner_Recorder> &rec_mng);
    ~Communication_Manager();
    void set_status(const Struct_Planner::Status &new_status);
    void set_calculate_handler(const calculate_handler &handler);
    void deliver(const std::string &msg);
    void shutdown();

};