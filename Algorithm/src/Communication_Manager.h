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
#include "common_libs/Server.h"

using boost::asio::ip::tcp;

class Communication_Manager {

private:
    boost::asio::io_context& io_context_;
    Server server_;
    tcp::endpoint endpoint_;
    int attemps_ = 0;
    void on_connect();
    void on_error(const boost::system::error_code& ec, const Type_Error &type_error);
    void on_message(const std::string& msg);

public:
    Communication_Manager(boost::asio::io_context& io_context, const tcp::endpoint& endpoint);

};