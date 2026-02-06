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
#include <map>
#include "common_libs/Server.h"
#include "structs/Structs_PLD.h"

using boost::asio::ip::tcp;

using message_handler = std::function<void(const std::string &)>;

class Communication_Manager {

private:
    boost::asio::io_context& io_context_;
    Server server_;
    tcp::endpoint endpoint_;
    boost::asio::steady_timer status_timer_;
    int attemps_ = 0;
    Structs_PLD::Status status_;
    message_handler message_handler_;
    std::mutex mutex_status_;
    std::mutex mutex_deliver_;
    std::atomic<bool> shutting_down_{false};
    unsigned int number_servers_ = 0;
    std::map<int, std::shared_ptr<Server>> servers_created_;

    void on_connect_client();
    void on_error_client(const boost::system::error_code& ec, const Type_Error &type_error);
    void on_message_client(const std::string& msg);
    void send_status_message(const boost::system::error_code& ec);
    Structs_PLD::Status get_status();

public:
    Communication_Manager(boost::asio::io_context& io_context, const tcp::endpoint& endpoint);
    ~Communication_Manager();
    void set_status(const Structs_PLD::Status &new_status);
    void set_message_handler(const message_handler &handler);
    void deliver(const std::string &msg);
    void shutdown();

    int create_server(Server::handlers &handler_obj, const std::string &ip, const std::string &port);
    void close_connection_to_server(const int &n);
    bool send_message_to_server(const int &n, const std::string &msg);

    boost::asio::io_context& get_io_context() const;

};