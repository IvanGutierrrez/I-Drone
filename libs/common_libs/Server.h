/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Server.h                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

using handler_error   = std::function<void(const boost::system::error_code&)>;  
using handler_message = std::function<void(const std::string&)>;

class Server {
public:

    struct handlers {
        handler_error call_error = 0;
        handler_message call_message = 0;
    };


    Server(boost::asio::io_context& io_context, const tcp::endpoint& endpoint);
    Server(boost::asio::io_context& io_context, const tcp::endpoint& endpoint, const handlers &handlers);
    ~Server();
    void set_handlers(const handlers &handlers);
    void start_listening();
    void connect(const tcp::endpoint& endpoint);
    void deliver(const std::string &message);
private:

    boost::asio::io_context& io_context_;
    tcp::acceptor acceptor_;
    handlers handlers_;
    std::shared_ptr<tcp::socket> current_client_;

    void start_read(std::shared_ptr<tcp::socket> socket);
};