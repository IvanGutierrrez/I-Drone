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

enum class Type_Error {
    CONNECTING,
    READING,
    SENDING
};

using handler_connect = std::function<void()>;
using handler_error   = std::function<void(const boost::system::error_code&, const Type_Error &)>;  
using handler_message = std::function<void(const std::string&)>;

class Server {
public:

    struct handlers {
        handler_connect call_connect = nullptr;
        handler_error call_error = nullptr;
        handler_message call_message = nullptr;
    };


    explicit Server(boost::asio::io_context& io_context);
    Server(boost::asio::io_context& io_context, const handlers &handlers);
    ~Server();
    void set_handlers(const handlers &handlers);
    void start_listening(const tcp::endpoint& endpoint);
    void accept_new_connection();
    void connect(const tcp::endpoint& endpoint);
    void deliver(const std::string &message);
    void server_close();
private:

    boost::asio::io_context& io_context_;
    tcp::acceptor acceptor_;
    handlers handlers_;
    std::shared_ptr<tcp::socket> current_client_;
    bool is_listening_;

    void notify_connecting_error(const boost::system::error_code &error);
    bool prepare_listening(const tcp::endpoint& endpoint);
    void on_accept(const boost::system::error_code& error, const std::shared_ptr<tcp::socket> &new_socket);
    void start_async_accept(const std::shared_ptr<tcp::socket> &new_socket);
    void start_read(std::shared_ptr<tcp::socket> socket);
};