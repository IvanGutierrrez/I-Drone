/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Communication_Manager.cpp                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Communication_Manager.h"

#include <thread>
#include <chrono>
#include "common_libs/Logger.h"
#include "common_libs/Enc_Dec_PLD.h"

constexpr int NUMBER_ATTEMPS_MAX = 10;
constexpr int RATE_STATUS_MESSAGE = 1;

Communication_Manager::Communication_Manager(boost::asio::io_context& io_context, 
                                             const tcp::endpoint& endpoint): io_context_(io_context),
                                                                             server_(io_context),
                                                                             endpoint_(endpoint),
                                                                             status_timer_(io_context_),
                                                                             status_(Structs_PLD::Status::WAITING_INFO)
{
    Server::handlers handler_obj;

    handler_obj.call_error = std::bind(&Communication_Manager::on_error_client, this, std::placeholders::_1, std::placeholders::_2);
    handler_obj.call_connect = std::bind(&Communication_Manager::on_connect_client, this);
    handler_obj.call_message = std::bind(&Communication_Manager::on_message_client, this, std::placeholders::_1);
    server_.set_handlers(handler_obj);

    std::stringstream ss;
    ss << "Start connecting to PLD at " << endpoint_.address().to_string() << ":" << endpoint_.port();
    Logger::log_message(Logger::Type::INFO, ss.str());

    server_.start_listening(endpoint_);
}

Communication_Manager::~Communication_Manager()
{
    shutdown();
    server_.server_close();
}

void Communication_Manager::shutdown()
{
    if (shutting_down_.exchange(true)) {
        return;
    }

    Logger::log_message(Logger::Type::INFO, "Communication_Manager shutting down");
    
    boost::system::error_code ec;
    status_timer_.cancel(ec);
    
    server_.server_close();
}

boost::asio::io_context& Communication_Manager::get_io_context() const
{
    return io_context_;
}

void Communication_Manager::on_connect_client()
{
    attemps_ = 0;
    Logger::log_message(Logger::Type::INFO,"Succesfully connected to Client");
    boost::system::error_code ec;
    send_status_message(ec);
}

void Communication_Manager::send_status_message(const boost::system::error_code& ec)
{
    if (shutting_down_ || ec == boost::asio::error::operation_aborted) {
        return;
    }
    
    if (ec) {
        Logger::log_message(Logger::Type::WARNING,"Problems with timer to send status message to Client");
        return;
    }   

    Logger::log_message(Logger::Type::INFO,"Sending status message");

    std::string message;
    if (!Enc_Dec_PLD::encode_status_pld(get_status(),message)) {
        Logger::log_message(Logger::Type::WARNING,"Problems encoding status message");
    } else {
        deliver(message);
    }
    
    if (!shutting_down_) {
        status_timer_.expires_after(std::chrono::seconds(RATE_STATUS_MESSAGE));
        status_timer_.async_wait(std::bind(&Communication_Manager::send_status_message, this, std::placeholders::_1));
    }
}

void Communication_Manager::set_status(const Structs_PLD::Status &new_status)
{
    std::lock_guard<std::mutex> lock(mutex_status_);
    status_ = new_status;
}

void Communication_Manager::on_error_client(const boost::system::error_code& ec, const Type_Error &type_error)
{    
    if (shutting_down_) return;
    
    boost::system::error_code cancel_ec;
    status_timer_.cancel(cancel_ec);
    
    if (get_status() == Structs_PLD::Status::FINISH) {
        Logger::log_message(Logger::Type::INFO,"Program task complete, leaving program...");
        io_context_.stop();
        return;
    } else if (get_status() == Structs_PLD::Status::ERROR) {
        Logger::log_message(Logger::Type::ERROR, "Error detected, shutting down program...");
        io_context_.stop();
        return;
    }

    Logger::log_message(Logger::Type::WARNING, "on_error callback triggered");
    std::string log;
    switch (type_error){
        case Type_Error::CONNECTING:
            log = "Error connecting to Client";
            break;
        case Type_Error::READING:
            log = "Error while reading a message from Client";
            break;
        case Type_Error::SENDING:
            log = "Error while sending a message to Client";
            break;
        default:
            log = "Unknown error communicating with Client";
            break;
    }
    Logger::log_message(Logger::Type::WARNING,log + ": " + ec.message());
    
    if (shutting_down_) {
        Logger::log_message(Logger::Type::INFO,"Shutting down, skipping reconnection");
        return;
    }

    Logger::log_message(Logger::Type::INFO,"Trying to reconnect to Client");
    server_.accept_new_connection();
}

void Communication_Manager::set_message_handler(const message_handler &handler)
{
    message_handler_ = std::move(handler);
}

void Communication_Manager::on_message_client(const std::string& msg)
{    
    if (shutting_down_) return;
    Logger::log_message(Logger::Type::INFO, "Message received from Client");

    if (message_handler_) {
        message_handler_(msg);
    } else {
        Logger::log_message(Logger::Type::WARNING, "No message handler set");
    }
}

void Communication_Manager::deliver(const std::string &msg)
{
    std::lock_guard<std::mutex> lock(mutex_deliver_);
    server_.deliver(msg);
}

Structs_PLD::Status Communication_Manager::get_status()
{
    std::lock_guard<std::mutex> lock(mutex_status_);
    return status_;
}

int Communication_Manager::create_server(Server::handlers &handler_obj, const std::string &ip, const std::string &port)
{

    boost::asio::ip::tcp::endpoint endpoint;
    try {
        boost::asio::ip::tcp::resolver resolver(io_context_);
        auto endpoints = resolver.resolve(ip,port);

        if (endpoints.begin() == endpoints.end()) {
            Logger::log_message(Logger::Type::ERROR, "No endpoints found for Server: " + ip + ":" + port);
            return -1;
        }

        endpoint = *endpoints.begin();
        Logger::log_message(Logger::Type::INFO,"Server endpoint: " + endpoint.address().to_string() + ":" + std::to_string(endpoint.port()) + " created");
    } catch (const std::exception& e) {
        Logger::log_message(Logger::Type::ERROR, std::string("Error creating Server endpoint: ") + e.what());
        return -1;
    }

    std::shared_ptr<Server> s = std::make_shared<Server>(io_context_);
     
    s->set_handlers(handler_obj);
    number_servers_++;
    servers_created_[number_servers_] = s;

    std::stringstream ss;
    ss << "Start listening to Server (" << number_servers_ << ") at " << endpoint.address().to_string() << ":" << endpoint.port();
    Logger::log_message(Logger::Type::INFO, ss.str());
    s->start_listening(endpoint);

    return number_servers_;
}

void Communication_Manager::close_connection_to_server(const int &n)
{
    std::stringstream ss;
    ss << "Closing connection to Server (" << n << ")";
    Logger::log_message(Logger::Type::INFO, ss.str());
    if (servers_created_.find(n) != servers_created_.end()) {
        servers_created_[n]->server_close();
    }
}

bool Communication_Manager::send_message_to_server(const int &n, const std::string &msg)
{
    if (servers_created_.find(n) != servers_created_.end()) {
        servers_created_[n]->deliver(msg);
        return true;
    } 
    return false;
}

