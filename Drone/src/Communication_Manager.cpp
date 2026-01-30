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
#include "common_libs/Enc_Dec_Drone.h"

constexpr int NUMBER_ATTEMPS_MAX = 10;
constexpr int RATE_STATUS_MESSAGE = 1;

Communication_Manager::Communication_Manager(boost::asio::io_context& io_context, 
                                             const tcp::endpoint& endpoint,
                                             const std::shared_ptr<Drone_Recorder> &rec_mng): io_context_(io_context),
                                                                                              server_(io_context),
                                                                                              endpoint_(endpoint),
                                                                                              recorder_ptr_(std::move(rec_mng)),
                                                                                              status_timer(io_context_),
                                                                                              status_(Struct_Drone::Status::STARTING_SIM)
{
    Server::handlers handler_obj;

    handler_obj.call_error = std::bind(&Communication_Manager::on_error, this, std::placeholders::_1, std::placeholders::_2);
    handler_obj.call_connect = std::bind(&Communication_Manager::on_connect, this);
    handler_obj.call_message = std::bind(&Communication_Manager::on_message, this, std::placeholders::_1);
    server_.set_handlers(handler_obj);

    std::stringstream ss;
    ss << "Start connecting to PLD at " << endpoint_.address().to_string() << ":" << endpoint_.port();
    Logger::log_message(Logger::Type::INFO, ss.str());

    server_.connect(endpoint_);
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
    status_timer.cancel(ec);
    
    if (retry_timer_) {
        retry_timer_->cancel(ec);
    }
    
    server_.server_close();
}


void Communication_Manager::on_connect()
{
    attemps_ = 0;
    Logger::log_message(Logger::Type::INFO,"Succesfully connected to PLD");
    boost::system::error_code ec;
    send_status_message(ec);
}

void Communication_Manager::send_status_message(const boost::system::error_code& ec)
{
    if (shutting_down_ || ec == boost::asio::error::operation_aborted) {
        return;
    }
    
    if (ec) {
        Logger::log_message(Logger::Type::WARNING,"Problems with timer to send status message to PLD module");
        return;
    }   

    Logger::log_message(Logger::Type::INFO,"Sending status message");

    std::string message;
    if (!Enc_Dec_Drone::encode_status_drone(get_status(),message)) {
        Logger::log_message(Logger::Type::WARNING,"Problems encoding status message");
    } else {
        deliver(message);
    }
    
    if (!shutting_down_) {
        status_timer.expires_after(std::chrono::seconds(RATE_STATUS_MESSAGE));
        status_timer.async_wait(std::bind(&Communication_Manager::send_status_message, this, std::placeholders::_1));
    }
}

void Communication_Manager::set_status(const Struct_Drone::Status &new_status)
{
    std::lock_guard<std::mutex> lock(mutex_status_);
    status_ = new_status;
}

void Communication_Manager::on_error(const boost::system::error_code& ec, const Type_Error &type_error)
{    
    if (shutting_down_) return;
    
    boost::system::error_code cancel_ec;
    status_timer.cancel(cancel_ec);
    
    if (get_status() == Struct_Drone::Status::FINISH) {
        Logger::log_message(Logger::Type::INFO,"Program task complete, leaving program...");
        io_context_.stop();
        return;
    } else if (get_status() == Struct_Drone::Status::ERROR) {
        Logger::log_message(Logger::Type::ERROR, "Error detected, shutting down program...");
        io_context_.stop();
        return;
    }

    Logger::log_message(Logger::Type::WARNING, "on_error callback triggered");
    std::string log;
    switch (type_error){
        case Type_Error::CONNECTING:
            log = "Error connecting to PLD";
            break;
        case Type_Error::READING:
            log = "Error while reading a message from PLD";
            break;
        case Type_Error::SENDING:
            log = "Error while sending a message to PLD";
            break;
        default:
            log = "Unknown error communicating with PLD";
            break;
    }
    Logger::log_message(Logger::Type::WARNING,log + ": " + ec.message());
    
    if (shutting_down_) {
        Logger::log_message(Logger::Type::INFO,"Shutting down, skipping reconnection");
        return;
    }
    
    attemps_++;
    if (attemps_ <= NUMBER_ATTEMPS_MAX)
    {
        retry_timer_ = std::make_shared<boost::asio::steady_timer>(io_context_, std::chrono::seconds(1));
        retry_timer_->async_wait([this](const boost::system::error_code& ec) {
            if (ec || shutting_down_) return;
            Logger::log_message(Logger::Type::INFO,"Trying to reconnect to PLD");
            server_.connect(endpoint_);
        });
    } else 
    {
        Logger::log_message(Logger::Type::ERROR,"Number of allowed attempts exceeded. Exiting program...");
        io_context_.stop();
    }
}

void Communication_Manager::set_message_handler(const message_handler &handler)
{
    message_handler_ = std::move(handler);
}

void Communication_Manager::on_message(const std::string& msg)
{    
    if (shutting_down_) return;
    Logger::log_message(Logger::Type::INFO, "Message received from PLD");

    std::string data = msg.substr(4);

    if (message_handler_) {
        message_handler_(data);
    } else {
        Logger::log_message(Logger::Type::WARNING, "No message handler set");
    }
}

void Communication_Manager::deliver(const std::string &msg)
{
    std::lock_guard<std::mutex> lock(mutex_deliver_);
    server_.deliver(msg);
}

Struct_Drone::Status Communication_Manager::get_status()
{
    std::lock_guard<std::mutex> lock(mutex_status_);
    return status_;
}
