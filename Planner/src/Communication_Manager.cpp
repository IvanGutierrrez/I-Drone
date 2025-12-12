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
#include "Planner_Recorder.h"
#include "common_libs/Logger.h"
#include "common_libs/Enc_Dec_Planner.h"
#include "common_libs/Enc_Dec_PLD.h"

constexpr int NUMBER_ATTEMPS_MAX = 10;
constexpr int RATE_STATUS_MESSAGE = 1;
constexpr int CALCULATION_POOL_SIZE = 1;

Communication_Manager::Communication_Manager(boost::asio::io_context& io_context, 
                                             const tcp::endpoint& endpoint,
                                             const std::shared_ptr<Planner_Recorder> &rec_mng): io_context_(io_context),
                                                                                                  calculation_pool_(CALCULATION_POOL_SIZE),
                                                                                                  server_(io_context),
                                                                                                  endpoint_(endpoint),
                                                                                                  recorder_ptr_(std::move(rec_mng)),
                                                                                                  status_timer(io_context_),
                                                                                                  status_(Struct_Planner::Status::EXPECTING_DATA)
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
    
    calculation_pool_.stop();
    Logger::log_message(Logger::Type::INFO, "Waiting for calculation threads to finish...");
    calculation_pool_.join();
    Logger::log_message(Logger::Type::INFO, "Calculation threads finished");
    
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
    if (!Enc_Dec_PLD::encode_status_planner(get_status(),message)) {
        Logger::log_message(Logger::Type::WARNING,"Problems encoding status message");
    } else {
        deliver(message);
    }
    
    if (!shutting_down_) {
        status_timer.expires_after(std::chrono::seconds(RATE_STATUS_MESSAGE));
        status_timer.async_wait(std::bind(&Communication_Manager::send_status_message, this, std::placeholders::_1));
    }
}

void Communication_Manager::set_status(const Struct_Planner::Status &new_status)
{
    std::lock_guard<std::mutex> lock(mutex_status_);
    status_ = new_status;
}

void Communication_Manager::on_error(const boost::system::error_code& ec, const Type_Error &type_error)
{    
    if (shutting_down_) return;
    Logger::log_message(Logger::Type::WARNING, "on_error callback triggered");
    
    boost::system::error_code cancel_ec;
    status_timer.cancel(cancel_ec);
    
    if (get_status() == Struct_Planner::Status::FINISH) {
        Logger::log_message(Logger::Type::INFO,"Program task complete, leaving program...");
        io_context_.stop();
        return;
    }
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

void Communication_Manager::set_calculate_handler(const calculate_handler &handler)
{
    calculate_handler_ = std::move(handler);
}

void Communication_Manager::on_message(const std::string& msg)
{    
    if (shutting_down_) return;
    Logger::log_message(Logger::Type::INFO, "Message received from PLD");

    if (status_ != Struct_Planner::Status::EXPECTING_DATA) {
        Logger::log_message(Logger::Type::WARNING, "Task in progress or done, message ignore");
        return;
    }

    std::string data = msg.substr(4);

    auto [type, decoded_msg] = Enc_Dec_Planner::decode_to_planner(data);

    switch (type) {
        case Enc_Dec_Planner::Planner::UNKNOWN:
            Logger::log_message(Logger::Type::WARNING, "Unknown message received");
            break;
        case Enc_Dec_Planner::Planner::ERROR:
            Logger::log_message(Logger::Type::WARNING, "Error decoding message");
            break;
        case Enc_Dec_Planner::Planner::ConfigMessage: {
            auto my_msg = dynamic_cast<PlannerMessage*>(decoded_msg.get());
            if (my_msg) {
                Logger::log_message(Logger::Type::INFO, "Configuration message received");

                Struct_Planner::SignalServerConfig signal_server;
                if (!Enc_Dec_Planner::decode_signal_server(my_msg->signal_server_config(), signal_server))
                {
                    Logger::log_message(Logger::Type::WARNING, "Unabled to decode Signal-Server message");
                    return;
                }

                Struct_Planner::DroneData drone_data;
                if (!Enc_Dec_Planner::decode_drone_data(my_msg->drone_data(), drone_data))
                {
                    Logger::log_message(Logger::Type::WARNING, "Unabled to decode drone data message");
                    return;
                }
                recorder_ptr_->write_message_received(signal_server,drone_data);
                
                boost::asio::post(calculation_pool_, [this, signal_server, drone_data]() {
                    if (!shutting_down_) {
                        calculate_handler_(signal_server, drone_data);
                    }
                });
            }
            break;
        }
        default:
            Logger::log_message(Logger::Type::WARNING, "Unhandled message type");
            break;
    }
}

void Communication_Manager::deliver(const std::string &msg)
{
    std::lock_guard<std::mutex> lock(mutex_deliver_);
    server_.deliver(msg);
}

Struct_Planner::Status Communication_Manager::get_status()
{
    std::lock_guard<std::mutex> lock(mutex_status_);
    return status_;
}
