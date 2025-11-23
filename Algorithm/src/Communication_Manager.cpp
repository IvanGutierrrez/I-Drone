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
#include "common_libs/Enc_Dec_Algo.h"
#include "common_libs/Enc_Dec_PLD.h"

constexpr int NUMBER_ATTEMPS_MAX = 10;
constexpr int RATE_STATUS_MESSAGE = 1;

Communication_Manager::Communication_Manager(boost::asio::io_context& io_context, 
                                             const tcp::endpoint& endpoint): io_context_(io_context), 
                                                                             server_(io_context),
                                                                             endpoint_(endpoint),
                                                                             status_timer(io_context_),
                                                                             status_(Struct_Algo::Status::EXPECTING_DATA)
{
    Server::handlers handler_obj;

    handler_obj.call_error = std::bind(&Communication_Manager::on_error, this, std::placeholders::_1, std::placeholders::_2);
    handler_obj.call_connect = std::bind(&Communication_Manager::on_connect, this);
    handler_obj.call_message = std::bind(&Communication_Manager::on_message, this, std::placeholders::_1);
    server_.set_handlers(handler_obj);

    Logger::log_message(Logger::TYPE::INFO,"Start listening to PLD");

    server_.connect(endpoint_);
}

void Communication_Manager::on_connect()
{
    attemps_ = 0;
    Logger::log_message(Logger::TYPE::INFO,"Succesfully connected to PLD");
    boost::system::error_code ec;
    send_status_message(ec);
}

void Communication_Manager::send_status_message(const boost::system::error_code& ec)
{
    if (ec) {
        Logger::log_message(Logger::TYPE::WARNING,"Problems with timer to send status message to PLD module");
        return;
    }   

    Logger::log_message(Logger::TYPE::INFO,"Sending status message");

    std::string message;
    if (!Enc_Dec_PLD::encode_status_algo(status_,message)) {
        Logger::log_message(Logger::TYPE::WARNING,"Problems encoding status message");
    } else {
        server_.deliver(message);
    }
    status_timer.expires_after(std::chrono::seconds(RATE_STATUS_MESSAGE));
    status_timer.async_wait(std::bind(&Communication_Manager::send_status_message, this, std::placeholders::_1));
}

void Communication_Manager::set_status(const Struct_Algo::Status &new_status)
{
    status_ = new_status;
}

void Communication_Manager::on_error(const boost::system::error_code& ec, const Type_Error &type_error)
{
    if (status_ == Struct_Algo::Status::FINISH) {
        Logger::log_message(Logger::TYPE::INFO,"Program task complete, leaving program...");
        io_context_.stop();
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
    Logger::log_message(Logger::TYPE::WARNING,log + ": " + ec.message());
    attemps_++;
    if (attemps_ <= NUMBER_ATTEMPS_MAX)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        Logger::log_message(Logger::TYPE::INFO,"Trying to connect to PLD");
        server_.connect(endpoint_);
    } else 
    {
        Logger::log_message(Logger::TYPE::ERROR,"Number of allowed attempts exceeded. Exiting program...");
        io_context_.stop();
    }
}

void Communication_Manager::set_calculate_handler(const calculate_handler &handler)
{
    calculate_handler_ = std::move(handler);
}

void Communication_Manager::on_message(const std::string& msg)
{
    Logger::log_message(Logger::TYPE::INFO, "Message received from PLD");

    if (status_ != Struct_Algo::Status::EXPECTING_DATA) {
        Logger::log_message(Logger::TYPE::WARNING, "Task in progress or done, message ignore");
        return;
    }

    std::string data = msg.substr(4);

    auto [type, decoded_msg] = Enc_Dec_Algo::decode_to_algo(data);

    switch (type) {
        case Enc_Dec_Algo::Algo::UNKNOWN:
            Logger::log_message(Logger::TYPE::WARNING, "Unknown message received");
            break;
        case Enc_Dec_Algo::Algo::ERROR:
            Logger::log_message(Logger::TYPE::WARNING, "Error decoding message");
            break;
        case Enc_Dec_Algo::Algo::ConfigMessage: {
            auto my_msg = dynamic_cast<AlgorithmMessage*>(decoded_msg.get());
            if (my_msg) {
                Logger::log_message(Logger::TYPE::INFO, "Configuration message received");

                Struct_Algo::SignalServerConfig signal_server;
                if (!Enc_Dec_Algo::decode_signal_server(my_msg->signal_server_config(), signal_server))
                {
                    Logger::log_message(Logger::TYPE::WARNING, "Unabled to decode Signal-Server message");
                    return;
                }

                Struct_Algo::DroneData drone_data;
                if (!Enc_Dec_Algo::decode_drone_data(my_msg->drone_data(), drone_data))
                {
                    Logger::log_message(Logger::TYPE::WARNING, "Unabled to decode drone data message");
                    return;
                }
                calculate_handler_(signal_server,drone_data);
            }
            break;
        }
        default:
            Logger::log_message(Logger::TYPE::WARNING, "Unhandled message type");
            break;
    }
}

void Communication_Manager::deliver(const std::string &msg)
{
    server_.deliver(msg);
}
