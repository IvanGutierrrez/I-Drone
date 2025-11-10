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
#include "common_libs/Enc_Dec_Msg.h"

constexpr int NUMBER_ATTEMPS_MAX = 10;

Communication_Manager::Communication_Manager(boost::asio::io_context& io_context, 
                                             const tcp::endpoint& endpoint): io_context_(io_context), 
                                                                             server_(io_context),
                                                                             endpoint_(endpoint)
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
    Logger::log_message(Logger::TYPE::INFO,"Succesfully connected to PLD");
}

void Communication_Manager::on_error(const boost::system::error_code& ec, const Type_Error &type_error)
{
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
    Logger::log_message(Logger::TYPE::WARNING,log + ": " + ec.what());
    attemps_++;
    if (attemps_ <= NUMBER_ATTEMPS_MAX)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        Logger::log_message(Logger::TYPE::INFO,"Trying to connect to PLD");
        server_.connect(endpoint_);
    } else 
    {
        Logger::log_message(Logger::TYPE::WARNING,"Number of allowed attempts exceeded. Exiting program...");
        io_context_.stop();
    }
}

void Communication_Manager::on_message(const std::string& msg)
{
    Logger::log_message(Logger::TYPE::INFO, "Message received from PLD");

    std::string data = msg.substr(4);

    auto [type, decoded_msg] = Enc_Dec::decode_to_algo(data);

    switch (type) {
        case Enc_Dec::Algo::UNKNOWN:
            Logger::log_message(Logger::TYPE::WARNING, "Unknown message received");
            break;
        case Enc_Dec::Algo::ERROR:
            Logger::log_message(Logger::TYPE::WARNING, "Error decoding message");
            break;
        case Enc_Dec::Algo::MyMessage: {
            auto my_msg = dynamic_cast<MyMessage*>(decoded_msg.get());
            if (my_msg) {
                std::stringstream log;
                log << "Message MyMessage decoded: name " << my_msg->name() << ", id " << my_msg->id();
                Logger::log_message(Logger::TYPE::INFO, log.str());
                // Process message
            }
            break;
        }
        default:
            Logger::log_message(Logger::TYPE::WARNING, "Unhandled message type");
            break;
    }
}
