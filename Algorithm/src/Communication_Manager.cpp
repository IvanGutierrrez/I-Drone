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
#include "Algorithm_Recorder.h"
#include "common_libs/Logger.h"
#include "common_libs/Enc_Dec_Algo.h"
#include "common_libs/Enc_Dec_PLD.h"

constexpr int NUMBER_ATTEMPS_MAX = 10;
constexpr int RATE_STATUS_MESSAGE = 1;
constexpr int NUM_THREADS = 1;

Communication_Manager::Communication_Manager(boost::asio::io_context& io_context, 
                                             const tcp::endpoint& endpoint,
                                             const std::shared_ptr<Algorithm_Recorder> &rec_mng): io_context_(io_context), 
                                                                                                  server_(io_context),
                                                                                                  endpoint_(endpoint),
                                                                                                  recorder_ptr_(std::move(rec_mng)),
                                                                                                  status_timer(io_context_),
                                                                                                  status_(Struct_Algo::Status::EXPECTING_DATA),
                                                                                                  pool_(NUM_THREADS)
{
    Server::handlers handler_obj;

    handler_obj.call_error = std::bind(&Communication_Manager::on_error, this, std::placeholders::_1, std::placeholders::_2);
    handler_obj.call_connect = std::bind(&Communication_Manager::on_connect, this);
    handler_obj.call_message = std::bind(&Communication_Manager::on_message, this, std::placeholders::_1);
    server_.set_handlers(handler_obj);

    Logger::log_message(Logger::Type::INFO,"Start listening to PLD");

    server_.connect(endpoint_);
}

Communication_Manager::~Communication_Manager()
{
    status_timer.cancel();
    pool_.stop();
    pool_.join();
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
    if (ec) {
        Logger::log_message(Logger::Type::WARNING,"Problems with timer to send status message to PLD module");
        return;
    }   

    Logger::log_message(Logger::Type::INFO,"Sending status message");

    std::string message;
    if (!Enc_Dec_PLD::encode_status_algo(get_status(),message)) {
        Logger::log_message(Logger::Type::WARNING,"Problems encoding status message");
    } else {
        deliver(message);
    }
    status_timer.expires_after(std::chrono::seconds(RATE_STATUS_MESSAGE));
    status_timer.async_wait(std::bind(&Communication_Manager::send_status_message, this, std::placeholders::_1));
}

void Communication_Manager::set_status(const Struct_Algo::Status &new_status)
{
    std::lock_guard<std::mutex> lock(mutex_status_);
    status_ = new_status;
}

void Communication_Manager::on_error(const boost::system::error_code& ec, const Type_Error &type_error)
{
    status_timer.cancel();
    std::lock_guard<std::mutex> lock(mutex_status_);
    if (get_status() == Struct_Algo::Status::FINISH) {
        Logger::log_message(Logger::Type::INFO,"Program task complete, leaving program...");
        io_context_.stop();
        pool_.stop();
        pool_.join();
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
    attemps_++;
    if (attemps_ <= NUMBER_ATTEMPS_MAX)
    {
        Logger::log_message(Logger::Type::INFO,"Trying to connect to PLD");
        boost::asio::steady_timer retry_timer(io_context_, std::chrono::seconds(1));
        retry_timer.async_wait([this](const boost::system::error_code&) {
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
    Logger::log_message(Logger::Type::INFO, "Message received from PLD");

    if (status_ != Struct_Algo::Status::EXPECTING_DATA) {
        Logger::log_message(Logger::Type::WARNING, "Task in progress or done, message ignore");
        return;
    }

    std::string data = msg.substr(4);

    auto [type, decoded_msg] = Enc_Dec_Algo::decode_to_algo(data);

    switch (type) {
        case Enc_Dec_Algo::Algo::UNKNOWN:
            Logger::log_message(Logger::Type::WARNING, "Unknown message received");
            break;
        case Enc_Dec_Algo::Algo::ERROR:
            Logger::log_message(Logger::Type::WARNING, "Error decoding message");
            break;
        case Enc_Dec_Algo::Algo::ConfigMessage: {
            auto my_msg = dynamic_cast<AlgorithmMessage*>(decoded_msg.get());
            if (my_msg) {
                Logger::log_message(Logger::Type::INFO, "Configuration message received");

                Struct_Algo::SignalServerConfig signal_server;
                if (!Enc_Dec_Algo::decode_signal_server(my_msg->signal_server_config(), signal_server))
                {
                    Logger::log_message(Logger::Type::WARNING, "Unabled to decode Signal-Server message");
                    return;
                }

                Struct_Algo::DroneData drone_data;
                if (!Enc_Dec_Algo::decode_drone_data(my_msg->drone_data(), drone_data))
                {
                    Logger::log_message(Logger::Type::WARNING, "Unabled to decode drone data message");
                    return;
                }
                recorder_ptr_->write_message_received(signal_server,drone_data);
                boost::asio::post(pool_, [this, signal_server, drone_data]() {
                    calculate_handler_(signal_server, drone_data);
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

Struct_Algo::Status Communication_Manager::get_status()
{
    std::lock_guard<std::mutex> lock(mutex_status_);
    return status_;
}
