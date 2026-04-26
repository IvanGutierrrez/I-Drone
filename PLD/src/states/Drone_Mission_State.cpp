/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Drone_Mission_State.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Drone_Mission_State.h"
#include "../State_Machine.h"
#include "Off_State.h"
#include "common_libs/Logger.h"
#include "common_libs/Enc_Dec_Drone.h"
#include "common_libs/Enc_Dec_PLD.h"
#include "structs/Structs_Planner.h"

constexpr int RATE_WAIT_FOR_MESSAGE = 300;
constexpr int RATE_FOR_SEND_MESSAGES = 1;
constexpr int NUMBER_ATTEMPS_MAX = 3;

Drone_Mission_State::Drone_Mission_State(std::shared_ptr<State_Machine> state_machine_ptr): State(state_machine_ptr),
                                                                                            wait_timer_(state_machine()->get_io_context()),
                                                                                            send_timer_(state_machine()->get_io_context())

{
}

Drone_Mission_State::~Drone_Mission_State() noexcept
{
    try {
        close_state();
    } catch (const std::exception &e) {
        Logger::log_message(Logger::Type::ERROR, std::string("Exception in ~Drone_Mission_State: ") + e.what());
    } catch (...) {
        Logger::log_message(Logger::Type::ERROR, "Unknown exception in ~Drone_Mission_State");
    }
}

void Drone_Mission_State::close_state()
{
    state_closing_ = true;
    wait_timer_.cancel();
    send_timer_.cancel();
    if (server_number_ != -1) {
        state_machine()->getCommunicationManager()->close_connection_to_server(server_number_);
        server_number_ = -1;
    }

    if(docker_manager_ && docker_manager_->is_container_running(config_.drone_module_data.docker_name)) {
        docker_manager_->stop_container(config_.drone_module_data.docker_name);
    }
}

void Drone_Mission_State::start()
{
    Logger::log_message(Logger::Type::INFO, "Entering Drone Mission State");
    state_machine()->getCommunicationManager()->set_status(Structs_PLD::Status::EXECUTING_MISSION);
    
    if (state_machine()->getRecorder()) {
        state_machine()->getRecorder()->write_state_transition("Planner_State", "Drone_Mission_State");
    }
    docker_manager_ = std::make_shared<Docker_Manager>(config_.drone_module_data.user,config_.drone_module_data.ssh_ip,config_.drone_module_data.docker_file,config_.drone_module_data.key);
    if (docker_manager_->test_connection()){
        std::stringstream log;
        log << "Connection successful to " << config_.drone_module_data.user << ":" << config_.drone_module_data.ssh_ip;
        Logger::log_message(Logger::Type::INFO, log.str());
    } else {
        std::stringstream log;
        log << "Unable to connect to " << config_.drone_module_data.user << ":" << config_.drone_module_data.ssh_ip << ". Transitioning to Off State";
        Logger::log_message(Logger::Type::ERROR, log.str());
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
        return;
    }

    Server::handlers handler_obj;
    handler_obj.call_error = [this](const boost::system::error_code& ec, const Type_Error &type_error) {
        on_error_drone(ec, type_error);
    };
    handler_obj.call_connect = [this]() {
        on_connect_drone();
    };
    handler_obj.call_message = [this](const std::string& msg) {
        on_message_drone(msg);
    };

    server_number_ = state_machine()->getCommunicationManager()->create_server(handler_obj,config_.drone_module_data.module_ip,config_.drone_module_data.port);

    if (server_number_ == -1) {
        Logger::log_message(Logger::Type::ERROR, "Unable to complete transition to Drone State, returning to Off State");
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
        return;
    }

    docker_manager_->start_container(config_.drone_module_data.docker_name);

    wait_timer_.expires_after(std::chrono::seconds(RATE_WAIT_FOR_MESSAGE));
    wait_timer_.async_wait([this](const boost::system::error_code& ec) {
        continue_start_process(ec);
    });
}

void Drone_Mission_State::end()
{
    Logger::log_message(Logger::Type::INFO, "Drone State functionality complete, transitioning to Off State to wait until next mission");
    
    close_state();
    auto off_state = std::make_unique<Off_State>(state_machine());
    state_machine()->transitionTo(std::move(off_state));
}

void Drone_Mission_State::continue_start_process(const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted) return;
    if (ec) {
        Logger::log_message(Logger::Type::ERROR, "Error in timer to continue start process in Drone State, transitioning to Off State");
        close_state();
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
        return;
    }

    Logger::log_message(Logger::Type::INFO, "Continuing start process in Drone State");
    
    if (!drone_module_running_){
        Logger::log_message(Logger::Type::WARNING, "Drone Module module is not running");
        attemps_++;
        if (attemps_ <= NUMBER_ATTEMPS_MAX) {
            if (server_number_ != -1){
                state_machine()->getCommunicationManager()->close_connection_to_server(server_number_);
                server_number_ = -1;
            }

            Server::handlers handler_obj;
            handler_obj.call_error = [this](const boost::system::error_code& ec, const Type_Error &type_error) {
                on_error_drone(ec, type_error);
            };
            handler_obj.call_connect = [this]() {
                on_connect_drone();
            };
            handler_obj.call_message = [this](const std::string& msg) {
                on_message_drone(msg);
            };

            server_number_ = state_machine()->getCommunicationManager()->create_server(handler_obj,config_.drone_module_data.module_ip,config_.drone_module_data.port);
            Logger::log_message(Logger::Type::INFO, "Retrying to start Drone module");
            wait_timer_.expires_after(std::chrono::seconds(RATE_WAIT_FOR_MESSAGE));
            wait_timer_.async_wait([this](const boost::system::error_code& ec) {
                continue_start_process(ec);
            });
            return;
        } else {
            Logger::log_message(Logger::Type::ERROR,"Number of allowed attempts exceeded. Transitioning to off state");
            close_state();
            auto off_state = std::make_unique<Off_State>(state_machine());
            state_machine()->transitionTo(std::move(off_state));
            return;
        }
    }

    if (last_status_ == Struct_Drone::Status::UNKNOWN) {
        attemps_++;
        if (attemps_ <= NUMBER_ATTEMPS_MAX) {
            wait_timer_.expires_after(std::chrono::seconds(RATE_WAIT_FOR_MESSAGE));
            wait_timer_.async_wait([this](const boost::system::error_code& ec) {
                continue_start_process(ec);
            });
            return;
        } else {
            Logger::log_message(Logger::Type::ERROR,"Number of allowed attempts exceeded. Transitioning to off state");
            close_state();
            auto off_state = std::make_unique<Off_State>(state_machine());
            state_machine()->transitionTo(std::move(off_state));
            return;
        }
    }

    if (config_.coor_points.empty()) {
        Logger::log_message(Logger::Type::ERROR,"There is no coordinate to send to Drone Module. Transitioning to off state");
        close_state();
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
        return;
    }

    Logger::log_message(Logger::Type::INFO, "Starting to send positions messages to Drone Module");

    drone_i_ = 0;
    coor_j_ = 0;

    send_timer_.expires_after(std::chrono::seconds(RATE_FOR_SEND_MESSAGES));
    send_timer_.async_wait([this](const boost::system::error_code& ec) {
        send_message(ec);
    });
}

void Drone_Mission_State::prepare_next_drone_message(Struct_Planner::Coordinate& coor, std::string& type)
{
    if (drone_i_ >= config_.coor_points.size()) {
        return;
    }

    if (coor_j_ >= config_.coor_points[drone_i_].size()) {
        drone_i_++;
        if (drone_i_ >= config_.coor_points.size()) {
            Logger::log_message(Logger::Type::INFO,"Starting all drones");
            coor = Struct_Planner::Coordinate(0,0);
            type = "START_ALL";
            return;
        }

        coor_j_ = 0;

        coor = config_.coor_points[drone_i_][0];
        if (1 == config_.coor_points[drone_i_].size()) {// If only have 1 coordinate add another one slightly offset for FINISH
            config_.coor_points[drone_i_].push_back(coor);
        }
        std::stringstream log;
        log << "Sending coordinate "<< coor <<" number " << coor_j_ << " to drone " << drone_i_;
        Logger::log_message(Logger::Type::INFO,log.str());
        type = "START";
        coor_j_ = 1;
        return;
    }

    coor = config_.coor_points[drone_i_][coor_j_];
    std::stringstream log;
    log << "Sending coordinate "<< coor <<" number " << coor_j_ << " to drone " << drone_i_;
    Logger::log_message(Logger::Type::INFO,log.str());
    if (coor_j_ == 0) {
        type = "START";
        if (coor_j_ == config_.coor_points[drone_i_].size() - 1) {// If only have 1 coordinate add another one slightly offset for FINISH
            config_.coor_points[drone_i_].push_back(coor);
        }
    } else if (coor_j_ == config_.coor_points[drone_i_].size() - 1) {
        type = "FINISH";
    } else {
        type = std::string();
    }

    coor_j_++;
}

void Drone_Mission_State::send_message(const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted) return;
    if (ec) {
        Logger::log_message(Logger::Type::ERROR, "Error in timer to continue start process in Drone State, transitioning to Off State");
        close_state();
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
        return;
    }

    Struct_Planner::Coordinate coor;
    std::string type;
    prepare_next_drone_message(coor, type);

    std::string message_to_drone;

    if (!Enc_Dec_Drone::create_message_to_drone(config_.drone_sim,coor,type,message_to_drone)) {
        std::stringstream log;
        log << "Unable to encode configuration message to Drone Module (" << config_.drone_sim << "). Transitioning to off state";
        Logger::log_message(Logger::Type::ERROR,log.str());
        close_state();
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
        return;
    }

    if (!state_machine()->getCommunicationManager()->send_message_to_server(server_number_,message_to_drone)){
        Logger::log_message(Logger::Type::ERROR,"Unable to send configuration message to Drone Module. Transitioning to off state");
        if (state_machine()->getRecorder()) {
            state_machine()->getRecorder()->write_error("Unable to send message to Drone Module");
        }
        close_state();
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
        return;
    }
    
    if (state_machine()->getRecorder()) {
        std::stringstream coord_info;
        coord_info << "Type: " << type << ", Coordinate: (" << coor.lat << ", " << coor.lon << ")";
        state_machine()->getRecorder()->write_message_sent("Drone", "COORDINATE", coord_info.str());
    }

    if (type == "START_ALL")
        return;

    send_timer_.expires_after(std::chrono::seconds(RATE_FOR_SEND_MESSAGES));
    send_timer_.async_wait([this](const boost::system::error_code& ec) {
        send_message(ec);
    });
}

void Drone_Mission_State::on_connect_drone()
{
    Logger::log_message(Logger::Type::INFO, "Successful connection to Drone Module");
    drone_module_running_ = true;
    attemps_ = 0;
}

void Drone_Mission_State::on_error_drone(const boost::system::error_code& ec, const Type_Error &type_error)
{
    if (state_closing_) {
        return;
    }

    wait_timer_.cancel();
    send_timer_.cancel();
    if (last_status_ == Struct_Drone::Status::FINISH) {
        Logger::log_message(Logger::Type::INFO,"Drone module task complete, closing connection");
        if (server_number_ != -1){
            state_machine()->getCommunicationManager()->close_connection_to_server(server_number_);
            server_number_ = -1;
        }
        return;
    }

    Logger::log_message(Logger::Type::WARNING, "on_error callback triggered in Drone connection");

    if (!docker_manager_->is_container_running(config_.drone_module_data.docker_name,false)) {
        Logger::log_message(Logger::Type::ERROR, "Drone docker is not running, transitioning to off state");
        close_state();
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
        return;
    }

    std::string log;
    switch (type_error){
        case Type_Error::CONNECTING:
            log = "Error connecting to Drone Module";
            break;
        case Type_Error::READING:
            log = "Error while reading a message from Drone Module";
            break;
        case Type_Error::SENDING:
            log = "Error while sending a message to Drone Module";
            break;
        default:
            log = "Unknown error communicating with Drone Module";
            break;
    }
    Logger::log_message(Logger::Type::WARNING,log + ": " + ec.message());
    
    attemps_++;
    if (attemps_ <= NUMBER_ATTEMPS_MAX) {
        Server::handlers handler_obj;
        handler_obj.call_error = [this](const boost::system::error_code& ec, const Type_Error &type_error) {
            on_error_drone(ec, type_error);
        };
        handler_obj.call_connect = [this]() {
            on_connect_drone();
        };
        handler_obj.call_message = [this](const std::string& msg) {
            on_message_drone(msg);
        };
        drone_module_running_ = false;

        server_number_ = state_machine()->getCommunicationManager()->create_server(handler_obj,config_.drone_module_data.module_ip,config_.drone_module_data.port);

        wait_timer_.expires_after(std::chrono::seconds(RATE_WAIT_FOR_MESSAGE));
        wait_timer_.async_wait([this](const boost::system::error_code& ec) {
            continue_start_process(ec);
        });
    } else {
        Logger::log_message(Logger::Type::ERROR,"Number of allowed attempts exceeded. Transitioning to off state");
        close_state();
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
    }
}

void Drone_Mission_State::on_message_drone(const std::string& msg)
{
    if (!drone_module_running_)
        drone_module_running_ = true;

    Logger::log_message(Logger::Type::INFO, "Message received from Drone module");

    std::string data = msg.substr(4);

    auto [type, decoded_msg] = Enc_Dec_PLD::decode_from_drone(data);
    if (type != Enc_Dec_PLD::PLD::STATUS_DRONE) {
        Logger::log_message(Logger::Type::WARNING, "Unable to decode Drone module message");
        if (state_machine()->getRecorder()) {
            state_machine()->getRecorder()->write_error("Unable to decode Drone module message");
            state_machine()->getRecorder()->write_raw_message("Drone", msg);
        }
        return;
    }

    auto my_msg = dynamic_cast<Status*>(decoded_msg.get());
    if (!my_msg || last_status_ == Struct_Drone::to_enum(my_msg->type_status())) {
        return;
    }

    if (state_machine()->getRecorder()) {
        state_machine()->getRecorder()->write_message_received("Drone", "STATUS", my_msg->type_status());
    }

    if (my_msg->type_status() == "ERROR"){
        Logger::log_message(Logger::Type::ERROR, "Drone module status has changed to ERROR, transitioning to off state");
        if (state_machine()->getRecorder()) {
            state_machine()->getRecorder()->write_error("Drone module status ERROR");
            state_machine()->getRecorder()->write_state_transition("Drone_Mission_State", "Off_State");
        }
        close_state();
        auto off_state = std::make_unique<Off_State>(state_machine());
        state_machine()->transitionTo(std::move(off_state));
        return;
    }

    last_status_ = Struct_Drone::to_enum(my_msg->type_status());
    Logger::log_message(Logger::Type::INFO, "Drone module status has changed to " + to_string(last_status_));
    if (last_status_ == Struct_Drone::Status::FINISH){
        end();
        return;
    }
}

void Drone_Mission_State::set_data(const Structs_PLD::Config_drone &config)
{
    config_ = config;
}

const char* Drone_Mission_State::state_name() const
{
    return "Drone Mission State";
}

void Drone_Mission_State::handle_finish_command()
{
    Logger::log_message(Logger::Type::WARNING, "FINISH command received in Drone Mission State, transitioning to Off State");
    close_state();
    auto off_state = std::make_unique<Off_State>(state_machine());
    state_machine()->transitionTo(std::move(off_state));
}