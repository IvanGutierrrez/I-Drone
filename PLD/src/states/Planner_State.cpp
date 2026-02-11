/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Planner_State.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Planner_State.h"
#include "Off_State.h"
#include "Drone_Mission_State.h"
#include "../State_Machine.h"
#include "common_libs/Logger.h"
#include "common_libs/Enc_Dec_Planner.h"
#include "common_libs/Enc_Dec_PLD.h"
#include "structs/Structs_Planner.h"

constexpr int RATE_WAIT_FOR_MESSAGE = 10;
constexpr int NUMBER_ATTEMPS_MAX = 3;

Planner_State::Planner_State(std::shared_ptr<State_Machine> state_machine_ptr): State(state_machine_ptr),
                                                                                server_number_(-1),
                                                                                wait_timer_(state_machine_ptr_->get_io_context()),
                                                                                planner_running_(false),
                                                                                response_message_received_(false),
                                                                                last_status_(Struct_Planner::Status::UNKNOWN),
                                                                                attemps_(0),
                                                                                state_closing_(false)
{
}

Planner_State::~Planner_State()
{
    wait_timer_.cancel();
    if (server_number_ != -1) {
        state_machine_ptr_->getCommunicationManager()->close_connection_to_server(server_number_);
        server_number_ = -1;
    }

    if(docker_manager_ && docker_manager_->is_container_running(config_.planner_module_data.docker_name)) {
        docker_manager_->stop_container(config_.planner_module_data.docker_name);
    }
}

void Planner_State::close_state()
{
    state_closing_ = true;
    wait_timer_.cancel();
    if (server_number_ != -1) {
        state_machine_ptr_->getCommunicationManager()->close_connection_to_server(server_number_);
        server_number_ = -1;
    }

    if(docker_manager_ && docker_manager_->is_container_running(config_.planner_module_data.docker_name)) {
        docker_manager_->stop_container(config_.planner_module_data.docker_name);
    }
}

void Planner_State::start()
{
    Logger::log_message(Logger::Type::INFO, "Entering Planner State");
    state_machine_ptr_->getCommunicationManager()->set_status(Structs_PLD::Status::PLANNING_MISSION);
    
    if (state_machine_ptr_->getRecorder()) {
        state_machine_ptr_->getRecorder()->write_state_transition("Off_State", "Planner_State");
    }
    docker_manager_ = std::make_shared<Docker_Manager>(config_.planner_module_data.user,config_.planner_module_data.ssh_ip,config_.planner_module_data.docker_file,config_.planner_module_data.key);
    if (docker_manager_->test_connection()){
        std::stringstream log;
        log << "Connection successful to " << config_.planner_module_data.user << ":" << config_.planner_module_data.ssh_ip;
        Logger::log_message(Logger::Type::INFO, log.str());
    } else {
        std::stringstream log;
        log << "Unable to connect to " << config_.planner_module_data.user << ":" << config_.planner_module_data.ssh_ip << ". Transitioning to Off State";
        Logger::log_message(Logger::Type::ERROR, log.str());
        std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
        state_machine_ptr_->transitionTo(std::move(off_state));
        return;
    }

    Server::handlers handler_obj;
    handler_obj.call_error = std::bind(&Planner_State::on_error_planner, this, std::placeholders::_1, std::placeholders::_2);
    handler_obj.call_connect = std::bind(&Planner_State::on_connect_planner, this);
    handler_obj.call_message = std::bind(&Planner_State::on_message_planner, this, std::placeholders::_1);

    server_number_ = state_machine_ptr_->getCommunicationManager()->create_server(handler_obj,config_.planner_module_data.module_ip,config_.planner_module_data.port);

    if (server_number_ == -1) {
        Logger::log_message(Logger::Type::ERROR, "Unable to complete transition to Planner State, returning to Off State");
        std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
        state_machine_ptr_->transitionTo(std::move(off_state));
        return;
    }

    docker_manager_->start_container(config_.planner_module_data.docker_name);

    wait_timer_.expires_after(std::chrono::seconds(RATE_WAIT_FOR_MESSAGE));
    wait_timer_.async_wait(std::bind(&Planner_State::continue_start_process, this, std::placeholders::_1));

}

void Planner_State::end()
{
    std::unique_ptr<Drone_Mission_State> mission_drone_state = std::make_unique<Drone_Mission_State>(state_machine_ptr_);
    mission_drone_state->set_data(data_next_state_);
    close_state();
    Logger::log_message(Logger::Type::INFO, "Planner State functionality complete, transitioning to the next state");
    
    state_machine_ptr_->transitionTo(std::move(mission_drone_state));
}

void Planner_State::handleMessage(const std::string &message)
{
    auto [type, proto_msg] = Enc_Dec_PLD::decode_from_client(message);
    
    if (type == Enc_Dec_PLD::PLD::CONFIG_MISSION) {
        Logger::log_message(Logger::Type::WARNING, "unexpected CONFIG MISSION message received in Planner State, ignoring");
        if (state_machine_ptr_->getRecorder()) {
            state_machine_ptr_->getRecorder()->write_message_received("Client", "CONFIG_MISSION", "Unexpected in Planner State");
        }
    } else if (type == Enc_Dec_PLD::PLD::COMMAND) {
        Command* command = dynamic_cast<Command*>(proto_msg.get());
        if (!command) {
            Logger::log_message(Logger::Type::WARNING, "Unable to decode command from Client");
            if (state_machine_ptr_->getRecorder()) {
                state_machine_ptr_->getRecorder()->write_error("Unable to decode COMMAND from Client");
                state_machine_ptr_->getRecorder()->write_raw_message("Client", message);
            }
            return;
        }

        if (state_machine_ptr_->getRecorder()) {
            state_machine_ptr_->getRecorder()->write_message_received("Client", "COMMAND", command->command());
        }

        if (command->command() == "FINISH") {
            Logger::log_message(Logger::Type::WARNING, "FINISH command received in Planner State, transitioning to Off State");
            close_state();
            std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
            state_machine_ptr_->transitionTo(std::move(off_state));
        } else {
            Logger::log_message(Logger::Type::WARNING, "Unexpected command received from Client: " + command->command());
            if (state_machine_ptr_->getRecorder()) {
                state_machine_ptr_->getRecorder()->write_error("Unexpected command: " + command->command());
            }
        }

    } else {
        Logger::log_message(Logger::Type::WARNING, "unexpected message received from Client, type: " + Enc_Dec_PLD::to_string(type));
        if (state_machine_ptr_->getRecorder()) {
            state_machine_ptr_->getRecorder()->write_error("Unexpected message type: " + Enc_Dec_PLD::to_string(type));
            state_machine_ptr_->getRecorder()->write_raw_message("Client", message);
        }
    }
}

void Planner_State::set_data(Structs_PLD::Config_mission config)
{
    config_ = config;
}

void Planner_State::continue_start_process(const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted) return;
    if (ec) {
        Logger::log_message(Logger::Type::ERROR, "Error in timer to continue start process in Planner State, transitioning to Off State");
        close_state();
        std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
        state_machine_ptr_->transitionTo(std::move(off_state));
        return;
    }

    Logger::log_message(Logger::Type::INFO, "Continuing start process in Planner State");
    
    if (!planner_running_){
        Logger::log_message(Logger::Type::WARNING, "Planner module is not running");
        attemps_++;
        if (attemps_ <= NUMBER_ATTEMPS_MAX) {
            if (server_number_ != -1){
                state_machine_ptr_->getCommunicationManager()->close_connection_to_server(server_number_);
                server_number_ = -1;
            }

            Server::handlers handler_obj;
            handler_obj.call_error = std::bind(&Planner_State::on_error_planner, this, std::placeholders::_1, std::placeholders::_2);
            handler_obj.call_connect = std::bind(&Planner_State::on_connect_planner, this);
            handler_obj.call_message = std::bind(&Planner_State::on_message_planner, this, std::placeholders::_1);

            server_number_ = state_machine_ptr_->getCommunicationManager()->create_server(handler_obj,config_.planner_module_data.module_ip,config_.planner_module_data.port);
            Logger::log_message(Logger::Type::INFO, "Retrying to start Planner module");
            wait_timer_.expires_after(std::chrono::seconds(RATE_WAIT_FOR_MESSAGE));
            wait_timer_.async_wait(std::bind(&Planner_State::continue_start_process, this, std::placeholders::_1));
            return;
        } else {
            Logger::log_message(Logger::Type::ERROR,"Number of allowed attempts exceeded. Transitioning to off state");
            close_state();
            std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
            state_machine_ptr_->transitionTo(std::move(off_state));
            return;
        }
    }

    if (last_status_ == Struct_Planner::Status::UNKNOWN) {
        attemps_++;
        if (attemps_ <= NUMBER_ATTEMPS_MAX) {
            wait_timer_.expires_after(std::chrono::seconds(RATE_WAIT_FOR_MESSAGE));
            wait_timer_.async_wait(std::bind(&Planner_State::continue_start_process, this, std::placeholders::_1));
            return;
        } else {
            Logger::log_message(Logger::Type::ERROR,"Number of allowed attempts exceeded. Transitioning to off state");
            close_state();
            std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
            state_machine_ptr_->transitionTo(std::move(off_state));
            return;
        }
    }

    Logger::log_message(Logger::Type::INFO, "Sending config message to Planner Module");

    std::string message_to_planner;

    if (!Enc_Dec_Planner::encode_config_message(config_.planner_info.signal_server_config,config_.planner_info.dron_data,message_to_planner)) {
        Logger::log_message(Logger::Type::ERROR,"Unable to encode configuration message to Planner. Transitioning to off state");
        close_state();
        std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
        state_machine_ptr_->transitionTo(std::move(off_state));
        return;
    }

    if (!state_machine_ptr_->getCommunicationManager()->send_message_to_server(server_number_,message_to_planner)){
        Logger::log_message(Logger::Type::ERROR,"Unable to send configuration message to Planner. Transitioning to off state");
        if (state_machine_ptr_->getRecorder()) {
            state_machine_ptr_->getRecorder()->write_error("Unable to send configuration message to Planner");
        }
        close_state();
        std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
        state_machine_ptr_->transitionTo(std::move(off_state));
        return;
    }
    
    if (state_machine_ptr_->getRecorder()) {
        state_machine_ptr_->getRecorder()->write_message_sent("Planner", "CONFIG_MESSAGE", "Configuration sent to Planner");
    }
}

void Planner_State::on_connect_planner()
{
    Logger::log_message(Logger::Type::INFO, "Successful connection to Planner Module");
    planner_running_ = true;
    attemps_ = 0;
}
    
void Planner_State::on_error_planner(const boost::system::error_code& ec, const Type_Error &type_error)
{
    if (state_closing_) {
        return;
    }

    wait_timer_.cancel();
    if (last_status_ == Struct_Planner::Status::FINISH) {
        Logger::log_message(Logger::Type::INFO,"Planner module task complete, closing connection");
        if (server_number_ != -1){
            state_machine_ptr_->getCommunicationManager()->close_connection_to_server(server_number_);
            server_number_ = -1;
        }
        return;
    }

    Logger::log_message(Logger::Type::WARNING, "on_error callback triggered in Planner connection");

    if (!docker_manager_->is_container_running(config_.planner_module_data.docker_name,false)) {
        Logger::log_message(Logger::Type::ERROR, "Planner docker is not running, transitioning to off state");
        close_state();
        std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
        state_machine_ptr_->transitionTo(std::move(off_state));
        return;
    }

    std::string log;
    switch (type_error){
        case Type_Error::CONNECTING:
            log = "Error connecting to Planner";
            break;
        case Type_Error::READING:
            log = "Error while reading a message from Planner";
            break;
        case Type_Error::SENDING:
            log = "Error while sending a message to Planner";
            break;
        default:
            log = "Unknown error communicating with Planner";
            break;
    }
    Logger::log_message(Logger::Type::WARNING,log + ": " + ec.message());
    
    attemps_++;
    if (attemps_ <= NUMBER_ATTEMPS_MAX) {
        Server::handlers handler_obj;
        handler_obj.call_error = std::bind(&Planner_State::on_error_planner, this, std::placeholders::_1, std::placeholders::_2);
        handler_obj.call_connect = std::bind(&Planner_State::on_connect_planner, this);
        handler_obj.call_message = std::bind(&Planner_State::on_message_planner, this, std::placeholders::_1);
        planner_running_ = false;

        server_number_ = state_machine_ptr_->getCommunicationManager()->create_server(handler_obj,config_.planner_module_data.module_ip,config_.planner_module_data.port);

        wait_timer_.expires_after(std::chrono::seconds(RATE_WAIT_FOR_MESSAGE));
        wait_timer_.async_wait(std::bind(&Planner_State::continue_start_process, this, std::placeholders::_1));
    } else {
        Logger::log_message(Logger::Type::ERROR,"Number of allowed attempts exceeded. Transitioning to off state");
        close_state();
        std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
        state_machine_ptr_->transitionTo(std::move(off_state));
    }
}
    
void Planner_State::on_message_planner(const std::string& msg)
{
    if (!planner_running_)
        planner_running_ = true;

    Logger::log_message(Logger::Type::INFO, "Message received from Planner");

    std::string data = msg.substr(4);

    auto [type, decoded_msg] = Enc_Dec_PLD::decode_from_planner(data);
    if (type == Enc_Dec_PLD::PLD::STATUS_Planner) {
        auto my_msg = dynamic_cast<Status*>(decoded_msg.get());
        if (my_msg && last_status_ != Struct_Planner::to_enum(my_msg->type_status())) {
            if (state_machine_ptr_->getRecorder()) {
                state_machine_ptr_->getRecorder()->write_message_received("Planner", "STATUS", my_msg->type_status());
            }
            if (my_msg->type_status() == "ERROR"){
                Logger::log_message(Logger::Type::ERROR, "Planner status has changed to ERROR, transitioning to off state");
                if (state_machine_ptr_->getRecorder()) {
                    state_machine_ptr_->getRecorder()->write_error("Planner status ERROR");
                    state_machine_ptr_->getRecorder()->write_state_transition("Planner_State", "Off_State");
                }
                close_state();
                std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
                state_machine_ptr_->transitionTo(std::move(off_state));
                return;
            }
            last_status_ = Struct_Planner::to_enum(my_msg->type_status());
            if (last_status_ == Struct_Planner::Status::FINISH && !response_message_received_){ //The next else if statement will transitionate to next state if the response is received
                Logger::log_message(Logger::Type::ERROR, "Planner status has changed to FINISH without receiving response message, transitioning to off state");
                if (state_machine_ptr_->getRecorder()) {
                    state_machine_ptr_->getRecorder()->write_error("Planner FINISH without response");
                    state_machine_ptr_->getRecorder()->write_state_transition("Planner_State", "Off_State");
                }
                close_state();
                std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
                state_machine_ptr_->transitionTo(std::move(off_state));
                return;
            }
            Logger::log_message(Logger::Type::INFO, "Planner status has changed to " + to_string(last_status_));
        }
    } else if (type == Enc_Dec_PLD::PLD::Planner_RESPONSE) {
        auto my_msg = dynamic_cast<PlannerResponseList*>(decoded_msg.get());
        if (my_msg) {
            Logger::log_message(Logger::Type::INFO, "Planner response received");
            if (state_machine_ptr_->getRecorder()) {
                state_machine_ptr_->getRecorder()->write_message_received("Planner", "PLANNER_RESPONSE", "Planner response received");
            }
            std::vector<std::vector<Struct_Planner::Coordinate>> result;
            if (!Enc_Dec_PLD::decode_planner_response(*my_msg, result)) {
                Logger::log_message(Logger::Type::ERROR, "Unable to decode planner response, transitioning to off state");
                if (state_machine_ptr_->getRecorder()) {
                    state_machine_ptr_->getRecorder()->write_error("Unable to decode planner response");
                    state_machine_ptr_->getRecorder()->write_state_transition("Planner_State", "Off_State");
                }
                close_state();
                std::unique_ptr<Off_State> off_state = std::make_unique<Off_State>(state_machine_ptr_);
                state_machine_ptr_->transitionTo(std::move(off_state));
                return;
            }
            response_message_received_ = true;
            data_next_state_.coor_points = result;
            data_next_state_.drone_module_data = config_.drone_module_data;
            data_next_state_.drone_sim = config_.drone_sim;
            
            // Transitionate to the next state
            end();
        }
    } else {
        Logger::log_message(Logger::Type::WARNING, "Unable to decode Planner message");
        if (state_machine_ptr_->getRecorder()) {
            state_machine_ptr_->getRecorder()->write_error("Unable to decode Planner message");
            state_machine_ptr_->getRecorder()->write_raw_message("Planner", msg);
        }
    }
}