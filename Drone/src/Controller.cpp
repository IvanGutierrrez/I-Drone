/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Controller.cpp                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Controller.h"
#include "common_libs/Logger.h"

Controller::Controller(std::shared_ptr<Communication_Manager> comm_mng, 
                       std::shared_ptr<Drone_Recorder> rec_mng,
                       std::shared_ptr<Engine> engine,
                       const Struct_Drone::Config_struct cnf) : comm_mng_ptr_(std::move(comm_mng)),
                                                                recorder_ptr_(std::move(rec_mng)),
                                                                engine_(std::move(engine)),
                                                                global_config_(cnf),
                                                                running_(true),
                                                                command_completed_(true),
                                                                first_complete_(false)
{
    comm_mng_ptr_->set_message_handler(std::bind(&Controller::handler_message, this, std::placeholders::_1));
    // TODO Set command_complete as a handler inside start engine
    engine_->start_engine();
    
}

Controller::~Controller()
{
    running_ = false;
    cv_add_command_.notify_one();
    cv_command_complete_.notify_one();
    
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void Controller::handler_message(const std::string &message)
{
    Logger::log_message(Logger::Type::INFO, "Adding new commands to the list (" + message + ")");
    
    {
        std::lock_guard<std::mutex> lock(commands_mutex_);
        commands_.push_back(message);
    }
    cv_add_command_.notify_one();
}

void Controller::process_commands()
{
    while (running_) {
        std::unique_lock<std::mutex> lock(commands_mutex_);
        
        cv_add_command_.wait(lock, [this] { return !commands_.empty() || !running_; });
        
        if (!running_) {
            break;
        }
        
        if (!commands_.empty()) {
            std::string command = commands_.front();
            commands_.erase(commands_.begin());
            lock.unlock();
            
            Logger::log_message(Logger::Type::INFO, "Processing command: " + command);
            command_completed_ = false;
            engine_->send_command(command);
            
            // Wait for command completion
            std::unique_lock<std::mutex> completion_lock(commands_mutex_);
            cv_command_complete_.wait(completion_lock, [this] { return command_completed_.load() || !running_; });
        }
    }
    
    Logger::log_message(Logger::Type::INFO, "Command processor thread stopped");
}

void Controller::command_complete()
{
    if (!first_complete_)
    {
        worker_thread_ = std::thread(&Controller::process_commands, this);
        first_complete_ = true;
    }
    Logger::log_message(Logger::Type::INFO, "Command completed, ready for next command");
    command_completed_ = true;
    cv_command_complete_.notify_one();
}