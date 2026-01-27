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
                       std::shared_ptr<Multi_Drone_Manager> drone_manager,
                       const Struct_Drone::Config_struct &cnf) : comm_mng_ptr_(std::move(comm_mng)),
                                                                recorder_ptr_(std::move(rec_mng)),
                                                                drone_manager_(std::move(drone_manager)),
                                                                global_config_(cnf)
{
    comm_mng_ptr_->set_message_handler(std::bind(&Controller::handler_message, this, std::placeholders::_1));

    Multi_Drone_Manager::Handlers handlers;
    handlers.all_missions_complete = std::bind(&Controller::mission_complete, this);
    handlers.error = std::bind(&Controller::error_processing_command, this);
    drone_manager_->set_handlers(handlers);
    drone_manager_->start_all();
}

Controller::~Controller()
{
}

void Controller::handler_message(const std::string &message)
{
    Logger::log_message(Logger::Type::INFO, "Adding new commands to the list");
    drone_manager_->dispatch_command(message);
}

void Controller::mission_complete()
{
    Logger::log_message(Logger::Type::INFO, "Mission completed, task finish");
    comm_mng_ptr_->set_status(Struct_Drone::Status::FINISH);
}

void Controller::error_processing_command()
{
    Logger::log_message(Logger::Type::ERROR, "Error processing command"); // TODO Think what to do
    comm_mng_ptr_->set_status(Struct_Drone::Status::ERROR);
}