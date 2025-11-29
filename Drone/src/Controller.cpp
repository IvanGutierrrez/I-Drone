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
                       const Struct_Drone::Config_struct cnf) : comm_mng_ptr_(std::move(comm_mng)),
                                                                recorder_ptr_(std::move(rec_mng)),
                                                                global_config_(cnf)
{
    comm_mng_ptr_->set_message_handler(std::bind(&Controller::handler_message, this, std::placeholders::_1));
}

Controller::~Controller()
{
}

void Controller::handler_message(const std::string &message)
{
    // TODO Check if the message make sense
    Logger::log_message(Logger::Type::INFO, "Adding new commands to the list (" + message + ")");
    commands_.push_back(message);
}