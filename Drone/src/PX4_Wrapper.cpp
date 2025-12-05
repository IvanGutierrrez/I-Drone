/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : PX4_Wrapper.cpp                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "PX4_Wrapper.h"
#include "common_libs/Logger.h"


PX4_Wrapper::PX4_Wrapper(const Struct_Drone::Config_struct &config) : Engine(config)
{
}

void PX4_Wrapper::start_engine()
{
    if (std::system(config_.command_px4.c_str()) != 0) {
        Logger::log_message(Logger::Type::ERROR, "Error starting PX4 Sim");
        return;
    }
    //TODO
}

void PX4_Wrapper::send_command(const std::string &command)
{
    // TODO Check if the message make sense
    // TODO Send
}
