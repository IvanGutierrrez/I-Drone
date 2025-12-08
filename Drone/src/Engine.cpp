/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Engine.cpp                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Engine.h"
#include "common_libs/Logger.h"

Engine::~Engine()
{
    Logger::log_message(Logger::Type::INFO, "Engine destroyed");
}

void Engine::start_engine()
{
}

void Engine::send_command(const std::string &command)
{
}

void Engine::set_handler(Handlers f)
{
    handlers_ = f;
}