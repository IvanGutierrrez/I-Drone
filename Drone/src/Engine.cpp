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
    // Intentionally empty: Engine acts as an interface-like base class.
    // Concrete engine implementations must provide their own startup logic.
}

void Engine::send_command(const std::string &command)
{
    // Intentionally empty: Engine acts as an interface-like base class.
    // Concrete engine implementations must provide command handling.
}

void Engine::set_start_signal(std::shared_future<void> signal)
{
    start_signal_ = signal;
}

void Engine::mark_commands_ready()
{
    // Intentionally empty: Engine acts as an interface-like base class.
    // Concrete engine implementations must define readiness behavior.
}

void Engine::set_handler(Handlers f)
{
    handlers_ = f;
}

void Engine::flush_recorder()
{
    // Intentionally empty: Engine acts as an interface-like base class.
    // Concrete engine implementations should flush their recorder resources.
}