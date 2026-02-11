/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Docker_Manager.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Docker_Manager.h"
#include <sstream>
#include "common_libs/Logger.h"

Docker_Manager::Docker_Manager(const std::string &user, 
                               const std::string &host, 
                               const std::string &file, 
                               const std::string &key): ssh_manager_(std::make_shared<SSH_Manager>(user, host, key)),
                                                        compose_file_(file)
{
}

Docker_Manager::~Docker_Manager() 
{
}

bool Docker_Manager::start_container(const std::string &container_name)
{
    std::string cmd = "docker compose -f " + compose_file_ + " up -d " + container_name;
    std::string output;
    bool success = ssh_manager_->execute_command(cmd,output);
    
    if (success) {
        Logger::log_message(Logger::Type::INFO, "Container '" + container_name + "' started successfully");
    } else {
        Logger::log_message(Logger::Type::ERROR, "Failed to start container '" + container_name + "'");
    }
    
    return success;
}

bool Docker_Manager::stop_container(const std::string &container_name)
{
    std::string cmd = "docker rm -f " + container_name;
    std::string output;
    bool success = ssh_manager_->execute_command(cmd,output);
    
    if (success) {
        Logger::log_message(Logger::Type::INFO, "Container '" + container_name + "' stopped successfully");
    } else {
        Logger::log_message(Logger::Type::ERROR, "Failed to stop container '" + container_name + "'");
    }
    
    return success;
}

bool Docker_Manager::is_container_running(const std::string &container_name, const bool &silent)
{
    std::string cmd = "docker compose -f " + compose_file_ + " ps --services --filter \"status=running\"";
    std::string output;
    bool success = ssh_manager_->execute_command(cmd,output);
    
    if (!success) {
        Logger::log_message(Logger::Type::ERROR, "Failed to check container status");
        return false;
    }

    bool is_running = (output.find(container_name) != std::string::npos);
    
    if (!silent) {
        if (is_running) {
            Logger::log_message(Logger::Type::INFO, "Container '" + container_name + "' is running");
        } else {
            Logger::log_message(Logger::Type::INFO, "Container '" + container_name + "' is not running");
        }
    }
    
    return is_running;
}

bool Docker_Manager::test_connection()
{
    return ssh_manager_->test_connection();
}
