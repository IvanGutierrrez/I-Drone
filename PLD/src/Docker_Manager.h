/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Docker_Manager.h                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "SSH_Manager.h"
#include <string>
#include <memory>
#include <filesystem>

class Docker_Manager {
public:
    Docker_Manager(const std::string &user, const std::string &host,
                   const std::string &compose_path, const std::string &key = "");

    ~Docker_Manager();

    bool start_container(const std::string &container_name);
    bool stop_container(const std::string &container_name);
    bool is_container_running(const std::string &container_name, const bool &silent = false);
    bool test_connection();

private:
    std::shared_ptr<SSH_Manager> ssh_manager_;
    std::string compose_file_;
};
