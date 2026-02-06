/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : SSH_Manager.h                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <string>
#include <optional>
#include "Logger.h"

class SSH_Manager {
public:
    SSH_Manager(const std::string &user, const std::string &host, 
                const std::string &password = "");

    ~SSH_Manager();

    bool execute_command(const std::string &command, std::string &output);
    bool test_connection();

private:
    std::string user_;
    std::string host_;
    std::string key_;
    std::string last_output_;
    bool has_password_;

    std::string build_ssh_command(const std::string &remote_command) const;
};
