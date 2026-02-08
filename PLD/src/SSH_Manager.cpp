/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : SSH_Manager.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "SSH_Manager.h"
#include <fstream>
#include <sstream>
#include <array>
#include <memory>
#include <cstdio>
#include "common_libs/Logger.h"

SSH_Manager::SSH_Manager(const std::string &user, 
                         const std::string &host,
                         const std::string &password): user_(user), 
                                                       host_(host),
                                                       key_(password),
                                                       has_password_(!password.empty())
{
}

SSH_Manager::~SSH_Manager()
{
}

bool SSH_Manager::execute_command(const std::string &command, std::string &output)
{
    std::string ssh_cmd = build_ssh_command(command);
    output.clear();
    
    std::array<char, 128> buffer;
    FILE* pipe_raw = popen(ssh_cmd.c_str(), "r");
    std::unique_ptr<FILE, int(*)(FILE*)> pipe(pipe_raw, pclose);
    
    if (!pipe) {
        Logger::log_message(Logger::Type::ERROR, "Failed to execute command");
        return false;
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        output += buffer.data();
    }

    int exit_code = pclose(pipe.release());
    
    return (exit_code == 0);
}

bool SSH_Manager::test_connection()
{
    std::string test_cmd = "echo 'connection_test'";
    std::string output;
    return execute_command(test_cmd,output);
}

std::string SSH_Manager::build_ssh_command(const std::string &remote_command) const
{
    std::ostringstream ssh_cmd;
    
    ssh_cmd << "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "
            << "-o LogLevel=ERROR "
            << "-o BatchMode=yes "
            << "-o ConnectTimeout=5 "
            << "-o PasswordAuthentication=no ";
    
    if (has_password_) {
        ssh_cmd << "-i " << key_ << " ";
    }
    
    ssh_cmd << user_ << "@" << host_ 
            << " '" << remote_command << "'";
    
    return ssh_cmd.str();
}
