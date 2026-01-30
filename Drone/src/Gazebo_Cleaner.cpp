/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Gazebo_Cleaner.cpp                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Gazebo_Cleaner.h"
#include "common_libs/Logger.h"
#include <fstream>
#include <vector>
#include <string>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstdlib>

void Gazebo_Cleaner::cleanup() {
    Logger::log_message(Logger::Type::INFO, "Cleaning up simulation processes...");
    
    std::ifstream pid_file("/tmp/simulation_processes.pid");
    if (!pid_file.is_open()) {
        Logger::log_message(Logger::Type::WARNING, "No simulation processes file found");
        return;
    }
    
    std::string line;
    std::vector<std::pair<std::string, pid_t>> processes;
    
    // Read all processes
    while (std::getline(pid_file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        size_t colon_pos = line.find(':');
        if (colon_pos != std::string::npos) {
            std::string name = line.substr(0, colon_pos);
            pid_t pid = std::stoi(line.substr(colon_pos + 1));
            processes.push_back({name, pid});
        }
    }
    pid_file.close();
    
    // Kill all registered processes
    for (const auto& proc : processes) {
        if (proc.second > 0) {
            Logger::log_message(Logger::Type::INFO, "Killing " + proc.first + " (PID: " + std::to_string(proc.second) + ")");
            
            // Try to kill process group first
            pid_t pgid = getpgid(proc.second);
            if (pgid > 0 && pgid != getpgrp()) {
                killpg(pgid, SIGKILL);
            } else {
                kill(proc.second, SIGKILL);
            }
        }
    }
    
    // Give processes time to terminate
    usleep(100000); // 100ms
    
    // Wait for all child processes to avoid zombies
    Logger::log_message(Logger::Type::INFO, "Reaping zombie processes...");
    int status;
    pid_t wpid;
    while ((wpid = waitpid(-1, &status, WNOHANG)) > 0) {
        Logger::log_message(Logger::Type::INFO, "Reaped process: " + std::to_string(wpid));
    }
    
    // Additional cleanup with pkill as fallback
    std::system("pkill -9 gzserver 2>/dev/null");
    std::system("pkill -9 gzclient 2>/dev/null");
    std::system("pkill -9 px4 2>/dev/null");
    
    // Wait again for any remaining zombies
    usleep(100000);
    while ((wpid = waitpid(-1, &status, WNOHANG)) > 0) {
        // Clean remaining zombies
    }
    
    // Remove the tracking file
    std::remove("/tmp/simulation_processes.pid");
    
    Logger::log_message(Logger::Type::INFO, "Simulation cleanup complete");
}
