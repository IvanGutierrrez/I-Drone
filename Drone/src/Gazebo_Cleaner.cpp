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
#include <vector>
#include <string>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstdio>
#include <cerrno>

namespace {

constexpr const char* kSimulationPidFile = "/opt/I-Drone/data/simulation_processes.pid";

int open_pid_file_secure()
{
    const int fd = open(kSimulationPidFile, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {
        if (errno != ENOENT) {
            Logger::log_message(Logger::Type::WARNING, "Unable to open simulation process file securely");
        }
        return -1;
    }

    struct stat file_stat {};
    if (fstat(fd, &file_stat) != 0) {
        Logger::log_message(Logger::Type::WARNING, "Unable to inspect simulation process file");
        close(fd);
        return -1;
    }

    if (!S_ISREG(file_stat.st_mode)) {
        Logger::log_message(Logger::Type::WARNING, "Simulation process file is not a regular file");
        close(fd);
        return -1;
    }

    const uid_t current_uid = getuid();
    if (file_stat.st_uid != current_uid && file_stat.st_uid != 0) {
        Logger::log_message(Logger::Type::WARNING, "Simulation process file owner is not trusted");
        close(fd);
        return -1;
    }

    if ((file_stat.st_mode & (S_IWGRP | S_IWOTH)) != 0) {
        Logger::log_message(Logger::Type::WARNING, "Simulation process file has unsafe write permissions");
        close(fd);
        return -1;
    }

    return fd;
}

} // namespace

void Gazebo_Cleaner::cleanup() {
    Logger::log_message(Logger::Type::INFO, "Cleaning up simulation processes...");

    const int pid_fd = open_pid_file_secure();
    if (pid_fd < 0) {
        Logger::log_message(Logger::Type::WARNING, "No simulation processes file found");
        return;
    }

    FILE* pid_file = fdopen(pid_fd, "r");
    if (!pid_file) {
        Logger::log_message(Logger::Type::WARNING, "Unable to read simulation processes file");
        close(pid_fd);
        return;
    }
    
    std::string line;
    std::vector<std::pair<std::string, pid_t>> processes;
    
    // Read all processes
    char line_buffer[256] = {0};
    while (std::fgets(line_buffer, sizeof(line_buffer), pid_file) != nullptr) {
        line.assign(line_buffer);
        if (!line.empty() && line.back() == '\n') {
            line.pop_back();
        }

        if (line.empty() || line[0] == '#') continue;
        
        size_t colon_pos = line.find(':');
        if (colon_pos != std::string::npos) {
            std::string name = line.substr(0, colon_pos);
            try {
                pid_t pid = static_cast<pid_t>(std::stol(line.substr(colon_pos + 1)));
                processes.push_back({name, pid});
            } catch (...) {
                Logger::log_message(Logger::Type::WARNING, "Invalid PID entry in simulation process file, skipping");
            }
        }
    }
    std::fclose(pid_file);
    
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
    std::remove(kSimulationPidFile);
    
    Logger::log_message(Logger::Type::INFO, "Simulation cleanup complete");
}
