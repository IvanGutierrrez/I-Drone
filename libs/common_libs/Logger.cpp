/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Logger.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Logger.h"
#include <iostream>
#include <chrono>

namespace Logger{

static std::unique_ptr<Recorder> rec_;
static bool initialized_ = false;

std::string to_string(const TYPE &t)
{
    switch (t){
        case TYPE::INFO :
            return "INFO";
        case TYPE::WARNING :
            return "WARNING";
        case TYPE::ERROR :
            return "ERROR";
        default:
            return "";
    }
}

bool initialize(const std::filesystem::path &path,
                       const std::string &name)
{
    try {
        std::stringstream filename;
        filename << name << getTimeFormatted();
        rec_ = std::make_unique<Recorder>(path,filename.str(),"log");
        initialized_ = true;
        return true;
    } catch (...) {
        return false;
    }
}

std::string getTimeFormatted() { // return time in doyyy_hhmm
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm* tm_info = std::localtime(&now_time);

    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%j", tm_info);
    std::string doy(buffer);

    int year = tm_info->tm_year % 100;
    std::ostringstream oss;
    oss << doy << std::setw(2) << std::setfill('0') << year
        << "_" 
        << std::setw(2) << std::setfill('0') << tm_info->tm_hour
        << std::setw(2) << std::setfill('0') << tm_info->tm_min;

    return oss.str();
}

std::string getCurrentTimestamp() { // return time in DD:MM:YYYYTHH:MM:SS
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    
    std::tm* tm_info = std::localtime(&now_time);

    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%d:%m:%YT%H:%M:%S", tm_info);

    return std::string(buffer);
}

void log_message(const TYPE &t, const std::string &log)
{
    if (!initialized_)
    {
        std::cout << "The logger is not initialized" << std::endl;
        return;
    }

    std::stringstream message;
    message << getCurrentTimestamp() 
            << std::setw(2) << "" 
            << "[" << std::left << std::setw(8) << to_string(t) << "]" 
            << std::setw(2) << "" << log << std::endl;

    if (!rec_->write(message.str()))
    {
        std::cout << "Error while dumping the log" << std::endl;
        return;
    }

    std::cout << message.str();
}

};