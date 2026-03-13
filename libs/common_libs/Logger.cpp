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
#include <fstream>
#include <cstdio>

namespace Logger{

namespace {
class Logger_State {
public:
    std::unique_ptr<Recorder> rec;
    bool initialized = false;
};

Logger_State& state()
{
    static Logger_State instance;
    return instance;
}
} // namespace

std::string to_string(const Type &t)
{
    switch (t){
        case Type::INFO :
            return "INFO";
        case Type::WARNING :
            return "WARNING";
        case Type::ERROR :
            return "ERROR";
        default:
            return "";
    }
}

bool initialize(const std::filesystem::path &path,
                       const std::string &name)
{
    auto &logger_state = state();
    try {
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
        }

        std::filesystem::path test_file = path / ".write_test";
        std::ofstream test_stream(test_file);
        if (!test_stream.is_open()) {
            std::cout << "Cannot write to directory: " << path << std::endl;
            return false;
        }
        test_stream.close();
        std::filesystem::remove(test_file);
        
        std::stringstream filename;
        filename << name << getTimeFormatted();
        logger_state.rec = std::make_unique<Recorder>(path,filename.str(),"log");
        logger_state.initialized = true;
        return true;
    } catch (...) {
        return false;
    }
}

std::string getTimeFormatted() { // return time in doyyy_hhmm
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_info {};
    localtime_r(&now_time, &tm_info);

    const int doy = tm_info.tm_yday + 1;
    const int year = tm_info.tm_year % 100;
    char buffer[16];
    std::snprintf(buffer, sizeof(buffer), "%03d%02d_%02d%02d", doy, year, tm_info.tm_hour, tm_info.tm_min);
    return std::string(buffer);
}

std::string getCurrentTimestamp() { // return time in DD/MM/YYYYTHH:MM:SS
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    
    std::tm tm_info {};
    localtime_r(&now_time, &tm_info);

    char buffer[24];
    std::snprintf(buffer,
                  sizeof(buffer),
                  "%02d/%02d/%04dT%02d:%02d:%02d",
                  tm_info.tm_mday,
                  tm_info.tm_mon + 1,
                  tm_info.tm_year + 1900,
                  tm_info.tm_hour,
                  tm_info.tm_min,
                  tm_info.tm_sec);
    return std::string(buffer);
}

void log_message(const Type &t, const std::string &log)
{
    auto &logger_state = state();
    if (!logger_state.initialized || !logger_state.rec)
    {
        std::cout << "The logger is not initialized" << std::endl;
        return;
    }

    std::stringstream message;
    message << getCurrentTimestamp() 
            << std::setw(2) << "" 
            << "[" << std::left << std::setw(8) << to_string(t) << "]" 
            << std::setw(2) << "" << log << std::endl;

    try {
        if (!logger_state.rec->write(message.str()))
        {
            std::cout << "Error while dumping the log" << std::endl;
            return;
        }
    } catch (...) {
        std::cout << "Exception while logging" << std::endl;
    }

    std::cout << message.str();
}

void close()
{
    auto &logger_state = state();
    if (logger_state.initialized && logger_state.rec) {
        logger_state.rec->close();
    }
}

};