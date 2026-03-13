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

namespace Logger{

namespace {
class Logger_State {
public:
    std::unique_ptr<Recorder> rec;
    bool initialized = false;
};

inline Logger_State logger_state;

std::string pad2(int value)
{
    std::string s = std::to_string(value);
    if (s.size() < 2) {
        s.insert(s.begin(), '0');
    }
    return s;
}

std::string pad3(int value)
{
    std::string s = std::to_string(value);
    while (s.size() < 3) {
        s.insert(s.begin(), '0');
    }
    return s;
}

Logger_State& state()
{
    return logger_state;
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
    return pad3(doy) + pad2(year) + "_" + pad2(tm_info.tm_hour) + pad2(tm_info.tm_min);
}

std::string getCurrentTimestamp() { // return time in DD/MM/YYYYTHH:MM:SS
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    
    std::tm tm_info {};
    localtime_r(&now_time, &tm_info);

    return pad2(tm_info.tm_mday) + "/" +
           pad2(tm_info.tm_mon + 1) + "/" +
           std::to_string(tm_info.tm_year + 1900) + "T" +
           pad2(tm_info.tm_hour) + ":" +
           pad2(tm_info.tm_min) + ":" +
           pad2(tm_info.tm_sec);
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